#define NRF_LOG_MODULE_NAME flash 
#define NRF_LOG_LEVEL 4

#include "MT25QL256ABA1EW7.h"
#include <math.h>
#include "hal.h"
#include "board_config.h"
#include "nrfx_spim.h"
#include "board_config.h"
#include "nrf_drv_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

NRF_LOG_MODULE_REGISTER();

uint8_t m_tx_buf[MT25_PAGE_SIZE + 5];
uint8_t m_rx_buf[MT25_PAGE_SIZE + 5];
uint8_t last_write_command = 0;

int32_t flash_read_flag_status_byte(uint8_t *flag_status);
int32_t flash_read_status_byte(uint8_t *status);
int32_t flash_read_sector_protection(uint16_t *sector_protection_reg);

int32_t flash_erase_all(void)
{
    int32_t err = 0;

    uint8_t status_byte;
    err = flash_read_status_byte(&status_byte);
    if(!err)
    {    
        if (
            status_byte & (
                MT25_STATUS_BP_3_MASK |
                MT25_STATUS_BP_2_MASK |
                MT25_STATUS_BP_1_MASK |
                MT25_STATUS_BP_0_MASK
            )
        )
        {
            err = MT25_ERROR_BANK_PROTECTED;
        }
    }

    uint16_t sector_protection_reg;
    err = flash_read_sector_protection(&sector_protection_reg);
    NRF_LOG_INFO("Sector protection: %d", sector_protection_reg);

    // write enable before erase
    uint8_t cmd = FLASH_CMD_WRITE_EN;
    err = hal_spi_write(&cmd, 1);
    if (err != 0)
    {
        return MT25_ERROR_WRITE_ENABLE_FAILED;
    }

    // enable flash reset
    cmd = FLASH_CMD_BULK_ERASE;
    err = hal_spi_write(&cmd, 1);
    last_write_command = cmd;
    if (err != 0)
    {
        return MT25_BULK_ERASE_CMD_FAILED;
    }

    err = flash_read_status_byte(&status_byte);
    NRF_LOG_INFO("status byte after die erase command %d", status_byte);

    wait_for_wip_bit(500);
    uint8_t flag_status;
    flash_read_flag_status_byte(&flag_status);
    if(flag_status & MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK)
    {
        err = MT25_ERROR_BUSY_AFTER_WIP_BIT_CLEAR;
    }

    return err;
}

int32_t flash_read_flag_status_byte(uint8_t *flag_status)
{
    int32_t err = 0;

    uint8_t cmd = FLASH_CMD_READ_FLAG_STATUS;
    uint8_t flag_status_byte[2] = {0x00, 0x00};

    err = hal_spi_transfer(&cmd, 1, flag_status_byte, 2);
    *flag_status = flag_status_byte[1];

    return err;
}

int32_t flash_read_sector_protection(uint16_t *sector_protection_reg)
{
    int32_t err = 0;

    uint8_t cmd = FLASH_CMD_READ_SECTOR_PROTECTION;
    uint8_t res[3] = {0x00, 0x00, 0x00};

    err = hal_spi_transfer(&cmd, 1, res, 3);
    *sector_protection_reg = res[1] + ( res[2] << 8 );

    return err;
}

int32_t flash_read_status_byte(uint8_t *status)
{
    int32_t err = 0;

    uint8_t cmd = FLASH_CMD_READ_STATUS;
    uint8_t status_byte[2] = {0x00, 0x00};

    err = hal_spi_transfer(&cmd, 1, status_byte, 2);
    *status = status_byte[1];

    return err;
}

// TODO: this blocking function needs a timeout and error mechanism 
int32_t wait_for_wip_bit(uint16_t delay)
{
    bool ready = false;
    int32_t err = 0;
    uint8_t flag_status;

    while (!ready && !err)
    {
        err = flash_read_flag_status_byte(&flag_status);
        ready = flag_status & MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK;
        if (delay)
        {
            hal_delay_ms(delay);
        }
    }

    return err;
}

int32_t init_flash()
{
    int32_t err = 0;

    // Allow any write or erase commands to terminate
    // (including Write Non-Voltile Configuration Register)
    // TODO: why does this wait_for_wip_bit() hang? SPI reads 0 when interrogating the status register.
    // wait_for_wip_bit(1);

    // Reset the Flash. 
    // This must be the first operation upon initialising, as the flash may have been
    // put into Deep Power Down mode prior to entering system_power_off.
    
    // enable flash reset
    uint8_t cmd = FLASH_CMD_RESET_ENABLE;
    err = hal_spi_write(&cmd, 1);

    if (err != 0)
    {
        return FLASH_ERROR_RESET_WRITE_ENABLE_FAILED;
    }

    // send reset command
    cmd = FLASH_CMD_RESET_MEMORY;
    err = hal_spi_write(&cmd, 1);

    if (err != 0)
    {
        return FLASH_ERROR_RESET_WRITE_FAILED;
    }

    err = flash_who_ami();
    if(err != 0)
    {
        return FLASH_ID_CHECK_FAILED;
    }

    // enter 4-byte address mode
    cmd = FLASH_CMD_ENTER_4_BYTE_ADDRESS_MODE;
    err = hal_spi_write(&cmd, 1);

    if (err != 0)
    {
        return FLASH_ERROR_4_BYTE_MODE_WRITE_FAILED;
    }

    uint8_t status_byte = 0;
    err = flash_read_status_byte(&status_byte);
    NRF_LOG_INFO("Status Byte: 0x%02x", status_byte);
    if(err != 0)
    {
        return FLASH_ERROR_STATUS_BYTE_READ_FAILED;
    }

    uint8_t flag_status_byte = 0;
    err = flash_read_flag_status_byte(&flag_status_byte);
    NRF_LOG_INFO("Flag Status Byte: 0x%02x", flag_status_byte);
    if(err != 0)
    {
        return FLASH_ERROR_FLAG_STATUS_BYTE_READ_FAILED;
    }

    if (flag_status_byte & MT25_STATUS_FLAG_ADDRESSING_MASK)
    {
        NRF_LOG_INFO("4 byte address mode sel");
    }
    else
    {
        NRF_LOG_INFO("3 byte address mode sel");
        return FLASH_ERROR_STUCK_IN_3_BYTE_MODE;
    }

    return err;
}

static uint8_t who_am_i[21];

int32_t flash_who_ami(void)
{
    int err = 0;
    memset(who_am_i, 0, 20);
    uint8_t cmd = FLASH_CMD_READ_ID;
    err = hal_spi_transfer(&cmd, 1, who_am_i, 21);

    if (err != 0)
    {
        NRF_LOG_INFO("SPI write failed");
        return FLASH_ERROR_ID_READ_FAILED;
    }

    if (who_am_i[1] != MT25_MICRON_MANUFACTURER_ID)
    {
        NRF_LOG_INFO("ID [1] = %d, not %d", who_am_i[1], MT25_MICRON_MANUFACTURER_ID);
        return FLASH_ID_CHECK_FAILED;
    }
    if (who_am_i[2] != MT25_MEMORY_TYPE_ID_3V)
    {
        NRF_LOG_INFO("ID [1] = %d, not %d", who_am_i[2], MT25_MEMORY_TYPE_ID_3V);

        return FLASH_ID_CHECK_FAILED;
    }
    if (who_am_i[3] != MT25_MEMORY_CAPACITY_ID_1GB)
    {
        NRF_LOG_INFO("ID [1] = %d, not %d", who_am_i[3], MT25_MEMORY_CAPACITY_ID_1GB);

        return FLASH_ID_CHECK_FAILED;
    }

    return err;
}

int32_t flash_lower_power_mode(bool set_low_power)
{
    int err = 0;

    if (set_low_power)
    {
        wait_for_wip_bit(0);
        uint8_t cmd = FLASH_CMD_ENTER_DEEP_POWER_DOWN;
        err = hal_spi_write(&cmd, 1);
    }
    else
    {
        uint8_t cmd = FLASH_CMD_RELEASE_FROM_DEEP_POWER_DOWN;
        err = hal_spi_write(&cmd, 1);
    }

    return err;
}

int32_t flash_write_enable(void)
{
    int err = 0;
    uint8_t cmd = FLASH_CMD_WRITE_EN;
    err = hal_spi_write(&cmd, 1);
    return err;
}

int32_t flash_erase_suspend(void)
{
    int err = 0;
    uint8_t cmd = FLASH_CMD_ERASE_SUSPEND;
    err = hal_spi_write(&cmd, 1);
    NRF_LOG_DEBUG("Flash erase suspended");
    return err;
}

int32_t flash_erase_resume(void)
{
    int err = 0;
    uint8_t cmd = FLASH_CMD_ERASE_RESUME;
    err = hal_spi_write(&cmd, 1);
    last_write_command = cmd;
    NRF_LOG_DEBUG("Flash erase resumed");
    return err;
}

int32_t flash_block_read(uint32_t address, uint8_t *buffer, size_t size)
{

    int err = 0;
    bool erase_suspended = false;
    bool erase_resumed = false;

    uint8_t flag_status;        
    err = flash_read_flag_status_byte(&flag_status);
    if(!err)
    {
        if(!(flag_status & MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK))
        {
            if(
                last_write_command == FLASH_CMD_ERASE_RESUME || 
                last_write_command == FLASH_CMD_4_BYTE_64KB_SECTOR_ERASE ||
                last_write_command == FLASH_CMD_4_BYTE_32KB_SUBSECTOR_ERASE || 
                last_write_command == FLASH_CMD_4_BYTE_4KB_SUBSECTOR_ERASE ||
                last_write_command == FLASH_CMD_BULK_ERASE
            )
            {
                err = flash_erase_suspend();
                erase_suspended = true;
                if(!err)
                {
                    bool ready = false;
                    for(uint8_t i = 0; i < 10 && !ready; i++)
                    {
                        err = flash_read_flag_status_byte(&flag_status);
                        if((flag_status & MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK) && (flag_status & MT25_STATUS_FLAG_ERASE_SUSPEND_MASK))
                        {
                            ready = true;
                        }
                        else
                        {
                            hal_delay_us(1000);
                        }
                    }
                    if(!ready)
                    {
                        NRF_LOG_WARNING("Failed to suspend erase operation for read at 0x%08x. Flag Status = 0x%02x", address, flag_status);
                        err = FLASH_ERROR_ERASE_SUSPEND_FAILED;
                    }
                }
            }
            else
            {
                wait_for_wip_bit(0);
            }
        }
    }
    if(!err)
    {        
        m_tx_buf[0] = FLASH_CMD_4_BYTE_READ;
        m_tx_buf[1] = (uint8_t)(address >> 24);
        m_tx_buf[2] = (uint8_t)(address >> 16);
        m_tx_buf[3] = (uint8_t)(address >> 8);
        m_tx_buf[4] = (uint8_t)(address);
        memset(m_rx_buf, 0, size);
        err = hal_spi_transfer(m_tx_buf, 5, m_rx_buf, size + 5);
        memcpy(buffer, &m_rx_buf[5], size);

        hal_wdt_feed();
    }
    if(!err && erase_suspended)
    {
        err = flash_erase_resume();
        erase_resumed = true;
    }

    if(erase_suspended && !erase_resumed)
    {
        NRF_LOG_ERROR("Erase suspended but not resumed in block read of %d", address);
        err = FLASH_ERROR_ERASE_SUSPENDED_BUT_NOT_RESUMED;
    }

    return err;
}

int32_t flash_block_write(uint32_t address, const uint8_t *buffer, size_t size)
{

    int err = 0;
    bool erase_suspended = false;
    bool erase_resumed = false;

    uint8_t flag_status;
    err = flash_read_flag_status_byte(&flag_status);
    if(!err)
    {
        if(!(flag_status & MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK))
        {
            if(
                last_write_command == FLASH_CMD_ERASE_RESUME || 
                last_write_command == FLASH_CMD_4_BYTE_64KB_SECTOR_ERASE ||
                last_write_command == FLASH_CMD_4_BYTE_32KB_SUBSECTOR_ERASE || 
                last_write_command == FLASH_CMD_4_BYTE_4KB_SUBSECTOR_ERASE ||
                last_write_command == FLASH_CMD_BULK_ERASE
            )
            {
                err = flash_erase_suspend();
                erase_suspended = true;

                if(!err)
                {
                    bool ready = false;
                    for(uint8_t i = 0; i < 10 && !ready; i++)
                    {
                        err = flash_read_flag_status_byte(&flag_status);
                        if((flag_status & MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK) && (flag_status & MT25_STATUS_FLAG_ERASE_SUSPEND_MASK))
                        {
                            ready = true;
                        }
                        else
                        {
                            hal_delay_us(1000);
                        }
                    }
                    if(!ready)
                    {
                        NRF_LOG_WARNING("Failed to suspend erase operation for write at 0x%08x. Flag Status = 0x%02x", address, flag_status);
                        err = FLASH_ERROR_ERASE_SUSPEND_FAILED;
                    }
                }
            }
            else
            {
                wait_for_wip_bit(0);
            }
        }
    }
    
    if(!err)
    {
        // write enable
        uint8_t cmd = FLASH_CMD_WRITE_EN;
        err = hal_spi_write(&cmd, 1);
    }
    if (!err)
    {

        m_tx_buf[0] = FLASH_CMD_4_BYTE_PAGE_PROGRAM;
        m_tx_buf[1] = (uint8_t)(address >> 24);
        m_tx_buf[2] = (uint8_t)(address >> 16);
        m_tx_buf[3] = (uint8_t)(address >> 8);
        m_tx_buf[4] = (uint8_t)(address);
        memcpy(&m_tx_buf[5], (uint8_t *)buffer, size);
        last_write_command = m_tx_buf[0];

        hal_wdt_feed();

        wait_for_wip_bit(0);
        err = hal_spi_write(m_tx_buf, 5 + size);

        uint8_t flag_status;            
        if(!err)
        {
            wait_for_wip_bit(0);
            err = flash_read_flag_status_byte(&flag_status);
        }
        if(!err)
        {
            if(flag_status & MT25_STATUS_FLAG_PROGRAM_MASK)
            {
                err = MT25_ERROR_PROGRAM_FAILED;
            }
        }
    }
    if(!err && erase_suspended)
    {
        err = flash_erase_resume();
        erase_resumed = true;
    }

    if(erase_suspended && !erase_resumed)
    {
        NRF_LOG_ERROR("Erase suspended but not resumed in block write of %d", address);
        err = FLASH_ERROR_ERASE_SUSPENDED_BUT_NOT_RESUMED;
    }

    return err;
}

int32_t flash_erase_subsector(uint32_t address)
{
    int err = 0;    

    // NRF_LOG_INFO("Flash block erase block %d", block)
    wait_for_wip_bit(0);
    
    uint8_t cmd = FLASH_CMD_WRITE_EN;
    err = hal_spi_write(&cmd, 1);

    m_tx_buf[0] = FLASH_CMD_4_BYTE_32KB_SUBSECTOR_ERASE;
    m_tx_buf[1] = (uint8_t)(address >> 24);
    m_tx_buf[2] = (uint8_t)(address >> 16);
    m_tx_buf[3] = (uint8_t)(address >> 8);
    m_tx_buf[4] = (uint8_t)(address);
    last_write_command = m_tx_buf[0];

    err = hal_spi_write(m_tx_buf, 5);

    hal_wdt_feed();

    return err;
}

int32_t flash_erase_sector(uint32_t address)
{
    int err = 0;    

    // NRF_LOG_INFO("Flash block erase block %d", block)
    wait_for_wip_bit(0);

    uint8_t flag_status;
    err = flash_read_flag_status_byte(&flag_status);
    if(flag_status & MT25_STATUS_FLAG_ERASE_MASK)
    {
        err = MT25_PREVIOUS_ERASE_SECTOR_FAILED;
    }

    if(!err)
    {    
        uint8_t cmd = FLASH_CMD_WRITE_EN;
        err = hal_spi_write(&cmd, 1);

        m_tx_buf[0] = FLASH_CMD_4_BYTE_64KB_SECTOR_ERASE;
        m_tx_buf[1] = (uint8_t)(address >> 24);
        m_tx_buf[2] = (uint8_t)(address >> 16);
        m_tx_buf[3] = (uint8_t)(address >> 8);
        m_tx_buf[4] = (uint8_t)(address);
        last_write_command = m_tx_buf[0];

        err = hal_spi_write(m_tx_buf, 5);
    }

    hal_wdt_feed();

    return err;
}

int32_t flash_erase_in_progress(bool *in_progress)
{
    int err = 0;

    *in_progress = false;
    uint8_t flag_status;
    err = flash_read_flag_status_byte(&flag_status);
    if(!err)
    {
        if(!(flag_status & MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK))
        {
            if(
                last_write_command == FLASH_CMD_ERASE_RESUME || 
                last_write_command == FLASH_CMD_4_BYTE_64KB_SECTOR_ERASE ||
                last_write_command == FLASH_CMD_4_BYTE_32KB_SUBSECTOR_ERASE || 
                last_write_command == FLASH_CMD_4_BYTE_4KB_SUBSECTOR_ERASE ||
                last_write_command == FLASH_CMD_BULK_ERASE
            )
            {
                *in_progress = true;
            }
        }
    }

    return err;
}
