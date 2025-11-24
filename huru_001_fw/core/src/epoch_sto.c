#define NRF_LOG_MODULE_NAME sto 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "storage.h"
#include "MT25QL256ABA1EW7.h"
#include "epoch.h"
#include "epoch_sto.h"

NRF_LOG_MODULE_REGISTER();

epoch_storing_status_t epoch_storing_status;

int32_t epoch_storage_init(void)
{
    int32_t err = 0;

    epoch_storing_status.state = epoch_storing_state_idle;
    
    return err;
}

int32_t epoch_storage_store(epoch_buffer_t *p_buffer)
{
    int32_t err = 0;

    if(epoch_storing_status.state != epoch_storing_state_idle)
    {
        NRF_LOG_INFO("Storage machine not ready");
        err = STO_ERROR_MACHINE_NOT_READY;
    }
    if(!err)
    {
        epoch_storing_status.p_epoch_storing = p_buffer;
    }

    return err;
}

epoch_storing_state_e epoch_storage_get_state(void)
{
    return epoch_storing_status.state;
}

int32_t epoch_storage_update(void)
{
    int32_t err = 0;
    uint32_t bytes_left_in_page = 0;
    static uint32_t section_length = 0;
    static uint32_t total_bytes_written = 0;
    static uint32_t pages_written = 0;
    static uint32_t section_bytes_written = 0;
    static uint8_t *p_section;
    uint32_t bytes_written;
    uint32_t bytes_left_in_section;
    epoch_buffer_t *p_epoch_storing = epoch_storing_status.p_epoch_storing;
    epoch_t *p_epoch = &(p_epoch_storing->epoch);

    err = storage_garbage_collector();
    if(!err)
    {

        epoch_storing_state_e next_state = epoch_storing_status.state;
        switch (epoch_storing_status.state)
        {
        case epoch_storing_state_idle:
            if (epoch_storing_status.p_epoch_storing != NULL)
            {
                next_state = epoch_storing_state_create_file;
            }
            break;

        case epoch_storing_state_create_file:
            err = storage_open_head(
                p_epoch->header_section.section_length +
                p_epoch->eeg_section.header.section_length +
                p_epoch->acc_x_section.header.section_length +
                p_epoch->acc_y_section.header.section_length +
                p_epoch->acc_z_section.header.section_length +
                p_epoch->mag_x_section.header.section_length +
                p_epoch->mag_y_section.header.section_length +
                p_epoch->mag_z_section.header.section_length +
                p_epoch->imp_section.header.section_length +
#if HURU_PROTOCOL_VERSION == 0x0103
                p_epoch->bat_section.header.section_length +
                p_epoch->temp_section.header.section_length +
#endif
                p_epoch->footer_section.section_length
            );
            if(err)
            {
                NRF_LOG_WARNING("Error %d opening head file %d", err, p_epoch->header_section.epoch_id);
            }
            else
            {
                NRF_LOG_INFO("Opened head file %d", p_epoch->header_section.epoch_id); //  p_epoch->header_section.epoch_id
            }
            total_bytes_written = 0;
            section_bytes_written = 0;
            section_length = p_epoch->header_section.section_length;
            NRF_LOG_DEBUG("Header Section: %d", section_length);
            p_section = (uint8_t *)&(p_epoch->header_section);
            next_state = epoch_storing_state_write_header_section;
            break;

        case epoch_storing_state_write_header_section:
        case epoch_storing_state_write_eeg_section:
        case epoch_storing_state_write_acc_x_section:
        case epoch_storing_state_write_acc_y_section:
        case epoch_storing_state_write_acc_z_section:
        case epoch_storing_state_write_mag_x_section:
        case epoch_storing_state_write_mag_y_section:
        case epoch_storing_state_write_mag_z_section:
        case epoch_storing_state_write_imp_section:
        case epoch_storing_state_write_bat_section:
        case epoch_storing_state_write_temp_section:
        case epoch_storing_state_write_footer_section:
            pages_written = total_bytes_written / MT25_PAGE_SIZE;
            bytes_left_in_page = MT25_PAGE_SIZE * (1+pages_written) - total_bytes_written;
            bytes_left_in_section = section_length - section_bytes_written;
            if (bytes_left_in_section > bytes_left_in_page)
            {
                NRF_LOG_DEBUG("Writing %d", bytes_left_in_page);
                err = storage_write_head(p_section + section_bytes_written, bytes_left_in_page, &bytes_written);
                if(err)
                {
                    NRF_LOG_WARNING("Error %d writing head file", err);
                }
                if (bytes_written != bytes_left_in_page)
                {
                    NRF_LOG_WARNING("Epoch file write incorrect byte count");
                    err = STO_ERROR_INCORRECT_WRITE_COUNT_1;
                }
                section_bytes_written += bytes_written;
                total_bytes_written += bytes_written;
            }
            else
            {
                NRF_LOG_DEBUG("Writing %d", bytes_left_in_section);
                err = storage_write_head(p_section + section_bytes_written, bytes_left_in_section, &bytes_written);
                if(err)
                {
                    NRF_LOG_WARNING("Error %d writing head file", err);
                }
                if (bytes_written != bytes_left_in_section)
                {
                    NRF_LOG_WARNING("Epoch file write incorrect byte count");
                    err = STO_ERROR_INCORRECT_WRITE_COUNT_2;
                }
                section_bytes_written = 0;
                total_bytes_written += bytes_written;
                switch (epoch_storing_status.state)
                {
                case epoch_storing_state_write_header_section:
                    p_section = (uint8_t *)&(p_epoch->eeg_section);                
                    section_length = p_epoch->eeg_section.header.section_length;
                    NRF_LOG_DEBUG("EEG Section: %d", section_length);
                    next_state = epoch_storing_state_write_eeg_section;
                    break;
                case epoch_storing_state_write_eeg_section:
                    p_section = (uint8_t *)&(p_epoch->acc_x_section);
                    section_length = p_epoch->acc_x_section.header.section_length;
                    NRF_LOG_DEBUG("ACC X Section: %d", section_length);
                    next_state = epoch_storing_state_write_acc_x_section;
                    break;
                case epoch_storing_state_write_acc_x_section:
                    p_section = (uint8_t *)&(p_epoch->acc_y_section);
                    section_length = p_epoch->acc_y_section.header.section_length;
                    NRF_LOG_DEBUG("ACC Y Section: %d", section_length);
                    next_state = epoch_storing_state_write_acc_y_section;
                    break;
                case epoch_storing_state_write_acc_y_section:
                    p_section = (uint8_t *)&(p_epoch->acc_z_section);
                    section_length = p_epoch->acc_z_section.header.section_length;
                    NRF_LOG_DEBUG("ACC_Z Section: %d", section_length);
                    next_state = epoch_storing_state_write_acc_z_section;
                    break;
                case epoch_storing_state_write_acc_z_section:
                    p_section = (uint8_t *)&(p_epoch->mag_x_section);
                    section_length = p_epoch->mag_x_section.header.section_length;
                    NRF_LOG_DEBUG("MAG X Section: %d", section_length);
                    next_state = epoch_storing_state_write_mag_x_section;
                    break;
                case epoch_storing_state_write_mag_x_section:
                    p_section = (uint8_t *)&(p_epoch->mag_y_section);
                    section_length = p_epoch->mag_y_section.header.section_length;
                    NRF_LOG_DEBUG("MAG Y Section: %d", section_length);
                    next_state = epoch_storing_state_write_mag_y_section;
                    break;
                case epoch_storing_state_write_mag_y_section:
                    p_section = (uint8_t *)&(p_epoch->mag_z_section);
                    section_length = p_epoch->mag_z_section.header.section_length;
                    NRF_LOG_DEBUG("MAG_Z Section: %d", section_length);
                    next_state = epoch_storing_state_write_mag_z_section;
                    break;
                case epoch_storing_state_write_mag_z_section:
                    p_section = (uint8_t *)&(p_epoch->imp_section);
                    section_length = p_epoch->imp_section.header.section_length;
                    NRF_LOG_DEBUG("IMP Section: %d", section_length);
                    next_state = epoch_storing_state_write_imp_section;
                    break;
                case epoch_storing_state_write_imp_section:
#if HURU_PROTOCOL_VERSION == 0x0103
                    p_section = (uint8_t *)&(p_epoch->bat_section);
                    section_length = p_epoch->bat_section.header.section_length;
                    NRF_LOG_DEBUG("BAT Section: %d", section_length);
                    next_state = epoch_storing_state_write_bat_section;
#else
                    p_section = (uint8_t *)&(p_epoch->footer_section);
                    section_length = p_epoch->footer_section.section_length;
                    NRF_LOG_DEBUG("FOOTER Section: %d", section_length);
                    next_state = epoch_storing_state_write_footer_section;
#endif
                    break;
                case epoch_storing_state_write_bat_section:
                    p_section = (uint8_t *)&(p_epoch->temp_section);
                    section_length = p_epoch->temp_section.header.section_length;
                    NRF_LOG_DEBUG("TEMP Section: %d", section_length);
                    next_state = epoch_storing_state_write_temp_section;
                    break;
                case epoch_storing_state_write_temp_section:
                    p_section = (uint8_t *)&(p_epoch->footer_section);
                    section_length = p_epoch->footer_section.section_length;
                    NRF_LOG_DEBUG("FOOTER Section: %d", section_length);
                    next_state = epoch_storing_state_write_footer_section;
                case epoch_storing_state_write_footer_section:
                    next_state = epoch_storing_state_close_file;
                    break;
                default:
                    next_state = epoch_storing_state_idle;
                    break;
                }
            }
            break;

        case epoch_storing_state_close_file:
            err = storage_close_head();
            if(err)
            {
                NRF_LOG_WARNING("Error %d closing head file");
            }
            else
            {
                NRF_LOG_INFO("Closed head file %d", p_epoch->header_section.epoch_id);
            }
            epoch_buffer_free(epoch_storing_status.p_epoch_storing);
            epoch_storing_status.p_epoch_storing = NULL;
            next_state = epoch_storing_state_idle;
            break;
        }

        if (epoch_storing_status.state != next_state)
        {
            NRF_LOG_DEBUG("State %d -> %d", epoch_storing_status.state, next_state);
            epoch_storing_status.state = next_state;
        }
    }

    return err;
}