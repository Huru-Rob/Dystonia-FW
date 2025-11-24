#define NRF_LOG_MODULE_NAME tra 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "huru_ser.h"
#include "storage.h"
#include "epoch.h"
#include "epoch_tra.h"
#include "serial.h"

NRF_LOG_MODULE_REGISTER();

epoch_transmission_status_t epoch_transmission_status;

int32_t epoch_transmission_init(void)
{
    int32_t err = 0;
    
    epoch_transmission_status.state = epoch_transmission_state_idle;
    epoch_transmission_status.send_next = false;
    epoch_transmission_status.send_previous = false;
    epoch_transmission_status.send_complete = false;
    epoch_transmission_status.tail_pending_deletion = false;

    return err;
}

epoch_transmission_state_e epoch_transmission_get_state(void)
{
    return epoch_transmission_status.state;
}

int32_t epoch_transmit_next(void)
{
    int32_t err = 0;
    epoch_transmission_status.send_next = true;
    return err;
}

int32_t epoch_re_transmit(void)
{
    int32_t err = 0;
    epoch_transmission_status.send_previous = true;
    return err;
}

int32_t epoch_transmit_complete(void)
{
    int32_t err = 0;
    epoch_transmission_status.send_complete = true;
    return err;
}

uint8_t transmit_buf[TRANSMISSION_DATA_SIZE + 1];
int32_t epoch_transmission_update(void)
{
    int32_t err = 0;
    static uint32_t tail_file_size = 0;
    static uint8_t messages_remaining = 0;
    uint32_t bytes_read;
    static uint32_t tail_file_bytes_read = 0;
    uint32_t num_epoch_files;

    ble_cus_t m_eeg_cus = get_m_eeg_cus();

    epoch_transmission_state_e next_state = epoch_transmission_status.state;
    switch (epoch_transmission_status.state)
    {

    case epoch_transmission_state_idle:
        if(pt10_status.ble_connected == false && pt10_status.uart_connected == false)
        {
            epoch_transmission_status.send_next = false;
            epoch_transmission_status.send_previous = false;
            epoch_transmission_status.send_complete = false;
        }
        else
        {
            if (
                (pt10_status.ble_connected && pt10_status.MTUSet && pt10_status.epoch_data_notify_en) ||
                (pt10_status.uart_connected)
            )
            {
                if (epoch_transmission_status.send_next == true)
                {
                    epoch_transmission_status.send_next = false;
                    if (epoch_transmission_status.tail_pending_deletion == true)
                    {
                        NRF_LOG_INFO("Next epoch requested. Pending deletion = true. Advancing tail");
                        err = storage_delete_tail_epoch();
                        epoch_transmission_status.tail_pending_deletion = false;
                    }
                    next_state = epoch_transmission_state_open_file;
                }
                else if (epoch_transmission_status.send_complete == true)
                {
                    epoch_transmission_status.send_complete = false;
                    if (epoch_transmission_status.tail_pending_deletion == true)
                    {
                        NRF_LOG_INFO("Epoch Transmission Complete. Pending deletion = true. Advancing tail");
                        err = storage_delete_tail_epoch();
                        epoch_transmission_status.tail_pending_deletion = false;
                    }
                    next_state = epoch_transmission_state_idle;
                }
                else if (epoch_transmission_status.send_previous == true)
                {
                    epoch_transmission_status.send_previous = false;
                    epoch_transmission_status.tail_pending_deletion = false;
                    next_state = epoch_transmission_state_open_file;
                }
            }
        }
        break;

    case epoch_transmission_state_open_file:
        if (!pt10_status.ble_connected && !pt10_status.uart_connected)
        {
            epoch_transmission_status.send_next = false;
            epoch_transmission_status.send_previous = false;
            epoch_transmission_status.send_complete = false;
            next_state = epoch_transmission_state_idle;
        }
        else
        {
            err = storage_num_epoch_files(&num_epoch_files);
            if(err)
            {
                NRF_LOG_WARNING("Error fetching epoch file count");
            }
            if (!err && num_epoch_files != 0)
            {
                err = storage_open_tail(&tail_file_size);
                if(err)
                {
                    NRF_LOG_WARNING("Error %d opening tail file", err);
                }
                else
                {
                    NRF_LOG_INFO("Opened file with %d bytes", tail_file_size);
                }
                tail_file_bytes_read = 0;
                if (!err)
                {
                    messages_remaining = (uint8_t)((tail_file_size-1) / TRANSMISSION_DATA_SIZE);
                    NRF_LOG_INFO("Computed %d messages total", messages_remaining);
                    next_state = epoch_transmission_state_read;
                }
            }
        }
        break;

    case epoch_transmission_state_read:
        if (!pt10_status.ble_connected && !pt10_status.uart_connected)
        {
            err = storage_close_tail();
            if(err)
            {
                NRF_LOG_WARNING("Error %d closing tail file", err);
            }
            epoch_transmission_status.send_next = false;
            epoch_transmission_status.send_previous = false;
            epoch_transmission_status.send_complete = false;
            next_state = epoch_transmission_state_idle;
        }
        else
        {
            NRF_LOG_DEBUG("composing message. %d remaining", messages_remaining);
            transmit_buf[0] = messages_remaining;
            if (tail_file_size - tail_file_bytes_read >= TRANSMISSION_DATA_SIZE)
            {                
                err = storage_read_tail(transmit_buf + 1, TRANSMISSION_DATA_SIZE, &bytes_read);
                if(err)
                {
                    NRF_LOG_WARNING("storage read error %d. Bytes requested %d; Bytes read %d", err, TRANSMISSION_DATA_SIZE, bytes_read);
                }
            }
            else
            {
                err = storage_read_tail(transmit_buf + 1, tail_file_size - tail_file_bytes_read, &bytes_read);
                if(err)
                {
                    NRF_LOG_WARNING("storage read error %d. Bytes requested %d; Bytes read %d", err, tail_file_size - tail_file_bytes_read, bytes_read);
                }
            }
            tail_file_bytes_read += bytes_read;
            next_state = epoch_transmission_state_transmit;            
        }
        break;

    case epoch_transmission_state_transmit:
        if (!pt10_status.ble_connected && !pt10_status.uart_connected)
        {
            err = storage_close_tail();
            if(err)
            {
                NRF_LOG_WARNING("Error %d when closing tail file");
            }
            epoch_transmission_status.send_next = false;
            epoch_transmission_status.send_previous = false;
            epoch_transmission_status.send_complete = false;
            next_state = epoch_transmission_state_idle;
        }
        else
        {
            if(pt10_status.uart_connected)
            {
                err = serial_send_epoch_data(transmit_buf, TRANSMISSION_DATA_SIZE + 1);
                if(!err)
                {
                    if (messages_remaining > 0)
                    {
                        messages_remaining -= 1;
                        NRF_LOG_DEBUG("Placed packet. %d remain.", messages_remaining);
                        next_state = epoch_transmission_state_read;
                    }
                    else
                    {
                        next_state = epoch_transmission_state_close_file;
                    }
                }
                else if(err == NRF_ERROR_NO_MEM)
                {
                    err = 0;
                    next_state = epoch_transmission_state_transmit;
                }
                else
                {
                    NRF_LOG_WARNING("err %d when sending UART data", err);
                    err = storage_close_tail();
                    next_state = epoch_transmission_state_idle;
                }
            }
            else
            {
                // NRF_LOG_DEBUG("Placing message. %d remaining", transmit_buf[0]);
                err = ble_cus_value_update(&m_eeg_cus, m_eeg_cus.epoch_data_handles, transmit_buf, TRANSMISSION_DATA_SIZE + 1, true);
                if(!err)
                {    
                    if (messages_remaining > 0)
                    {
                        messages_remaining -= 1;
                        NRF_LOG_DEBUG("Placed packet. %d remain.", messages_remaining);
                        next_state = epoch_transmission_state_read;
                    }
                    else
                    {
                        next_state = epoch_transmission_state_close_file;
                    }
                }
                else if(err == NRF_ERROR_RESOURCES)
                {
                    // Not really an error. Just no space in the queue because the central is busy
                    err = 0;
                    next_state = epoch_transmission_state_transmit;
                }
                else
                {
                    NRF_LOG_WARNING("err %d when updating characteristic", err);
                    err = storage_close_tail();
                    next_state = epoch_transmission_state_idle;
                }
            }
        }
        break;

    case epoch_transmission_state_close_file:
        NRF_LOG_INFO("Complete - closing file and marking for deletion");
        err = storage_close_tail();
        if(err)
        {
            NRF_LOG_WARNING("Error %d closing tail file", err);
        }
        epoch_transmission_status.tail_pending_deletion = true;
        next_state = epoch_transmission_state_idle;
        break;
    }

    if (epoch_transmission_status.state != next_state)
    {
        NRF_LOG_DEBUG("State %d -> %d", epoch_transmission_status.state, next_state);
        epoch_transmission_status.state = next_state;
    }

    return err;
}