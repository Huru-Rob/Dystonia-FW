#define NRF_LOG_MODULE_NAME ble
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "bluetooth.h"
#include "nrf_queue.h"
#include "pt10.h"
#include "huru_ser.h"
#include "nrf_log.h"
#include "hal.h"
#include "board_config.h"

NRF_LOG_MODULE_REGISTER();

NRF_QUEUE_DEF(uint8_t, m_eeg_command_buf, 32, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(uint16_t, m_notification_queue, 8, NRF_QUEUE_MODE_NO_OVERFLOW);

int32_t add_handle_to_notification_queue(uint16_t *handle)
{
    int32_t err = 0;

    err = nrf_queue_write(&m_notification_queue, handle, 1);
    NRF_LOG_DEBUG("Push: %d (err = %d)", *handle, err);

    return err;
}

int32_t get_handle_from_notification_queue(uint16_t *handle)
{
    int32_t err = 0;

    err = nrf_queue_read(&m_notification_queue, handle, 1);

    return err;
}

int32_t hvn_tx_buffers_free(void)
{
    return HVN_TX_QUEUE_SIZE - nrf_queue_utilization_get(&m_notification_queue);
}

int32_t bluetooth_init(void)
{
    int32_t err = 0;

    nrf_queue_reset(&m_eeg_command_buf);
    nrf_queue_reset(&m_notification_queue);

    return err;
}

pt10_bluetooth_state_t _bluetooth_state = BLUETOOTH_STATE_UNKNOWN;

int32_t bluetooth_buffer_handler(void)
{
    int32_t err = 0;

    // eeg first
    uint8_t data[5];
    size_t length = 5;
    int32_t queue_ret = nrf_queue_read(&m_eeg_command_buf, data, length);

    // const to point to the command in the received data
    const uint8_t command_position = 0;

    if (queue_ret == NRF_SUCCESS)
    {
        uint8_t reply_msg[5];
        memset(&reply_msg, 0, 5);

        reply_msg[0] = data[command_position];
        NRF_LOG_INFO("Command");
        NRF_LOG_HEXDUMP_INFO(data, 5);
        pt10_cmd_processor(data, reply_msg);
        NRF_LOG_INFO("Response");
        NRF_LOG_HEXDUMP_INFO(reply_msg, 5);
        eeg_reply_to_write(reply_msg);
    }
    else
    {
        // No messages in the queue
    }

    return err;
}

int32_t send_reply_msg(int16_t connection, uint16_t characteristic, uint8_t *data, uint16_t length)
{
    int32_t err = 0;
    ble_gatts_hvx_params_t params_hvx;

    if (connection == BLE_CONN_HANDLE_INVALID)
    {
        return BLE_CONN_HANDLE_INVALID;
    }

    memset(&params_hvx, 0, sizeof(params_hvx));
    params_hvx.handle = characteristic;
    params_hvx.p_data = data;
    params_hvx.p_len = &length;
    params_hvx.type = BLE_GATT_HVX_NOTIFICATION;
    err = sd_ble_gatts_hvx(connection, &params_hvx);
    if (!err)
    {
        NRF_LOG_DEBUG("Command reply handle = %d", params_hvx.handle);
        add_handle_to_notification_queue(&params_hvx.handle);
    }

    return err;
}

int32_t eeg_reply_to_write(uint8_t *reply_msg)
{
    int32_t err = 0;
    ble_cus_t m_cus_eeg = get_m_eeg_cus();

    err = send_reply_msg(m_cus_eeg.conn_handle, m_cus_eeg.cmd_value_handles.value_handle, reply_msg, 5);
    return err;
}

pt10_bluetooth_state_t get_bluetooth_state(void)
{
    return _bluetooth_state;
}

int32_t pt10_bt_eeg_command_irq(uint8_t *data, uint8_t len)
{
    int32_t err = 0;
    if (len != 5)
    {
        return BLUETOOTH_ERROR_COMMAND_LENGTH;
    }
    size_t length = 5;

    err = nrf_queue_write(&m_eeg_command_buf, data, length);

    return err;
}