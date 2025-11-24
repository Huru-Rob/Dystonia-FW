#define NRF_LOG_MODULE_NAME ser 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <string.h>
#include "sdk_common.h"
#include "ble_srv_common.h"
#include "huru_ser.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "pt10.h"
#include "epoch.h"
#include "bluetooth.h"

NRF_LOG_MODULE_REGISTER();

BLE_CUS_DEF(m_cus_eeg);

ble_cus_t get_m_eeg_cus(void) { return m_cus_eeg; }

static void on_cus_evt_eeg(ble_cus_t *p_cus_service,
                           ble_cus_evt_t *p_evt)
{
    NRF_LOG_DEBUG("cus evt eeg");
    switch (p_evt->evt_type)
    {
    case BLE_CUS_EVT_NOTIFICATION_ENABLED:
        
        if (p_evt->handle == p_cus_service->epoch_data_handles.cccd_handle)
        {
            NRF_LOG_INFO("epoch data notify 1");
            pt10_status.epoch_data_notify_en = true;
        }
        if (p_evt->handle == p_cus_service->lead_onoff_value_handles.cccd_handle)
        {
            NRF_LOG_INFO("lead on/off notify 1");
            pt10_status.lead_onoff_notify_en = true;
        }
        if (p_evt->handle == p_cus_service->lead_impedance_value_handles.cccd_handle)
        {
            NRF_LOG_INFO("lead impedance notify 1");
            pt10_status.lead_impedance_notify_en = true;
        }
        if (p_evt->handle == p_cus_service->eeg_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("eeg stream notify 1");
            pt10_status.eeg_stream_notify_en = true;
        }
        if (p_evt->handle == p_cus_service->acc_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("acc stream notify 1");
            pt10_status.acc_stream_notify_en = true;
        }
        if (p_evt->handle == p_cus_service->imp_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("imp stream notify 1");
            pt10_status.imp_stream_notify_en = true;
        }
        if (p_evt->handle == p_cus_service->mag_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("mag stream notify 1");
            pt10_status.mag_stream_notify_en = true;
        }

        break;

    case BLE_CUS_EVT_NOTIFICATION_DISABLED:

        if (p_evt->handle == p_cus_service->epoch_data_handles.cccd_handle)
        {
            NRF_LOG_INFO("epoch data notify 0");
            pt10_status.epoch_data_notify_en = false;
        }
        if (p_evt->handle == p_cus_service->lead_onoff_value_handles.cccd_handle)
        {
            NRF_LOG_INFO("lead on/off notify 0");
            pt10_status.lead_onoff_notify_en = false;
        }
        if (p_evt->handle == p_cus_service->lead_impedance_value_handles.cccd_handle)
        {
            NRF_LOG_INFO("lead impedance notify 0");
            pt10_status.lead_impedance_notify_en = false;
        }
        if (p_evt->handle == p_cus_service->eeg_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("eeg stream notify 0");
            pt10_status.eeg_stream_notify_en = false;
        }
        if (p_evt->handle == p_cus_service->acc_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("acc stream notify 0");
            pt10_status.acc_stream_notify_en = false;
        }
        if (p_evt->handle == p_cus_service->acc_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("imp stream notify 0");
            pt10_status.imp_stream_notify_en = false;
        }
        if (p_evt->handle == p_cus_service->mag_stream_handles.cccd_handle)
        {
            NRF_LOG_INFO("mag stream notify 0");
            pt10_status.mag_stream_notify_en = false;
        }
        break;

    case BLE_CUS_EVT_CONNECTED:
        NRF_LOG_DEBUG("eeg cus conn");
        break;

    case BLE_CUS_EVT_DISCONNECTED:
        NRF_LOG_DEBUG("eeg cus disconn");
        break;

    default:
        // No implementation needed.
        break;
    }
}

int32_t custom_init(void)
{

    ret_code_t err_code;

    ble_cus_init_t cus_init_eeg;
    memset(&cus_init_eeg, 0, sizeof(cus_init_eeg));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init_eeg.custom_value_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init_eeg.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init_eeg.custom_value_char_attr_md.write_perm);

    // Initialize CUS Service init structure to zero.
    cus_init_eeg.evt_handler = on_cus_evt_eeg;
    err_code = ble_cus_init_eeg(&m_cus_eeg, &cus_init_eeg);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

static uint32_t custom_value_char_add(ble_cus_t *p_cus, uint16_t target_service_handle, ble_gatts_char_handles_t *target_char_handle, const ble_cus_init_t *p_cus_init, const uint16_t char_uuid, uint16_t max_length)
{
    uint32_t err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t attr_char_value;
    ble_uuid_t ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    // cccd_md.write_perm = p_cus_init->custom_value_char_attr_md.cccd_write_perm;
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = &cccd_md;
    char_md.p_sccd_md = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm = p_cus_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_cus_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc = BLE_GATTS_VLOC_STACK; // Note: not BLE_GATTS_VLOC_USER - so we do not have to provide a static RAM buffer for the value
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = char_uuid;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = max_length;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = max_length;

    err_code = sd_ble_gatts_characteristic_add(target_service_handle, &char_md,
                                               &attr_char_value,
                                               target_char_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_cus_init_eeg(ble_cus_t *p_cus, const ble_cus_init_t *p_cus_init)
{
    if (p_cus == NULL || p_cus_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code;
    ble_uuid_t ble_uuid;

    p_cus->evt_handler = p_cus_init->evt_handler;
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = HURU_SERVICE_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cus->uuid_type;
    ble_uuid.uuid = HURU_SERVICE_UUID;

    // Initialize service structure

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->eeg_service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // our_char_add(p_cus);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->cmd_value_handles, p_cus_init, HURU_CMD_CHARACTERISTIC, 5);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->epoch_data_handles, p_cus_init, EPOCH_DATA_CHARACTERISTIC, 244);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->eeg_stream_handles, p_cus_init, EEG_STREAM_CHARACTERISTIC, 244);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->acc_stream_handles, p_cus_init, ACC_STREAM_CHARACTERISTIC, 244);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->imp_stream_handles, p_cus_init, CON_STREAM_CHARACTERISTIC, 244);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->lead_onoff_value_handles, p_cus_init, LEAD_ONOFF_CHARACTERISTIC, 1);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->lead_impedance_value_handles, p_cus_init, LEAD_IMPEDANCE_CHARACTERISTIC, 4);
    err_code = custom_value_char_add(p_cus, p_cus->eeg_service_handle, &p_cus->mag_stream_handles, p_cus_init, MAG_STREAM_CHARACTERISTIC, 244);

    return err_code;
}

static void _on_write(ble_cus_t *p_cus, ble_evt_t const *p_ble_evt)
{
    // ble_gatts_evt_write_t p_evt_write = p_ble_evt->evt.gatts_evt.params.write;
    ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    uint8_t data[5];
    memset(data, 0, 5);
    uint32_t length = p_evt_write->len;
    memcpy(data, p_evt_write->data, length);

    NRF_LOG_DEBUG("onwrite %u to handle %d",length, p_evt_write->handle);

    // eeg cmd
    //  if (p_evt_write.handle == m_cus_eeg.cmd_value_handles.value_handle)
    if (p_evt_write->handle == p_cus->cmd_value_handles.value_handle)
    {

        NRF_LOG_DEBUG("Write to the command characteristic");
        pt10_bt_eeg_command_irq(data, length);
    }

    // if (p_evt_write.handle == m_cus_info.info_value_handles.value_handle)
    // {
    //     pt10_bt_info_command_irq(data,length);

    // }

    // // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if (
        (
            (p_evt_write->handle == p_cus->epoch_data_handles.cccd_handle) ||
            (p_evt_write->handle == p_cus->lead_onoff_value_handles.cccd_handle) ||
            (p_evt_write->handle == p_cus->lead_impedance_value_handles.cccd_handle) ||
            (p_evt_write->handle == p_cus->eeg_stream_handles.cccd_handle) ||
            (p_evt_write->handle == p_cus->acc_stream_handles.cccd_handle) ||
            (p_evt_write->handle == p_cus->imp_stream_handles.cccd_handle) ||
            (p_evt_write->handle == p_cus->mag_stream_handles.cccd_handle)
            
        ) &&
        (p_evt_write->len == 2))
    {

        // CCCD written, call application event handler
        if (p_cus->evt_handler != NULL)
        {

            ble_cus_evt_t evt;

            evt.handle = p_evt_write->handle;
            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CUS_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_cus->evt_handler(p_cus, &evt);
        }
    }
}

static void _on_connect(ble_cus_t *p_cus, ble_evt_t const *p_ble_evt)
{
    NRF_LOG_DEBUG("_on_connect");
    if (p_cus->evt_handler != NULL)
    {
        p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        NRF_LOG_DEBUG("Stored conn handle %d", p_ble_evt->evt.gap_evt.conn_handle);
        ble_cus_evt_t evt;

        evt.evt_type = BLE_CUS_EVT_CONNECTED;

        p_cus->evt_handler(p_cus, &evt);
    }
}

static void _on_disconnect(ble_cus_t *p_cus, ble_evt_t const *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_cus_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
    ble_cus_t *p_cus = (ble_cus_t *)p_context;

    if (p_cus == NULL || p_ble_evt == NULL)
    {
        NRF_LOG_INFO("xxx");

        return;
    }
    NRF_LOG_DEBUG("evt 1: id 0x%02x", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        _on_connect(p_cus, p_ble_evt);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        _on_disconnect(p_cus, p_ble_evt);
        break;
    case BLE_GATTS_EVT_WRITE:
        _on_write(p_cus, p_ble_evt);
        break;
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    {
        NRF_LOG_DEBUG("notify complete eeg");

        ble_gatts_evt_hvn_tx_complete_t const *tx_complete = &p_ble_evt->evt.gatts_evt.params.hvn_tx_complete;

        NRF_LOG_DEBUG("%d bufs released", tx_complete->count);
        for (uint8_t i = 0; i < tx_complete->count; i++)
        {
            uint16_t handle;
            int32_t err;
            err = get_handle_from_notification_queue(&handle);

            if (!err)
            {
                ble_cus_t m_eeg_cus = get_m_eeg_cus();

                if (handle == m_eeg_cus.epoch_data_handles.value_handle)
                {
                    NRF_LOG_DEBUG("Released buffer with handle %d for epoch data", handle);
                }
                if (handle == m_eeg_cus.lead_impedance_value_handles.value_handle)
                {
                    NRF_LOG_DEBUG("Released buffer with handle %d for lead impedance value", handle);
                }
                if (handle == m_eeg_cus.lead_onoff_value_handles.value_handle)
                {
                    NRF_LOG_DEBUG("Released buffer with handle %d for lead on/off value", handle);
                }
            }
        }
    }
    break;

    default:
        // No implementation needed.
        // NRF_LOG_INFO("%d",p_ble_evt->header.evt_id);
        // if(p_ble_evt->header.evt_id == 58){
        //     NRF_LOG_INFO("err intresting");

        // }

        break;
    }
}

uint32_t ble_cus_value_update(ble_cus_t *p_cus, ble_gatts_char_handles_t char_handle, uint8_t *custom_value, uint8_t length, bool notify)
{

    if (p_cus == NULL)
    {
        return NRF_ERROR_NULL;
    }

    NRF_LOG_DEBUG("updating a cus value. conn_handle: %d, char_handle: %d, ", p_cus->conn_handle, char_handle.value_handle);

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len = length;
    gatts_value.offset = 0;
    gatts_value.p_value = custom_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cus->conn_handle,
                                      char_handle.value_handle,
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("gatts value set err: %i", err_code);
        return err_code;
    }

    if (notify == true)
    {

        if ((p_cus->conn_handle != BLE_CONN_HANDLE_INVALID))
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = char_handle.value_handle;
            hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_params);
            if (!err_code)
            {
                NRF_LOG_DEBUG("Cus value update handle = %d", hvx_params.handle);
                add_handle_to_notification_queue(&hvx_params.handle);
            }
            else
            {
                if(err_code != NRF_ERROR_RESOURCES)
                {
                    NRF_LOG_WARNING("gatts hvx err: %i", err_code);
                }
                return err_code;
            }
        }
        else
        {
            NRF_LOG_WARNING("Invalid handle: %d", p_cus->conn_handle);
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}