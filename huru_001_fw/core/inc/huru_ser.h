#ifndef __HURU_SER_H
#define __HURU_SER_H
#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_bas.h"
#include "ble_dis.h"


#define HVN_TX_QUEUE_SIZE (12)
//https://devzone.nordicsemi.com/guides/short-range-guides/b/bluetooth-low-energy/posts/ble-services-a-beginners-tutorial
//https://yupana-engineering.com/online-reverse-byte-array

//RAW
//"a3770000-ef30-11ec-8e0a-0242ac120002"
#define HURU_SERVICE_UUID_BASE          {{0x02, 0x00, 0x12, 0xac, 0x42, 0x02, 0x0a, 0x8e, 0xec, 0x11, 0x30, 0xef, 0x70, 0x2f, 0x77, 0xa3}}
//"a377c019-ef30-11ec-8e0a-0242ac120002"
#define HURU_SERVICE_UUID               (0xc019)
//"a37753f9-ef30-11ec-8e0a-0242ac120002"
#define HURU_CMD_CHARACTERISTIC         (0x53f9)
//"a377f4b6-ef30-11ec-8e0a-0242ac120002"
#define EPOCH_DATA_CHARACTERISTIC       (0xf4b6)
//"a377de40-ef30-11ec-8e0a-0242ac120002"
#define EEG_STREAM_CHARACTERISTIC       (0xDE40)
//"a377de42-ef30-11ec-8e0a-0242ac120002"
#define ACC_STREAM_CHARACTERISTIC       (0xDE42)
//"a377de44-ef30-11ec-8e0a-0242ac120002"
#define CON_STREAM_CHARACTERISTIC       (0xDE44)
//"a377de46-ef30-11ec-8e0a-0242ac120002"
#define LEAD_ONOFF_CHARACTERISTIC       (0xDE46)
//"a377de48-ef30-11ec-8e0a-0242ac120002"
#define LEAD_IMPEDANCE_CHARACTERISTIC   (0xDE48)
//"a377de4a-ef30-11ec-8e0a-0242ac120002"
#define MAG_STREAM_CHARACTERISTIC       (0xDE4A)


// ble_gatts_evt_write_t
#define BLE_CUS_DEF(_name)                                                                          \
static ble_cus_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_cus_on_ble_evt, &_name)

typedef enum
{
    BLE_CUS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_CUS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_CUS_EVT_DISCONNECTED,
    BLE_CUS_EVT_CONNECTED
} ble_cus_evt_type_t;

typedef struct
{
    ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
    uint16_t handle;
} ble_cus_evt_t;

typedef struct ble_cus_s ble_cus_t;

typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);

typedef struct
{
    ble_cus_evt_handler_t         evt_handler;                    
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;

struct ble_cus_s
{
    ble_cus_evt_handler_t         evt_handler;                    
    uint16_t                      eeg_service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    uint16_t                      lead_service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */


    ble_gatts_char_handles_t      epoch_data_handles;                      /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      eeg_stream_handles;                      /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      acc_stream_handles;                      /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      mag_stream_handles;
    ble_gatts_char_handles_t      imp_stream_handles;                      /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      cmd_value_handles;                /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      err_value_handles;                /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      lead_onoff_value_handles;               /**< Handles related to the Custom Value characteristic. */
    ble_gatts_char_handles_t      lead_impedance_value_handles;               /**< Handles related to the Custom Value characteristic. */

    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

typedef struct {
    uint16_t                            connection;
	uint8_t                         	uuid;
    uint16_t                        	service_handle; 
	ble_gatts_char_handles_t        	char_handle_raw; 
	ble_gatts_char_handles_t        	char_handle_control;
	ble_gatts_char_handles_t        	char_handle_error;

} pt10_bt_eeg_service_t;

typedef struct
{
    uint16_t                    conn_handle; 
    uint16_t                    service_handle;        
    // OUR_JOB: Step 2.D, Add handles for our characteristic
    ble_gatts_char_handles_t    char_handles;
} ble_os_t;





/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_cus_init_eeg(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);
uint32_t ble_cus_init_lead(ble_cus_t * p_cus, const ble_cus_init_t * p_cus_init);

int32_t custom_init(void);
void ble_cus_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);
void ble_cus_on_ble_evt_2( ble_evt_t const * p_ble_evt, void * p_context);
ble_cus_t get_m_eeg_cus(void);

bool is_there_space_in_notify_queue(void);
uint32_t ble_cus_value_update(ble_cus_t * p_cus,  ble_gatts_char_handles_t char_handle, uint8_t* custom_value,uint8_t length, bool notify);


// static uint32_t add_char (uint16_t svc_handle, uint8_t base_uuid_id, ble_gatts_char_handles_t* char_handle,  char* desc,  uint16_t size, uint16_t uuid, security_req_t sec, bool read, bool notify, bool write, bool write_wor);



#endif