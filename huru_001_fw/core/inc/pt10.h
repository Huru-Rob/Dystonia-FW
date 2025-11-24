#ifndef PT10_H_INCLUDED
#define PT10_H_INCLUDED
#include <stdint.h>
#include <stdbool.h>

#include "power.h"
#include "AFE4960_ex.h"
#include "afe.h"

#define PT10_ERROR_BASE             (0xD00)
#define PT10_ERROR_BAT_TOO_LOW      (PT10_ERROR_BASE + 0x01)

#define EPOCH_DURATION_SECONDS      (30)
#define ON_DOCK_UPDATE_MS           (1000)
#define IDLE_UPDATE_MS              (1000)
#define SHUTDOWN_UPDATE_MS          (1000)
#define INIT_UPDATE_MS              (1000)
#define RECORDING_UPDATE_MS         (1000)
#define SLEEP_UPDATE_MS             (1000)
#define SYNTHESIZE_UPDATE_MS        (1000)
#define DEEP_SLEEP_PREP_UPDATE_MS   (1000)
#define PANIC_UPDATE_MS             (1000)
#define IDLE_TO_SLEEP_TIMEOUT_MS    (2*60*1000)
#define STREAMING_UPDATE_MS         (1000)
#define HIGH_Z_TIMEOUT_MS           (2*60*1000)
#define SHUTDOWN_PAUSE_MS           (1000)
#define WAKE_FROM_SLEEP_PERIOD_S    (5)
#define RTC_S_TO_TICKS(s)           (s*8)
#define RTC_PRESCALE_FOR_8HZ_TICK   (4095)
#define RTC_PRESCALE_FOR_32HZ_TICK  (1024)

#define POST_BLE_PASS               (1<<0)
#define POST_ADC_PASS               (1<<1)
#define POST_LED_PASS               (1<<2)
#define POST_I2C_PASS               (1<<3)
#define POST_AFE_PASS               (1<<4)
#define POST_RTC_PASS               (1<<5)
#define POST_SPI_PASS               (1<<6)
#define POST_FLASH_PASS             (1<<7)
#define POST_STORAGE_PASS           (1<<8)
#define POST_ACC_PASS               (1<<9)
#define POST_MAG_PASS               (1<<10)
#define POST_ESB_PASS               (1<<11)


//we don't have states for charging, low bat etc. we just get held in the idle state if we fail checks on these
typedef enum {
    PT10_STATE_RESET = 0,       // 0
    PT10_STATE_INIT,            // 1
    PT10_STATE_IDLE,            // 2
    PT10_STATE_STREAMING,       // 3
    PT10_STATE_SLEEP,           // 4
    PT10_STATE_SHUTDOWN,        // 5
    PT10_STATE_RECORDING,       // 6
    PT10_STATE_ERROR,           // 7
    PT10_STATE_ON_DOCK,         // 8
    PT10_STATE_RESTART,         // 9
    PT10_STATE_EMPTY_BATTERY,   // 10
    PT10_STATE_SHUTINGDOWN,     // 11
    PT10_STATE_EXT_CONTROL,     // 12
    PT10_STATE_SYNTHESIZE,      // 13
    PT10_STATE_CURRENT_MEASUREMENT, // 14
    PT10_STATE_DEEP_SLEEP,      // 15
    PT10_STATE_DEEP_SLEEP_PREP, // 16
    PT10_STATE_PANIC,           // 17
}pt10_states_e;

typedef enum{

    PT10_CMD_SEND_NEXT = 2,
    PT10_CMD_SEND_PREVIOUS = 3,
    PT10_CMD_SEND_COMPLETE = 4,

    PT10_CMD_SET_ID = 5,
    PT10_CMD_GET_ID = 6,

    PT10_CMD_SET_TIME = 7,
    PT10_CMD_GET_TIME = 8,

    PT10_CMD_FILES_REMAIN = 9,
    PT10_CMD_BLOCKS_USED = 10,
    PT10_CMD_CLEAN_FS = 11,

    PT10_CMD_GET_LEAD_DETECT_STATUS = 12,
    PT10_CMD_SELECT_SENSOR = 13,
    PT10_CMD_SET_AFE_MODE = 14,
    PT10_CMD_SYNTHESIZE_EPOCHS = 15,
    PT10_CMD_SET_INA_GAINS = 16,
    PT10_CMD_GET_STATE = 17,
    PT10_CMD_GET_VERTICAL_SCALING = 18,
    PT10_CMD_GET_BATTERY = 19,
    PT10_CMD_GET_RESETREASON = 20,
    PT10_CMD_GET_BOOT_TIME = 21,
    PT10_CMD_GET_FIRMWARE_VERSION = 22,
    PT10_CMD_GET_HARDWARE_VERSION = 23,
    PT10_CMD_GET_POST_RESULT = 24,
    PT10_CMD_GET_PANIC_CODE = 25,
    PT10_CMD_GET_STORE_REASON = 26,
    PT10_CMD_GET_OSCILLTRACK_FREQUENCY = 27,
    PT10_CMD_SET_OSCILLTRACK_FREQUENCY = 28,
    PT10_CMD_GET_OSCILLTRACK_TRIGGER_PHASE = 29,
    PT10_CMD_SET_OSCILLTRACK_TRIGGER_PHASE = 30,
    PT10_CMD_GET_OSCILLTRACK_SUPPRESSION_DUTY_CYCLE = 31,
    PT10_CMD_SET_OSCILLTRACK_SUPPRESSION_DUTY_CYCLE = 32,
    PT10_CMD_STORE_OSCILLTRACK_PARAMETERS = 33,
    PT10_CMD_GET_OSCILLTRACK_FLAGS = 34,
    PT10_CMD_SET_OSCILLTRACK_FLAGS = 35,
    PT10_CMD_GET_CONVERGENCE_GAIN = 36,
    PT10_CMD_SET_CONVERGENCE_GAIN = 37,
    PT10_CMD_CONFIGURE_VTS_HAPTIC = 38,
    PT10_CMD_CONFIGURE_VTS_SLEEP = 39,

    PT10_CMD_ENTER_DEEP_SLEEP_MODE = 0xFD,
    PT10_CMD_ENTER_DFU = 0xFE,
    PT10_CMD_ENTER_SHIP_MODE = 0xFF,

} PT10_CMDs_t;

#define DEBUG_NONE                          0
#define DEBUG_EEG                           1
#define DEBUG_ACCELEROMETER                 2
#define DEBUG_IMPEDANCE                     4
#define DEBUG_MAGNETOMETER                  8

#define STICKY_UPDATES                      (5)
#define GOOD_CONNECTION_UPDATES             (5)

#define PT10_LOG_FLUSH(x) while(NRF_LOG_PROCESS(x)){}

typedef struct
{
    pt10_states_e device_state;
    bool MTUSet;
    bool ble_connected;
    bool mtu_updated;
    uint32_t reset_reason;
    uint32_t boot_time;
    bool epoch_data_notify_en;
    bool eeg_stream_notify_en;
    bool acc_stream_notify_en;
    bool mag_stream_notify_en;
    bool imp_stream_notify_en;
    bool lead_onoff_notify_en;
    bool lead_impedance_notify_en;
    bool usb_cdc_connected;
    bool uart_connected;
    bool streaming_mode;
    bool enter_deep_sleep;
    bool synthesize;
    int32_t num_synthesized_epochs;
    uint32_t post_result;
    afe_mode_e afe_mode;

    uint8_t debug_sensor;
} pt10_status_t;

extern pt10_status_t pt10_status;

void pt10_init(void);
void pt10_run(void);
pt10_states_e pt10_get_state(void);
void pt10_send_next_epoch(void);
void pt10_resend_epoch(void);
void pt10_set_afe_mode(afe_mode_e mode);
void pt10_set_streaming_sensor(uint8_t sensor);
int32_t pt10_set_ina_gains(ina_gain_t eeg1_ina_gain, bioz_ina_gain_t eeg2_ina_gain);
void pt10_synthesize_epochs(int32_t num_epochs);
int32_t enter_bootloader_for_dfu(void);
int32_t sw_reset(void);
bool pt10_check_connection_impedance(void);
void pt10_enter_deep_sleep_mode(void);
int32_t pt10_set_device_id(int32_t new_device_id);

// 
pt10_status_t return_device_status(void);
pt10_status_t* return_device_status_pointer(void);
int32_t pt10_cmd_processor(uint8_t *command, uint8_t *response);
void pt10_set_uart_connected(bool connected);
bool pt10_get_uart_connected(void);

#endif