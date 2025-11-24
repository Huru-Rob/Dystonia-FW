#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stdint.h"
#include "stdbool.h"
// #include "pt10.h"

#define BLUETOOTH_ERROR_BASE            (0xC00)
#define BLUETOOTH_ERROR_COMMAND_LENGTH  (BLUETOOTH_ERROR_BASE + 0x01)

typedef enum {
    BLUETOOTH_STATE_ERROR = 0,
    BLUETOOTH_STATE_UNKNOWN = 1,

} pt10_bluetooth_state_t;
int32_t bluetooth_init(void);
int32_t eeg_reply_to_write(uint8_t * reply_msg);

int32_t pt10_bt_eeg_command_irq(uint8_t*data, uint8_t len); 
int32_t bluetooth_buffer_handler(void);
int32_t add_handle_to_notification_queue(uint16_t *handle);
int32_t get_handle_from_notification_queue(uint16_t *handle);
int32_t hvn_tx_buffers_free(void);

int32_t request_to_send_stop_message(void);
bool has_stop_message_been_read(void);

pt10_bluetooth_state_t get_bluetooth_state(void);
// void set_bluetooth_state(pt10_bluetooth_state_t bluetooth_state);

#endif