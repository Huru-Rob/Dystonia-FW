#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#define UART_BUF_SIZE   (256)
#define HELLO_SIZE      (5)
#define PING_TIMEOUT_MS (50)
#define RX_TX_TURNAROUND_MS (2)
// The connection timeout needs to accommodate the second UART connection
// available on our own UART, as it might be awaiting a timeout from 
// an empty slot. That's why it's 250ms and not 150ms
#define UART_CONNECTION_TIMEOUT_MS  (250)
#define SERIAL_ERR_NO_ROOM_IN_QUEUE (-15)

typedef enum {
    serial_state_idle = 0,
    serial_state_rx,
    serial_state_wait_rx_abort,
    serial_state_tx,    
    serial_state_wait_tx_done
} serial_state_e;

typedef enum {
    ping_timeout_none = 0,
    ping_timeout_ping,
    ping_timeout_rxtx_turnaround,
    ping_timeout_cmd
} ping_timeout_e;

typedef enum {
    serial_payload_none = 0,
    serial_payload_device_id,
    serial_payload_command,
    serial_payload_command_response,
    serial_payload_sensor_data,
    serial_payload_epoch_data,
    serial_payload_command_timeout,
    serial_payload_ping,
    serial_payload_ping_response,
    serial_payload_crc_error, // for use by the multi-dock to signal to app
    serial_payload_device_id_timeout,
    serial_payload_ping_timeout
} serial_payload_type_e;

typedef struct __attribute__ ((__packed__)) {
    uint32_t device_id;
    uint8_t payload_type;
    uint8_t payload_length;
} serial_packet_header_t;

typedef union {
    serial_packet_header_t header;
    uint8_t bytes[256];
} serial_packet_t;

void serial_init(void);
void serialStopRx(void);
int32_t serialStartRx(void);
void serialUpdate(void);
void serialStart(void);
void serialStop(void);
int32_t serial_send_stream_data(uint8_t *data, uint32_t length);
int32_t serial_send_epoch_data(uint8_t *data, uint32_t length);

#endif