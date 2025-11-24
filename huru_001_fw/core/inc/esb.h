#ifndef ESB_H_INCLUDED
#define ESB_H_INCLUDED

#define NUM_PRX_PERIPHERALS         (2)
#define ESB_PIPE_BRIDGE             (0)
#define ESB_PIPE_VTS                (0)
#define ESB_PING_MS                 (1000)

typedef enum {
    vts_command_send_haptic = 0x00,
    vts_command_configure_haptic = 0x01,
    vts_command_configure_sleep = 0x02
} vts_command_e;

typedef enum {
    prx_peripheral_stimulator = 0,
    prx_peripheral_usb_bridge,
    prx_peripheral_none
} prx_peripheral_e;

typedef enum {
    esb_payload_command = 1,
    esb_payload_command_response = 2,
    esb_payload_oscilltrack_data = 3,
    esb_payload_timeout = 4,
    esb_payload_ping = 5,
    esb_payload_ping_response = 6,
    esb_payload_bridge_command = 7
} esb_payload_type_e;

typedef struct __attribute__ ((__packed__)) {
    uint8_t payload_type;
    uint8_t payload_length;
} esb_packet_header_t;

typedef union {
    esb_packet_header_t header;
    uint8_t bytes[32]; 
} esb_packet_t;

typedef enum {
    esb_state_reset = 0,
    esb_state_unconnected,
    esb_state_unconnected_wait_tx_done,
    esb_state_connected,
    esb_state_connected_wait_tx_done,
} esb_state_e;

typedef struct {
    esb_state_e     state;    
    bool            stimulus_ready;
    bool            oscilltrack_data_ready;
    bool            command_response_ready;
    bool            vts_command_ready;
    bool            tx_done;
    bool            ping_timeout;
    bool            ping_response;
} esb_status_t;

int32_t esb_init(void);
void esb_start_rx(void);
int32_t esb_update(void);
int32_t esb_set_oscilltrack_data(uint8_t *data, uint8_t length);
int32_t esb_set_stimulus(void);
void esb_configure_vts_haptic(uint8_t *haptic_parameters);
void esb_configure_vts_sleep(uint8_t *sleep_parameters);

#endif