#ifndef LED_H_INCLUDED
#define LED_H_INCLUDED

#define LED_ERROR_BASE          (0x700)
#define LED_ERROR_QUEUE_OVERRUN (LED_ERROR_BASE + 0x01)
#define LED_PWM_FREQ        (NRF_PWM_CLK_1MHz)
#define LED_PWM_TOP         (1000)
#define LED_PWM_BUF_SIZE    (100) // 100 samples * 20 repeats * 1ms = 2000ms
#define LED_MS_TO_SA(t)     (t/20)


#define LED_QUEUE_LENGTH    (10)

typedef enum {
    led_red = 0,
    led_green,
    led_blue,
    max_leds
} led_idx_e;

typedef struct {
    uint8_t pattern_id;
    int32_t repeats;
} led_queue_item_t;

typedef enum {
    led_pattern_off = 0,    
    // On Dock Conditions
    led_pattern_charging_0,
    led_pattern_charging_1,
    led_pattern_charging,
    led_pattern_charge_complete,
    // Off Dock Conditions
    led_pattern_unconnected_unattached,
    led_pattern_unconnected_attached,
    led_pattern_connected_unattached,
    led_pattern_connected_attached,    
    // Miscellaneous Conditions
    led_pattern_init,
    led_pattern_streaming,
    led_pattern_streaming_attached,
    led_pattern_streaming_unattached,
    led_pattern_synthesizing,
    led_pattern_shutdown,
    led_pattern_memory_full,
    led_pattern_error,
    led_pattern_sleep,
    led_pattern_deep_sleep_prep,
    led_pattern_panic,
} led_pattern_e;

typedef enum {
    led_state_off = 0,
    led_state_init,
    led_state_error,
    led_state_charging,
    led_state_charge_complete,
    led_state_unconnected_unattached,
    led_state_connected_unattached,
    led_state_unconnected_attached,
    led_state_connected_attached,
    led_state_streaming,
    led_state_streaming_attached,
    led_state_streaming_unattached,
    led_state_synthesizing,
    led_state_memory_full,
    led_state_shutdown,
    led_state_sleep,
    led_state_deep_sleep_prep,
    led_state_panic,
} led_state_e;



// #define LED_R_MAX       (95*LED_PWM_TOP/100)
// #define LED_G_MAX       (25*LED_PWM_TOP/100)
// #define LED_B_MAX       (95*LED_PWM_TOP/100)

#define LED_R_MAX       (24*LED_PWM_TOP/100)
#define LED_G_MAX       (6*LED_PWM_TOP/100)
#define LED_B_MAX       (24*LED_PWM_TOP/100)

int32_t led_init(void);
int32_t led_set_for_state(led_state_e state);
int32_t led_start(void);
int32_t led_stop(void);
void led_set_rgb(uint8_t r_val, uint8_t g_val, uint8_t b_val);
bool leds_running(void);

#endif