#define NRF_LOG_MODULE_NAME led 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "board_config.h"
#include "led.h"
#include "nrfx_pwm.h"

NRF_LOG_MODULE_REGISTER();

static nrfx_pwm_t m_pwm0 = NRFX_PWM_INSTANCE(0);

static bool _leds_running = false;
led_queue_item_t led_queue[LED_QUEUE_LENGTH];
uint8_t led_queue_idx;

static nrf_pwm_values_individual_t pwm_values[2][LED_PWM_BUF_SIZE];
uint8_t idle_buf;

nrf_pwm_sequence_t pwm_sequence_0 = {
    .length = LED_PWM_BUF_SIZE * 4,
    .repeats = 20,
    .end_delay = 0,
    .values.p_individual = pwm_values[0],
};

nrf_pwm_sequence_t pwm_sequence_1 = {
    .length = LED_PWM_BUF_SIZE * 4,
    .repeats = 20,
    .end_delay = 0,
    .values.p_individual = pwm_values[1],
};

int32_t led_load_pattern(uint8_t buf, led_pattern_e pattern);

void pwm_handler(nrfx_pwm_evt_type_t event_type)
{
    
    switch(event_type)
    {
    case NRFX_PWM_EVT_END_SEQ0:
        idle_buf = 0;
        break;
    case NRFX_PWM_EVT_END_SEQ1:
        idle_buf = 1;
        break;
    default:
        break;
    }

    if(led_queue[0].repeats > 0)
    {
        NRF_LOG_DEBUG("LED: remaining repeats: %d", led_queue[0].repeats);
        led_queue[0].repeats--;
    }
    
    if(led_queue[0].repeats == 0)
    {
        if(led_queue_idx > 0)
        {
            for(uint8_t i = 0; i < LED_QUEUE_LENGTH-1; i++)
            {
                led_queue[i].pattern_id = led_queue[i+1].pattern_id;
                led_queue[i].repeats = led_queue[i+1].repeats;
            }
            led_queue_idx--;
        }
    }

    if(led_queue[0].repeats == -1 || led_queue[0].repeats > 0)
    {
        NRF_LOG_DEBUG("LED: Load id %d into buf %d", led_queue[0].pattern_id, idle_buf);
        led_load_pattern(idle_buf, led_queue[0].pattern_id);
    }
    else
    {
        if(led_queue_idx > 1)
        {
            NRF_LOG_DEBUG("LED: Load id %d into buf %d", led_queue[1].pattern_id, idle_buf);
            led_load_pattern(idle_buf, led_queue[1].pattern_id);
        }
    }

    if(led_queue[0].pattern_id == led_pattern_off)    
    {
        led_stop();                
    }
}

int32_t led_init(void)
{
    int32_t err = 0;

    led_load_pattern(0, led_pattern_off);
    led_load_pattern(1, led_pattern_off);

    led_queue[0].pattern_id = led_pattern_off;
    led_queue_idx = 0;
    idle_buf = 0;

    return err;
}

int32_t led_start(void)
{
    int32_t err = 0;

    NRF_LOG_INFO("LED: start");
    nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG;

    config.output_pins[0] = PIN_LED_R | NRFX_PWM_PIN_INVERTED;
    config.output_pins[1] = PIN_LED_G | NRFX_PWM_PIN_INVERTED;
    config.output_pins[2] = PIN_LED_B | NRFX_PWM_PIN_INVERTED;

    config.output_pins[3] = NRFX_PWM_PIN_NOT_USED;
    config.base_clock = LED_PWM_FREQ;
    config.top_value = LED_PWM_TOP;
    config.load_mode = NRF_PWM_LOAD_INDIVIDUAL;
    config.step_mode = NRF_PWM_STEP_AUTO;

    err = nrfx_pwm_init(&m_pwm0, &config, pwm_handler);
    APP_ERROR_CHECK(err);

    err = nrfx_pwm_complex_playback(&m_pwm0, &pwm_sequence_0, &pwm_sequence_1, 1, 
        NRFX_PWM_FLAG_LOOP |
        NRFX_PWM_FLAG_SIGNAL_END_SEQ0 |
        NRFX_PWM_FLAG_SIGNAL_END_SEQ1 |
        NRFX_PWM_FLAG_NO_EVT_FINISHED
        );
    APP_ERROR_CHECK(err);
    _leds_running = true;

    return err;
}

int32_t led_stop(void)
{
    int32_t err = 0;

    NRF_LOG_INFO("LED: stop");
    nrfx_pwm_stop(&m_pwm0, false);    
    _leds_running = false;
    idle_buf = 0;
    nrfx_pwm_uninit(&m_pwm0);
    
    return err;
}

bool leds_running(void)
{  
    return _leds_running;  
}

const uint16_t pwm_max[max_leds] = {
    LED_R_MAX,
    LED_G_MAX,
    LED_B_MAX};

void led_load_ramped_flash(uint8_t buf, led_idx_e led, uint16_t on_s, uint16_t off_s, uint16_t ramp_s, uint8_t intensity_percent, uint16_t t_offset)
{
    uint16_t i = 0;
    uint16_t j = 0;

    NRF_LOG_DEBUG("Setting Ramped LED");

    if (led >= max_leds)
    {
        return;
    }

    uint16_t ramp_start_s;
    uint16_t cycle_start_s = 0;
    do
    {
        ramp_start_s = j;        
        cycle_start_s = j;
        while (j < cycle_start_s + ramp_s)
        {
            i = (j + t_offset) % LED_PWM_BUF_SIZE;
            switch (led)
            {
            case led_red:
                pwm_values[buf][i].channel_0 = (uint16_t)((uint32_t)(j - ramp_start_s) * pwm_max[led] / ramp_s * intensity_percent / 100);
                break;
            case led_green:
                pwm_values[buf][i].channel_1 = (uint16_t)((uint32_t)(j - ramp_start_s) * pwm_max[led] / ramp_s * intensity_percent / 100);
                break;
            case led_blue:
                pwm_values[buf][i].channel_2 = (uint16_t)((uint32_t)(j - ramp_start_s) * pwm_max[led] / ramp_s * intensity_percent / 100);
                break;
            default:
                break;
            }
            j += 1;
        }
        NRF_LOG_DEBUG("Ended P1 j=%d",j);
        while (j < cycle_start_s + on_s)
        {
            i = (j + t_offset) % LED_PWM_BUF_SIZE;
            switch (led)
            {
            case led_red:
                pwm_values[buf][i].channel_0 = (uint16_t)((uint32_t)pwm_max[led] * intensity_percent / 100);
                break;
            case led_green:
                pwm_values[buf][i].channel_1 = (uint16_t)((uint32_t)pwm_max[led] * intensity_percent / 100);
                break;
            case led_blue:
                pwm_values[buf][i].channel_2 = (uint16_t)((uint32_t)pwm_max[led] * intensity_percent / 100);
                break;
            default:
                break;
            }
            j += 1;
        }
        NRF_LOG_DEBUG("Ended P2 j=%d",j);
        ramp_start_s = j;
        while (j < cycle_start_s + on_s + ramp_s)
        {
            i = (j + t_offset) % LED_PWM_BUF_SIZE;
            switch (led)
            {
            case led_red:
                pwm_values[buf][i].channel_0 = (uint16_t)((uint32_t)(pwm_max[led] - (j - ramp_start_s) * pwm_max[led] / ramp_s) * intensity_percent / 100);
                break;
            case led_green:
                pwm_values[buf][i].channel_1 = (uint16_t)((uint32_t)(pwm_max[led] - (j - ramp_start_s) * pwm_max[led] / ramp_s) * intensity_percent / 100);
                break;
            case led_blue:
                pwm_values[buf][i].channel_2 = (uint16_t)((uint32_t)(pwm_max[led] - (j - ramp_start_s) * pwm_max[led] / ramp_s) * intensity_percent / 100);
                break;
            default:
                break;
            }
            j += 1;
        }
        NRF_LOG_DEBUG("Ended P3 j=%d",j);
        while (j < cycle_start_s + on_s + off_s)
        {
            i = (j + t_offset) % LED_PWM_BUF_SIZE;
            switch (led)
            {
            case led_red:
                pwm_values[buf][i].channel_0 = 0;
                break;
            case led_green:
                pwm_values[buf][i].channel_1 = 0;
                break;
            case led_blue:
                pwm_values[buf][i].channel_2 = 0;
                break;
            default:
                break;
            }
            j += 1;
        }
        NRF_LOG_DEBUG("Ended P4 j=%d",j);
    } while (j < LED_PWM_BUF_SIZE);
    NRF_LOG_DEBUG("Ramp Done");
}


void led_load_solid(uint8_t buf, uint8_t led, uint16_t pwm_value)
{
    for (uint16_t i = 0; i < LED_PWM_BUF_SIZE; i++)
    {
        switch (led)
        {
        case led_red:
            //NRF_LOG_INFO("Red[%d]",i);
            pwm_values[buf][i].channel_0 = pwm_value;
            break;
        case led_green:
            //NRF_LOG_INFO("Grn[%d]",i);
            pwm_values[buf][i].channel_1 = pwm_value;
            break;
        case led_blue:
            //NRF_LOG_INFO("Blu[%d]",i);
            pwm_values[buf][i].channel_2 = pwm_value;
            break;
        default:
            break;
        }
    }
}

void led_load_rgb(uint8_t buf, uint8_t r_val, uint8_t g_val, uint8_t b_val)
{
    led_load_solid(buf, led_red, (uint16_t)((uint32_t)r_val * LED_R_MAX / 100));
    led_load_solid(buf, led_green, (uint16_t)((uint32_t)g_val * LED_G_MAX / 100));
    led_load_solid(buf, led_blue, (uint16_t)((uint32_t)b_val * LED_B_MAX / 100));
}


void led_load_on(uint8_t buf, uint8_t led)
{
    led_load_solid(buf, led, pwm_max[led]);
}

void led_load_off(uint8_t buf, uint8_t led)
{
    led_load_solid(buf, led, 0);
}

int32_t led_queue_reset(void)
{
    int32_t err = 0;

    NRF_LOG_DEBUG("LED: Queue Reset");
    if(_leds_running)
    {
        led_queue[0].repeats = 0;
        led_queue_idx = 1;
    }
    else
    {
        led_queue_idx = 0;
    }

    return err;
}

int32_t led_queue_add(led_pattern_e pattern, uint32_t repeats)
{
    int32_t err = 0;

    NRF_LOG_DEBUG("LED: Adding %d to queue at idx %d", pattern, led_queue_idx);    

    if(led_queue_idx >= LED_QUEUE_LENGTH)
    {
        err = LED_ERROR_QUEUE_OVERRUN;
    }

    if(!err)
    {
        led_queue[led_queue_idx].pattern_id = pattern;
        led_queue[led_queue_idx].repeats = repeats;         

        if(led_queue_idx == 0)
        {
            NRF_LOG_DEBUG("LED: Load id %d into buf %d", pattern, idle_buf);
            led_load_pattern(idle_buf, pattern);
            if(repeats == -1 || repeats > 1)
            {
                NRF_LOG_DEBUG("LED: Load id %d into buf %d", pattern, 1-idle_buf);
                led_load_pattern(1-idle_buf, pattern);
            }
        }
        else
        {
            if(led_queue_idx == 1 && led_queue[0].repeats == 0)
            {
                NRF_LOG_DEBUG("LED: Load id %d into buf %d", pattern, idle_buf);
                led_load_pattern(idle_buf, pattern);
            }
        }

        led_queue_idx++;
            
    }

    return err;
}

int32_t led_set_for_state(led_state_e state)
{
    int32_t err = 0;

    NRF_LOG_DEBUG("LED: set for state %d", state);    

    led_queue_reset();

    switch(state)
    {
    case led_state_off:
    case led_state_shutdown:
        led_queue_add(led_pattern_off, -1);
        break;
    case led_state_charging:
        led_queue_add(led_pattern_charging, -1);
        break;
    case led_state_init:
        led_queue_add(led_pattern_init, -1);
        break;
    case led_state_error:
        led_queue_add(led_pattern_error, -1);
        break;
    case led_state_charge_complete:
        led_queue_add(led_pattern_charge_complete, -1);
        break;
    case led_state_unconnected_unattached:
        led_queue_add(led_pattern_unconnected_unattached, -1);
        break;
    case led_state_unconnected_attached:
        led_queue_add(led_pattern_unconnected_attached, 10);
        led_queue_add(led_pattern_off, -1);
        break;
    case led_state_connected_unattached:
        led_queue_add(led_pattern_connected_unattached, -1);
        break;
    case led_state_connected_attached:
        led_queue_add(led_pattern_connected_attached, 10);
        led_queue_add(led_pattern_off, -1);
        break;
    case led_state_streaming:
        led_queue_add(led_pattern_streaming, -1);
        break;
    case led_state_streaming_attached:
        led_queue_add(led_pattern_streaming_attached, -1);
        break;
    case led_state_streaming_unattached:
        led_queue_add(led_pattern_streaming_unattached, -1);
        break;
    case led_state_synthesizing:
        led_queue_add(led_pattern_synthesizing, -1);
        break;
    case led_state_memory_full:   
        led_queue_add(led_pattern_memory_full, -1);
        break;
    case led_state_sleep:
        led_queue_add(led_pattern_sleep, 1);
        led_queue_add(led_pattern_off, -1);
        break;
    case led_state_deep_sleep_prep:
        led_queue_add(led_pattern_deep_sleep_prep, -1);
        break;
    case led_state_panic:
        led_queue_add(led_pattern_panic, -1);
        break;
    }

    if(!_leds_running)
    {
        // ("LED: starting");
        led_start();
    }

    return err;

}

int32_t led_load_pattern(uint8_t buf, led_pattern_e pattern)
{
    int32_t err = 0;
    
    switch (pattern)
    {

    case led_pattern_off:
        led_load_off(buf, led_red);
        led_load_off(buf, led_green);
        led_load_off(buf, led_blue);
        break;

    case led_pattern_init:
        led_load_off(buf, led_red);
        led_load_ramped_flash(buf, led_green, LED_MS_TO_SA(75), LED_MS_TO_SA(75), LED_MS_TO_SA(50), 100, 0);
        led_load_off(buf, led_blue);
        break;

    case led_pattern_error:
        led_load_ramped_flash(buf, led_red, LED_MS_TO_SA(100), LED_MS_TO_SA(100), LED_MS_TO_SA(20), 100, LED_MS_TO_SA(0));
        led_load_ramped_flash(buf, led_green, LED_MS_TO_SA(100), LED_MS_TO_SA(100), LED_MS_TO_SA(20), 100, LED_MS_TO_SA(0));
        led_load_off(buf, led_blue);
        break;

    case led_pattern_charging_0:
        led_load_off(buf, led_red);
        led_load_ramped_flash(buf, led_green, LED_MS_TO_SA(1000), LED_MS_TO_SA(1000), LED_MS_TO_SA(1000), 30, LED_MS_TO_SA(0));
        led_load_off(buf, led_blue);            
        break;

    case led_pattern_charging_1:
        led_load_off(buf, led_red);
        led_load_off(buf, led_green);
        led_load_off(buf, led_blue);            
        break;

    case led_pattern_charging:
        led_load_on(buf, led_red);
        led_load_solid(buf, led_green, pwm_max[led_green]*5/10);
        led_load_off(buf, led_blue);
        break;

    case led_pattern_charge_complete:
        led_load_off(buf, led_red);
        led_load_on(buf, led_green);
        led_load_off(buf, led_blue);
        break;      

    case led_pattern_memory_full:
        led_load_on(buf, led_red);
        led_load_off(buf, led_green);
        led_load_on(buf, led_blue);
        break;

    case led_pattern_unconnected_unattached:
    case led_pattern_connected_unattached:
    case led_pattern_streaming_unattached:
        led_load_ramped_flash(buf, led_red, LED_MS_TO_SA(250), LED_MS_TO_SA(750), LED_MS_TO_SA(250), 100, 0);
        led_load_off(buf, led_green);
        led_load_off(buf, led_blue);
        break;

    case led_pattern_connected_attached:
    case led_pattern_unconnected_attached:
        led_load_off(buf, led_red);
        led_load_on(buf, led_green);
        led_load_off(buf, led_blue);
        break;

    case led_pattern_shutdown:
        led_load_ramped_flash(buf, led_red, LED_MS_TO_SA(333), LED_MS_TO_SA(667), LED_MS_TO_SA(100), 100, LED_MS_TO_SA(0));
        led_load_ramped_flash(buf, led_green, LED_MS_TO_SA(333), LED_MS_TO_SA(667), LED_MS_TO_SA(100), 100, LED_MS_TO_SA(667));
        led_load_ramped_flash(buf, led_blue, LED_MS_TO_SA(333), LED_MS_TO_SA(667), LED_MS_TO_SA(100), 100, LED_MS_TO_SA(1333));
        break;

    case led_pattern_streaming:
    case led_pattern_streaming_attached:
        led_load_off(buf, led_red);       
        led_load_on(buf, led_green);
        led_load_solid(buf, led_blue, pwm_max[led_blue]*3/10);
        break;

    case led_pattern_synthesizing:
        led_load_ramped_flash(buf, led_red, LED_MS_TO_SA(125), LED_MS_TO_SA(375), LED_MS_TO_SA(125), 100, LED_MS_TO_SA(375));
        led_load_ramped_flash(buf, led_blue, LED_MS_TO_SA(125), LED_MS_TO_SA(125), LED_MS_TO_SA(125), 100, LED_MS_TO_SA(0));
        led_load_ramped_flash(buf, led_green, LED_MS_TO_SA(125), LED_MS_TO_SA(750), LED_MS_TO_SA(125), 100, LED_MS_TO_SA(125));            
        break;

    case led_pattern_sleep:
        led_load_ramped_flash(buf, led_red, LED_MS_TO_SA(125), LED_MS_TO_SA(375), LED_MS_TO_SA(125), 100, LED_MS_TO_SA(375));
        led_load_ramped_flash(buf, led_blue, LED_MS_TO_SA(125), LED_MS_TO_SA(125), LED_MS_TO_SA(125), 100, LED_MS_TO_SA(0));
        led_load_ramped_flash(buf, led_green, LED_MS_TO_SA(125), LED_MS_TO_SA(750), LED_MS_TO_SA(125), 100, LED_MS_TO_SA(125));  
        break;

    case led_pattern_deep_sleep_prep:
        led_load_off(buf, led_red);
        led_load_ramped_flash(buf, led_green, LED_MS_TO_SA(250), LED_MS_TO_SA(750), LED_MS_TO_SA(100), 100, LED_MS_TO_SA(0));
        led_load_ramped_flash(buf, led_blue, LED_MS_TO_SA(250), LED_MS_TO_SA(750), LED_MS_TO_SA(100), 100, LED_MS_TO_SA(500));
        break;

    case led_pattern_panic:
        led_load_ramped_flash(buf, led_red, LED_MS_TO_SA(50), LED_MS_TO_SA(950), LED_MS_TO_SA(25), 100, LED_MS_TO_SA(0));
        led_load_off(buf, led_green);
        led_load_off(buf, led_blue);
        break;
    }
    

    // if(!err)
    // {
    //     err = led_start();
    // }

    return err;
}
