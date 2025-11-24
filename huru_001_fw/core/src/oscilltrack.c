#define NRF_LOG_MODULE_NAME osc
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdint.h>
#include <stdbool.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "pt10.h"
#include "nordic_common.h"
#include "nrf.h"
#include "board_config.h"
#include "app_error.h"
#include "app_timer.h"
#include "oscilltrack.h"
#include "hal.h"
#include "esb.h"
#include "afe.h"
#include "AFE4960.h"
#include "math.h"
#include "internal_flash.h"

NRF_LOG_MODULE_REGISTER();

// APP_TIMER_DEF(oscilltrack_timer);

osc_state_t osc_state = {
    .suppressionCount = 0,
    .a = 0.0,
    .b = 0.0
};
int32_t oscilltrack_eeg_rx_buf[AFE_FIFO_SIZE*2];

bool m_pulse;
float m_g = 125.0f / OSCILLTRACK_FS;
float m_fc;
float m_phTrig;
float m_suppressionDutyCycle;
const float m_pulseWidth_us = 100.0f;
float m_w;
const float m_hp = TWO_PI * OSCILLTRACK_HPFC / OSCILLTRACK_FS;
uint8_t m_suppressionReset;
float m_eeg_vertical_scale = 0.0;
bool m_stimulusOn = true;
bool m_50HzFilterOn = true;
uint8_t m_decimation_count = OSCILLTRACK_DECIMATION_FACTOR;

// bool oscilltrack_timeout = false;
// void oscilltrack_timer_handler(void *p_context)
// {
//     NRF_LOG_DEBUG("Oscilltrack Timer elapsed %d", (uint32_t)p_context);
//     oscilltrack_timeout = true;
// }

void oscilltrack_set_frequency(int32_t frequency_hz)
{
    m_fc = (float)frequency_hz;
    m_w = TWO_PI * m_fc / OSCILLTRACK_FS;
}

void oscilltrack_set_trigger_phase(int32_t trigger_phase_degrees)
{
    m_phTrig = DEG_TO_RAD(trigger_phase_degrees);
}

void oscilltrack_set_suppression_duty_cycle(int32_t suppression_duty_cycle_percent)
{
    m_suppressionDutyCycle = (float)suppression_duty_cycle_percent / 100.0f;
    m_suppressionReset = (uint8_t)((float)OSCILLTRACK_FS / m_fc * (float)m_suppressionDutyCycle);
}

void oscilltrack_set_convergence_gain(int32_t convergence_gain)
{
    m_g = (float)convergence_gain / (float)OSCILLTRACK_FS;
}

void oscilltrack_set_flags(uint32_t flags)
{
    m_stimulusOn = ((flags >> OSCILLTRACK_STIMULUS_ON_BITPOS) & 0x00000001) == 1 ? true : false;
    m_50HzFilterOn = ((flags >> OSCILLTRACK_50HZ_FILTER_ON_BITPOS) & 0x00000001) == 1 ? true : false;
}

int32_t oscilltrack_init(void)
{
    int32_t err = 0;

    // app_timer_create(&oscilltrack_timer, APP_TIMER_MODE_SINGLE_SHOT, oscilltrack_timer_handler);    

    osc_state.packet_id = 0;

    int32_t frequency_hz;
    get_oscilltrack_frequency(&frequency_hz);
    if(frequency_hz > 50 || frequency_hz < 5.0 )
    {
        frequency_hz = 35;
        set_oscilltrack_frequency(frequency_hz);
    }
    oscilltrack_set_frequency(frequency_hz);

    int32_t trigger_phase_degrees;
    get_oscilltrack_trigger_phase(&trigger_phase_degrees);
    if(trigger_phase_degrees < -179 || trigger_phase_degrees > 179)
    {
        trigger_phase_degrees = 0;
        set_oscilltrack_trigger_phase(trigger_phase_degrees);
    }
    oscilltrack_set_trigger_phase(trigger_phase_degrees);

    int32_t suppression_duty_cycle_percent;
    get_oscilltrack_suppression_duty_cycle(&suppression_duty_cycle_percent);
    if(suppression_duty_cycle_percent < 5 || suppression_duty_cycle_percent > 95)
    {
        suppression_duty_cycle_percent = 80;
        set_oscilltrack_suppression_duty_cycle(suppression_duty_cycle_percent);
    }
    oscilltrack_set_suppression_duty_cycle(suppression_duty_cycle_percent);

    int32_t convergence_gain;
    get_oscilltrack_convergence_gain(&convergence_gain);
    if(convergence_gain < 0 || convergence_gain > 500)
    {
        convergence_gain = 125;
        set_oscilltrack_convergence_gain(convergence_gain);
    }
    oscilltrack_set_convergence_gain(convergence_gain);

    uint32_t flags;
    get_oscilltrack_flags(&flags);
    oscilltrack_set_flags(flags);

    return err;
}

int32_t oscilltrack_start(void)
{
    int32_t err = 0; 
    
    // app_timer_start(oscilltrack_timer, APP_TIMER_TICKS(OSCILLTRACK_UPDATE_MS), (void *)NULL);

    return err;
}

int32_t oscilltrack_stop(void)
{
    int32_t err = 0;

    // app_timer_stop(oscilltrack_timer);

    return err;
}

float oscilltrackHighPass(float sample)
{
    static float x = 0;
    float s = (float)sample - x;
    x = x + m_hp * s;
    return s;
}

void oscilltrackTrk(float input)
{
    static float s = 0.0, c = 1.0;

    float e = (input - osc_state.re) * m_g;
    osc_state.a += s * e;
    osc_state.b += c * e;
    
    osc_state.theta += m_w;
    if(osc_state.theta >= PI)
    {
        osc_state.theta -= TWO_PI;        
    }

    s = sinf(osc_state.theta);
    c = cosf(osc_state.theta);

    osc_state.re = osc_state.a * s + osc_state.b * c;
    osc_state.im = osc_state.b * s - osc_state.a * c;
}

bool oscilltrackPhasePulse(float ph)
{
    static bool previousAboveTrig = false;
    float phRotated;
    bool aboveTrig;
    bool pulse = false;
    
    phRotated = ph - m_phTrig;
    if(phRotated >= PI)
    {
        phRotated -= TWO_PI;
    }
    if(phRotated < -PI)
    {
        phRotated += TWO_PI;
    }
    aboveTrig = (phRotated >= 0);
    if(aboveTrig && !previousAboveTrig && phRotated < HALF_PI)
    {
        if(m_stimulusOn)
        {
            pulse = (osc_state.suppressionCount == 0); 
            osc_state.suppressionCount = m_suppressionReset;  
        }     
    }
    previousAboveTrig = aboveTrig;
    osc_state.suppressionCount = (osc_state.suppressionCount > 0 ) ? osc_state.suppressionCount - 1 : 0;

    return pulse;
}

const biquad_t line_filter_section_1 = {
    .b = {0.91687724, -1.50207655,  0.91687724},
    .a = {1.        , -1.51533877,  0.91195527}
};
const biquad_t line_filter_section_2 = {
    .b = {1.        , -1.63825263,  1.        },
    .a = {1.        , -1.61932094,  0.92183891}
};

float oscilltrack_line_filter(float input)
{
    float output;

    // Section 1
    static float d1[3] = {0.0};
    float y1;

    y1    = line_filter_section_1.b[0] * input                                   + d1[0];
    d1[0] = line_filter_section_1.b[1] * input - line_filter_section_1.a[1] * y1 + d1[1];
    d1[1] = line_filter_section_1.b[2] * input - line_filter_section_1.a[2] * y1;

    // Section 2
    static float d2[3] = {0.0};
    output = line_filter_section_2.b[0] * y1                                       + d2[0];
    d2[0]  = line_filter_section_2.b[1] * y1 - line_filter_section_2.a[1] * output + d2[1];
    d2[1]  = line_filter_section_2.b[2] * y1 - line_filter_section_2.a[2] * output;

    return output;
}

int32_t oscilltrack_update(void)
{
    int32_t err = 0;
    uint8_t samples_read;

    if(afe_samples_available())
    {
        err = afe_samples_fetch(oscilltrack_eeg_rx_buf, 0, &samples_read);
        // NRF_LOG_INFO("%d eeg samples read", samples_read);
        m_eeg_vertical_scale = afe_get_eeg_vertical_scaling();

        float eeg_sample = (float)oscilltrack_eeg_rx_buf[0] * m_eeg_vertical_scale * 1.0e6f;        
        hal_gpio_set(PIN_RTC_EVI);
        osc_state.filtered = oscilltrackHighPass(eeg_sample);
        if(m_50HzFilterOn)
        {
            osc_state.filtered = oscilltrack_line_filter(osc_state.filtered);
        }
        oscilltrackTrk(osc_state.filtered);
        osc_state.ph = atan2f(osc_state.im, osc_state.re);
        bool pulse = oscilltrackPhasePulse(osc_state.ph);
        if(pulse)
        {
            esb_set_stimulus();
        }
        hal_gpio_clear(PIN_RTC_EVI);

        if(m_decimation_count > 0)
        {
            m_decimation_count -= 1;
            if(m_decimation_count == 0)
            {
                esb_set_oscilltrack_data((uint8_t *)&osc_state, sizeof(osc_state_t));
                osc_state.packet_id += 1;
                m_decimation_count = OSCILLTRACK_DECIMATION_FACTOR;
            }
        }        
        
    }

    

    // if(oscilltrack_timeout == true)
    // {
    //     app_timer_start(oscilltrack_timer, APP_TIMER_TICKS(OSCILLTRACK_UPDATE_MS), (void *)NULL);
    //     oscilltrack_timeout = false;
    //     osc_state.packet_id += 1;
    //     esb_send_oscilltrack_data(prx_peripheral_usb_bridge, (uint8_t *)&osc_state, sizeof(osc_state_t));
    // }
    
    return err;
}