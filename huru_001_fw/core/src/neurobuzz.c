#define NRF_LOG_MODULE_NAME neurobuzz
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdint.h>
#include <stdbool.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "pt10.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_timer.h"
#include "board_config.h"
#include "neurobuzz.h"
#include "oscilltrack.h"
#include "esb.h"
#include "internal_flash.h"
#include "hal.h"

NRF_LOG_MODULE_REGISTER();

APP_TIMER_DEF(neurobuzz_timer);
void neurobuzz_machine(void *p_context);

uint32_t last_ui_command_serial_number = 0;

neurobuzz_status_t neurobuzz_status = {
    .state = neurobuzz_state_reset,
    .next_state = neurobuzz_state_reset,
    .switching_mode = slow_switching_mode_mark,
    .pulse_counter = 0,
    .short_press = false,
    .long_press = false,
    .very_long_press = false,
    .double_click = false,
    .refined_excl_bad_phases_it = 1,
    .phase_switch_update_counter = NEUROBUZZ_LONG_DUR_MS / NEUROBUZZ_IDLE_UPDATE_PERIOD,
    .num_phases = 0,
    .current_phase_idx = 0,
    .long_dur = NEUROBUZZ_LONG_DUR_MS,
    .short_dur = NEUROBUZZ_SHORT_DUR_MS,
    .switch_stim_gap = NEUROBUZZ_SWITCH_STIM_GAP_MS,
    .step_near_preferred_phase = NEUROBUZZ_STEP_NEAR_PREFERRED_PHASE,
    .disable_refinement = false
};

int16_t default_phases[NEUROBUZZ_MAX_PHASES] = {-90, 90, -150, -30, 150, 30};
uint8_t n_default_phases = 6;
int16_t excl_bad_phases[NEUROBUZZ_MAX_PHASES];
uint8_t n_phases;
neurobuzz_state_snapshot_t snapshot;



void neurobuzz_reset_phases(void)
{
    NRF_LOG_INFO("Setting excl_bad_phases list to default");
    for(uint8_t i=0; i < n_default_phases; i++)
    {
        excl_bad_phases[i] = default_phases[i];
    }
    neurobuzz_status.num_phases = n_default_phases;
    neurobuzz_status.current_phase_idx = 0;
    neurobuzz_status.refined_excl_bad_phases_it = 1;
    neurobuzz_status.disable_refinement = false;
}

void neurobuzz_initialise_phases(void)
{
    NRF_LOG_INFO("Setting default phase list to default");
    default_phases[0] = -90;
    default_phases[1] = 90;
    default_phases[2] = -150;
    default_phases[3] = -30;
    default_phases[4] = 150;
    default_phases[5] = 30;    
    neurobuzz_reset_phases();
    neurobuzz_status.num_phases = n_default_phases;
    neurobuzz_status.current_phase_idx = 0;
    neurobuzz_status.refined_excl_bad_phases_it = 1;
    neurobuzz_status.disable_refinement = false;
}

int32_t neurobuzz_init(void)
{
    int32_t err = 0;

    get_neurobuzz_long_dur(&neurobuzz_status.long_dur);
    if(neurobuzz_status.long_dur < 10000 || neurobuzz_status.long_dur > 300000)
    {
        neurobuzz_status.long_dur = 120000;
        set_neurobuzz_long_dur(neurobuzz_status.long_dur);
    }

    get_neurobuzz_short_dur(&neurobuzz_status.short_dur);
    if(neurobuzz_status.short_dur < 5000 || neurobuzz_status.short_dur > 60000)
    {
        neurobuzz_status.short_dur = 15000;
        set_neurobuzz_short_dur(neurobuzz_status.short_dur);
    }

    get_neurobuzz_switch_stim_gap(&neurobuzz_status.switch_stim_gap);
    if(neurobuzz_status.switch_stim_gap > 30000)
    {
        neurobuzz_status.switch_stim_gap = 1500;
        set_neurobuzz_switch_stim_gap(neurobuzz_status.switch_stim_gap);
    }

    for(uint8_t i = 0; i < NEUROBUZZ_MAX_PHASES; i++)
    {
        excl_bad_phases[i] = 0;
    }
    get_neurobuzz_phases(default_phases, &n_default_phases);
    neurobuzz_reset_phases();
    bool corrupt = false;
    if(neurobuzz_status.num_phases > NEUROBUZZ_MAX_PHASES)
    {
        corrupt = true;
    }
    
    for(uint8_t i = 0; i < neurobuzz_status.num_phases && i < NEUROBUZZ_MAX_PHASES; i++)
    {
        if(excl_bad_phases[i] < -180 || excl_bad_phases[i] > 180)
        {
            corrupt = true;
        }
    }

    if(corrupt)
    {
        neurobuzz_initialise_phases();
        set_neurobuzz_phases(excl_bad_phases, neurobuzz_status.num_phases);
    }
    

    get_neurobuzz_step_near_preferred_phase(&neurobuzz_status.step_near_preferred_phase);
    if(neurobuzz_status.step_near_preferred_phase > 90 || neurobuzz_status.step_near_preferred_phase < 5)
    {
        neurobuzz_status.step_near_preferred_phase = 20;
        set_neurobuzz_step_near_preferred_phase(neurobuzz_status.step_near_preferred_phase);
    }

    uint8_t disabled;
    get_neurobuzz_disable_refinement(&disabled);
    if(disabled > 1)
    {
        neurobuzz_status.disable_refinement = false; 
        set_neurobuzz_disable_refinement(false);       
    }
    else
    {
        neurobuzz_status.disable_refinement = (disabled == 1);
    }

    app_timer_create(&neurobuzz_timer, APP_TIMER_MODE_SINGLE_SHOT, neurobuzz_machine);
    neurobuzz_reset_phases();
    return err;
}

int32_t neurobuzz_update(void)
{
    int32_t err = 0;
    oscilltrack_update();
    return err;
}


void neurobuzz_start(void)
{
    NRF_LOG_INFO("Start");
    app_timer_start(neurobuzz_timer, APP_TIMER_TICKS(100), NULL);
}

void neurobuzz_stop(void)
{
    NRF_LOG_INFO("Stop");
    app_timer_stop(neurobuzz_timer);
}

void neurobuzz_advance_phase(void)
{
    int16_t old_phase = excl_bad_phases[neurobuzz_status.current_phase_idx];
    neurobuzz_status.current_phase_idx += 1;
    if(neurobuzz_status.current_phase_idx >= neurobuzz_status.num_phases)
    {
        neurobuzz_status.current_phase_idx = 0;
    }
    int16_t new_phase = excl_bad_phases[neurobuzz_status.current_phase_idx];
    oscilltrack_set_trigger_phase(excl_bad_phases[neurobuzz_status.current_phase_idx]);
    NRF_LOG_INFO("advancing phase %d -> %d", old_phase, new_phase);    
}

void neurobuzz_refine_phases(void)
{
    int16_t phase_step = neurobuzz_status.step_near_preferred_phase / neurobuzz_status.refined_excl_bad_phases_it;
    neurobuzz_status.num_phases = 3;
    excl_bad_phases[1] = excl_bad_phases[0];
    excl_bad_phases[0] = excl_bad_phases[1] - phase_step;
    excl_bad_phases[2] = excl_bad_phases[1] + phase_step;
    neurobuzz_status.current_phase_idx = 1;

    if(neurobuzz_status.refined_excl_bad_phases_it < 20)
    {
        neurobuzz_status.refined_excl_bad_phases_it += 1;
    }

}

void neurobuzz_reject_current_phase(void)
{
    if(neurobuzz_status.num_phases > 1)
    {
        uint8_t idx_to_delete = neurobuzz_status.current_phase_idx;
        if(neurobuzz_status.current_phase_idx > 0)
        {
            neurobuzz_status.current_phase_idx -= 1;
        }
        else
        {
            neurobuzz_status.current_phase_idx = neurobuzz_status.num_phases - 2;
        }
        for(
            uint8_t i = idx_to_delete; 
            i < neurobuzz_status.num_phases - 1; 
            i++
        )
        {
            excl_bad_phases[i] = excl_bad_phases[i+1];
        }
        neurobuzz_status.num_phases -= 1;
    }
    else
    {
        if(neurobuzz_status.disable_refinement == false)
        {
            neurobuzz_refine_phases();
        }
    }
    NRF_LOG_INFO("New excl_bad_phases list:");
    for(uint8_t i = 0; i < neurobuzz_status.num_phases; i++)
    {
        if(i == neurobuzz_status.current_phase_idx)
        {
            NRF_LOG_INFO("%d (<-- Current)", excl_bad_phases[i]);            
        }
        else
        {
            NRF_LOG_INFO("%d", excl_bad_phases[i]);  
        }
    }

    NRF_LOG_INFO("setting new phase to %d", excl_bad_phases[neurobuzz_status.current_phase_idx]);
    oscilltrack_set_trigger_phase(excl_bad_phases[neurobuzz_status.current_phase_idx]);

}

void neurobuzz_machine(void *p_context)
{
    NRF_LOG_DEBUG("Machine");
    uint32_t next_update_ms = 1000;

    neurobuzz_status.next_state = neurobuzz_status.state;
    switch(neurobuzz_status.state)
    {

    case neurobuzz_state_reset:

        neurobuzz_status.next_state = neurobuzz_state_idle;
        next_update_ms = 100;
        break;

    case neurobuzz_state_idle:

        if(neurobuzz_status.short_press == true)
        {
            neurobuzz_status.short_press = false;
            neurobuzz_reject_current_phase();
            oscilltrack_blank_stimulus(true);
            neurobuzz_status.next_state = neurobuzz_state_short_press_blank;
            next_update_ms = neurobuzz_status.switch_stim_gap;
        }
        else if(neurobuzz_status.long_press == true)
        {
            neurobuzz_status.long_press = false;
            switch(neurobuzz_status.switching_mode)
            {
            case slow_switching_mode_mark:
            case slow_switching_mode_space:
                neurobuzz_status.switching_mode = slow_hold;
                break;
            case fast_switching_mode:
                neurobuzz_status.switching_mode = fast_hold;
                break;
            case slow_hold:
            case fast_hold:
                break;
            }
            oscilltrack_blank_stimulus(true);
            neurobuzz_status.pulse_counter = (3+1);
            neurobuzz_status.next_state = neurobuzz_state_long_press_pulse;
            next_update_ms = LONG_PRESS_PULSE_PERIOD;
        }
        else if(neurobuzz_status.very_long_press == true)
        {
            neurobuzz_status.very_long_press = false;
            neurobuzz_reset_phases();
            neurobuzz_status.switching_mode = slow_switching_mode_mark;
            neurobuzz_status.phase_switch_update_counter = neurobuzz_status.long_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
            oscilltrack_blank_stimulus(true);
            neurobuzz_status.pulse_counter = 40;
            neurobuzz_status.next_state = neurobuzz_state_very_long_press_pulse;
            next_update_ms = VERY_LONG_PRESS_PULSE_PERIOD;
        }
        else if(neurobuzz_status.double_click == true)
        {
            neurobuzz_status.double_click = false;
            oscilltrack_blank_stimulus(true);
            
            neurobuzz_status.next_state = neurobuzz_state_double_click_pulse;
            switch(neurobuzz_status.switching_mode)
            {
            case slow_switching_mode_mark:
            case slow_switching_mode_space:
                neurobuzz_status.switching_mode = fast_switching_mode;
                neurobuzz_status.pulse_counter = (30+1);
                neurobuzz_status.phase_switch_update_counter = neurobuzz_status.short_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
                next_update_ms = DOUBLE_CLICK_PULSE_PERIOD_SLOW_TO_FAST;
                break;
            case fast_switching_mode:
                neurobuzz_status.switching_mode = slow_switching_mode_mark;
                neurobuzz_status.pulse_counter = (16+1);
                neurobuzz_status.phase_switch_update_counter = neurobuzz_status.long_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
                next_update_ms = DOUBLE_CLICK_PULSE_PERIOD_FAST_TO_SLOW;
                break;
            case slow_hold:
                neurobuzz_status.switching_mode = slow_switching_mode_mark;
                neurobuzz_status.pulse_counter = (30+1);
                neurobuzz_status.phase_switch_update_counter = neurobuzz_status.long_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
                next_update_ms = DOUBLE_CLICK_PULSE_PERIOD_FAST_TO_SLOW;                
                break;
            case fast_hold:
                neurobuzz_status.switching_mode = fast_switching_mode;
                neurobuzz_status.pulse_counter = (16+1);
                neurobuzz_status.phase_switch_update_counter = neurobuzz_status.short_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
                next_update_ms = DOUBLE_CLICK_PULSE_PERIOD_SLOW_TO_FAST;                
                break;
            }
        }
        else
        {
            switch(neurobuzz_status.switching_mode)
            {

            case slow_hold:
            case fast_hold:
                break;

            case slow_switching_mode_mark:
                if(neurobuzz_status.phase_switch_update_counter > 0)
                {
                    neurobuzz_status.phase_switch_update_counter -= 1;
                }
                if(neurobuzz_status.phase_switch_update_counter == 0)
                {
                    neurobuzz_advance_phase();
                    neurobuzz_status.phase_switch_update_counter = NEUROBUZZ_SWITCH_STIM_UPDATES;
                    neurobuzz_status.switching_mode = slow_switching_mode_space;
                    oscilltrack_blank_stimulus(true);
                }
                break;

            case slow_switching_mode_space:
                if(neurobuzz_status.phase_switch_update_counter > 0)
                {
                    neurobuzz_status.phase_switch_update_counter -= 1;
                }
                if(neurobuzz_status.phase_switch_update_counter == 0)
                {
                    neurobuzz_status.phase_switch_update_counter = neurobuzz_status.long_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
                    neurobuzz_status.switching_mode = slow_switching_mode_mark;
                    oscilltrack_blank_stimulus(false);
                }
                break;

            case fast_switching_mode:
                if(neurobuzz_status.phase_switch_update_counter > 0)
                {
                    neurobuzz_status.phase_switch_update_counter -= 1;
                }
                if(neurobuzz_status.phase_switch_update_counter == 0)
                {
                    neurobuzz_advance_phase();
                    neurobuzz_status.phase_switch_update_counter = neurobuzz_status.short_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
                }
                break;
            }
            next_update_ms = NEUROBUZZ_IDLE_UPDATE_PERIOD;
        }
        break;

    case neurobuzz_state_short_press_blank:
        oscilltrack_blank_stimulus(false);
        switch(neurobuzz_status.switching_mode)
        {
        case slow_hold:
            break;
        case slow_switching_mode_mark:
        case slow_switching_mode_space:
            neurobuzz_status.switching_mode = slow_switching_mode_mark;
            neurobuzz_status.phase_switch_update_counter = neurobuzz_status.long_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
            break;
        case fast_hold:
            break;
        case fast_switching_mode:
            neurobuzz_status.phase_switch_update_counter = neurobuzz_status.short_dur / NEUROBUZZ_IDLE_UPDATE_PERIOD;
            break;
        }
        neurobuzz_status.next_state = neurobuzz_state_idle;
        next_update_ms = 100; 
        break;   

    case neurobuzz_state_long_press_pulse:
        NRF_LOG_INFO("long press pulse");
        esb_set_stimulus();
        if(neurobuzz_status.pulse_counter > 0)        
        {
            neurobuzz_status.pulse_counter -= 1;
        }
        if(neurobuzz_status.pulse_counter == 0)
        {
            oscilltrack_blank_stimulus(false);
            neurobuzz_status.next_state = neurobuzz_state_idle;
            next_update_ms = 100;
        }
        else
        {
            next_update_ms = LONG_PRESS_PULSE_PERIOD;
        }
        break;

    case neurobuzz_state_very_long_press_pulse:
        NRF_LOG_INFO("very long press pulse");
        esb_set_stimulus();
        if(neurobuzz_status.pulse_counter > 0)        
        {
            neurobuzz_status.pulse_counter -= 1;
        }
        if(neurobuzz_status.pulse_counter == 0)
        {
            oscilltrack_blank_stimulus(false);
            neurobuzz_status.next_state = neurobuzz_state_idle;
            next_update_ms = 100;
        }
        else
        {
            next_update_ms = VERY_LONG_PRESS_PULSE_PERIOD;
        }
        break;

    case neurobuzz_state_double_click_pulse:
        NRF_LOG_INFO("double-click pulse");
        esb_set_stimulus();
        if(neurobuzz_status.pulse_counter > 0)        
        {
            neurobuzz_status.pulse_counter -= 1;
        }
        if(neurobuzz_status.pulse_counter == 0)
        {
            oscilltrack_blank_stimulus(false);
            neurobuzz_status.next_state = neurobuzz_state_idle;
            next_update_ms = 100;
        }
        else
        {
            switch(neurobuzz_status.switching_mode)
            {
            case slow_switching_mode_mark:
            case slow_switching_mode_space:
            case slow_hold:
                next_update_ms = DOUBLE_CLICK_PULSE_PERIOD_FAST_TO_SLOW;
                break;
            case fast_switching_mode:
            case fast_hold:
                next_update_ms = DOUBLE_CLICK_PULSE_PERIOD_SLOW_TO_FAST;
                break;
            }
        }
        break;

    }


    if(neurobuzz_status.next_state != neurobuzz_status.state)
    {
        NRF_LOG_INFO("state %d -> %d", neurobuzz_status.state, neurobuzz_status.next_state);
        neurobuzz_status.state = neurobuzz_status.next_state;
    }

    app_timer_start(neurobuzz_timer, APP_TIMER_TICKS(next_update_ms), NULL);
}

void neurobuzz_ui_short_press(uint32_t serial_number)
{
    NRF_LOG_INFO("UI Short Press");
    if(serial_number == last_ui_command_serial_number)
    {
        NRF_LOG_INFO("Duplicate command - rejected");
    }
    else
    {
        neurobuzz_status.short_press = true;
    }
    last_ui_command_serial_number = serial_number;
}

void neurobuzz_ui_long_press(uint32_t serial_number)
{
    NRF_LOG_INFO("UI Long Press");
    if(serial_number == last_ui_command_serial_number)
    {
        NRF_LOG_INFO("Duplicate command - rejected");
    }
    else
    {
        neurobuzz_status.long_press = true;
    }
    last_ui_command_serial_number = serial_number;
}

void neurobuzz_ui_very_long_press(uint32_t serial_number)
{
    NRF_LOG_INFO("UI Very Long Press");
    if(serial_number == last_ui_command_serial_number)
    {
        NRF_LOG_INFO("Duplicate command - rejected");
    }
    else
    {
        neurobuzz_status.very_long_press = true;
    }
    last_ui_command_serial_number = serial_number;
}

void neurobuzz_ui_double_click(uint32_t serial_number)
{
    NRF_LOG_INFO("UI Double-Click");
    if(serial_number == last_ui_command_serial_number)
    {
        NRF_LOG_INFO("Duplicate command - rejected");
    }
    else
    {
        neurobuzz_status.double_click = true;
    }
    last_ui_command_serial_number = serial_number;
}

void neurobuzz_get_first_phase(int16_t *phase)
{
    memcpy((uint8_t *)&(snapshot.phases[0]), (uint8_t *)&(excl_bad_phases[0]), sizeof(excl_bad_phases));
    snapshot.current_phase_idx = neurobuzz_status.current_phase_idx;
    snapshot.num_phases = neurobuzz_status.num_phases;
    
    *phase = snapshot.phases[0];
    snapshot.transmit_phase_idx = 0;
}

void neurobuzz_get_next_phase(int16_t *phase)
{
    snapshot.transmit_phase_idx += 1;
    if(snapshot.transmit_phase_idx >= snapshot.num_phases)
    {
        *phase = 32767;
    }
    else
    {
        *phase = snapshot.phases[snapshot.transmit_phase_idx];
    }
}

void neurobuzz_get_current_phase_idx(uint8_t *idx)
{
    *idx = snapshot.current_phase_idx;
}

void neurobuzz_set_long_dur(uint32_t dur)
{
    neurobuzz_status.long_dur = dur;
}
void neurobuzz_get_long_dur(uint32_t *dur)
{
    *dur = neurobuzz_status.long_dur;
}
void neurobuzz_set_short_dur(uint32_t dur)
{
    neurobuzz_status.short_dur = dur;
}
void neurobuzz_get_short_dur(uint32_t *dur)
{
    *dur = neurobuzz_status.short_dur;
}
void neurobuzz_set_switch_stim_gap(uint32_t gap)
{
    neurobuzz_status.switch_stim_gap = gap;
}
void neurobuzz_get_switch_stim_gap(uint32_t *gap)
{
    *gap = neurobuzz_status.switch_stim_gap;
}
void neurobuzz_set_step_near_preferred_phase(int16_t phase)
{
    neurobuzz_status.step_near_preferred_phase = phase;
}
void neurobuzz_get_step_near_preferred_phase(int16_t *phase)
{
    *phase = neurobuzz_status.step_near_preferred_phase;
}
void neurobuzz_set_first_default_phase(int16_t phase)
{
    default_phases[0] = phase;
    n_default_phases = 1;
}
void neurobuzz_set_next_default_phase(int16_t phase)
{
    if(n_default_phases < NEUROBUZZ_MAX_PHASES)
    {
        default_phases[n_default_phases] = phase;
        n_default_phases++;
    }
}
void neurobuzz_disable_refinement(bool disabled)
{
    neurobuzz_status.disable_refinement = disabled;
}