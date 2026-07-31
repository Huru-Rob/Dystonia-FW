#ifndef NEUROBUZZ_H_INCLUDED
#define NEUROBUZZ_H_INCLUDED


#define NEUROBUZZ_IDLE_UPDATE_PERIOD    (500)
#define NEUROBUZZ_LONG_DUR_MS           (120000)
#define NEUROBUZZ_SHORT_DUR_MS          (15000)
#define NEUROBUZZ_SWITCH_STIM_GAP_MS    (1500)
#define NEUROBUZZ_SWITCH_STIM_UPDATES   (NEUROBUZZ_SWITCH_STIM_GAP_MS/NEUROBUZZ_IDLE_UPDATE_PERIOD)

#define LONG_PRESS_N_PULSES             (75)
#define LONG_PRESS_N_BURSTS             (2)
#define LONG_PRESS_BURST_GAP            (1000)
#define LONG_PRESS_PULSE_PERIOD         (10)

#define VERY_LONG_PRESS_N_PULSES        (200)
#define VERY_LONG_PRESS_PULSE_PERIOD    (10)

#define DOUBLE_CLICK_N_PULSES_SLOW_TO_FAST          (20)
#define DOUBLE_CLICK_N_PULSES_SLOW_HOLD_TO_SLOW     (30)
#define DOUBLE_CLICK_N_PULSES_FAST_TO_SLOW          (50)
#define DOUBLE_CLICK_N_PULSES_FAST_HOLD_TO_FAST     (16)
#define DOUBLE_CLICK_PULSE_PERIOD_SLOW_TO_FAST      (10)
#define DOUBLE_CLICK_PULSE_PERIOD_FAST_TO_SLOW      (10)

#define NEUROBUZZ_MAX_PHASES            (6)
#define NEUROBUZZ_STEP_NEAR_PREFERRED_PHASE (20)


typedef enum {
    slow_hold = 0,
    fast_hold,
    slow_switching_mode_mark,
    slow_switching_mode_space,
    fast_switching_mode
} switching_mode_e;

typedef enum {
    neurobuzz_state_reset = 0,
    neurobuzz_state_idle,    
    neurobuzz_state_short_press_blank,
    neurobuzz_state_long_press_pulse,
    neurobuzz_state_very_long_press_pulse,
    neurobuzz_state_double_click_pulse
} neurobuzz_state_e;

typedef struct {
    neurobuzz_state_e state;
    neurobuzz_state_e next_state;    /* data */
    uint8_t pulse_counter;
    uint8_t burst_counter;
    uint16_t phase_switch_update_counter;
    switching_mode_e switching_mode;
    bool short_press;
    bool long_press;
    bool very_long_press;
    bool double_click;
    uint8_t refined_excl_bad_phases_it;
    uint8_t num_phases;
    uint8_t current_phase_idx;
    uint32_t long_dur;
    uint32_t short_dur;
    uint32_t switch_stim_gap;
    int16_t step_near_preferred_phase;
    bool disable_refinement;
} neurobuzz_status_t;

typedef struct {
    int16_t phases[NEUROBUZZ_MAX_PHASES];
    uint8_t num_phases;
    uint8_t current_phase_idx;
    uint8_t transmit_phase_idx;
} neurobuzz_state_snapshot_t;


int32_t neurobuzz_update(void);
int32_t neurobuzz_init(void);

void neurobuzz_start(void);
void neurobuzz_stop(void);

void neurobuzz_reset_phases(void);
void neurobuzz_ui_short_press(uint32_t serial_number);
void neurobuzz_ui_long_press(uint32_t serial_number);
void neurobuzz_ui_very_long_press(uint32_t serial_number);
void neurobuzz_ui_double_click(uint32_t serial_number);
void neurobuzz_get_first_phase(int16_t *phase);
void neurobuzz_get_next_phase(int16_t *phase);
void neurobuzz_get_current_phase_idx(uint8_t *idx);
void neurobuzz_set_first_default_phase(int16_t phase);
void neurobuzz_set_next_default_phase(int16_t phase);
void neurobuzz_disable_refinement(bool disable);
void neurobuzz_set_long_dur(uint32_t dur);
void neurobuzz_get_long_dur(uint32_t *dur);
void neurobuzz_set_short_dur(uint32_t dur);
void neurobuzz_get_short_dur(uint32_t *dur);
void neurobuzz_set_switch_stim_gap(uint32_t gap);
void neurobuzz_get_switch_stim_gap(uint32_t *gap);
void neurobuzz_set_step_near_preferred_phase(int16_t phase);
void neurobuzz_get_step_near_preferred_phase(int16_t *phase);


#endif