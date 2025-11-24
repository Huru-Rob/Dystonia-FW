#ifndef INTERNAL_FLASH_H_INCLUDED
#define INTERNAL_FLASH_H_INCLUDED

typedef enum {
    store_reason_none = 0,
    store_reason_blank_device_id = 1,
    store_reason_blank_deep_sleep_flag = 2,
    store_reason_blank_all = 3,
    store_reason_set_device_id = 4,
    store_reason_set_deep_sleep_flag = 5,
    store_reason_clear_deep_sleep_flag = 6,
    store_reason_set_oscilltrack_parameters = 7,
} store_reason_e;

// This structure maintains compatibility with PT10A in CLIC form
typedef struct {
    uint32_t device_id;
    uint32_t deep_sleep_flag;
    uint32_t store_reason;
    uint32_t bluetooth_mode; // particularly - keep this and set it to one to ensure that bluetooth comes up if re-programmed to be a CLIC device
    int32_t frequency;
    int32_t trigger_phase;
    int32_t suppression_duty_cycle;
    int32_t convergence_gain;
    uint32_t flags;
} nv_params_t;

//internal flash
int32_t init_internal_flash(void);

int32_t save_info_to_local_flash(uint32_t store_reason);
int32_t read_stored_data(void);

int32_t set_device_id(uint32_t device_id);
int32_t get_device_id(uint32_t * device_id);

int32_t set_deep_sleep_flag(uint32_t deep_sleep_flag);
int32_t get_deep_sleep_flag(uint32_t *deep_sleep_flag);

int32_t set_oscilltrack_frequency(int32_t frequency);
int32_t get_oscilltrack_frequency(int32_t *frequency);
int32_t set_oscilltrack_trigger_phase(int32_t trigger_phase);
int32_t get_oscilltrack_trigger_phase(int32_t *trigger_phase);
int32_t set_oscilltrack_suppression_duty_cycle(int32_t suppression_duty_cycle);
int32_t get_oscilltrack_suppression_duty_cycle(int32_t *suppression_duty_cycle);
int32_t set_oscilltrack_convergence_gain(int32_t convergence_gain);
int32_t get_oscilltrack_convergence_gain(int32_t *convergence_gain);
int32_t get_oscilltrack_flags(uint32_t *flags);
int32_t set_oscilltrack_flags(uint32_t flags);

uint32_t get_store_reason(void);



#endif