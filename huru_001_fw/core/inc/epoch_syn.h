#ifndef EPOCH_SYN_H_INCLUDED
#define EPOCH_SYN_H_INCLUDED

#define SYN_ERROR_BASE                      (0x500)
#define SYN_ERROR_MACHINE_NOT_IDLE          (SYN_ERROR_BASE + 0x01)

#define P_EEG (EEG_SAMPLE_RATE*EPOCH_DURATION_SECONDS*5/7)
#define M_EEG ((1<<20)/(P_EEG/2))
#define C_EEG (M_EEG*P_EEG/4)

#define P_ACC_X (ACC_SAMPLE_RATE*EPOCH_DURATION_SECONDS*13/23)
#define M_ACC_X ((1<<15)/(P_ACC_X/2))
#define C_ACC_X (M_ACC_X*P_ACC_X/4)

#define P_ACC_Y (ACC_SAMPLE_RATE*EPOCH_DURATION_SECONDS*17/23)
#define M_ACC_Y ((1<<15)/(P_ACC_Y/2))
#define C_ACC_Y (M_ACC_Y*P_ACC_Y/4)

#define P_ACC_Z (ACC_SAMPLE_RATE*EPOCH_DURATION_SECONDS*19/23)
#define M_ACC_Z ((1<<15)/(P_ACC_Z/2))
#define C_ACC_Z (M_ACC_Z*P_ACC_Z/4)

#define P_MAG_X (MAG_SAMPLE_RATE*EPOCH_DURATION_SECONDS*13/23)
#define M_MAG_X ((1<<15)/(P_MAG_X/2))
#define C_MAG_X (M_MAG_X*P_MAG_X/4)

#define P_MAG_Y (MAG_SAMPLE_RATE*EPOCH_DURATION_SECONDS*17/23)
#define M_MAG_Y ((1<<15)/(P_MAG_Y/2))
#define C_MAG_Y (M_MAG_Y*P_MAG_Y/4)

#define P_MAG_Z (MAG_SAMPLE_RATE*EPOCH_DURATION_SECONDS*19/23)
#define M_MAG_Z ((1<<15)/(P_MAG_Z/2))
#define C_MAG_Z (M_MAG_Z*P_MAG_Z/4)

#define P_AMP (IMP_SAMPLE_RATE*EPOCH_DURATION_SECONDS*37/23)
#define M_AMP ((1<<20)/(P_AMP/2))
#define C_AMP (M_AMP*P_AMP/4)

typedef enum {
    epoch_synthesize_state_idle = 0,
    epoch_synthesize_state_synthesizing,
    epoch_synthesize_state_storing,
    epoch_synthesize_state_wait
} epoch_synthesize_state_e;

typedef struct {
    epoch_synthesize_state_e state;  
    int32_t epochs_remaining;  
    bool second_tick;
    bool epoch_boundary;

} epoch_synthesize_status_t;

int32_t epoch_synthesize_init(void);
epoch_synthesize_state_e epoch_synthesize_get_state(void);
int32_t epoch_synthesize_begin(int32_t num_epochs);
int32_t epoch_synthesize_update(void);
epoch_synthesize_state_e epoch_synthesize_get_state(void);

#endif