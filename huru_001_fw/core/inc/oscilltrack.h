#ifndef OSCILLTRACK_H_INCLUDED
#define OSCILLTRACK_H_INCLUDED

#define OSCILLTRACK_UPDATE_MS           (10)
#define OSCILLTRACK_DECIMATION_FACTOR   (4)

#define PI                      (3.1415927f)
#define TWO_PI                  (2.0f * PI)
#define HALF_PI                 (PI / 2.0f)

#define OSCILLTRACK_FS          (512.0f)
#define OSCILLTRACK_FC          (35.0f)
#define OSCILLTRACK_HPFC        (5.0f)
#define DEG_TO_RAD(d)           (d * 2.0f * PI / 360.0f)

#define OSCILLTRACK_STIMULUS_ON_BITPOS      (0)
#define OSCILLTRACK_50HZ_FILTER_ON_BITPOS   (1)

typedef struct  __attribute__ ((__packed__)) {
    uint8_t     packet_id;          // 1
    float       filtered;      // 5
    float       theta;              // 9
    float       re;                 // 13
    float       im;                 // 17
    float       ph;                 // 21
    float       a;                  // 25
    float       b;                  // 29
    uint8_t     suppressionCount;   // 30
} osc_state_t;

typedef struct {
    float   b[3];
    float   a[3];
} biquad_t;

int32_t oscilltrack_init(void);
int32_t oscilltrack_start(void);
int32_t oscilltrack_stop(void);
int32_t oscilltrack_update(void);
void oscilltrack_set_frequency(int32_t frequency_hz);
void oscilltrack_set_trigger_phase(int32_t trigger_phase_degrees);
void oscilltrack_set_suppression_duty_cycle(int32_t suppression_duty_cycle_percent);
void oscilltrack_set_convergence_gain(int32_t convergeance_gain);
void oscilltrack_set_flags(uint32_t flags);

#endif