#ifndef __ACC_WRAPPER_H
#define __ACC_WRAPPER_H

#include "stdio.h"
#include "stdint.h"
#include <stdbool.h>
#include "LIS2DTW12.h"

#define ACC_ERROR_BASE              (0xB00)
#define ACC_ERROR_INVALID_ID        (ACC_ERROR_BASE + 0x01)

#define ACC_FIFO_SIZE               (32)
#define ACC_FULL_SCALE              (LIS2DTW12_2g)

#define ROOM_TEMP_K                 (273+25)
#define ACC_TEMP_VERTICAL_SCALE     (1.0/16)

typedef enum {
    acc_mode_reset = 0,
    acc_mode_wakeup,
    acc_mode_stream,
    acc_mode_off
} acc_mode_e;

typedef struct  __attribute__ ((__packed__)) {
    int16_t x;
    int16_t y;
    int16_t z;
} acc_sample_t;

int32_t acc_init(void);
int32_t acc_read_fifo(acc_sample_t *buf, uint8_t *samples_read);

void acc_set_data_rate(lis2dtw12_odr_t data_rate);
int32_t acc_set_mode(acc_mode_e mode);
bool acc_get_wake_event(void);
void acc_clear_wake_event(void);
int32_t acc_tap_src_get(void);

bool acc_data_available(void);
float acc_get_vertical_scaling(void);
int32_t acc_get_temperature(uint16_t *temperature_k16);


#endif