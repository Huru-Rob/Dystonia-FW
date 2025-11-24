#ifndef MAG_H_INCLUDED
#define MAG_H_INCLUDED

#define MAG_ERROR_BASE              (0x800)
#define MAG_ERROR_INCORRECT_ID      (MAG_ERROR_BASE + 0x01)
#define MAG_FIFO_SIZE               (1)

typedef enum {
    mag_mode_reset = 0,
    mag_mode_off,
    mag_mode_stream
} mag_mode_e;

typedef struct  __attribute__ ((__packed__)) {
    int16_t x;
    int16_t y;
    int16_t z;
} mag_sample_t;

int32_t mag_init(void);
int32_t mag_set_mode(mag_mode_e mode);
float mag_get_vertical_scaling(void);
bool mag_data_available(void);
int32_t mag_read_fifo(mag_sample_t *buf, uint8_t *samples_read);

#endif