#ifndef AFE_H_INCLUDED
#define AFE_H_INCLUDED

#include "nrfx_gpiote.h"
#include "AFE4960_ex.h"

#define AFE_ERROR_BASE                  (0xa00)
#define AFE_ERROR_INCORRECT_ID          (AFE_ERROR_BASE + 0x01)
#define AFE_ERROR_INVALID_GAIN          (AFE_ERROR_BASE + 0x02)
#define AFE_ERROR_CONTROL0_READ_FAILED  (AFE_ERROR_BASE + 0x03)
#define AFE_ERROR_CONTROL4_READ_FAILED  (AFE_ERROR_BASE + 0x04)

#define AC_AMP_TO_KILOHMS       (11.623/1000.0)
#define KILOHMS_TO_AC_AMP       (1.0/AC_AMP_TO_KILOHMS)
#define AFE_R_PROTECT_KILOHMS   (9.1)
#define AFE_R_SERIES_KILOHMS    (1.1)
#define AFE_LOWER_LIMIT_KILOHMS (1.0)

typedef enum {
    afe_mode_off = 0,
    afe_mode_passive,
    afe_mode_active,
    afe_mode_uninitialised
} afe_mode_e;

typedef enum
{
    AFE_LEAD_UNK = 0,
    AFE_LEAD_OFF= 1,
    AFE_LEAD_ON = 2,
} afe_lead_status_t;

typedef enum
{
    AFE_LEAD_DETECT_OFF = 0,
    AFE_LEAD_DETECT_AC,
    AFE_LEAD_DETECT_DC
} lead_detect_mode_t;

typedef struct 
{
    bool ac_lead_detect_en;
    bool dc_lead_detect_en;
    uint8_t flags;
    uint32_t ac_amplitude;
} afe_lead_detect_t;

int32_t afe_init(void);
void afe_set_power(bool afe_enabled, bool lna_enabled);
int32_t afe_set_mode(afe_mode_e mode);
int32_t afe_pause(void);
int32_t afe_resume(void);

int32_t service_afe_fifo(void); //will probs need to pass file into this
int32_t update_lead_detect_status(void);
uint8_t afe_get_lead_detect_flags(void);
uint32_t afe_get_lead_detect_amplitude(void);
uint32_t afe_get_offset_corrected_lead_detect_amplitude(void);
uint32_t afe_get_lead_detect_amplitude_lpf(void);

int32_t afe_get_fifo_level(uint8_t *fifo_level);
bool afe_samples_available(void);
uint32_t afe_fifo_fill_time_ms(void);
int32_t afe_clear_fifo(void);
int32_t afe_samples_fetch(int32_t *buf, uint8_t n_samples, uint8_t *samples_read);

float afe_get_eeg_vertical_scaling(void);
float afe_get_impedance_vertical_scaling(void);
int32_t afe_set_ina_gains(ina_gain_t eeg1_ina_gain, bioz_ina_gain_t eeg2_ina_gain);

// int32_t afe_set_read_write_mode(uint8_t write_enabled);

int32_t afe_sw_reset(void);
int32_t afe_sw_powerdown(void);

void afe_in_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

int32_t afe_dump_registers(void);

#endif