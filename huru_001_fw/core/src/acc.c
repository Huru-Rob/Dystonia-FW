#define NRF_LOG_MODULE_NAME acc
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "acc.h"
#include "hal.h"
#include <string.h>
#include "LIS2DTW12.h"
#include "stdio.h"
#include "stdint.h"
#include "epoch.h"
#include "nrf_log.h"
#include "board_config.h"
#include "hal.h"

NRF_LOG_MODULE_REGISTER();

stmdev_ctx_t acc_ctx;
acc_mode_e _current_acc_mode = acc_mode_reset;

int32_t write_acc(void *bus, uint8_t reg, const uint8_t *data, uint16_t len)
{
    int32_t error = 0;

    hal_i2c_write(0, LIS2DTW12_I2C_ADD_H, reg, data, len);

    return error;
}

int32_t read_acc(void *bus, uint8_t reg, uint8_t *data, uint16_t len)
{
    int32_t error = 0;

    error = hal_i2c_read(0, LIS2DTW12_I2C_ADD_H, reg, data, len);

    return error;
}

bool _acc_wake_event = false;
bool _acc_data_ready = false;
void acc_int_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (_current_acc_mode == acc_mode_wakeup)
    {
        _acc_wake_event = true;
    }
    if (_current_acc_mode == acc_mode_stream)
    {
        _acc_data_ready = true;
    }
}

void acc_clear_wake_event(void)
{
    _acc_wake_event = false;
}

bool acc_get_wake_event(void)
{
    return _acc_wake_event;
}

int32_t acc_init(void)
{
    int32_t err = 0;

    acc_ctx.write_reg = write_acc;
    acc_ctx.read_reg = read_acc;

    // hal_delay_ms(20);

    uint8_t who_am_i = 0;

    lis2dtw12_device_id_get(&acc_ctx, &who_am_i);

    if (who_am_i != LIS2DTW12_ID)
    {
        err = ACC_ERROR_INVALID_ID;
    }

    lis2dtw12_full_scale_set(&acc_ctx, ACC_FULL_SCALE);
    lis2dtw12_data_rate_set(&acc_ctx, LIS2DTW12_XL_ODR_OFF);
    lis2dtw12_power_mode_set(&acc_ctx, LIS2DTW12_CONT_LOW_PWR_12bit);
    lis2dtw12_fifo_watermark_set(&acc_ctx, 20); // fifo set at 20 out of 32 we need to service the acc atleast (32-wtm)*(1/data rate) i.e. (32-20)*(1/50) = 240ms
    lis2dtw12_fifo_mode_set(&acc_ctx, LIS2DTW12_BYPASS_MODE);
    lis2dtw12_auto_increment_set(&acc_ctx, 1);
    lis2dtw12_pin_polarity_set(&acc_ctx, 0); // H_LACTIVE = 0 => active high
    lis2dtw12_int_notification_set(&acc_ctx, 0); // LIR = 0 => pulsed not latched interrupts
    lis2dtw12_pin_mode_set(&acc_ctx, 0); // PP_OD = 0 => push pull interrupt pin mode
    lis2dtw12_auto_increment_set(&acc_ctx, 1); // IF_ADD_INC = 1 => auto-increment read address


    // setup the pin change int on the adc rdy pin
    ret_code_t err_code;
    if (!nrfx_gpiote_is_init())
    {
        err_code = nrfx_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrfx_gpiote_in_config_t in_config_1 = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    err_code = nrfx_gpiote_in_init(PIN_ACC_INT, &in_config_1, acc_int_pin_handler);
    APP_ERROR_CHECK(err_code);

    return err;
}

int32_t acc_set_mode(acc_mode_e mode)
{
    int32_t err = 0;
    if (mode != _current_acc_mode)
    {

        lis2dtw12_ctrl4_int1_pad_ctrl_t val = {0};

        switch (mode)
        {

        case acc_mode_reset:
            break;

        case acc_mode_off:
            val.int1_fth = 0;
            lis2dtw12_fifo_mode_set(&acc_ctx, LIS2DTW12_BYPASS_MODE);
            lis2dtw12_pin_int1_route_set(&acc_ctx, &val);
            lis2dtw12_act_mode_set(&acc_ctx, LIS2DTW12_NO_DETECTION);

#if TAP_DETECT == 1
            lis2dtw12_tap_detection_on_x_set(&acc_ctx, 0);
            lis2dtw12_tap_detection_on_y_set(&acc_ctx, 0);
            lis2dtw12_tap_detection_on_z_set(&acc_ctx, 0);
#endif

            nrfx_gpiote_in_event_disable(PIN_ACC_INT);

            // turn off
            lis2dtw12_data_rate_set(&acc_ctx, LIS2DTW12_XL_ODR_OFF);
            lis2dtw12_power_mode_set(&acc_ctx, LIS2DTW12_CONT_LOW_PWR_12bit);
            break;

        case acc_mode_stream:
            val.int1_fth = 1;
            lis2dtw12_pin_int1_route_set(&acc_ctx, &val);

            lis2dtw12_act_mode_set(&acc_ctx, LIS2DTW12_NO_DETECTION);

#if TAP_DETECT == 1
            lis2dtw12_tap_dur_set(&acc_ctx, 0); // tap duration
            lis2dtw12_tap_quiet_set(&acc_ctx, 1);
            lis2dtw12_tap_shock_set(&acc_ctx, 2);
            lis2dtw12_tap_threshold_x_set(&acc_ctx, 0x6);
            lis2dtw12_tap_threshold_y_set(&acc_ctx, 0x6);
            lis2dtw12_tap_threshold_z_set(&acc_ctx, 0x6);
            lis2dtw12_tap_detection_on_x_set(&acc_ctx, 1); // enable x axis tap
            lis2dtw12_tap_detection_on_y_set(&acc_ctx, 1); // enable y axis tap
            lis2dtw12_tap_detection_on_z_set(&acc_ctx, 1); // enable z axis tap
            lis2dtw12_tap_mode_set(&acc_ctx, 0); // Single tap only
            lis2dtw12_int_notification_set(&acc_ctx, 1); // latched mode
#endif

            nrfx_gpiote_in_event_enable(PIN_ACC_INT, true);

            val.int1_fth = 1;
            lis2dtw12_pin_int1_route_set(&acc_ctx, &val);
            lis2dtw12_power_mode_set(&acc_ctx, LIS2DTW12_CONT_LOW_PWR_12bit);
            lis2dtw12_data_rate_set(&acc_ctx, LIS2DTW12_XL_ODR_50Hz);
            lis2dtw12_fifo_mode_set(&acc_ctx, LIS2DTW12_STREAM_MODE);
            break;

        case acc_mode_wakeup:
#if TAP_DETECT == 1
            lis2dtw12_tap_detection_on_x_set(&acc_ctx, 0);
            lis2dtw12_tap_detection_on_y_set(&acc_ctx, 0);
            lis2dtw12_tap_detection_on_z_set(&acc_ctx, 0);
#endif
            lis2dtw12_fifo_mode_set(&acc_ctx, LIS2DTW12_BYPASS_MODE);
            lis2dtw12_data_rate_set(&acc_ctx, LIS2DTW12_XL_ODR_200Hz);
            lis2dtw12_wkup_dur_set(&acc_ctx, 2);
            lis2dtw12_wkup_threshold_set(&acc_ctx, 10);
            lis2dtw12_act_sleep_dur_set(&acc_ctx, 2);
            lis2dtw12_act_mode_set(&acc_ctx, LIS2DTW12_DETECT_STAT_MOTION);

            val.int1_wu = 1;
            lis2dtw12_pin_int1_route_set(&acc_ctx, &val);

            nrfx_gpiote_in_event_enable(PIN_ACC_INT, true);

            _acc_wake_event = false;

            break;
        }

        _current_acc_mode = mode;
    }

    return err;
}

bool acc_data_available()
{
    return _acc_data_ready;
}

uint8_t acc_fifo_buf[ACC_FIFO_SIZE * 6];
int32_t acc_read_fifo(acc_sample_t *buf, uint8_t *samples_read)
{
    int32_t err = 0;
    uint8_t fifo_level = 0;

    err = lis2dtw12_fifo_data_level_get(&acc_ctx, &fifo_level);  
    if(!err)
    {  
        NRF_LOG_DEBUG("Fetching %d samples", fifo_level);
        err = lis2dtw12_read_reg(&acc_ctx, LIS2DTW12_OUT_X_L, acc_fifo_buf, fifo_level * 6);
        for(uint8_t i = 0; i < fifo_level; i++)
        {
            buf[i].x = (int16_t)acc_fifo_buf[i*6+0]+((int16_t)acc_fifo_buf[i*6+1]<<8);
            buf[i].y = (int16_t)acc_fifo_buf[i*6+2]+((int16_t)acc_fifo_buf[i*6+3]<<8);
            buf[i].z = (int16_t)acc_fifo_buf[i*6+4]+((int16_t)acc_fifo_buf[i*6+5]<<8);
        }
        *samples_read = fifo_level;
        _acc_data_ready = false;
    }
    return err;
}

int32_t acc_tap_src_get(void)
{
    int32_t err = 0;

    lis2dtw12_tap_src_t tap_src;
    err = lis2dtw12_tap_src_get(&acc_ctx, &tap_src);
    uint8_t val = tap_src.single_tap;
    NRF_LOG_INFO("Tap SRC: %d", val);

    return err;
}

void acc_set_data_rate(lis2dtw12_odr_t data_rate)
{
    lis2dtw12_data_rate_set(&acc_ctx, data_rate);
}

float acc_get_vertical_scaling(void)
{
    
    float acc_full_scale;
    switch(ACC_FULL_SCALE)
    {
    case LIS2DTW12_2g:
        acc_full_scale = 2.0;
        break;
    case LIS2DTW12_4g:
        acc_full_scale = 4.0;
        break;
    case LIS2DTW12_8g:
        acc_full_scale = 8.0;
        break;
    case LIS2DTW12_16g:
        acc_full_scale = 16.0;
        break;
    }

    return acc_full_scale/(float)(1<<15);

}

int32_t acc_get_temperature(uint16_t *temperature_k16)
{
    int32_t err = 0;

    int16_t lis2dtw12_temp;
    err = lis2dtw12_temperature_raw_get(&acc_ctx, &lis2dtw12_temp);
    *temperature_k16 = (uint16_t)((lis2dtw12_temp / 16) + (ROOM_TEMP_K) * 16);
    NRF_LOG_INFO("TEMP: %d", *temperature_k16);

    return err;
}