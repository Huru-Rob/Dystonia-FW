#define NRF_LOG_MODULE_NAME mag 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "stdint.h"
#include "stdbool.h"
#include "board_config.h"
#include "nrf_log.h"
#include "app_timer.h"
#include "mag.h"
#include "LIS2MDL.h"
#include "hal.h"

NRF_LOG_MODULE_REGISTER();

stmdev_ctx_t mag_ctx;
mag_mode_e _current_mag_mode = mag_mode_reset;
bool _mag_data_ready = false;

int32_t write_mag(void *bus, uint8_t reg, const uint8_t *data, uint16_t len)
{
    int32_t error = 0;

    hal_i2c_write(0, LIS2MDL_I2C_ADD, reg, data, len);

    return error;
}

int32_t read_mag(void *bus, uint8_t reg, uint8_t *data, uint16_t len)
{
    int32_t error = 0;

    error = hal_i2c_read(0, LIS2MDL_I2C_ADD, reg, data, len);

    return error;
}

void mag_int_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    _mag_data_ready = true;
}

int32_t mag_init(void)
{
    int32_t err = 0;

    mag_ctx.write_reg = write_mag;
    mag_ctx.read_reg = read_mag;

    // hal_delay_ms(20);

    uint8_t who_am_i = 0;

    lis2mdl_device_id_get(&mag_ctx, &who_am_i);

    if (who_am_i != LIS2MDL_ID)
    {
        NRF_LOG_WARNING("Init error. ID reads %d", who_am_i);
        err = MAG_ERROR_INCORRECT_ID;
    }

    if(!err)
    {
        lis2mdl_reset_set(&mag_ctx, 1);
        lis2mdl_reset_set(&mag_ctx, 0);
    }

    lis2mdl_data_format_set(&mag_ctx, 0);
    lis2mdl_low_pass_bandwidth_set(&mag_ctx, LIS2MDL_ODR_DIV_4);
    lis2mdl_data_rate_set(&mag_ctx, LIS2MDL_ODR_10Hz);
    lis2mdl_drdy_on_pin_set(&mag_ctx, 1);        
    lis2mdl_offset_temp_comp_set(&mag_ctx, 1); 
    lis2mdl_power_mode_set(&mag_ctx, LIS2MDL_LOW_POWER); 
    lis2mdl_set_rst_mode_set(&mag_ctx, LIS2MDL_SET_SENS_ONLY_AT_POWER_ON); // TODO: seems not to reduce power consumption - so revert to default?
    
            

    if(!err)
    {
        nrfx_gpiote_in_config_t in_config_1 = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
        err = nrfx_gpiote_in_init(PIN_MAG_INT, &in_config_1, mag_int_pin_handler);
        APP_ERROR_CHECK(err);
    }

    return err;
}

int32_t mag_set_mode(mag_mode_e mode)
{
    int32_t err = 0;
    

    if (mode != _current_mag_mode)
    {

        switch (mode)
        {

        case mag_mode_reset:
            break;

        case mag_mode_off:
            nrfx_gpiote_in_event_disable(PIN_MAG_INT);
            lis2mdl_operating_mode_set(&mag_ctx, LIS2MDL_POWER_DOWN);
            break;

        case mag_mode_stream:
            mag_sample_t dummy_sample;
            uint8_t dummy_count;
            mag_read_fifo(&dummy_sample, &dummy_count);
            nrfx_gpiote_in_event_enable(PIN_MAG_INT, true);            
            lis2mdl_operating_mode_set(&mag_ctx, LIS2MDL_CONTINUOUS_MODE);            
            break;
        }
    }

    _current_mag_mode = mode;

    // uint8_t res[3];
    // int32_t ret;
    // ret = lis2mdl_read_reg(&mag_ctx, LIS2MDL_CFG_REG_A, res+0, 1);
    // ret = lis2mdl_read_reg(&mag_ctx, LIS2MDL_CFG_REG_B, res+1, 1);
    // ret = lis2mdl_read_reg(&mag_ctx, LIS2MDL_CFG_REG_C, res+2, 1);
    // NRF_LOG_INFO("Config[0,1,2] %d %d %d", res[0], res[1], res[2]);

    return err;
}

bool mag_data_available(void)
{
    return _mag_data_ready;
}

uint8_t _mag_fifo_buf[6];
int32_t mag_read_fifo(mag_sample_t *buf, uint8_t *samples_read)
{
    int32_t err = 0;
    lis2mdl_read_reg(&mag_ctx, LIS2MDL_OUTX_L_REG, _mag_fifo_buf, 6);
    buf[0].x = _mag_fifo_buf[0] + (_mag_fifo_buf[1] << 8);
    buf[0].y = _mag_fifo_buf[2] + (_mag_fifo_buf[3] << 8);
    buf[0].z = _mag_fifo_buf[4] + (_mag_fifo_buf[5] << 8);
    *samples_read = 1;
    _mag_data_ready = false;
    return err;
}

float mag_get_vertical_scaling(void)
{
    float vertical_scaling = 0.0;

    // return the vertical scaling in gauss per LSB
    vertical_scaling = lis2mdl_from_lsb_to_mgauss(1) * 1.0e-3;

    return vertical_scaling;
}