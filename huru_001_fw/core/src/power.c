#define NRF_LOG_MODULE_NAME power 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "power.h"
#include "board_config.h"
#include "hal.h"
#include "nrfx_saadc.h"
#include "nrf_log.h"

NRF_LOG_MODULE_REGISTER();

bool _usb_power_state = false;
uint8_t _battery_percent = 0;

int16_t _vbat_mv;
int16_t _vbat_lpf_mv;
int16_t _vbat_array[VBAT_FILTER_LENGTH];
uint8_t _vbat_array_idx = 0;
bool _vbat_filter_init = false;

pt10_battery_state_e _pt10_battery_state = BATTERY_STATE_UNKNOWN;
pt10_charge_state_e _pt10_charge_state = BATTERY_CHARGE_DISCONNECTED;

void set_usb_power_state(bool usb_power_state)
{
    _usb_power_state = usb_power_state;
}
bool get_usb_power_state(void)
{
    return _usb_power_state;
}

uint8_t get_battery_percent(void)
{
    return _battery_percent;
}

uint16_t get_battery_mv(void)
{
    return _vbat_lpf_mv;
}

int32_t update_battery_voltage(void)
{
    int32_t err = 0;

    nrf_saadc_value_t raw_value = 0;
    hal_gpio_set(PIN_VBAT_ADC_EN);
    hal_delay_us(VBAT_EN_RISE_TIME_US);
    // read adc pin
    err = nrfx_saadc_sample_convert(PIN_VBAT_ADC, &raw_value);

    hal_gpio_clear(PIN_VBAT_ADC_EN);

    if (err != 0)
    {
        return err;
    }

    _vbat_mv = (int32_t)raw_value * ((int32_t)(ADC_REF_MV / (RDIV_GAIN * PREAMP_GAIN))) >> ADC_BITS;

    if (!_vbat_filter_init)
    {
        for (int j = 0; j < VBAT_FILTER_LENGTH; j++)
        {
            _vbat_array[j] = _vbat_mv;
        }
        _vbat_filter_init = true;
    }

    _vbat_array[_vbat_array_idx] = _vbat_mv;
    int32_t vbat_total = 0;
    _vbat_array_idx = (_vbat_array_idx + 1) % VBAT_FILTER_LENGTH;
    for (uint8_t i = 0; i < VBAT_FILTER_LENGTH; i++)
    {
        vbat_total += _vbat_array[i];
    }
    _vbat_lpf_mv = vbat_total >> VBAT_FILTER_LENGTH_LOG2;
    NRF_LOG_DEBUG("Battery %dmv", _vbat_lpf_mv);

    _battery_percent = (_vbat_lpf_mv - BATTERY_EMPTY_MV) * (100) / (BATTERY_FULL_MV - BATTERY_EMPTY_MV);
    if (_battery_percent > 100)
    {
        _battery_percent = 100;
    }

    return err;
}

void update_power(void)
{
    // read battery voltage
    update_battery_voltage();

    // read charger state
    bool charger_err = hal_gpio_pin_read(PIN_CHARGE_ERROR);
    bool charger_status = hal_gpio_pin_read(PIN_CHARGE_STAT);

    // usb power state is updated in an interrupt, but will often lag behind the status pin

    // update charger status
    if (!charger_err)
    { // err pin is pulled up and active low
        // charger error
        _pt10_charge_state = BATTERY_CHARGE_ERROR;
    }
    else if (!charger_status)
    {

        _pt10_charge_state = BATTERY_CHARGE_CHARGING;
    }
    else if (!_usb_power_state)
    {
        // no error and no usb power, we're disconnected
        _pt10_charge_state = BATTERY_CHARGE_DISCONNECTED;
    }
    else if (_usb_power_state & charger_status)
    {
        // no error, usb power and charger status is high means that we've finished charging
        _pt10_charge_state = BATTERY_CHARGE_COMPLETED;
    }
    else
    {
        // not sure when we'd end up here but just in case
        _pt10_charge_state = BATTERY_CHARGE_DISCONNECTED;
    }

    // update battery state
    if (_battery_percent > 95)
    {
        _pt10_battery_state = BATTERY_STATE_FULL_BATTERY; // we should defer to "_pto09_charge_status" tp indicate this
    }
    else if (_battery_percent > 10)
    {
        _pt10_battery_state = BATTERY_STATE_IN_USE_BATTERY;
    }
    else if (_battery_percent > 5)
    {
        _pt10_battery_state = BATTERY_STATE_LOW_BATTERY;
    }
    else
    {
        _pt10_battery_state = BATTERY_STATE_EMPTY_BATTERY;
    }
}

pt10_battery_state_e get_battery_state(void)
{
    return _pt10_battery_state;
}
pt10_charge_state_e get_charge_state(void)
{
    return _pt10_charge_state;
}
