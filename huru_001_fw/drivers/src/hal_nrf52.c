#include "hal.h"
#include "board_config.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrfx_twi.h"
#include "nrfx_twim.h"
#include "nrfx_spim.h"
#include "nrfx_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrfx_wdt.h"
#include "nrf_pwr_mgmt.h"
#include "nrfx_uarte.h"
#include "serial.h"

nrfx_wdt_channel_id m_channel_id;

static const nrfx_twim_t m_twi0 = NRFX_TWIM_INSTANCE(0); // acc
static const nrfx_twim_t m_twi1 = NRFX_TWIM_INSTANCE(1); // afe
static const nrfx_spim_t m_spi2 = NRFX_SPIM_INSTANCE(3);



external_pin_function_e external_pin_function = external_pin_function_default;

// wdt
void hal_wdt_feed(void)
{
#if USE_WATCHDOG == 1
	nrfx_wdt_channel_feed(m_channel_id);
#endif
}

void hal_wdt_event_handler(void)
{
}

void hal_wdt_init(void)
{
	nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;
	int32_t err_code = nrfx_wdt_init(&config, hal_wdt_event_handler);
	APP_ERROR_CHECK(err_code);
	err_code = nrfx_wdt_channel_alloc(&m_channel_id);
	APP_ERROR_CHECK(err_code);
	nrfx_wdt_enable();
}

void hal_gpio_init(void)
{
	hal_gpio_set(PIN_LED_R);
	nrf_gpio_cfg_output(PIN_LED_R);
	hal_gpio_set(PIN_LED_G);
	nrf_gpio_cfg_output(PIN_LED_G);
	hal_gpio_set(PIN_LED_B);
	nrf_gpio_cfg_output(PIN_LED_B);

	hal_gpio_clear(PIN_VBAT_ADC_EN);
	nrf_gpio_cfg_output(PIN_VBAT_ADC_EN);
	hal_gpio_clear(PIN_EN_SHIP_MODE);
	nrf_gpio_cfg_output(PIN_EN_SHIP_MODE);
	hal_gpio_clear(PIN_EEG_nRESET);
	nrf_gpio_cfg_output(PIN_EEG_nRESET);

	hal_gpio_clear(PIN_1V8_EN);
	nrf_gpio_cfg_output(PIN_1V8_EN);
	hal_gpio_set(PIN_LNA_3V0);
	nrf_gpio_cfg_output(PIN_LNA_3V0);

	nrf_gpio_cfg_input(PIN_CHARGE_STAT, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(PIN_CHARGE_ERROR, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(PIN_RTC_INT, NRF_GPIO_PIN_PULLUP);
	hal_gpio_clear(PIN_RTC_EVI);
    nrf_gpio_cfg_output(PIN_RTC_EVI);
}


// This function is intended to set the board up from a fresh power-up
// into a configuration supporting the lowest possible power consumption
void hal_set_board_for_shutdown(void)
{
    hal_gpio_set(PIN_LED_R);
	nrf_gpio_cfg_output(PIN_LED_R);

	hal_gpio_set(PIN_LED_G);
    nrf_gpio_cfg_output(PIN_LED_G);
	
	hal_gpio_set(PIN_LED_B);
	nrf_gpio_cfg_output(PIN_LED_B);

    hal_gpio_clear(PIN_VBAT_ADC_EN);
    nrf_gpio_cfg_output(PIN_VBAT_ADC_EN);
    hal_gpio_clear(PIN_EN_SHIP_MODE);
	nrf_gpio_cfg_output(PIN_EN_SHIP_MODE);
	nrf_gpio_cfg_default(PIN_CHARGE_STAT);
	nrf_gpio_cfg_default(PIN_CHARGE_ERROR);

    hal_gpio_clear(PIN_EEG_nRESET);
	nrf_gpio_cfg_output(PIN_EEG_nRESET);
    hal_gpio_clear(PIN_1V8_EN);
	nrf_gpio_cfg_output(PIN_1V8_EN);
	hal_gpio_clear(PIN_LNA_3V0);
	nrf_gpio_cfg_output(PIN_LNA_3V0);

    hal_gpio_set(PIN_SPI1_nCS);
	nrf_gpio_cfg_output(PIN_SPI1_nCS);
    hal_gpio_clear(PIN_SPI1_SDO);
	nrf_gpio_cfg_output(PIN_SPI1_SDO);
    hal_gpio_clear(PIN_SPI1_CLK);
	nrf_gpio_cfg_output(PIN_SPI1_CLK);
    nrf_gpio_cfg_input(PIN_SPI1_SDI, NRF_GPIO_PIN_PULLDOWN);

    //hal_gpio_set(PIN_I2C2_SCL);
	hal_gpio_clear(PIN_I2C2_SCL);
	nrf_gpio_cfg_output(PIN_I2C2_SCL);
	//nrf_gpio_cfg_default(PIN_I2C2_SCL);    
    //hal_gpio_set(PIN_I2C2_SDA);
	hal_gpio_clear(PIN_I2C2_SDA);
	nrf_gpio_cfg_output(PIN_I2C2_SDA);
	//nrf_gpio_cfg_default(PIN_I2C2_SDA);    

    hal_gpio_set(PIN_I2C0_SCL);
	nrf_gpio_cfg_output(PIN_I2C0_SCL);
	hal_gpio_set(PIN_I2C0_SDA);
	nrf_gpio_cfg_output(PIN_I2C0_SDA);

	//nrf_gpio_cfg_default(PIN_RESET);
	halSetExternalPinFunction(external_pin_function_default);

    hal_gpio_clear(PIN_RTC_EVI);
    nrf_gpio_cfg_output(PIN_RTC_EVI);

}


int32_t halSetExternalPinFunction(external_pin_function_e function)
{
	int32_t err = 0;

	// First go to the default state
	switch(external_pin_function)
	{
	case external_pin_function_default:
		break;
	case external_pin_function_sticky:
		nrf_gpio_cfg_default(PIN_RESET);
		break;
	case external_pin_function_uart_rx:
		nrf_gpio_cfg_default(PIN_RESET);
		break;
	case external_pin_function_uart_tx:
		break;
	}

	// Now configure the new function
	switch(function)
	{

	case external_pin_function_default:
		nrf_gpio_cfg_default(PIN_RESET);
		break;

	case external_pin_function_sticky:
		nrf_gpio_cfg_input(PIN_RESET, NRF_GPIO_PIN_PULLUP);
		break;

	case external_pin_function_uart_rx:
		break;

	case external_pin_function_uart_tx:
		break;
	}

	external_pin_function = function;

	return err;
}

void hal_gpio_toggle(uint16_t pin_num)
{
	nrf_gpio_pin_toggle(pin_num);
}

void hal_gpio_write(uint16_t pin_num, bool set)
{
	nrf_gpio_pin_write(pin_num, set);
}

void hal_gpio_set(uint16_t pin_num)
{
	nrf_gpio_pin_write(pin_num, 1);
}
void hal_gpio_clear(uint16_t pin_num)
{
	nrf_gpio_pin_write(pin_num, 0);
}

bool hal_gpio_pin_read(uint16_t pin_num)
{
	return nrf_gpio_pin_read(pin_num);
}
void hal_delay_ms(uint32_t ms)
{
	nrf_delay_ms(ms);
}
void hal_delay_us(uint32_t us)
{
	nrf_delay_us(us);
}

// time
bool hal_timeout(uint32_t *timestamp, bool init, uint32_t expiry)
{
	uint32_t difference = 0;
	bool expired = false;

	// We want to initialise out timeout
	if (init)
	{
		*timestamp = app_timer_cnt_get();
		return false;
	}

	// Compare the difference to see if we have expired and set flag
	difference = app_timer_cnt_diff_compute(app_timer_cnt_get(), *timestamp);
	if (difference > APP_TIMER_TICKS(expiry) || expiry == 0)
	{
		expired = true;
		*timestamp = app_timer_cnt_get();
	}

	return expired;
}

volatile bool twi0_xfer_complete = true;
volatile bool twi1_xfer_complete = true;

bool wait_for_twi0(void)
{
	while (!twi0_xfer_complete)
	{
	}
	return true;
}

bool wait_for_twi1(void)
{
	while (!twi1_xfer_complete)
	{
		// NRF_LOG_INFO("%i",twi1_xfer_complete);
	}
	// NRF_LOG_INFO("%i",twi1_xfer_complete);

	return true;
}

void twi_handler(nrfx_twim_evt_t const *p_event, void *p_context)

{
	NRF_LOG_INFO("i2c1 evt");

	if ((p_event->type == NRFX_TWIM_EVT_DONE) &&
		(p_event->xfer_desc.type == NRFX_TWIM_XFER_TX))
	{
		twi1_xfer_complete = true;
	}
}

// i2c
bool _i2c0_initialised = false;
int32_t hal_i2c0_init(void)
{
	ret_code_t err_code = 0;

	if(!_i2c0_initialised)
	{
		const nrfx_twim_config_t twi_0 = {
			.scl = PIN_I2C0_SCL,
			.sda = PIN_I2C0_SDA,
			.frequency = NRF_TWIM_FREQ_100K,
			.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
			.hold_bus_uninit = false,
		};

		err_code = nrfx_twim_init(&m_twi0, &twi_0, NULL, NULL);
		APP_ERROR_CHECK(err_code);

		nrfx_twim_enable(&m_twi0);

		_i2c0_initialised = true;
	}

	return err_code;
}

int32_t hal_i2c0_uninit(void)
{
	int32_t err = 0;

	if(_i2c0_initialised)
	{
		nrfx_twim_disable(&m_twi0);
		nrfx_twim_uninit(&m_twi0);

		hal_gpio_set(PIN_I2C0_SCL);
		nrf_gpio_cfg_output(PIN_I2C0_SCL);
		hal_gpio_set(PIN_I2C0_SDA);
		nrf_gpio_cfg_output(PIN_I2C0_SDA);

		_i2c0_initialised = false;
	}

	return err;
}

bool _i2c2_initialised = false;
int32_t hal_i2c2_init(void)
{
	ret_code_t err_code = 0;

	if(!_i2c2_initialised)
	{
		const nrfx_twim_config_t twi_1 = {
			.scl = PIN_I2C2_SCL,
			.sda = PIN_I2C2_SDA,
			.frequency = NRF_TWIM_FREQ_400K,
			.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
			.hold_bus_uninit = false, // TODO check this assign
		};

		err_code = nrfx_twim_init(&m_twi1, &twi_1, NULL, NULL);
		APP_ERROR_CHECK(err_code);

		nrfx_twim_enable(&m_twi1);

		_i2c2_initialised = true;
	}
	return err_code;
}

int32_t hal_i2c2_uninit(void)
{
	int32_t err = 0;
	if(_i2c2_initialised)
	{
		nrfx_twim_disable(&m_twi1);
		nrfx_twim_uninit(&m_twi1);
		nrf_gpio_cfg_default(PIN_I2C2_SCL);
		nrf_gpio_cfg_default(PIN_I2C2_SDA);
		_i2c2_initialised = false;
	}

	return err;
}

// spi
bool _spi_2_initialised = false;
int32_t hal_spi_init(void)
{
	ret_code_t err_code = 0;

	if(!_spi_2_initialised)
	{
		nrfx_spim_config_t spi_2 = NRFX_SPIM_DEFAULT_CONFIG;

		spi_2.miso_pin = PIN_SPI1_SDI;
		spi_2.mosi_pin = PIN_SPI1_SDO;
		spi_2.sck_pin = PIN_SPI1_CLK;
		spi_2.ss_pin = PIN_SPI1_nCS;
		spi_2.ss_active_high = false;
		// 16MHz is as fast as the PT10 board can go with Standard Drive Strength
		// High drive strength introduces ringing onto the clock and produces multiple edges.
		spi_2.frequency = NRF_SPIM_FREQ_16M;

		err_code = nrfx_spim_init(&m_spi2, &spi_2, NULL, NULL);
		APP_ERROR_CHECK(err_code);
		_spi_2_initialised = true;
	}
	return err_code;
}

int32_t hal_spi_uninit(void)
{
	int32_t err = 0;

	if(_spi_2_initialised)
	{
		nrfx_spim_uninit(&m_spi2);

		nrf_gpio_cfg_input(PIN_SPI1_SDI, NRF_GPIO_PIN_PULLDOWN);

		hal_gpio_clear(PIN_SPI1_SDO);
		nrf_gpio_cfg_output(PIN_SPI1_SDO);

		hal_gpio_clear(PIN_SPI1_CLK);
		nrf_gpio_cfg_output(PIN_SPI1_CLK);

		hal_gpio_set(PIN_SPI1_nCS);
		nrf_gpio_cfg_output(PIN_SPI1_nCS);

		_spi_2_initialised = false;
	}

	return err;
}

int32_t hal_adc_raw_to_mv(nrf_saadc_value_t raw_value, float *voltage)
{
	int err = 0;

	return err;
}

// adc
bool _saadc_initialised = false;

int32_t hal_adc_init(void)
{
	int32_t err = 0;

	if(!_saadc_initialised)
	{
		nrfx_saadc_config_t saadc_conf = NRFX_SAADC_DEFAULT_CONFIG;
		saadc_conf.resolution = NRF_SAADC_RESOLUTION_12BIT;
		ret_code_t err_code = nrfx_saadc_init(&saadc_conf, NULL);
		APP_ERROR_CHECK(err_code);

		nrf_saadc_channel_config_t chan_conf = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(0);
		chan_conf.gain = NRF_SAADC_GAIN1_6;
		chan_conf.reference = NRF_SAADC_REFERENCE_INTERNAL; // 0.6V
		chan_conf.acq_time = NRF_SAADC_ACQTIME_3US;
		chan_conf.pin_p = NRF_SAADC_INPUT_AIN2;
		err_code = nrfx_saadc_channel_init(0, &chan_conf);
		APP_ERROR_CHECK(err_code);
		_saadc_initialised = true;
	}
	return err;
}

int32_t hal_adc_uninit(void)
{
	int32_t err = 0;
	if(_saadc_initialised)
	{
		nrfx_saadc_uninit();
		_saadc_initialised = false;
	}
	return err;
}

uint8_t data_holder[32];
// unchecked
int32_t hal_i2c_write(uint8_t bus, uint8_t device_address, uint8_t reg, const uint8_t *data, uint16_t len)
{
	int32_t error = 0;
	nrfx_twim_t active_bus;

	if (len > 32)
	{
		return HAL_ERROR_I2C_CMD_TOO_LONG;
	}
	data_holder[0] = reg;

	for (uint8_t i = 1; i <= len; i++)
	{
		data_holder[i] = data[i - 1];
	}

	if (bus == 0)
	{
		active_bus = m_twi0;
	}
	else if (bus == 1)
	{
		active_bus = m_twi1;
	}
	else
	{
		error = HAL_ERROR_INVALID_I2C_ID;
		return error;
	}

	ret_code_t err_code = nrfx_twim_tx(&active_bus, device_address, data_holder, len + 1, false);

	if (err_code)
	{
		NRF_LOG_INFO("i2c write err");
		return err_code;
	}

	return error;
}

// unchecked
int32_t hal_i2c_read(uint8_t bus, uint8_t device_address, uint8_t reg, uint8_t *data, uint16_t len)
{
	int32_t error = 0;
	nrfx_twim_t active_bus;

	if (bus == 0)
	{
		active_bus = m_twi0;
	}
	else if (bus == 1)
	{
		active_bus = m_twi1;
	}
	else
	{
		error = HAL_ERROR_INVALID_I2C_ID;
		return error;
	}

	twi1_xfer_complete = false;
	ret_code_t err_code = nrfx_twim_tx(&active_bus, device_address, &reg, 1, true);
	// wait_for_twi1();
	err_code += nrfx_twim_rx(&active_bus, device_address, data, len);
	if (err_code)
	{
		error = HAL_ERROR_I2C_READ;
	}

	return error;
}
uint8_t hal_rx_buf[1024 + 5];

int32_t hal_spi_transfer(uint8_t *tx, uint32_t tx_size, uint8_t *rx, uint32_t rx_size)
{
	nrfx_spim_xfer_desc_t transfer = NRFX_SPIM_XFER_TRX(tx, tx_size, rx, rx_size);
	// rx = &hal_rx_buf[tx_size];
	// nrfx_err_t err =  nrfx_spim_xfer_dcx(&m_spi2, &transfer, 0,1); // cmd for use with spim3 & extended options
	nrfx_err_t err = nrfx_spim_xfer(&m_spi2, &transfer, 0);

	return err;
}

int32_t hal_spi_read(uint8_t *add, uint8_t add_size, uint8_t *rx, uint32_t rx_size)
{

	// send start address
	nrfx_spim_xfer_desc_t transfer = NRFX_SPIM_XFER_TX(add, add_size);
	nrfx_err_t err = nrfx_spim_xfer(&m_spi2, &transfer, 0);
	nrfx_spim_xfer_desc_t receiver = NRFX_SPIM_XFER_RX(rx, rx_size);
	err = nrfx_spim_xfer(&m_spi2, &receiver, 0);

	return err;
}

int32_t hal_spi_write(uint8_t *tx, uint32_t tx_size)
{

	nrfx_spim_xfer_desc_t transfer = NRFX_SPIM_XFER_TX(tx, tx_size);
	nrfx_err_t err = nrfx_spim_xfer(&m_spi2, &transfer, 0);

	return err;
}

