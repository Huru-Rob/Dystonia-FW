#ifndef __HAL_H
#define __HAL_H
#include <stdint.h>
#include <stdbool.h>
#include "nrf_gpio.h"
#include "RV3028.h"

#define HAL_ERROR_BASE                  (0xE00)
#define HAL_ERROR_I2C_CMD_TOO_LONG      (HAL_ERROR_BASE + 0x01)
#define HAL_ERROR_INVALID_I2C_ID        (HAL_ERROR_BASE + 0x02)
#define HAL_ERROR_I2C_READ              (HAL_ERROR_BASE + 0x03)

typedef enum {
    external_pin_function_default = 0,
    external_pin_function_sticky,
    external_pin_function_uart_rx,
    external_pin_function_uart_tx
} external_pin_function_e;

#define VBAT_MV_PER_LSB (0.0366210938F)
// 0.6/16384

#define VBAT_RDIV_RATIO (2.0F)
// R1 10k, R2 10k

void hal_gpio_init(void);
void hal_gpio_toggle(uint16_t pin_num);
void hal_gpio_set(uint16_t pin_num);
void hal_gpio_clear(uint16_t pin_num);
void hal_delay_ms(uint32_t ms);
void hal_delay_us(uint32_t ms);
void hal_gpio_write(uint16_t pin_num,bool set);
bool hal_gpio_pin_read(uint16_t pin_num);
int32_t halSetExternalPinFunction(external_pin_function_e function);

int32_t hal_spi_init(void);
int32_t hal_spi_uninit(void);
void wait_for_flash_spi(void);
int32_t hal_spi_read (uint8_t* add, uint8_t add_size, uint8_t* rx, uint32_t rx_size );
int32_t hal_spi_write ( uint8_t* tx, uint32_t tx_size );

int32_t hal_spi_transfer (uint8_t* tx, uint32_t tx_size, uint8_t* rx, uint32_t rx_size );

#define ACC_I2C_BUS 0
#define AFE_I2C_BUS 1

int32_t hal_i2c0_init(void);
int32_t hal_i2c0_uninit(void);
int32_t hal_i2c2_init(void);
int32_t hal_i2c2_uninit(void);

void hal_i2c_write_single(uint8_t bus, uint8_t device_address, uint8_t register,uint8_t value);
void hal_i2c_read_single(uint8_t bus, uint8_t device_address, uint8_t register, uint8_t * value);

int32_t hal_i2c_write(uint8_t bus, uint8_t device_address, uint8_t reg, const uint8_t *data, uint16_t len);
int32_t hal_i2c_read(uint8_t bus, uint8_t device_address, uint8_t reg, uint8_t *data ,uint16_t len);

void hal_wdt_feed(void);
void hal_wdt_init(void);

int32_t hal_adc_init(void);
int32_t hal_adc_uninit(void);

void hal_set_board_for_shutdown(void);

bool hal_timeout (uint32_t* timestamp, bool init, uint32_t expiry);


#endif