#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "nrf_gpio.h"
#include "pt10.h"

#define FIRMWARE_VERSION_TXT "V0.1.5"
#define FIRMWARE_VERSION ((0 << 16) | (1 << 8) | (5 << 0))
#define HARDWARE_VERSION 0x01

#define PT10A //PT10A hardware
#define ADVERTISING_NAME_PT10A
#define HURU_PROTOCOL_VERSION (0x0102)

// undef this to use the randomised MAC address assigned to the nRF52 device during manufacture
#undef USE_FIXED_MAC_ADDR

// For a firmware release, all of the following enables should be set to 1
#define EXTERNAL_PIN_SUPPORTED      (1) // Set to 1 to enable the use of the RESET PIN for sticky pad detection or UART dock
#define ENABLE_BLE                  (0) // Set to 0 to disable BLE for debug purposes
#define USE_WATCHDOG                (1) // Set to 1 to enable the watchdog
#define AFE_OVERSAMPLING            (1) // Set to 1 to oversample EEG by 4x and decimate to ~250sa/sec
#define AFE_CLOCK_SYNC              (1) // Set to 1 to synchronise AFE with RTC clock (sample rate = 256sa/sec)
#define POWER_MANAGEMENT            (1) // Set to 0 to prevent the board from going to low power mode when idle
#define TAP_DETECT                  (0) // Set to 1 to enable display of attachment status on detection of tap

#if !defined( PT10A)
    #error ("need a board")
#endif

#define PT10A_ON_PT09_HARDWARE (0)
#if PT10A_ON_PT09_HARDWARE == 0

    // PT10A GPIO Definitions
    //afe
    #define PIN_EEG_INT         NRF_GPIO_PIN_MAP(0,27)
    #define PIN_EEG_INT_2       NRF_GPIO_PIN_MAP(1,9)
    #define PIN_EEG_nRESET      NRF_GPIO_PIN_MAP(0,8)

    //afe
    //i2c2
    #define PIN_I2C2_SCL        NRF_GPIO_PIN_MAP(0,30)
    #define PIN_I2C2_SDA        NRF_GPIO_PIN_MAP(0,28)

    //acc
    //i2c0
    #define PIN_I2C0_SCL        NRF_GPIO_PIN_MAP(1,0)
    #define PIN_I2C0_SDA        NRF_GPIO_PIN_MAP(0,21)
    #define PIN_ACC_INT         NRF_GPIO_PIN_MAP(0,9)
    #define PIN_MAG_INT         NRF_GPIO_PIN_MAP(0,23)

    //flash
    //spi1
    #define PIN_SPI1_SDO        NRF_GPIO_PIN_MAP(0,12)
    #define PIN_SPI1_SDI        NRF_GPIO_PIN_MAP(1,13)
    #define PIN_SPI1_CLK        NRF_GPIO_PIN_MAP(0,15)
    #define PIN_SPI1_nCS        NRF_GPIO_PIN_MAP(1,11)

    //rtc
    #define PIN_RTC_INT         NRF_GPIO_PIN_MAP(1,4)
    #define PIN_RTC_EVI         NRF_GPIO_PIN_MAP(1,7)

    //led
    #define PIN_LED_R           NRF_GPIO_PIN_MAP(0,3)
    #define PIN_LED_B           NRF_GPIO_PIN_MAP(1,14)
    #define PIN_LED_G           NRF_GPIO_PIN_MAP(0,2)

    //charging
    #define PIN_EN_SHIP_MODE    NRF_GPIO_PIN_MAP(0,6)
    #define PIN_CHARGE_STAT     NRF_GPIO_PIN_MAP(1,8) 
    #define PIN_CHARGE_ERROR    NRF_GPIO_PIN_MAP(0,7)

    //board
    #define PIN_1V8_EN          NRF_GPIO_PIN_MAP(0,26)
    #define PIN_LNA_3V0         NRF_GPIO_PIN_MAP(1,10)
    #define PIN_VBAT_ADC        NRF_GPIO_PIN_MAP(0,4)
    #define PIN_VBAT_ADC_EN     NRF_GPIO_PIN_MAP(0,14)

    #define PIN_RESET           NRF_GPIO_PIN_MAP(0,18)

#else

    // PT09A GPIO Definitions
    //afe
    #define PIN_EEG_INT         NRF_GPIO_PIN_MAP(0,27)
    #define PIN_EEG_INT_2       NRF_GPIO_PIN_MAP(1,9)
    #define PIN_EEG_nRESET      NRF_GPIO_PIN_MAP(0,8)

    //afe
    //i2c2
    #define PIN_I2C2_SCL        NRF_GPIO_PIN_MAP(0,21)
    #define PIN_I2C2_SDA        NRF_GPIO_PIN_MAP(1,0)

    //acc
    //i2c0
    #define PIN_I2C0_SCL        NRF_GPIO_PIN_MAP(0,30)
    #define PIN_I2C0_SDA        NRF_GPIO_PIN_MAP(0,28)
    #define PIN_ACC_INT         NRF_GPIO_PIN_MAP(1,14)
    #define PIN_MAG_INT         NRF_GPIO_PIN_MAP(0,23)

    //flash
    //spi1
    #define PIN_SPI1_SDO        NRF_GPIO_PIN_MAP(0,12)
    #define PIN_SPI1_SDI        NRF_GPIO_PIN_MAP(1,13)
    #define PIN_SPI1_CLK        NRF_GPIO_PIN_MAP(0,15)
    #define PIN_SPI1_nCS        NRF_GPIO_PIN_MAP(1,11)

    //rtc
    #define PIN_RTC_INT         NRF_GPIO_PIN_MAP(1,4)
    #define PIN_RTC_EVI         NRF_GPIO_PIN_MAP(1,7)

    //led
    #define PIN_LED_R           NRF_GPIO_PIN_MAP(0,3)
    #define PIN_LED_B           NRF_GPIO_PIN_MAP(0,9)
    #define PIN_LED_G           NRF_GPIO_PIN_MAP(0,5)

    //charging
    #define PIN_EN_SHIP_MODE    NRF_GPIO_PIN_MAP(0,6)
    #define PIN_CHARGE_STAT     NRF_GPIO_PIN_MAP(1,8) 
    #define PIN_CHARGE_ERROR    NRF_GPIO_PIN_MAP(0,7)

    //board
    #define PIN_1V8_EN          NRF_GPIO_PIN_MAP(0,26)
    #define PIN_LNA_3V0         NRF_GPIO_PIN_MAP(0,2)
    #define PIN_VBAT_ADC        NRF_GPIO_PIN_MAP(0,4)
    #define PIN_VBAT_ADC_EN     NRF_GPIO_PIN_MAP(0,14)

    #define PIN_RESET           NRF_GPIO_PIN_MAP(0,18)

#endif

#endif