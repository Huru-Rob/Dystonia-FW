#define NRF_LOG_MODULE_NAME hw_rtc 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "hw_rtc.h"
#include "RV3028.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrfx_gpiote.h"
#include "board_config.h"
#include "hal.h"

NRF_LOG_MODULE_REGISTER();

void (*rtc_int_callback)(void);

rv3028_t RTC;
rv3028_init_t RTC_Init = {
    // Use this settings to enable the battery backup
    .BatteryMode = RV3028_BAT_LSM,
    .Resistance = RV3028_TCT_3K,
    .EnableBSIE = false,
    .EnableCharge = false,

    // Use this settings to configure the time stamp function
    //.TSMode = RV3028_TS_BAT,
    .EnableTS = false,
    //.EnableTSOverwrite = true,

    // Use this settings to enable the clock output
    .Frequency = RV3028_CLKOUT_32KHZ,
    .EnableClkOut = true,

    // Use this settings for the event input configuration
    .EnableEventInt = false,
    .EventHighLevel = false,
    .Filter = RV3028_FILTER_256HZ,

    // Set the current time
    .HourMode = RV3028_HOURMODE_24,
    .p_CurrentTime = NULL,
    .CurrentUnixTime = 0,

    // Use this settings for the Power On Reset interrupt
    .EnablePOR = false,

    // Use this settings for the password function
    .Password = PASSWORD,
    //.Password = 0x00,
};

rv3028_error_t RV3028_Write(uint8_t Device_Addr, uint8_t Reg_Addr, const uint8_t *p_Reg_Data, uint32_t Length)
{
    rv3028_error_t err = RV3028_NO_ERROR;
    err = hal_i2c_write(0, Device_Addr, Reg_Addr, p_Reg_Data, Length);
    return err;
}

rv3028_error_t RV3028_Read(uint8_t Device_Addr, uint8_t Reg_Addr, uint8_t *p_Reg_Data, uint32_t Length)
{
    rv3028_error_t err = RV3028_NO_ERROR;
    err = hal_i2c_read(0, Device_Addr, Reg_Addr, p_Reg_Data, Length);
    return err;
}

rv3028_error_t RV3028_Interface(rv3028_t *p_Device)
{
    if (p_Device == NULL)
    {
        return RV3028_INVALID_PARAM;
    }

    p_Device->p_Read = RV3028_Read;
    p_Device->p_Write = RV3028_Write;
    p_Device->DeviceAddr = RV3028_ADDRESS;

    return RV3028_NO_ERROR;
}

void hw_rtc_in_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Callback function when the interrupt is triggered
    if (pin == PIN_RTC_INT)
    {
        if (action == NRF_GPIOTE_POLARITY_HITOLO)
        {
            if (rtc_int_callback != NULL)
            {
                rtc_int_callback();
            }
        }
    }
}

int32_t hw_rtc_init(void)
{
    rv3028_error_t ErrorCode;

    ErrorCode = RV3028_Interface(&RTC);
    if (ErrorCode == RV3028_NO_ERROR)
    {
        RTC.IsInitialized = true;
        // uint8_t regbuf[0x40];
        // RV3028_RegDump(&RTC, regbuf, sizeof(regbuf));
        ErrorCode = RV3028_DisableWP(&RTC, PASSWORD);
    }
    if(ErrorCode == RV3028_NO_ERROR)
    {        
        ErrorCode = RV3028_Init(&RTC_Init, &RTC);
    }

    if (ErrorCode == RV3028_NO_ERROR)
    {
        uint8_t Status;

        NRF_LOG_INFO("RTC initialized...");
        NRF_LOG_INFO("  HID: %u", RTC.HID);
        NRF_LOG_INFO("  VID: %u", RTC.VID);

        // hal_delay_ms(1000);

        RV3028_GetFlags(&RTC, &Status);
        NRF_LOG_INFO("  Status: 0x%x", Status);

        // Check for a Power On Reset and clear the flag
        if (Status & RV3028_FLAG_POR)
        {
            NRF_LOG_INFO("  Power On Reset...");
            RV3028_ClearFlags(&RTC, RV3028_FLAG_POR);
        }

        hw_rtc_disable_periodic_countdown_timer();        
    }

    if (ErrorCode != RV3028_NO_ERROR)
    {
        NRF_LOG_WARNING("Can not initialize RTC. Error: %u", ErrorCode);
    }

    if(ErrorCode == RV3028_NO_ERROR)
    {
        // setup the pin change int on the adc rdy pin
        ret_code_t err_code;
        if (!nrfx_gpiote_is_init())
        {
            err_code = nrfx_gpiote_init();
            APP_ERROR_CHECK(err_code);
        }

        nrfx_gpiote_in_config_t in_config_1 = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
        in_config_1.pull = NRF_GPIO_PIN_PULLUP;
        err_code = nrfx_gpiote_in_init(PIN_RTC_INT, &in_config_1, hw_rtc_in_pin_handler);
        APP_ERROR_CHECK(err_code);

        nrfx_gpiote_in_event_enable(PIN_RTC_INT, true);
    }

    return ErrorCode;
}

int32_t hw_rtc_get_unix_time(uint32_t *unix_time)
{
    rv3028_error_t ErrorCode;

    ErrorCode = RV3028_GetUnixTime(&RTC, unix_time);
    return (int32_t)ErrorCode;
}

int32_t hw_rtc_set_unix_time(uint32_t unix_time)
{
    rv3028_error_t ErrorCode;

    ErrorCode = RV3028_UnlockWP(&RTC, PASSWORD);
    if (!ErrorCode)
    {
        ErrorCode = RV3028_SetUnixTime(&RTC, unix_time);
    }
    if (!ErrorCode)
    {
        ErrorCode = RV3028_UnlockWP(&RTC, 0);
    }

    return (int32_t)ErrorCode;
}

int32_t hw_rtc_enable_periodic_update(void)
{
    int32_t ErrorCode;

    ErrorCode = RV3028_UnlockWP(&RTC, PASSWORD);
    if(!ErrorCode)
    {
        rv3028_tu_config_t tu_config = 
        {
            .UseClockOut = false,
            .UseInt = true,
            .Frequency = RV3028_PERIODIC_UPDATE_1HZ
        };
        ErrorCode = RV3028_InitPeriodicTimeUpdate(&RTC, &tu_config);
    }    
    if(!ErrorCode)
    {
        ErrorCode = RV3028_UnlockWP(&RTC, 0);
    }
    return ErrorCode;
}

int32_t hw_rtc_disable_periodic_update(void)
{
    int32_t ErrorCode;

    ErrorCode = RV3028_UnlockWP(&RTC, PASSWORD);
    if(!ErrorCode)
    {
        ErrorCode = RV3028_DisablePeriodicTimeUpdate(&RTC);
    }
    if(!ErrorCode)
    {
        ErrorCode = RV3028_UnlockWP(&RTC, 0);
    }
    return ErrorCode;
}

int32_t hw_rtc_enable_periodic_countdown_timer(uint16_t seconds)
{
    int32_t ErrorCode;

    ErrorCode = RV3028_UnlockWP(&RTC, PASSWORD);
    if(!ErrorCode)
    {
        rv3028_cd_config_t cd_config =
            {
                .UseClockOut = false,
                .UseInt = true,
                .EnableRepeat = true,
                .Frequency = RV3028_COUNTDOWN_1HZ,
                .Value = seconds};
        ErrorCode = RV3028_InitCountdown(&RTC, &cd_config);
    }
    if (!ErrorCode)
    {
        ErrorCode = RV3028_UnlockWP(&RTC, 0);
    }

    return ErrorCode;
}

int32_t hw_rtc_disable_periodic_countdown_timer(void)
{
    int32_t ErrorCode;

    ErrorCode = RV3028_UnlockWP(&RTC, PASSWORD);
    if(!ErrorCode)
    {
        ErrorCode = RV3028_DisableCountdown(&RTC);
    }
    if (!ErrorCode)
    {
        ErrorCode = RV3028_UnlockWP(&RTC, 0);
    }

    return ErrorCode;
}

int32_t hw_rtc_update_periodic_timeout(uint16_t seconds)
{
    int32_t ErrorCode;

    ErrorCode = RV3028_UnlockWP(&RTC, PASSWORD);
    if(!ErrorCode)
    {
        ErrorCode = RV3028_UpdateCountdown(&RTC, seconds);
    }
    if (!ErrorCode)
    {
        ErrorCode = RV3028_UnlockWP(&RTC, 0);
    };

    return ErrorCode;
}

void hw_rtc_register_countdown_timer_callback(void (*callback)(void))
{
    rtc_int_callback = callback;
}

int32_t hw_rtc_enable_clkout(bool enable, bool disable_sync)
{
    int32_t ErrorCode = 0;
    ErrorCode = RV3028_UnlockWP(&RTC, PASSWORD);
    if(!ErrorCode)
    {
        ErrorCode = RV3028_EnableClkOut(&RTC, false, false);
    }
    if (!ErrorCode)
    {
        ErrorCode = RV3028_UnlockWP(&RTC, 0);
    }
    return ErrorCode;
}

int32_t hw_rtc_dump_registers(void)
{
    int32_t ErrorCode = 0;

    for(uint8_t i = 0; i <= 0x3f; i++)
    {
        uint8_t Temp;
        ErrorCode = RV3028_Read(RTC.DeviceAddr, i, &Temp, sizeof(Temp));
        NRF_LOG_INFO("%02x: %02x", i, Temp);
    }

    return ErrorCode;
}