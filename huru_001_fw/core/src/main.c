#define NRF_LOG_MODULE_NAME main
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pt10.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"

// #include "nrfx_clock.h"
#include "nrf_drv_clock.h"
#include "nrfx_timer.h"
#include "nrfx_nvmc.h"

#include "app_util.h"

#include "nrf_drv_power.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_fstorage_sd.h"
#include "nrf_log_backend_flash.h"

#include "board_config.h"
#include "epoch.h"
#include "epoch_tra.h"
#include "epoch_syn.h"
#include "epoch_sto.h"
#include "huru_ser.h"
#include "hal.h"
#include "usb.h"
#include "internal_flash.h"

NRF_LOG_MODULE_REGISTER();

#define MANUFACTURER_NAME "HURU" /**< Manufacturer. Will be passed to Device Information Service. */
#ifdef PT10A
#define MODEL "PT10A"
#ifdef ADVERTISING_NAME_CLIC
# char DEVICE_NAME[] = "CLIC-%02d-%02d-%05d";
#define NAME_STR_SIZE (17)
#endif
#ifdef ADVERTISING_NAME_PT10A
char DEVICE_NAME[] = "PT10A-%05d";
#define NAME_STR_SIZE (13)
#endif
#endif

char m_device_formatted_name[NAME_STR_SIZE] = {0};

uint32_t m_ble_id;
uint32_t m_deep_sleep_flag;

BLE_BAS_DEF(m_bas);

static void idle_state_handle(void);


/**
 * @brief Handler for timer events.
 */
void timer_rtc_event_handler(nrf_timer_event_t event_type, void *p_context)
{

    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:

        break;

    default:
        // Do nothing.
        break;
    }
}

void clock_event_handler(nrfx_clock_evt_type_t evt)
{
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    // ret_code_t err_code = nrfx_clock_init(clock_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
    nrfx_clock_lfclk_start();

    while (!nrf_drv_clock_lfclk_is_running())
    // while(!nrfx_clock_lfclk_is_running())
    {
        hal_wdt_feed();
        /* Just waiting */
    }
    
    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void log_resetreason(void)
{
    /* Reset reason */
    uint32_t rr = nrf_power_resetreas_get();
    NRF_LOG_INFO("Reset reasons:");
    if (0 == rr)
    {
        NRF_LOG_INFO("- NONE");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_RESETPIN_MASK))
    {
        NRF_LOG_INFO("- RESETPIN");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_DOG_MASK     ))
    {
        NRF_LOG_INFO("- DOG");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_SREQ_MASK    ))
    {
        NRF_LOG_INFO("- SREQ");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_LOCKUP_MASK  ))
    {
        NRF_LOG_INFO("- LOCKUP");
    }
    if (0 != (rr & NRF_POWER_RESETREAS_OFF_MASK     ))
    {
        NRF_LOG_INFO("- OFF");
    }
#if defined(NRF_POWER_RESETREAS_LPCOMP_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_LPCOMP_MASK  ))
    {
        NRF_LOG_INFO("- LPCOMP");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_DIF_MASK     ))
    {
        NRF_LOG_INFO("- DIF");
    }
#if defined(NRF_POWER_RESETREAS_NFC_MASK)
    if (0 != (rr & NRF_POWER_RESETREAS_NFC_MASK     ))
    {
        NRF_LOG_INFO("- NFC");
    }
#endif
    if (0 != (rr & NRF_POWER_RESETREAS_VBUS_MASK    ))
    {
        NRF_LOG_INFO("- VBUS");
    }
    pt10_status.reset_reason = rr;
    nrf_power_resetreas_clear(0xffffffff);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    bool power_management = true;

    if(pt10_get_state() == PT10_STATE_ON_DOCK)
    {
        power_management = false;
    }
    if(NRF_LOG_PROCESS() == true)
    {
        power_management = false;
    }
    if(pt10_get_state() == PT10_STATE_SYNTHESIZE)
    {
        power_management = false;
    }
    if(epoch_storage_get_state() != epoch_storing_state_idle)
    {
        power_management = false;
    }
    if(
        (epoch_transmission_get_state() != epoch_transmission_state_idle) &&
        (epoch_transmission_get_state() != epoch_transmission_state_open_file)
    )
    {
        power_management = false;
    }
    if(pt10_get_state() == PT10_STATE_STREAMING && pt10_status.uart_connected == true)
    {
        power_management = false;
    }
    if(power_management == true)
    {
#if POWER_MANAGEMENT == 1 

        hal_spi_uninit(); // Dangerous if there's an erase operation in progress?
        hal_i2c0_uninit();
        hal_adc_uninit();
        hal_i2c2_uninit();

        nrf_pwr_mgmt_run();
        
        hal_i2c2_init();
        hal_adc_init();
        hal_i2c0_init();
        hal_spi_init();

#endif
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

/**@brief Function for application main entry.
 */
int main(void)
{

#if EXTERNAL_PIN_SUPPORTED == 1
    // Check the two PSELRESET registers for PIN_RESET functionality.
    // If it seems to be there, then disable it and restart. 
    // This is to work around a boot-loader which might insist on re-enabling CONFIG_GPIO_AS_PINRESET
    if ((NRF_UICR->PSELRESET[0] == PIN_RESET) && (NRF_UICR->PSELRESET[1] == PIN_RESET))
    {
        // Write only one of the PSELRESET registers to zero. 
        // The mis-match will disable pin reset functionality.
        // The assertion of both CONNECT flags will keep the start-up code in the bootloader happy.
        nrfx_nvmc_word_write((uint32_t)&(NRF_UICR->PSELRESET[0]), 0x00000000);
        NVIC_SystemReset();
    }
#endif

    hal_delay_ms(1);

    // #if CONFIG_JLINK_MONITOR_ENABLED
    NVIC_SetPriority(DebugMonitor_IRQn, _PRIO_SD_LOW);
    // #endif    

    hal_gpio_init();

    clocks_start();

    // Initialize.
    log_init();

    // get device ID
    init_internal_flash();
    read_stored_data();
    get_device_id(&m_ble_id);
    get_deep_sleep_flag(&m_deep_sleep_flag);

    if(m_deep_sleep_flag == 1)
    {
        pt10_check_connection_impedance();
        // If we get here, then we're now out of deep sleep mode
    }

#ifdef ADVERTISING_NAME_CLIC
    uint32_t month = m_ble_id / 1e7;
    uint32_t year = (m_ble_id - (month * 1e7)) / 1e5;
    uint32_t serial = m_ble_id - (month * 1e7) - (year * 1e5);
    snprintf(m_device_formatted_name, NAME_STR_SIZE, DEVICE_NAME, month, year, serial);
#endif
#ifdef ADVERTISING_NAME_PT10A
    snprintf(m_device_formatted_name, NAME_STR_SIZE, DEVICE_NAME, m_ble_id);
#endif
    
    NRF_LOG_INFO("%s", NRF_LOG_PUSH(m_device_formatted_name)); 
    
    timers_init();
    power_management_init();

    log_resetreason();
    nrf_power_resetreas_clear(nrf_power_resetreas_get());

    int32_t err = 0;
    err = usb_init();
    if(!err)
    {
        NRF_LOG_INFO("USB ok");
    }
    else
    {
        NRF_LOG_WARNING("USB err");
    }

    // Start execution.
    NRF_LOG_INFO("PT10 started.");
    // application_timers_start();

    pt10_init();

#if USE_WATCHDOG == 1
    hal_wdt_init();   
#endif

    PT10_LOG_FLUSH();

    // Enter main loop.
    for (;;)
    {
        pt10_run();
        hal_wdt_feed();
        idle_state_handle();
    }
}

/**
 * @}
 */
