#define NRF_LOG_MODULE_NAME pt10 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "pt10.h"
#include "epoch.h"
#include "epoch_tra.h"
#include "epoch_sto.h"
#include "epoch_syn.h"
#include "stream.h"
#include "storage.h"
#include "hw_rtc.h"
#include "bluetooth.h"
#include "power.h"
#include "hal.h"
#include "board_config.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_info.h"
#include "nrfx_rtc.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrfx_power.h"
#include "nrf_gpio.h"
#include "hal.h"
#include "AFE4960.h"
#include "acc.h"
#include "mag.h"
#include "LIS2DTW12.h"
#include "MT25QL256ABA1EW7.h"
#include "huru_ser.h"
#include "led.h"
#include "afe.h"
#include "internal_flash.h"
#include "usb.h"
#include "serial.h"
#include "oscilltrack.h"
#include "esb.h"

NRF_LOG_MODULE_REGISTER();


void application_timers_stop(void);

bool _rtc_wake_event = false;
int32_t _panic_code = 0;
uint32_t _state_entry_ticks = 0;
uint32_t _state_elapsed_ticks = 0;
uint32_t _high_z_timeout_updates = 0;
pt10_charge_state_e last_charge_state;
bool last_connection_state;
APP_TIMER_DEF(_state_timer);

pt10_status_t pt10_status = {
    .device_state = PT10_STATE_RESET,
    .usb_cdc_connected = false,
    .streaming_mode = false,
    .afe_mode = afe_mode_active,
    .ble_connected = false,
    .uart_connected = false,
    .boot_time = 0,
    .epoch_data_notify_en = false,
    .eeg_stream_notify_en = false,
    .acc_stream_notify_en = false,
    .mag_stream_notify_en = false,
    .imp_stream_notify_en = false,
    .lead_onoff_notify_en = false,
    .lead_impedance_notify_en = false,
    .MTUSet = 0,
    .enter_deep_sleep = false,
    .debug_sensor = DEBUG_NONE,
};

void PT10_STATE_timer_handler(void *p_context);

pt10_status_t *return_device_status_pointer(void)
{
    return &pt10_status;
}
pt10_status_t return_device_status(void)
{
    return pt10_status;
}

void pt10_send_next_epoch(void)
{
    epoch_transmit_next();
}

void pt10_resend_epoch(void)
{
    epoch_re_transmit();
}

void pt10_send_complete(void)
{
    epoch_transmit_complete();
}

void pt10_set_streaming_sensor(uint8_t sensor)
{
    if(sensor == DEBUG_NONE || sensor == DEBUG_EEG || sensor == DEBUG_ACCELEROMETER || sensor == DEBUG_MAGNETOMETER || sensor == DEBUG_IMPEDANCE)
    {
        pt10_status.debug_sensor = sensor;
    }
}

void pt10_set_afe_mode(afe_mode_e mode)
{
    pt10_status.afe_mode = mode;
    afe_set_mode(mode);
}

int32_t pt10_set_ina_gains(ina_gain_t eeg1_ina_gain, bioz_ina_gain_t eeg2_ina_gain)
{
    int32_t err = 0;
    err = afe_set_ina_gains(eeg1_ina_gain, eeg2_ina_gain);
    return err;
}

void pt10_enter_deep_sleep_mode(void)
{
    pt10_status.enter_deep_sleep = true;
}

void pt10_synthesize_epochs(int32_t num_epochs)
{
    if(pt10_status.device_state == PT10_STATE_ON_DOCK)
    {
        pt10_status.num_synthesized_epochs = num_epochs;
        pt10_status.synthesize = true;
    }    
}

void pt10_init(void)
{

    int32_t local_err = 0;
    int32_t init_err = 0;

    pt10_status.post_result = 0;

    local_err = esb_init();
    if(local_err != 0)
    {
        NRF_LOG_WARNING("esb err");
        init_err += 1;
    }
    else
    {
        NRF_LOG_INFO("esb ok");
        pt10_status.post_result |= POST_ESB_PASS;
    }

    local_err = 0;
    local_err = hal_adc_init();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("adc err");
        init_err += 1;
    }
    else
    {
        NRF_LOG_INFO("adc ok");
        pt10_status.post_result |= POST_ADC_PASS;
    }
    PT10_LOG_FLUSH();

    local_err = 0;
    hal_gpio_init();
    local_err = led_init();
    if(local_err != 0)
    {
        NRF_LOG_WARNING("led err");
    }
    else
    {
        NRF_LOG_INFO("led ok");
        pt10_status.post_result |= POST_LED_PASS;
        led_set_for_state(led_state_init);
    }    

    local_err = hal_i2c0_init();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("i2c err");
        init_err += 1;
    }
    else
    {
        NRF_LOG_INFO("i2c0 ok");
        pt10_status.post_result |= POST_I2C_PASS;
    }
    hal_wdt_feed();    
    PT10_LOG_FLUSH();

    local_err = 0;
    local_err = afe_init();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("afe err");
        init_err += 1;
    }
    else
    {
        NRF_LOG_INFO("afe ok");
        pt10_status.post_result |= POST_AFE_PASS;
    }
    local_err = 0;
    afe_set_mode(afe_mode_off);
    PT10_LOG_FLUSH();

    local_err = hw_rtc_init();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("rtc err");
        init_err = 1;
    }
    else
    {
        uint32_t unix_time;
        NRF_LOG_INFO("rtc ok");
        pt10_status.post_result |= POST_RTC_PASS;
        int32_t err = hw_rtc_get_unix_time(&unix_time);
        if(!err)
        {
            pt10_status.boot_time = unix_time;
        }
    }

    // init spi
    local_err = hal_spi_init();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("spi err");
        init_err = 1;
    }
    else
    {
        NRF_LOG_INFO("spi ok");
        pt10_status.post_result |= POST_SPI_PASS;
    }
    hal_wdt_feed();
    PT10_LOG_FLUSH();     

    // init flash
    local_err = init_flash();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("flash err");
        init_err = 1;
    }
    else
    {
        NRF_LOG_INFO("flash ok");
        pt10_status.post_result |= POST_FLASH_PASS;

        hal_wdt_feed();
        PT10_LOG_FLUSH();
        
        // init storage
        local_err = storage_init(false);
        if (local_err != 0)
        {
            NRF_LOG_WARNING("storage err");
            init_err = 1;
        }
        else
        {
            NRF_LOG_INFO("storage ok");
            pt10_status.post_result |= POST_STORAGE_PASS;
        }
    }
    hal_wdt_feed();
    PT10_LOG_FLUSH();


    local_err = acc_init();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("acc err");
        init_err += 1;
    }
    else
    {
        NRF_LOG_INFO("acc ok");
        pt10_status.post_result |= POST_ACC_PASS;
    }
    PT10_LOG_FLUSH();
    local_err = 0;

    local_err = mag_init();
    if (local_err != 0)
    {
        NRF_LOG_WARNING("mag err");
        init_err += 1;
    }
    else
    {
        NRF_LOG_INFO("mag ok");
        pt10_status.post_result |= POST_MAG_PASS;
    }
    PT10_LOG_FLUSH();
    local_err = 0;

    update_power();

    // epoch_data_init();
    // streaming_init();
    // serial_init();
    oscilltrack_init();

    ret_code_t err_code = app_timer_create(&_state_timer, APP_TIMER_MODE_REPEATED, PT10_STATE_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(_state_timer, APP_TIMER_TICKS(INIT_UPDATE_MS), NULL);
    APP_ERROR_CHECK(err_code);

    oscilltrack_start();

    if (init_err == 0)
    {        
        NRF_LOG_INFO("init() ok.");
        last_charge_state = get_charge_state();
        pt10_status.device_state = PT10_STATE_INIT;
        led_set_for_state(led_state_init);
        _state_entry_ticks = app_timer_cnt_get();
    }
    else
    {
        NRF_LOG_WARNING("init() fail err = %i", init_err);
        pt10_status.device_state = PT10_STATE_ERROR;
        led_set_for_state(led_state_error);
        _state_entry_ticks = app_timer_cnt_get();
    }

    PT10_LOG_FLUSH();
}

pt10_states_e pt10_get_state(void)
{
    return pt10_status.device_state;
}

int32_t update_connection_quality(bool enabled)
{
    int32_t err = 0;
    uint8_t amp_buf[sizeof(uint32_t)];
    uint32_t ac_amplitude = 0;

    if (enabled)
    {
        err = update_lead_detect_status();
        ac_amplitude = afe_get_offset_corrected_lead_detect_amplitude();
    }

    amp_buf[0] = (ac_amplitude >> 24) & 0Xff;
    amp_buf[1] = (ac_amplitude >> 16) & 0xff;
    amp_buf[2] = (ac_amplitude >> 8) & 0xff;
    amp_buf[3] = (ac_amplitude >> 0) & 0xff;

    uint8_t flags = afe_get_lead_detect_flags();

    // NRF_LOG_INFO("DC: %d {L1: %d, L2: %d} AC: %d {Amp: %d}",
    //     (flags & (1<<DC_LEAD_DETECT_VALID_BITPOS)) ? 1 : 0,
    //     (flags & (1<<DC_LEAD_DETECT_ELECTRODE_1_BITPOS)) ? 1 : 0,
    //     (flags & (1<<DC_LEAD_DETECT_ELECTRODE_2_BITPOS)) ? 1 : 0,
    //     (flags & (1<<AC_LEAD_DETECT_VALID_BITPOS)) ? 1 : 0,
    //     ac_amplitude
    // );

    if (pt10_status.ble_connected)
    {

        ble_cus_t eeg_cus = get_m_eeg_cus();

        // TODO:
        // Alex' iOS app doesn't read the impedance characteristic, even though it turns 
        // notifications on. So we get an error 19 while trying to push more data to 
        // the characteristic via hvx
        err = ble_cus_value_update(&eeg_cus, eeg_cus.lead_impedance_value_handles, amp_buf, 4, pt10_status.lead_impedance_notify_en);
        if(err != 0 && err != 19)
        {
            NRF_LOG_WARNING("error %d updating impedance characteristic", err);
        }
        if(err == 0 || err == 19)
        err = ble_cus_value_update(&eeg_cus, eeg_cus.lead_onoff_value_handles, &flags, 1, pt10_status.lead_onoff_notify_en);
        if(err != 0 && err != 19)
        {
            NRF_LOG_WARNING("error %d updating impedance characteristic", err);
        }
        if(err == 19)
        {
            err = 0;
        }
    }

    return err;
}

const nrfx_rtc_t rtc2 = NRFX_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */

static void rtc_handler(nrfx_rtc_int_type_t int_type)
{
    if (int_type == NRFX_RTC_INT_COMPARE2)
    {
        _rtc_wake_event = true;
    }
}

static void rtc_config(uint32_t prescaler, uint32_t eighths)

{
    static bool rtc_init = false;
    uint32_t err_code;

    if (!rtc_init)
    {

        // Initialize RTC instance
        nrfx_rtc_config_t config = NRFX_RTC_DEFAULT_CONFIG;

        config.prescaler = prescaler; // Gives 8Hz timer tick
        err_code = nrfx_rtc_init(&rtc2, &config, rtc_handler);
        APP_ERROR_CHECK(err_code);

        rtc_init = true;
    }
    // //Enable tick event & interrupt
    // nrf_drv_rtc_tick_enable(&rtc,true);

    // Set compare channel to trigger interrupt after 10 seconds
 
    // err_code = nrfx_rtc_cc_set(&rtc2, 2, RTC_S_TO_TICKS(WAKE_FROM_SLEEP_PERIOD_S), true);
    err_code = nrfx_rtc_cc_set(&rtc2, 2, eighths, true);
    APP_ERROR_CHECK(err_code);

    // Power on RTC instance
    nrfx_rtc_enable(&rtc2);
    // _wake_event = true;
}

bool _state_timer_elapsed = false;
void PT10_STATE_timer_handler(void *p_context)
{
    _state_timer_elapsed = true;
}

void pt10_run(void)
{
    int32_t err = 0;
    pt10_states_e next_state = pt10_status.device_state;
    _state_elapsed_ticks = app_timer_cnt_diff_compute(app_timer_cnt_get(), _state_entry_ticks);
    uint32_t state_update_ticks = 0;
    static uint8_t good_connection_counter = GOOD_CONNECTION_UPDATES;   

    // err |= bluetooth_buffer_handler();
    // serialUpdate();
    
        
    switch (pt10_status.device_state)
    {
    case PT10_STATE_INIT:

        if (_state_timer_elapsed)
        {
            _state_timer_elapsed = false; 

            update_power();
            pt10_charge_state_e current_charge_state = get_charge_state();
            pt10_battery_state_e current_bat_state = get_battery_state();
            last_charge_state = current_charge_state;
            
            uint32_t deep_sleep_flag = 0;
            get_deep_sleep_flag(&deep_sleep_flag);
            
            if(deep_sleep_flag == 1)
            {
                NRF_LOG_INFO("returning from deep sleep");                
                set_deep_sleep_flag(0);
                save_info_to_local_flash(store_reason_clear_deep_sleep_flag);
                err |= led_set_for_state(led_state_unconnected_unattached);
                state_update_ticks = APP_TIMER_TICKS(IDLE_UPDATE_MS);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                err |= afe_set_mode(pt10_status.afe_mode);
                last_connection_state = pt10_status.ble_connected;
                next_state = PT10_STATE_IDLE;
            }
            
            // State transitions to the ON_DOCK state
            if (current_charge_state == BATTERY_CHARGE_CHARGING)
            {
                // serialStart();
                err |= afe_set_mode(pt10_status.afe_mode); // TODO: just for now - to aid testing
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                oscilltrack_start(); // TODO: just for now - to aid testing
                err |= led_set_for_state(led_state_charging);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                next_state = PT10_STATE_ON_DOCK;
            }
            else if (current_charge_state == BATTERY_CHARGE_COMPLETED)
            {
                // serialStart();
                err |= afe_set_mode(pt10_status.afe_mode); // TODO: just for now - to aid testing
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                oscilltrack_start();
                err |= led_set_for_state(led_state_charge_complete);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                next_state = PT10_STATE_ON_DOCK;
            }

            // State transitions to the IDLE state
            // if reset was caused by a reason from which we would want
            // to resume recording
            else if (
                !(pt10_status.reset_reason & NRF_POWER_RESETREAS_SREQ_MASK) && 
                (
                    (pt10_status.reset_reason & NRF_POWER_RESETREAS_DOG_MASK) ||
                    (pt10_status.reset_reason & NRF_POWER_RESETREAS_LOCKUP_MASK)
                )
            )
            {
                
                if (pt10_status.ble_connected)
                {
                    err |= led_set_for_state(led_state_connected_unattached);
                }
                else
                {
                    err |= led_set_for_state(led_state_unconnected_unattached);
                }
                
                state_update_ticks = APP_TIMER_TICKS(IDLE_UPDATE_MS);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                err |= afe_set_mode(pt10_status.afe_mode);
                last_connection_state = pt10_status.ble_connected;
                // serialStop();
                good_connection_counter = GOOD_CONNECTION_UPDATES;
                next_state = PT10_STATE_IDLE;
            }

            // State Transition to the SHUTDOWN STATE            
            else if (current_bat_state == BATTERY_STATE_EMPTY_BATTERY || current_bat_state == BATTERY_STATE_LOW_BATTERY)
            {
                err |= led_set_for_state(led_state_shutdown);
                // err |= epoch_stop_recording();
                err |= afe_set_mode(afe_mode_off);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                state_update_ticks = APP_TIMER_TICKS(SHUTDOWN_UPDATE_MS);
                next_state = PT10_STATE_SHUTDOWN;
            }

        }

        break;

    case PT10_STATE_ON_DOCK:

        esb_update();

        // err |= epoch_recording_update();
        // err |= epoch_storage_update();
        // err |= epoch_synthesize_update();
        // err |= epoch_transmission_update();

        err |= oscilltrack_update();

        if (_state_timer_elapsed)
        {
            _state_timer_elapsed = false;

            update_power();
            err |= update_connection_quality(true);
            bool can_fit_epoch = false;
            err |= storage_can_fit_epoch(&can_fit_epoch);

            pt10_charge_state_e current_charge_state = get_charge_state();
            if(current_charge_state != last_charge_state)
            {
                if (current_charge_state == BATTERY_CHARGE_COMPLETED)
                {
                    err |= led_set_for_state(led_state_charge_complete);
                }
                else if (current_charge_state == BATTERY_CHARGE_CHARGING)
                {
                    err |= led_set_for_state(led_state_charging);
                }
                last_charge_state = current_charge_state;
            }

            // State Transitions to the IDLE state
            else if (
                // We prevented transitioning to the recording state when the power drops
                // if there was a UART connection. This dates from when the multi-dock
                // raised and lowered the power during scanning.
                // I think it's safe to remove this requirement now.
                // pt10_status.uart_connected != true &&
                current_charge_state == BATTERY_CHARGE_DISCONNECTED &&
                pt10_status.debug_sensor == DEBUG_NONE
            )
            {                
                if (pt10_status.ble_connected)
                {
                    err |= led_set_for_state(led_state_connected_unattached);
                }
                else
                {
                    err |= led_set_for_state(led_state_unconnected_unattached);
                }                
                state_update_ticks = APP_TIMER_TICKS(IDLE_UPDATE_MS);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                err |= afe_set_mode(pt10_status.afe_mode);
                last_connection_state = pt10_status.ble_connected;
                // serialStop();
                good_connection_counter = GOOD_CONNECTION_UPDATES;
                next_state = PT10_STATE_IDLE;
            }
            
        }
        break;

    case PT10_STATE_IDLE:

        esb_update();

        err |= oscilltrack_update();

        if (_state_timer_elapsed)
        {
            _state_timer_elapsed = false;

            update_power();
            err |= update_connection_quality(true);
            
            if (afe_get_lead_detect_amplitude_lpf() > AFE_GOOD_CONNECTION_AMPLITUDE)
            {
                good_connection_counter = GOOD_CONNECTION_UPDATES;
            }
            else if(good_connection_counter > 0)
            {
                good_connection_counter -= 1;
            }
            NRF_LOG_INFO("Amplitude %d, count = %d", afe_get_lead_detect_amplitude_lpf(), good_connection_counter);   

            pt10_charge_state_e current_charge_state = get_charge_state();
            pt10_battery_state_e current_bat_state = get_battery_state();

            // State Transition to the SHUTDOWN STATE
            
            if (current_bat_state == BATTERY_STATE_EMPTY_BATTERY || current_bat_state == BATTERY_STATE_LOW_BATTERY)
            {
                err |= led_set_for_state(led_state_shutdown);
                state_update_ticks = APP_TIMER_TICKS(SHUTDOWN_UPDATE_MS);
                next_state = PT10_STATE_SHUTDOWN;
            } 

            // State Transition to the DEEP_SLEEP_PREP State
            else if(pt10_status.enter_deep_sleep == true)
            {
                pt10_status.enter_deep_sleep = false;
                err |= led_set_for_state(led_state_deep_sleep_prep);
                state_update_ticks = APP_TIMER_TICKS(DEEP_SLEEP_PREP_UPDATE_MS);
                next_state = PT10_STATE_DEEP_SLEEP_PREP;
            }

            // State Transitions to the ON_DOCK STATE

            else if (current_charge_state == BATTERY_CHARGE_CHARGING)
            {
                // err |= afe_set_mode(afe_mode_off);
                // err |= acc_set_mode(acc_mode_off);
                // err |= mag_set_mode(mag_mode_off);
                err |= led_set_for_state(led_state_charging);                
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }
            else if (current_charge_state == BATTERY_CHARGE_COMPLETED)
            {
                // err |= afe_set_mode(afe_mode_off);
                // err |= acc_set_mode(acc_mode_off);
                // err |= mag_set_mode(mag_mode_off);
                err |= led_set_for_state(led_state_charge_complete);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }
            // State transitions to the RECORDING State
            else if (good_connection_counter == 0)
            {
                if (pt10_status.ble_connected)
                {
                    err |= led_set_for_state(led_state_connected_attached);
                }
                else
                {
                    err |= led_set_for_state(led_state_unconnected_attached);
                }
                state_update_ticks = APP_TIMER_TICKS(RECORDING_UPDATE_MS);
                // err |= epoch_start_recording();
                _high_z_timeout_updates = HIGH_Z_TIMEOUT_MS / RECORDING_UPDATE_MS;
                next_state = PT10_STATE_RECORDING;
            }

            // State Transitions to the SLEEP State

            else if(_state_elapsed_ticks > APP_TIMER_TICKS(IDLE_TO_SLEEP_TIMEOUT_MS))
            {
                // uint32_t num_epoch_files;
                // err |= storage_num_epoch_files(&num_epoch_files);
                if (
                    (!pt10_status.ble_connected))
                {
                    err |= led_set_for_state(led_state_sleep);
                    state_update_ticks = APP_TIMER_TICKS(SLEEP_UPDATE_MS);
                    err |= acc_set_mode(acc_mode_wakeup);
                    err |= afe_set_mode(afe_mode_off);
                    err |= mag_set_mode(mag_mode_off);
                    flash_lower_power_mode(true);
            
                    next_state = PT10_STATE_SLEEP;
                }
            }

            
        }

        break;

    case PT10_STATE_RECORDING:

        esb_update();
        
        err |= oscilltrack_update();

        if (_state_timer_elapsed)
        {
            _state_timer_elapsed = false;

            err |= update_connection_quality(true);
            // epoch_update_connection_quality();
            update_power();
            
#if TAP_DETECT == 1
            acc_tap_src_get();
#endif

            pt10_charge_state_e current_charge_state = get_charge_state();
            pt10_battery_state_e current_bat_state = get_battery_state();                         
            
            if (afe_get_lead_detect_amplitude_lpf() < AFE_GOOD_CONNECTION_AMPLITUDE)
            {
                _high_z_timeout_updates = HIGH_Z_TIMEOUT_MS / RECORDING_UPDATE_MS;
            }
            // NRF_LOG_INFO("amplitude %d, updates %d", afe_get_lead_detect_amplitude_lpf(), _high_z_timeout_updates);
            if (_high_z_timeout_updates > 0)
            {
                _high_z_timeout_updates -= 1;
            }

            // State Transitions to the IDLE STATE
            if (_high_z_timeout_updates == 0)
            {
                if (pt10_status.ble_connected)
                {
                    err |= led_set_for_state(led_state_connected_unattached);
                }
                else
                {
                    err |= led_set_for_state(led_state_unconnected_unattached);
                }
                // err |= epoch_stop_recording();
                state_update_ticks = APP_TIMER_TICKS(IDLE_UPDATE_MS);
                next_state = PT10_STATE_IDLE;
            }            

            // State Transition to the SHUTDOWN STATE            
            else if (current_bat_state == BATTERY_STATE_EMPTY_BATTERY || current_bat_state == BATTERY_STATE_LOW_BATTERY)
            {
                err |= led_set_for_state(led_state_shutdown);
                // err |= epoch_stop_recording();
                err |= afe_set_mode(afe_mode_off);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                state_update_ticks = APP_TIMER_TICKS(SHUTDOWN_UPDATE_MS);
                next_state = PT10_STATE_SHUTDOWN;
            }

            // State Transitions to the ON DOCK STATE
            else if (current_charge_state == BATTERY_CHARGE_CHARGING)
            {
                err |= led_set_for_state(led_state_charging);
                // err |= epoch_stop_recording();
                // err |= afe_set_mode(afe_mode_off);
                // err |= acc_set_mode(acc_mode_off);
                // err |= mag_set_mode(mag_mode_off);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }
            else if (current_charge_state == BATTERY_CHARGE_COMPLETED)
            {
                err |= led_set_for_state(led_state_charge_complete);
                // err |= epoch_stop_recording();
                // err |= afe_set_mode(afe_mode_off);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }
            
        }
        break;

    case PT10_STATE_SHUTDOWN:
        if (_state_elapsed_ticks > APP_TIMER_TICKS(SHUTDOWN_PAUSE_MS) && !leds_running())
        {
            hal_gpio_set(PIN_EN_SHIP_MODE);
        }
        // hal_gpio_set(PIN_EN_SHIP_MODE);
        break;

    case PT10_STATE_SLEEP:

        // shutdown sensors here...
        //  turn off the LEDs

        if(!leds_running())
        {
            NRF_LOG_INFO("about to sleep");
            // PT10_LOG_FLUSH();

            rtc_config(RTC_PRESCALE_FOR_8HZ_TICK, 8 * WAKE_FROM_SLEEP_PERIOD_S);

            // start RTC
            nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);
            // go to sleep
            //  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
            _rtc_wake_event = false;
            acc_clear_wake_event();

            hal_adc_uninit();
            hal_spi_uninit();
            hal_i2c2_uninit();
            hal_i2c0_uninit();

            // nrf_drv_clock_hfclk_release();
            while (!_rtc_wake_event && !acc_get_wake_event() && !get_usb_power_state())
            {
                __SEV();
                __WFE();
                __WFI();
                // sd_app_evt_wait();
            }
            // nrf_drv_clock_hfclk_request(NULL);

            hal_i2c0_init();
            hal_i2c2_init();
            hal_spi_init();
            hal_adc_init();

            NRF_LOG_INFO("awake");
            // PT10_LOG_FLUSH();

            // and resume/wake up sensors here
            nrfx_rtc_counter_clear(&rtc2);
            nrfx_rtc_disable(&rtc2);

            // err |= acc_set_mode(acc_mode_off);

            update_power();
            pt10_battery_state_e current_bat_state = get_battery_state();
            pt10_charge_state_e current_charge_state = get_charge_state();            
            NRF_LOG_INFO("Battery %d%%; Charge State %d; Battery State %d", get_battery_percent(), current_charge_state, current_bat_state);

            // State Transition to the SHUTDOWN STATE
            if (current_bat_state == BATTERY_STATE_EMPTY_BATTERY || current_bat_state == BATTERY_STATE_LOW_BATTERY)
            {
                err |= led_set_for_state(led_state_shutdown);
                state_update_ticks = APP_TIMER_TICKS(SHUTDOWN_UPDATE_MS);
                next_state = PT10_STATE_SHUTDOWN;
            }

            // State Transitions to the ON_DOCK STATE
            else if (current_charge_state == BATTERY_CHARGE_CHARGING)
            {
                err |= flash_lower_power_mode(false);
                err |= afe_set_mode(pt10_status.afe_mode);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                err |= led_set_for_state(led_state_charging);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }
            else if (current_charge_state == BATTERY_CHARGE_COMPLETED)
            {
                err |= flash_lower_power_mode(false);
                err |= led_set_for_state(led_state_charge_complete);
                err |= afe_set_mode(pt10_status.afe_mode);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }

            // State Transition to the IDLE STATE
            else if (acc_get_wake_event())
            {
                err |= flash_lower_power_mode(false);
                err |= afe_set_mode(pt10_status.afe_mode);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(mag_mode_off);
                bool can_fit_epoch = false;
                err |= storage_can_fit_epoch(&can_fit_epoch);
                if(!can_fit_epoch)
                {
                    err |= led_set_for_state(led_state_memory_full);
                }
                else
                {
                    if (pt10_status.ble_connected)
                    {
                        err |= led_set_for_state(led_state_connected_unattached);
                    }
                    else
                    {
                        err |= led_set_for_state(led_state_unconnected_unattached);
                    }
                }   
                state_update_ticks = APP_TIMER_TICKS(IDLE_UPDATE_MS);
                next_state = PT10_STATE_IDLE;
            }

            // State Transition to the SLEEP STATE
            else if (_rtc_wake_event)
            {
                // Do nothing.
                // Remain in PT10_STATE_SLEEP and go back to sleep
                // on the next update.
            }
            
        }

        break;

    case PT10_STATE_DEEP_SLEEP_PREP:

        if (_state_timer_elapsed)
        {
            _state_timer_elapsed = false;

            update_power();
            err |= update_connection_quality(true);

            pt10_battery_state_e current_bat_state = get_battery_state();
            pt10_charge_state_e current_charge_state = get_charge_state(); 

            if (afe_get_lead_detect_amplitude_lpf() < AFE_GOOD_CONNECTION_AMPLITUDE)
            {
                good_connection_counter = GOOD_CONNECTION_UPDATES;
            }
            else if(good_connection_counter > 0)
            {
                good_connection_counter -= 1;
            }
            NRF_LOG_INFO("Amplitude %d, count = %d", afe_get_lead_detect_amplitude_lpf(), good_connection_counter);

            // State Transition to the SHUTDOWN STATE
            if (current_bat_state == BATTERY_STATE_EMPTY_BATTERY || current_bat_state == BATTERY_STATE_LOW_BATTERY)
            {
                err |= led_set_for_state(led_state_shutdown);
                err |= afe_set_mode(afe_mode_off);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(acc_mode_off);
                state_update_ticks = APP_TIMER_TICKS(SHUTDOWN_UPDATE_MS);
                next_state = PT10_STATE_SHUTDOWN;
            }

            // State Transitions to the DEEP_SLEEP_STATE
            else if(good_connection_counter == 0)
            {
                // The impedance between the electrodes is high
                led_set_for_state(led_state_sleep);
                next_state = PT10_STATE_DEEP_SLEEP;
            }

            // State Transitions to the ON_DOCK state
            else if (current_charge_state == BATTERY_CHARGE_CHARGING)
            {
                err |= led_set_for_state(led_state_charging);
                err |= afe_set_mode(pt10_status.afe_mode);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(acc_mode_off);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }
            else if (current_charge_state == BATTERY_CHARGE_COMPLETED)
            {
                err |= led_set_for_state(led_state_charge_complete);
                err |= afe_set_mode(pt10_status.afe_mode);
                err |= acc_set_mode(acc_mode_off);
                err |= mag_set_mode(acc_mode_off);
                state_update_ticks = APP_TIMER_TICKS(ON_DOCK_UPDATE_MS);
                last_charge_state = current_charge_state;
                // serialStart();
                next_state = PT10_STATE_ON_DOCK;
            }
        }
        break;

    case PT10_STATE_DEEP_SLEEP:

        if(!leds_running())
        {
            afe_set_mode(afe_mode_off);
            acc_set_mode(acc_mode_off);
            mag_set_mode(mag_mode_off);
            set_deep_sleep_flag(1);
            save_info_to_local_flash(store_reason_set_deep_sleep_flag);
            flash_lower_power_mode(true);
            hw_rtc_enable_periodic_countdown_timer(5);
            hal_set_board_for_shutdown();
            nrf_gpio_cfg_sense_input(PIN_RTC_INT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW); 
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            while(1)
            {
                // We don't get here. 
                // this is for emulated shutdown only.
            }
        }

        break;

    case PT10_STATE_ERROR:
        break;

    default:
        break;
    }

    if (next_state != pt10_status.device_state)
    {
        NRF_LOG_INFO("State %d -> %d", pt10_status.device_state, next_state);
        pt10_status.device_state = next_state;
        app_timer_stop(_state_timer);
        app_timer_start(_state_timer, state_update_ticks, NULL);
        _state_entry_ticks = app_timer_cnt_get();        
    }

    if(err)
    {
        NRF_LOG_WARNING("Error %d during main loop", err);
    }

    // PT10_LOG_FLUSH();

}

bool pt10_check_connection_impedance(void)
{
    afe_set_mode(afe_mode_passive);
    PT10_LOG_FLUSH();    

    nrf_drv_clock_lfclk_request(NULL);
    nrfx_clock_lfclk_start();

    while (!nrf_drv_clock_lfclk_is_running())
    {
        hal_wdt_feed();
        /* Just waiting */
    }

    rtc_config(RTC_PRESCALE_FOR_32HZ_TICK, 1);

    // start RTC
    nrf_rtc_task_trigger(NRF_RTC2, NRF_RTC_TASK_START);
    _rtc_wake_event = false;

    nrf_drv_clock_hfclk_release();
    while (!_rtc_wake_event && !acc_get_wake_event() && !get_usb_power_state())
    {
        __SEV();
        __WFE();
        __WFI();
        // sd_app_evt_wait();
    }

    update_lead_detect_status();
    uint8_t flags = afe_get_lead_detect_flags();
    uint32_t ac_amplitude = afe_get_offset_corrected_lead_detect_amplitude();
    NRF_LOG_INFO("Flags: %d, Amplitude: %d", flags, ac_amplitude);
    PT10_LOG_FLUSH();
    afe_set_mode(afe_mode_off);

    if(ac_amplitude > AFE_GOOD_CONNECTION_AMPLITUDE)
    {
        nrf_gpio_cfg_sense_input(PIN_RTC_INT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW); 
        hal_set_board_for_shutdown();
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }

    return true;
}

int32_t enter_bootloader_for_dfu(void)
{
    NRF_LOG_INFO("In ble_dfu_buttonless_bootloader_start_finalize");

#if ENABLE_BLE == 1

    uint32_t err_code;

    err_code = sd_power_gpregret_clr(0, 0xffffffff);
    VERIFY_SUCCESS(err_code);
    NRF_LOG_INFO("dfu init point 1");

    err_code = sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    VERIFY_SUCCESS(err_code);
    NRF_LOG_INFO("dfu init point 2");
#else
    nrf_power_gpregret_ext_set(0, BOOTLOADER_DFU_START);
    NRF_LOG_INFO("dfu init point 1");
#endif

    // Indicate that the Secure DFU bootloader will be entered
    // m_dfu.evt_handler(BLE_DFU_EVT_BOOTLOADER_ENTER);

    // Signal that DFU mode is to be enter to the power management module
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
    NRF_LOG_WARNING("shouldn't be reached");

    return NRF_SUCCESS;
}

int32_t sw_reset(void)
{

    NVIC_SystemReset();

    return NRF_SUCCESS;
}

int32_t pt10_set_device_id(int32_t new_device_id)
{
    int32_t err = 0;

    err = set_device_id(new_device_id);
    if(!err)
    {
        err = save_info_to_local_flash(store_reason_set_device_id);
    }

    if (err != 0)
    {
        NRF_LOG_WARNING("failed to save new id");
    }
    else
    {
        hal_wdt_feed();
        hal_delay_ms(200);
        hal_wdt_feed();
        hal_delay_ms(200);
        sw_reset();
    }

    return err;

}

int32_t pt10_cmd_processor(uint8_t *command, uint8_t *response)
{
    int32_t err = 0;

    memset(response, 0, 5);

    response[0] = command[0];

    switch(command[0])
    {

    case PT10_CMD_SEND_NEXT:
        pt10_send_next_epoch();
        break;

    case PT10_CMD_SEND_PREVIOUS:
        pt10_resend_epoch();
        break;

    case PT10_CMD_SEND_COMPLETE:
        pt10_send_complete();
        break;

    case PT10_CMD_SET_ID:
    {
        uint32_t new_device_id = 0;
        new_device_id = (
            ((uint32_t)command[1] << 24) | 
            ((uint32_t)command[2] << 16) | 
            ((uint32_t)command[3] << 8) | 
            ((uint32_t)command[4])
        );
        pt10_set_device_id(new_device_id);
    }
    break;

    case PT10_CMD_GET_ID:
        uint32_t current_device_id = 0;
        err = get_device_id(&current_device_id);

        if (err != 0)
        {
            current_device_id = 0;
        }
        response[1] = (current_device_id >> 24) & 0xff;
        response[2] = (current_device_id >> 16) & 0xff;
        response[3] = (current_device_id >> 8) & 0xff;
        response[4] = (current_device_id) & 0xff;
        break;

    case PT10_CMD_SET_TIME:
    {
        // uint32_t epoch = (command[4]<<24) |(command[3]<<16) |(command[2]<<8) |(command[1])  ;

        uint32_t epoch = (command[1] << 24) | (command[2] << 16) | (command[3] << 8) | (command[4]);
        err = hw_rtc_set_unix_time(epoch);
        NRF_LOG_INFO("time set %i", epoch);
        if(!err)
        {
            err = hw_rtc_get_unix_time(&epoch);
            response[0] = PT10_CMD_SET_TIME;
            response[1] = (epoch >> 24) & 0xff;
            response[2] = (epoch >> 16) & 0xff;
            response[3] = (epoch >> 8) & 0xff;
            response[4] = (epoch) & 0xff;
        }
        epoch = 0;
    }
    break;

    case PT10_CMD_GET_TIME:
    {
        uint32_t epoch = 0;

        err = hw_rtc_get_unix_time(&epoch);
        if(!err)
        {
            response[1] = (epoch >> 24) & 0xff;
            response[2] = (epoch >> 16) & 0xff;
            response[3] = (epoch >> 8) & 0xff;
            response[4] = (epoch) & 0xff;
        }

        epoch = 0;
    }
    break;

    case PT10_CMD_FILES_REMAIN:
    {

        uint32_t files_in_fs = 0;

        err = storage_num_epoch_files(&files_in_fs);
        if (err)
        {
            files_in_fs = 0xFFFFFFFF;
        }
        response[1] = (files_in_fs >> 24) & 0xff;
        response[2] = (files_in_fs >> 16) & 0xff;
        response[3] = (files_in_fs >> 8) & 0xff;
        response[4] = (files_in_fs) & 0xff;
    }
    break;

    case PT10_CMD_BLOCKS_USED:
    {
        uint32_t blocks_in_use = 0;

        err = storage_blocks_used(&blocks_in_use);
        if (err)
        {
            blocks_in_use = 0xFFFFFFFF;
        }
        response[1] = (blocks_in_use >> 24) & 0xff;
        response[2] = (blocks_in_use >> 16) & 0xff;
        response[3] = (blocks_in_use >> 8) & 0xff;
        response[4] = (blocks_in_use) & 0xff;
    }
    break;

    case PT10_CMD_CLEAN_FS:
    {
        err = storage_clear_fs();

        if (err == 0)
        {
            response[4] = 0x01;
        }
    }
    break;

    case PT10_CMD_GET_LEAD_DETECT_STATUS:
    {
        uint8_t flags = afe_get_lead_detect_flags();
        uint32_t ac_amplitude = afe_get_offset_corrected_lead_detect_amplitude();
        // NRF_LOG_INFO("Flags: %d AC Amplitude: %d", flags, ac_amplitude);

        response[1] = flags;
        response[2] = (uint8_t)((ac_amplitude >> 16) & 0xff);
        response[3] = (uint8_t)((ac_amplitude >> 8) & 0xff);
        response[4] = (uint8_t)((ac_amplitude >> 0) & 0xff);
    }
    break;

    case PT10_CMD_SELECT_SENSOR:
    {
        uint8_t sensor = command[1];
        pt10_set_streaming_sensor(sensor);            
    }
        break;

    case PT10_CMD_SET_AFE_MODE:
    {
        uint8_t mode = command[1];
        if(mode == afe_mode_passive || mode == afe_mode_active)
        {
            pt10_set_afe_mode(mode);
        }
    }
        break;

    case PT10_CMD_SET_INA_GAINS:
    {
        ina_gain_t eeg1_ina_gain = command[3];
        bioz_ina_gain_t eeg2_ina_gain = command[4];
        pt10_set_ina_gains(eeg1_ina_gain, eeg2_ina_gain);
    }
        break;

    case PT10_CMD_SYNTHESIZE_EPOCHS:
        uint32_t num_epochs = (command[1] << 24) | (command[2] << 16) | (command[3] << 8) | (command[4]);
        pt10_synthesize_epochs(num_epochs);
        break;

    case PT10_CMD_GET_STATE:
        response[4] = pt10_status.device_state;
        response[3] = epoch_transmission_get_state();
        response[2] = epoch_storage_get_state();
        response[1] = epoch_recording_get_state();
        break;

    case PT10_CMD_GET_VERTICAL_SCALING:
        float vertical_scaling;
        switch(command[4])
        {
            case eeg_data_section_id:
                vertical_scaling = afe_get_eeg_vertical_scaling();
                break;
            case acc_x_section_id:
            case acc_y_section_id:
            case acc_z_section_id:
                vertical_scaling = acc_get_vertical_scaling();
                break;
            case imp_section_id:
                vertical_scaling = afe_get_impedance_vertical_scaling();
                break;                
        }
        memcpy(response+1, (uint8_t *)&vertical_scaling, 4);
        break;

    case PT10_CMD_GET_BATTERY:
        uint8_t battery_percent = get_battery_percent();
        response[4] = battery_percent;
        break;

    case PT10_CMD_GET_RESETREASON:
        response[1] = ( pt10_status.reset_reason >> 24 ) & 0xff;
        response[2] = ( pt10_status.reset_reason >> 16 ) & 0xff;
        response[3] = ( pt10_status.reset_reason >> 8 ) & 0xff;
        response[4] = ( pt10_status.reset_reason >> 0 ) & 0xff;
        break;

    case PT10_CMD_GET_BOOT_TIME:
        response[1] = ( pt10_status.boot_time >> 24 ) & 0xff;
        response[2] = ( pt10_status.boot_time >> 16 ) & 0xff;
        response[3] = ( pt10_status.boot_time >> 8 ) & 0xff;
        response[4] = ( pt10_status.boot_time >> 0 ) & 0xff;
        break;

    case PT10_CMD_GET_FIRMWARE_VERSION:
        response[1] = (uint8_t)((FIRMWARE_VERSION >> 24) & 0xff);
        response[2] = (uint8_t)((FIRMWARE_VERSION >> 16) & 0xff);
        response[3] = (uint8_t)((FIRMWARE_VERSION >> 8) & 0xff);
        response[4] = (uint8_t)((FIRMWARE_VERSION >> 0) & 0xff);
        break;

    case PT10_CMD_GET_HARDWARE_VERSION:
        response[1] = (uint8_t)((HARDWARE_VERSION >> 24) & 0xff);
        response[2] = (uint8_t)((HARDWARE_VERSION >> 16) & 0xff);
        response[3] = (uint8_t)((HARDWARE_VERSION >> 8) & 0xff);
        response[4] = (uint8_t)((HARDWARE_VERSION >> 0) & 0xff);
        break;

    case PT10_CMD_GET_POST_RESULT:
        response[1] = (uint8_t)((pt10_status.post_result >> 24) & 0xff);
        response[2] = (uint8_t)((pt10_status.post_result >> 16) & 0xff);
        response[3] = (uint8_t)((pt10_status.post_result >> 8) & 0xff);
        response[4] = (uint8_t)((pt10_status.post_result >> 0) & 0xff);
        break;

    case PT10_CMD_GET_PANIC_CODE:
        response[1] = (uint8_t)((_panic_code >> 24) & 0xff);
        response[2] = (uint8_t)((_panic_code >> 16) & 0xff);
        response[3] = (uint8_t)((_panic_code >> 8) & 0xff);
        response[4] = (uint8_t)((_panic_code >> 0) & 0xff);
        break;

    case PT10_CMD_GET_STORE_REASON:
    {
        uint32_t store_reason = get_store_reason();
        response[1] = (uint8_t)((store_reason >> 24) & 0xff);
        response[2] = (uint8_t)((store_reason >> 16) & 0xff);
        response[3] = (uint8_t)((store_reason >> 8) & 0xff);
        response[4] = (uint8_t)((store_reason >> 0) & 0xff);
    }
        break;

    case PT10_CMD_GET_OSCILLTRACK_FREQUENCY:
    {
        int32_t frequency;
        get_oscilltrack_frequency(&frequency);
        response[1] = (uint8_t)((frequency >> 24) & 0xff);
        response[2] = (uint8_t)((frequency >> 16) & 0xff);
        response[3] = (uint8_t)((frequency >> 8) & 0xff);
        response[4] = (uint8_t)((frequency >> 0) & 0xff);
    }
        break;

    case PT10_CMD_SET_OSCILLTRACK_FREQUENCY:
    {
        int32_t frequency;
        frequency = (
            ((uint32_t)command[1] << 24) | 
            ((uint32_t)command[2] << 16) | 
            ((uint32_t)command[3] << 8) | 
            ((uint32_t)command[4])
        );
        set_oscilltrack_frequency(frequency); 
        oscilltrack_set_frequency(frequency);
    }
        break;

    case PT10_CMD_GET_OSCILLTRACK_TRIGGER_PHASE:
    {
        int32_t trigger_phase;
        get_oscilltrack_trigger_phase(&trigger_phase);
        response[1] = (uint8_t)((trigger_phase >> 24) & 0xff);
        response[2] = (uint8_t)((trigger_phase >> 16) & 0xff);
        response[3] = (uint8_t)((trigger_phase >> 8) & 0xff);
        response[4] = (uint8_t)((trigger_phase >> 0) & 0xff);
    }
        break;

    case PT10_CMD_SET_OSCILLTRACK_TRIGGER_PHASE:
    {
        int32_t trigger_phase;
        trigger_phase = (
            ((uint32_t)command[1] << 24) | 
            ((uint32_t)command[2] << 16) | 
            ((uint32_t)command[3] << 8) | 
            ((uint32_t)command[4])
        );
        set_oscilltrack_trigger_phase(trigger_phase); 
        oscilltrack_set_trigger_phase(trigger_phase);
    }
        break;

    case PT10_CMD_GET_OSCILLTRACK_SUPPRESSION_DUTY_CYCLE:
    {
        int32_t suppression_duty_cycle;
        get_oscilltrack_suppression_duty_cycle(&suppression_duty_cycle);
        response[1] = (uint8_t)((suppression_duty_cycle >> 24) & 0xff);
        response[2] = (uint8_t)((suppression_duty_cycle >> 16) & 0xff);
        response[3] = (uint8_t)((suppression_duty_cycle >> 8) & 0xff);
        response[4] = (uint8_t)((suppression_duty_cycle >> 0) & 0xff);
    }
        break;

    case PT10_CMD_SET_OSCILLTRACK_SUPPRESSION_DUTY_CYCLE:
    {
        int32_t suppression_duty_cycle;
        suppression_duty_cycle = (
            ((uint32_t)command[1] << 24) | 
            ((uint32_t)command[2] << 16) | 
            ((uint32_t)command[3] << 8) | 
            ((uint32_t)command[4])
        );
        set_oscilltrack_suppression_duty_cycle(suppression_duty_cycle); 
        oscilltrack_set_suppression_duty_cycle(suppression_duty_cycle);
    }
        break;

    case PT10_CMD_GET_CONVERGENCE_GAIN:
    {
        int32_t convergence_gain;
        get_oscilltrack_convergence_gain(&convergence_gain);
        response[1] = (uint8_t)((convergence_gain >> 24) & 0xff);
        response[2] = (uint8_t)((convergence_gain >> 16) & 0xff);
        response[3] = (uint8_t)((convergence_gain >> 8) & 0xff);
        response[4] = (uint8_t)((convergence_gain >> 0) & 0xff);
    }
        break;

    case PT10_CMD_SET_CONVERGENCE_GAIN:
    {
        int32_t convergence_gain;
        convergence_gain = (
            ((uint32_t)command[1] << 24) | 
            ((uint32_t)command[2] << 16) | 
            ((uint32_t)command[3] << 8) | 
            ((uint32_t)command[4])
        );
        set_oscilltrack_convergence_gain(convergence_gain); 
        oscilltrack_set_convergence_gain(convergence_gain);
    }
        break;

    case PT10_CMD_STORE_OSCILLTRACK_PARAMETERS:
        save_info_to_local_flash(store_reason_set_oscilltrack_parameters);
        break;

    case PT10_CMD_GET_OSCILLTRACK_FLAGS:
    {
        uint32_t flags;
        get_oscilltrack_flags(&flags);
        response[1] = (uint8_t)((flags >> 24) & 0xff);
        response[2] = (uint8_t)((flags >> 16) & 0xff);
        response[3] = (uint8_t)((flags >> 8) & 0xff);
        response[4] = (uint8_t)((flags >> 0) & 0xff);
    }
        break;

    case PT10_CMD_SET_OSCILLTRACK_FLAGS:
    {
        int32_t flags;
        flags = (
            ((uint32_t)command[1] << 24) | 
            ((uint32_t)command[2] << 16) | 
            ((uint32_t)command[3] << 8) | 
            ((uint32_t)command[4])
        );
        set_oscilltrack_flags(flags); 
        oscilltrack_set_flags(flags);
    }
        break;

    case PT10_CMD_CONFIGURE_VTS_HAPTIC:
        esb_configure_vts_haptic(command+1);
        break;

    case PT10_CMD_CONFIGURE_VTS_SLEEP:
        esb_configure_vts_sleep(command+1);
        break;

    case PT10_CMD_ENTER_DEEP_SLEEP_MODE:
        pt10_enter_deep_sleep_mode();
        break;

    case PT10_CMD_ENTER_SHIP_MODE:
        hal_delay_ms(10);
        hal_wdt_feed();
        hal_gpio_set(PIN_EN_SHIP_MODE);
        break;

    case PT10_CMD_ENTER_DFU:
        update_power();

        if (get_battery_state() == (BATTERY_STATE_LOW_BATTERY || BATTERY_STATE_EMPTY_BATTERY))
        {
            NRF_LOG_INFO("bat too low for dfu");
            // for()
            return PT10_ERROR_BAT_TOO_LOW;
        }
        hal_wdt_feed();
        hal_delay_ms(200);
        hal_wdt_feed();
        hal_delay_ms(200);
        hal_wdt_feed();
        hal_delay_ms(200);
        hal_wdt_feed();
        enter_bootloader_for_dfu();

        // hal_gpio_set(PIN_CHARGE_EN_SHIP_MODE);
        break;

    default:
        break;
    }

    return err;

}

void pt10_set_uart_connected(bool connected)
{
    NRF_LOG_INFO("Uart connection: %d", connected);
    pt10_status.uart_connected = connected;
}

bool pt10_get_uart_connected(void)
{
    return pt10_status.uart_connected;
}