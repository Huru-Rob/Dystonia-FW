#define NRF_LOG_MODULE_NAME int_flash 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdint.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_queue.h"
#include "app_error.h"
#include "nrf_fstorage_nvmc.h"
#include "internal_flash.h"
#include "hal.h"
#include "oscilltrack.h"
#include "neurobuzz.h"

NRF_LOG_MODULE_REGISTER();

///////////////////////////////// F internal storage

nv_params_t nv_params = {
    .device_id = 0,
    .deep_sleep_flag = 0,
    .frequency = 35,
    .suppression_duty_cycle = 50,
    .trigger_phase = 0,
    .convergence_gain = 125
};

static void fstorage_evt_handler(nrf_fstorage_evt_t *p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
    {
        /* Set a handler for fstorage events. */
        .evt_handler = fstorage_evt_handler,

        /* These below are the boundaries of the flash space assigned to this instance of fstorage.
         * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
         * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
         * last page of flash available to write data. */
        .start_addr = 0xA0000,
        .end_addr = 0xA1000,
};

/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
// static uint32_t nrf5_flash_end_addr_get()
// {
//     uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
//     uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
//     uint32_t const code_sz         = NRF_FICR->CODESIZE;

//     return (bootloader_addr != 0xFFFFFFFF ?
//             bootloader_addr : (code_sz * page_sz));
// }

static void fstorage_evt_handler(nrf_fstorage_evt_t *p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
    case NRF_FSTORAGE_EVT_WRITE_RESULT:
    {
        NRF_LOG_DEBUG("--> Event received: wrote %d bytes at address 0x%x.",
          p_evt->len, p_evt->addr);
    }
    break;

    case NRF_FSTORAGE_EVT_ERASE_RESULT:
    {
        NRF_LOG_DEBUG("--> Event received: erased %d page from address 0x%x.",
          p_evt->len, p_evt->addr);
    }
    break;

    default:
        break;
    }
}

void wait_for_flash_ready(nrf_fstorage_t const *p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        hal_delay_ms(1); // not best practice

        // idle_state_handle();
        // power_manage();
    }
}

int32_t init_internal_flash(void)
{
    int32_t err = 0;

    nrf_fstorage_api_t *p_fs_api;
    p_fs_api = &nrf_fstorage_nvmc;
    err = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(err);
    return err;
}

int32_t set_deep_sleep_flag(uint32_t deep_sleep_flag)
{
    int32_t err = 0;
    nv_params.deep_sleep_flag = deep_sleep_flag;
    return err;
}

int32_t get_deep_sleep_flag(uint32_t *deep_sleep_flag)
{
    int32_t err = 0;
    *deep_sleep_flag = nv_params.deep_sleep_flag;
    return err;
}

int32_t set_oscilltrack_frequency(int32_t frequency)
{
    int32_t err = 0;
    nv_params.frequency = frequency;
    return err;
}
int32_t get_oscilltrack_frequency(int32_t *frequency)
{
    int32_t err = 0;
    *frequency = nv_params.frequency;
    return err;
}
int32_t set_oscilltrack_trigger_phase(int32_t trigger_phase)
{   
    int32_t err = 0;
    nv_params.trigger_phase = trigger_phase;
    return err;
}
int32_t get_oscilltrack_trigger_phase(int32_t *trigger_phase)
{
    int32_t err = 0;
    *trigger_phase = nv_params.trigger_phase;
    return err;
}
int32_t set_oscilltrack_suppression_duty_cycle(int32_t suppression_duty_cycle)
{
    int32_t err = 0;
    nv_params.suppression_duty_cycle = suppression_duty_cycle;
    return err;
}
int32_t get_oscilltrack_convergence_gain(int32_t *convergence_gain)
{
    int32_t err = 0;
    *convergence_gain = nv_params.convergence_gain;
    return err;
}
int32_t set_oscilltrack_convergence_gain(int32_t convergence_gain)
{
    int32_t err = 0;
    nv_params.convergence_gain = convergence_gain;
    return err;
}
int32_t get_oscilltrack_suppression_duty_cycle(int32_t *suppression_duty_cycle)
{
    int32_t err = 0;
    *suppression_duty_cycle = nv_params.suppression_duty_cycle;
    return err;
}
int32_t set_oscilltrack_flags(uint32_t flags)
{
    int32_t err = 0;
    nv_params.flags = flags;
    return err;
}
int32_t get_oscilltrack_flags(uint32_t *flags)
{
    int32_t err = 0;
    *flags = nv_params.flags;
    return err;
}

int32_t set_neurobuzz_phases(int16_t *phases, uint8_t num_phases)
{
    int32_t err = 0;
    for(uint8_t i = 0; i < num_phases; i++)
    {
        nv_params.phases[i] = phases[i];
    }
    nv_params.num_phases = num_phases;
    return err;
}
int32_t get_neurobuzz_phases(int16_t *phases, uint8_t *num_phases)
{
    int32_t err = 0;
    for(uint8_t i = 0; i < nv_params.num_phases; i++)
    {
        phases[i] = nv_params.phases[i];
    }
    *num_phases = nv_params.num_phases;
    return err;
}
int32_t get_neurobuzz_long_dur(uint32_t *long_dur){
    int32_t err = 0;
    *long_dur = nv_params.long_dur;
    return err;
}
int32_t set_neurobuzz_long_dur(uint32_t long_dur){
    int32_t err = 0;
    nv_params.long_dur = long_dur;
    return err;
}
int32_t get_neurobuzz_short_dur(uint32_t *short_dur){
    int32_t err = 0;
    *short_dur = nv_params.short_dur;
    return err;
}
int32_t set_neurobuzz_short_dur(uint32_t short_dur){
    int32_t err = 0;
    nv_params.short_dur = short_dur;
    return err;
}
int32_t get_neurobuzz_switch_stim_gap(uint32_t *switch_stim_gap){
    int32_t err = 0;
    *switch_stim_gap = nv_params.switch_stim_gap;    
    return err;
}
int32_t set_neurobuzz_switch_stim_gap(uint32_t switch_stim_gap){
    int32_t err = 0;
    nv_params.switch_stim_gap = switch_stim_gap;
    return err;
}
int32_t get_neurobuzz_step_near_preferred_phase(int16_t *step){
    int32_t err = 0;
    *step = nv_params.step_near_preferred_phase;
    return err;
}
int32_t set_neurobuzz_step_near_preferred_phase(int16_t step){
    int32_t err = 0;
    nv_params.step_near_preferred_phase = step;
    return err;
}
int32_t set_neurobuzz_first_default_phase(int16_t phase)
{
    int32_t err = 0;
    nv_params.phases[0] = phase;
    nv_params.num_phases = 1;
    return err;
}
int32_t set_neurobuzz_next_default_phase(int16_t phase)
{
    int32_t err = 0;
    if(nv_params.num_phases < NEUROBUZZ_MAX_PHASES)
    {
        nv_params.phases[nv_params.num_phases] = phase;
        nv_params.num_phases += 1;
    }
    return err;
}
int32_t set_neurobuzz_disable_refinement(bool disabled)
{
    int32_t err = 0;
    nv_params.disable_refinement = disabled ? 1:0;
    return err;
}
int32_t get_neurobuzz_disable_refinement(uint8_t *disabled)
{
    int32_t err = 0;
    *disabled = nv_params.disable_refinement;
    return err;
}

void init_nv_params(void)
{
    nv_params.device_id = 255;
    nv_params.bluetooth_mode = 1;
    nv_params.deep_sleep_flag = 0;
    nv_params.bluetooth_mode = 1;
    nv_params.frequency = 35;
    nv_params.suppression_duty_cycle = 80;
    nv_params.trigger_phase = 0;
    nv_params.convergence_gain = 125;
    nv_params.flags = (1 << OSCILLTRACK_STIMULUS_ON_BITPOS);
    nv_params.long_dur = NEUROBUZZ_LONG_DUR_MS;
    nv_params.short_dur = NEUROBUZZ_SHORT_DUR_MS;
    nv_params.switch_stim_gap = NEUROBUZZ_SWITCH_STIM_GAP_MS;
    nv_params.step_near_preferred_phase = NEUROBUZZ_STEP_NEAR_PREFERRED_PHASE;
    int16_t default_phases[NEUROBUZZ_MAX_PHASES] = {-90, 90, -150, -30, 150, 30};
    for(uint8_t i = 0; i < NEUROBUZZ_MAX_PHASES; i++)
    {
        nv_params.phases[i] = default_phases[i];
    } 
    nv_params.num_phases = 6;

}

int32_t save_info_to_local_flash(uint32_t reason)
{
    int32_t err = 0;

    wait_for_flash_ready(&fstorage);

    err = nrf_fstorage_erase(&fstorage, 0xA0000, 1, NULL);
    APP_ERROR_CHECK(err);
    wait_for_flash_ready(&fstorage);

    nv_params.store_reason = reason;

    err = nrf_fstorage_write(&fstorage, 0xA0000, &nv_params, sizeof(nv_params), NULL);
    APP_ERROR_CHECK(err);
    wait_for_flash_ready(&fstorage);

    return err;
}

int32_t read_stored_data(void)
{
    int32_t err = 0;

    wait_for_flash_ready(&fstorage);

    err = nrf_fstorage_read(&fstorage, 0xA0000, &nv_params, sizeof(nv_params));
    NRF_LOG_INFO("Device ID: %d", nv_params.device_id);
    NRF_LOG_INFO("Store Reason: %d", nv_params.store_reason);
    NRF_LOG_INFO("Frequency: %d", nv_params.frequency);
    NRF_LOG_INFO("Trigger Phase: %d", nv_params.trigger_phase);
    NRF_LOG_INFO("Suppression Duty Cycle: %d", nv_params.suppression_duty_cycle);
    NRF_LOG_INFO("convergence Gain: %d", nv_params.convergence_gain);
    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(err);

    if(nv_params.store_reason == 0xFFFFFFFF)
    {
        NRF_LOG_INFO("Blank non volatile parameters: Initialising")
        init_nv_params();
        save_info_to_local_flash(store_reason_blank_all);
    }

    return err;
}

uint32_t get_store_reason(void)
{
    return nv_params.store_reason;
}