#define NRF_LOG_MODULE_NAME afe 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdio.h>
#include <stdlib.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "afe.h"
#include "AFE4960.h"
#include "hal.h"
#include "epoch.h"

NRF_LOG_MODULE_REGISTER();

const afe_init_t afe_common_config[] = {
    {CONTROL0_ADDR, AFE_COMMON_CONTROL0},
    {CONTROL2_ADDR, AFE_COMMON_CONTROL2},
    {ACT_CTRL_ADDR, AFE_COMMON_ACT_CTRL},
    {CTRL_PDN_ADDR, AFE_COMMON_CTRL_PDN},
    //{RAC_CONTROL_ADDR, AFE_COMMON_RAC_CONTROL},
    {CONTROL4_ADDR, AFE_COMMON_CONTROL4},
    {CONTROL7_ADDR, AFE_COMMON_CONTROL7},
    {CONTROL8_ADDR, AFE_COMMON_CONTROL8},
    {CONTROL9_ADDR, AFE_COMMON_CONTROL9},
    {CONTROL10_ADDR, AFE_COMMON_CONTROL10},
    {CONTROL13_ADDR, AFE_COMMON_CONTROL13},
    {CONTROL19_ADDR, AFE_COMMON_CONTROL19},
    {CONTROL20_ADDR, AFE_COMMON_CONTROL20},
    {AC_DEMOD_ADDR, AFE_COMMON_CONFIG_AC_DEMOD},
    {MASK_INT_REG_ADDR, AFE_COMMON_MASK_INT_REG},
    {CONTROL25_ADDR, AFE_COMMON_CONTROL25},
    {CONFIG_CLK_CKT_MIX1_ADDR, AFE_COMMON_CONFIG_CLK_CKT_MIX1},
    {CONFIG_CLK_DIV_MIX1_ADDR, AFE_COMMON_CONFIG_CLK_DIV_MIX1},
    {CONFIG_PRPCT_MIX1_ADDR, AFE_COMMON_CONFIG_PRPCT_MIX1},
    {CONFIG_TS_9TO16_MIX1_ADDR, AFE_COMMON_CONFIG_TS_9TO16_MIX1},
    {CONFIG_NUM_TS_MIX1_ADDR, AFE_COMMON_CONFIG_NUM_TS_MIX1},
    {CONFIG_ECG1_MIX1_ADDR, AFE_COMMON_CONFIG_ECG1_MIX1},
    {CONFIG_ECG2_MIX1_ADDR, AFE_COMMON_CONFIG_ECG2_MIX1},
    {CONFIG_BIA_SW_MATRIX_STATIC_MIX1_ADDR, AFE_COMMON_CONFIG_BIA_SW_MATRIX_STATIC_MIX1},
    {CONFIG_BIA_EXC_MIX1_ADDR, AFE_COMMON_CONFIG_BIA_EXC_MIX1},
    {CONFIG_BIA_TX_MIX1_ADDR, AFE_COMMON_CONFIG_BIA_TX_MIX1},
    {CONFIG_BIA_RX_MIX1_ADDR, AFE_COMMON_CONFIG_BIA_RX_MIX1},
    {CONFIG_ELECTRODE_BIAS1_MIX1_ADDR, AFE_COMMON_CONFIG_ELECTRODE_BIAS1_MIX1},
    {CONFIG_ELECTRODE_BIAS2_MIX1_ADDR, AFE_COMMON_CONFIG_ELECTRODE_BIAS2_MIX1},
    {CONFIG_COMP_ANA_MIX1_ADDR, AFE_COMMON_CONFIG_COMP_ANA_MIX1},
    {CONFIG_IACTIVE_MIX1_ADDR, AFE_COMMON_CONFIG_IACTIVE_MIX1},
    {CONFIG_COMP_DIG_MIX1_ADDR, AFE_COMMON_CONFIG_COMP_DIG_MIX1},
};

const afe_init_t afe_passive_config[] = {
    {CONFIG_MODE_SEL_ADDR, AFE_PASSIVE_CONFIG_MODE_SEL},
    {CONFIG_ECG_CH2_ADDR, AFE_PASSIVE_CONFIG_ECG_CH2},
    {CONFIG_TS_1TO8_MIX1_ADDR, AFE_PASSIVE_CONFIG_TS_1TO8_MIX1},
    {CONFIG_BIA_PROC_MIX1_ADDR, AFE_PASSIVE_CONFIG_BIA_PROC_MIX1},
};

const afe_init_t afe_active_config[] = {
    {CONFIG_MODE_SEL_ADDR, AFE_ACTIVE_CONFIG_MODE_SEL},
    {CONFIG_ECG_CH2_ADDR, AFE_ACTIVE_CONFIG_ECG_CH2},
    {CONFIG_TS_1TO8_MIX1_ADDR, AFE_ACTIVE_CONFIG_TS_1TO8_MIX1},
    {CONFIG_BIA_PROC_MIX1_ADDR, AFE_ACTIVE_CONFIG_BIA_PROC_MIX1}};

afe_mode_e _current_afe_mode = afe_mode_uninitialised;
ina_gain_t _eeg1_ina_gain = EEG1_INA_GAIN;
bioz_ina_gain_t _eeg2_ina_gain = EEG2_INA_GAIN;

uint8_t _watermark_lvl = 0;
int32_t afe_fifo_output[AFE_FIFO_SIZE];
// uint8_t _last_fifo_level = 0;

uint8_t _afe_fifo_rdy_int = 0;

afe_lead_detect_t afe_lead_detector = {
    .ac_amplitude = 0,
    .flags = 0,
    .ac_lead_detect_en = false,
    .dc_lead_detect_en = false};

void afe_hw_reset(bool reset)
{
    if(reset)
    {
        hal_gpio_clear(PIN_EEG_nRESET); 
    }
    else
    {
        hal_gpio_set(PIN_EEG_nRESET); 
    }
}

int32_t afe_init(void)
{
    int32_t err = 0;

    if (!err)
    {
        // Turn everything on
        hal_i2c2_init();
        afe_hw_reset(true);
        hal_gpio_set(PIN_1V8_EN);
        hal_delay_ms(2);
        afe_hw_reset(false);
        err = afe_sw_reset();
    }

    if (!err)
    {
        // Check that we have the right device
        uint32_t read_designid_val;
        uint32_t design_id;
        err = AFE4960_read_reg(READ_DESIGNID_ADDR, &read_designid_val);
        if (!err)
        {
            design_id = (read_designid_val & DESIGN_ID_MASK) >> DESIGN_ID_BITPOS;
            if (design_id != AFE4960_DESIGN_ID)
            {
                err = AFE_ERROR_INCORRECT_ID;
            }
        }
    }

    if (!err)
    {
        // setup the pin change int on the adc rdy pin
        if (!nrfx_gpiote_is_init())
        {
            err = nrfx_gpiote_init();
            APP_ERROR_CHECK(err);
        }
    }

    if (!err)
    {
        nrfx_gpiote_in_config_t in_config_1 = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);

        err = nrfx_gpiote_in_init(PIN_EEG_INT, &in_config_1, afe_in_pin_handler);
        APP_ERROR_CHECK(err);
    }

    _current_afe_mode = afe_mode_uninitialised;
    afe_set_mode(afe_mode_off);

    return err;
}

int32_t afe_write_configuration(const afe_init_t *config, int32_t length)
{
    int32_t err = 0;

    for (uint8_t i = 0; i < length; i++)
    {
        err = AFE4960_write_reg(config[i].addr, config[i].value);
        if (err != 0)
        {
            NRF_LOG_WARNING("Init err");
            return config[i].addr;
        }
    }

    return err;
}

void afe_set_power(bool afe_enabled, bool lna_enabled)
{

    if(afe_enabled)
    {
        hal_gpio_set(PIN_1V8_EN);
    }
    else
    {
        hal_gpio_clear(PIN_1V8_EN);
    }

    if(lna_enabled)
    {
        hal_gpio_set(PIN_LNA_3V0);
    }
    else
    {
        hal_gpio_clear(PIN_LNA_3V0);
    }

}

int32_t afe_set_mode(afe_mode_e mode)
{
    int32_t err = 0;

    if (mode != _current_afe_mode)
    {
        switch(mode)
        {

        case afe_mode_off:
            // Turn everything off
            afe_pause();
            afe_sw_powerdown();

            nrfx_gpiote_in_event_disable(PIN_EEG_INT);

            afe_hw_reset(true);   
            hal_i2c2_uninit();

            afe_set_power(false, false);
            break;

        case afe_mode_passive:
        case afe_mode_active:
            // The new mode is an active mode
            if (_current_afe_mode == afe_mode_passive || _current_afe_mode == afe_mode_active)
            {
                afe_pause();
            }
            else
            {
                // Turn the AFE on
                afe_hw_reset(true);
                afe_set_power(true, false);
                hal_delay_ms(2);
                hal_i2c2_init();
                // hal_delay_ms(2);                
                afe_hw_reset(false);

                // Check that we have the right device
                uint32_t read_designid_val;
                uint32_t design_id;
                err = AFE4960_read_reg(READ_DESIGNID_ADDR, &read_designid_val);
                if (!err)
                {
                    design_id = (read_designid_val & DESIGN_ID_MASK) >> DESIGN_ID_BITPOS;
                    if (design_id != AFE4960_DESIGN_ID)
                    {
                        err = AFE_ERROR_INCORRECT_ID;
                    }
                }

                // TODO: This software reset should not be necessary
                // but without it, the AFE doesn't run reliably
                if(!err)
                {
                    err = afe_sw_reset();
                }

                if(!err)
                {
                    err = afe_write_configuration(afe_common_config,
                            sizeof(afe_common_config) / sizeof(afe_common_config[0]));
                }
                
            }

            if (!err)
            {
                if (mode == afe_mode_active)
                {
                    afe_set_power(true, true);
                    err = afe_write_configuration(afe_active_config,
                                                  sizeof(afe_active_config) / sizeof(afe_active_config[0]));
                }
                else
                {
                    afe_set_power(true, false);

                    err = afe_write_configuration(afe_passive_config,
                                                  sizeof(afe_passive_config) / sizeof(afe_passive_config[0]));
                }
            }
            if (!err)
            {
                afe_set_ina_gains(_eeg1_ina_gain, _eeg2_ina_gain);
            }
            if (!err)
            {
                err = afe_resume();
            }
            if(!err)
            {
                nrfx_gpiote_in_event_enable(PIN_EEG_INT, true);
            }
            break;        

        default:
            break;
        }
        _current_afe_mode = mode;
    }

    return err;
}

int32_t afe_get_fifo_level(uint8_t *fifo_level)
{

    int32_t err = 0;
    uint32_t read_pointer_val;

    err = AFE4960_read_reg(READ_POINTER_ADDR, &read_pointer_val);
    if (err != 0)
    {
        return err;
    }
    *fifo_level = ((read_pointer_val & REG_POINTER_DIFF_MASK) >> REG_POINTER_DIFF_BITPOS) + 1;

    return err;
}

int32_t afe_pause(void)
{
    int32_t err = 0;

    afe_lead_detector.ac_lead_detect_en = false;
    afe_lead_detector.dc_lead_detect_en = false;

    // Stop timing engine
    err = AFE4960_write_reg(RAC_CONTROL_ADDR, 0);
    if (err != 0)
    {
        return err;
    }
    // hal_delay_ms(5);

    uint32_t control0_val;
    err = AFE4960_read_reg(CONTROL0_ADDR, &control0_val);
    if (err != 0)
    {
        return err;
    }
    control0_val &= ~FIFO_EN_MASK;              // disable the FIFO
    control0_val |= (1 << TM_COUNT_RST_BITPOS); // reset the timer
    err = AFE4960_write_reg(CONTROL0_ADDR, control0_val);
    if (err != 0)
    {
        return err;
    }

    _afe_fifo_rdy_int = 0;
    return err;
}

int32_t afe_resume(void)
{

    int32_t err = 0;

    const uint32_t control0_val = (1 << FIFO_EN_BITPOS) | (1 << RW_CONT_BITPOS);
    // const uint32_t control0_val = (1 << FIFO_EN_BITPOS);
    err = AFE4960_write_reg(CONTROL0_ADDR, control0_val);
    if (err != 0)
    {
        return err;
    }

    // hal_delay_ms(5);

    const uint32_t rac_control_val = (1 << TIMER_ENABLE_BITPOS) | (1 << RAC_COUNTER_ENABLE_BITPOS);
    err = AFE4960_write_reg(RAC_CONTROL_ADDR, rac_control_val);
    if (err != 0)
    {
        return err;
    }

    afe_lead_detector.ac_lead_detect_en = true;
    afe_lead_detector.dc_lead_detect_en = false;

    return err;
}

void afe_in_pin_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // Callback function when the interrupt is triggered
    if (pin == PIN_EEG_INT)
    {        
        _afe_fifo_rdy_int = 1;        
    }
}

bool afe_samples_available(void)
{
    return _afe_fifo_rdy_int;
}

uint32_t afe_fifo_fill_time_ms(void)
{
    uint32_t ms = 0;
    switch(_current_afe_mode)
    {
    case afe_mode_passive:
        ms = AFE_COMMON_WATERMARK_LEVEL * 1000 / EEG_SAMPLE_RATE;
        break;
    case afe_mode_active:
        ms = AFE_COMMON_WATERMARK_LEVEL / 2 * 1000 / EEG_SAMPLE_RATE;
        break;
    default:
        break;
    }

    return ms;

}

uint8_t afe_fifo_buf[AFE_FIFO_SIZE * 2 * 3];

int32_t afe_clear_fifo(void)
{
    int32_t err = 0;
    uint8_t fifo_level = 0;
    err = afe_get_fifo_level(&fifo_level); 
    uint16_t samples_to_read;       
    if (!err)
    {
        // Data is read out of the FIFO big-endian (MSB first)
        switch (_current_afe_mode)
        {
        case afe_mode_passive:
            samples_to_read = fifo_level;
            break;
        case afe_mode_active:
            samples_to_read = fifo_level;
            break;
        default:
            samples_to_read = 0;
            break;
        }
    } 
    if(!err)
    {
        err = hal_i2c_read(AFE_I2C_BUS, AFE_I2C_ADD, AFE_FIFO_ADDR, afe_fifo_buf, 3 * samples_to_read);
    }
    _afe_fifo_rdy_int = false;
    return err;
}


int32_t afe_samples_fetch(int32_t *buf, uint8_t n_samples, uint8_t *samples_read)
{
    int32_t err = 0;

    *samples_read = 0;
    if (_afe_fifo_rdy_int)
    {
        uint16_t samples_to_read;
                
        if(n_samples != 0)
        {
            samples_to_read = n_samples;
        }
        else
        {
            uint8_t fifo_level = 0;
            err = afe_get_fifo_level(&fifo_level);        
            if (!err)
            {
                // Data is read out of the FIFO big-endian (MSB first)
                switch (_current_afe_mode)
                {
                case afe_mode_passive:
                    samples_to_read = fifo_level;
                    break;
                case afe_mode_active:
                    samples_to_read = fifo_level;
                    break;
                default:
                    samples_to_read = 0;
                    break;
                }
            }            
        }
        if(!err)
        {
            err = hal_i2c_read(AFE_I2C_BUS, AFE_I2C_ADD, AFE_FIFO_ADDR, afe_fifo_buf, 3 * samples_to_read);
        }
        if (!err)
        {
            uint8_t fifo_samples_placed = 0;
            int32_t *dest = buf;
            for (uint8_t fifo_samples_read = 0; fifo_samples_read < samples_to_read; fifo_samples_read++)
            {
                int32_t sample =
                    (afe_fifo_buf[fifo_samples_read * 3 + 0] << 16) +
                    (afe_fifo_buf[fifo_samples_read * 3 + 1] << 8) +
                    (afe_fifo_buf[fifo_samples_read * 3 + 2]);
                // Grab the frame sync from bit 23
                bool frame_sync = (sample & 0x800000) ? true : false;

                // Mask off all but the low 22 bits.
                sample &= 0x003fffff;
                // Sign extend to 32 bits.
                if ((sample & 0x00200000))
                {
                    sample = sample | 0xFFE00000;
                }
                // In Passive mode, only ECG Channel 1 is stored in the FIFO
                // In Active mode, ECG Channel 1 and ECG Channel 2 (BIOZ) are both stored in the FIFO
                //      and BIOZ is the second sample in the RAC (frame sync identifies the first)
                if (
                    (_current_afe_mode == afe_mode_passive) ||
                    (_current_afe_mode == afe_mode_active && !frame_sync))
                {
                    *dest++ = sample;
                    fifo_samples_placed++;
                }
            }
            *samples_read = fifo_samples_placed;
            NRF_LOG_DEBUG("Placed %d of %d", fifo_samples_placed, samples_to_read);
        }
        _afe_fifo_rdy_int = false;
    }
    

    return err;
}

int32_t update_lead_detect_status(void)
{
    int32_t err = 0;
    uint32_t intr_soft_values;

    if (afe_lead_detector.ac_lead_detect_en == true)
    {
        err = AFE4960_read_reg(READ_AC_LEAD_ADDR, &afe_lead_detector.ac_amplitude);
        if (!err)
        {
            afe_lead_detector.flags |= (1 << AC_LEAD_DETECT_VALID_BITPOS);
        }
    }
    else
    {
        afe_lead_detector.ac_amplitude = 0xffffffff;
        afe_lead_detector.flags &= ~(1 << AC_LEAD_DETECT_VALID_BITPOS);
    }

    if (afe_lead_detector.dc_lead_detect_en == true)
    {
        err = AFE4960_read_reg(INTR_SOFT_ADDR, &intr_soft_values);
        if (!err)
        {
            afe_lead_detector.flags |= (1 << DC_LEAD_DETECT_VALID_BITPOS);
            afe_lead_detector.flags &= ~(
                (1 << DC_LEAD_DETECT_ELECTRODE_1_BITPOS) |
                (1 << DC_LEAD_DETECT_ELECTRODE_2_BITPOS));
            afe_lead_detector.flags |=
                (((intr_soft_values & LEAD_STATUS1_MASK) ? 1 : 0) << DC_LEAD_DETECT_ELECTRODE_1_BITPOS);
            afe_lead_detector.flags |=
                (((intr_soft_values & LEAD_STATUS2_MASK) ? 1 : 0) << DC_LEAD_DETECT_ELECTRODE_2_BITPOS);
        }
    }
    else
    {
        afe_lead_detector.flags &= ~(1 << DC_LEAD_DETECT_VALID_BITPOS);
    }

    afe_lead_detector.flags &= ~AFE_MODE_MASK;
    afe_lead_detector.flags |= (_current_afe_mode << AFE_MODE_BITPOS) & (AFE_MODE_MASK);

    return err;
}

uint8_t afe_get_lead_detect_flags(void)
{
    return afe_lead_detector.flags;
}

uint32_t afe_get_lead_detect_amplitude(void)
{
    return afe_lead_detector.ac_amplitude;
}

uint32_t afe_get_offset_corrected_lead_detect_amplitude(void)
{
    uint32_t uncorrected_amplitude = afe_get_lead_detect_amplitude();
    uint32_t correction = (uint32_t)(2.0*(AFE_R_PROTECT_KILOHMS+AFE_R_SERIES_KILOHMS)*KILOHMS_TO_AC_AMP);
    uint32_t lower_limit = (uint32_t)(AFE_LOWER_LIMIT_KILOHMS*KILOHMS_TO_AC_AMP);
    uint32_t corrected_amplitude;
    if(uncorrected_amplitude == 0xffffffff)
    {
        corrected_amplitude = lower_limit;
    }
    else
    {
        if(uncorrected_amplitude >= (correction + lower_limit))
        {
            corrected_amplitude = uncorrected_amplitude - correction;
        }
        else
        {
            corrected_amplitude = lower_limit;
        }
    }
    // NRF_LOG_DEBUG("Uncorrected %d, correction %d, lower limit %d, corrected %d", uncorrected_amplitude, correction, lower_limit, corrected_amplitude);    
    return corrected_amplitude;
}

uint32_t afe_get_lead_detect_amplitude_lpf(void)
{
    static uint32_t amplitude_buf[5] = {400};
    static uint8_t i = 0;
    uint32_t average = 0;

    amplitude_buf[i] = afe_get_lead_detect_amplitude();
    i = (i+1)%5;
    for(uint8_t j = 0; j < 5; j++)
    {
        average += amplitude_buf[j];
    }
    average /= 5;

    return average;

}

int32_t afe_sw_reset(void)
{

    int32_t err = 0;
    uint32_t current_crtl0_val;

    err = AFE4960_read_reg(CONTROL0_ADDR, &current_crtl0_val);
    if (err != 0)
    {
        return AFE_ERROR_CONTROL0_READ_FAILED;
    }

    current_crtl0_val |= (1 << SW_RESET_BITPOS);
    err = AFE4960_write_reg(CONTROL0_ADDR, current_crtl0_val);

    // hal_delay_ms(20);
    return err;
}

int32_t afe_sw_powerdown(void)
{

    int32_t err = 0;
    uint32_t current_crtl4_val;

    err = AFE4960_read_reg(CONTROL4_ADDR, &current_crtl4_val);
    if (err != 0)
    {
        return AFE_ERROR_CONTROL4_READ_FAILED;
    }

    current_crtl4_val |= (1 << PDNAFE_BITPOS);
    err = AFE4960_write_reg(CONTROL4_ADDR, current_crtl4_val);

    return err;
}

float afe_get_eeg_vertical_scaling(void)
{

    float vertical_scale;
    float ina_gain = 1.0f;
    float preamp_gain = 1.0f;

    switch(_current_afe_mode)
    {

    case afe_mode_passive:
        switch(_eeg1_ina_gain)
        {
        case INA_GAIN_2:
            ina_gain = 2.0f;
            break;
        case INA_GAIN_3:
            ina_gain = 3.0f;
            break;
        case INA_GAIN_4:
            ina_gain = 4.0f;
            break;
        case INA_GAIN_5:
            ina_gain = 5.0f;
            break;
        case INA_GAIN_6:
            ina_gain = 6.0f;
            break;
        case INA_GAIN_9:
            ina_gain = 9.0f;
            break;
        case INA_GAIN_12:
            ina_gain = 12.0f;
            break;
        }
        preamp_gain = 1.0;
        break;

    case afe_mode_active:
        switch(_eeg2_ina_gain)
        {
        case BIOZ_INA_GAIN_2:
            ina_gain = 2.0f;
            break;
        case BIOZ_INA_GAIN_3:
            ina_gain = 3.0f;
            break;
        case BIOZ_INA_GAIN_5:
            ina_gain = 5.0f;
            break;
        case BIOZ_INA_GAIN_9:
            ina_gain = 9.0f;
            break;
        case BIOZ_INA_GAIN_12:
            ina_gain = 12.0f;
            break;
        case BIOZ_INA_GAIN_DNU1:
        case BIOZ_INA_GAIN_DNU2:
        case BIOZ_INA_GAIN_DNU3:
            ina_gain = 1.0f;
            break;
        }
#ifdef ECG2_SHORTED
        preamp_gain = 1.0;
#else
        preamp_gain = 12.96f;
#endif
        break;

    default:
        ina_gain = 1.0f;
        preamp_gain = 1.0f;
        break;

    }

    vertical_scale = 1.11f / (ina_gain * preamp_gain * (float)(1<<21));

    return vertical_scale;

}

int32_t afe_set_ina_gains(ina_gain_t eeg1_ina_gain, bioz_ina_gain_t eeg2_ina_gain)
{
    int32_t err = 0;

    _eeg1_ina_gain = eeg1_ina_gain;
    _eeg2_ina_gain = eeg2_ina_gain;
    
    switch(eeg1_ina_gain)
    {
    case INA_GAIN_2:
    case INA_GAIN_3:
    case INA_GAIN_4:
    case INA_GAIN_5:
    case INA_GAIN_6:
    case INA_GAIN_9:
    case INA_GAIN_12:
        uint32_t config_ecg1_mix1_val;
        err = AFE4960_read_reg(CONFIG_ECG1_MIX1_ADDR, &config_ecg1_mix1_val);
        if(!err)
        {
            config_ecg1_mix1_val &= ~ECG_INA_GAIN_MASK;
            config_ecg1_mix1_val |= (eeg1_ina_gain << ECG_INA_GAIN_BITPOS) & ECG_INA_GAIN_MASK;
            err = AFE4960_write_reg(CONFIG_ECG1_MIX1_ADDR, config_ecg1_mix1_val);
        }
        break;
    default:
        err = AFE_ERROR_INVALID_GAIN;
        // Invalid gain value
        // Do nothing
        break;
    }

    switch(eeg2_ina_gain)
    {
    case BIOZ_INA_GAIN_2:
    case BIOZ_INA_GAIN_3:
    case BIOZ_INA_GAIN_5:
    case BIOZ_INA_GAIN_9:
    case BIOZ_INA_GAIN_12:
        uint32_t config_bia_rx_mix1_val;
        if(!err)
        {
            err = AFE4960_read_reg(CONFIG_BIA_RX_MIX1_ADDR, &config_bia_rx_mix1_val);
        }
        if(!err)
        {
            config_bia_rx_mix1_val &= ~BIOZ_INA_GAIN_MASK;
            config_bia_rx_mix1_val |= (eeg2_ina_gain << BIOZ_INA_GAIN_BITPOS) & BIOZ_INA_GAIN_MASK;
            err = AFE4960_write_reg(CONFIG_BIA_RX_MIX1_ADDR, config_bia_rx_mix1_val);
        }
        break;
    default:
        err = AFE_ERROR_INVALID_GAIN;
        // invalid ina gain
        // do nothing
        break;
    }

    return err;

}

float afe_get_impedance_vertical_scaling(void)
{
    // This value is calibrated with AC_LEAD_DEMOD_CFG = 0x66
    return (AC_AMP_TO_KILOHMS);
}

const uint32_t dump_addresses[] = {
        CONTROL0_ADDR,
        CONTROL2_ADDR,
        CONFIG_MODE_SEL_ADDR,
        ACT_CTRL_ADDR,
        CTRL_PDN_ADDR,
        RAC_CONTROL_ADDR,
        CONTROL4_ADDR,
        READ_DESIGNID_ADDR,
        CONTROL7_ADDR,
        CONTROL8_ADDR,
        INA_GAIN_ERROR_ADDR,
        CONTROL9_ADDR,
        CONTROL10_ADDR,
        CONTROL13_ADDR,
        CONTROL19_ADDR,
        CONTROL20_ADDR,
        READ_AC_LEAD_ADDR,
        AC_DEMOD_ADDR,
        MASK_INT_REG_ADDR,
        INTR_SOFT_ADDR,
        CONTROL25_ADDR,
        CONFIG_CLK_CKT_MIX1_ADDR,
        CONFIG_CLK_DIV_MIX1_ADDR,
        CONFIG_PRPCT_MIX1_ADDR,
        CONFIG_TS_1TO8_MIX1_ADDR,
        CONFIG_TS_9TO16_MIX1_ADDR,
        CONFIG_NUM_TS_MIX1_ADDR,
        CONFIG_ECG1_MIX1_ADDR,
        CONFIG_ECG2_MIX1_ADDR,
        CONFIG_ECG_CH2_ADDR,
        CONFIG_BIA_SW_MATRIX_STATIC_MIX1_ADDR,
        CONFIG_BIA_EXC_MIX1_ADDR,
        CONFIG_BIA_TX_MIX1_ADDR,
        CONFIG_BIA_RX_MIX1_ADDR,
        CONFIG_BIA_PROC_MIX1_ADDR,
        CONFIG_ELECTRODE_BIAS1_MIX1_ADDR,
        CONFIG_ELECTRODE_BIAS2_MIX1_ADDR,
        CONFIG_COMP_ANA_MIX1_ADDR,
        CONFIG_IACTIVE_MIX1_ADDR,
        CONFIG_COMP_DIG_MIX1_ADDR,
        CONFIG_PACE_DETECT_ADDR,
        CONFIG_PACE_SIG_CHAIN_ADDR,
        CONFIG_PACE_DIG_REG1_ADDR,
        CONFIG_PACE_DIG_REG2_ADDR,
        CONFIG_PACE_DIG_REG3_ADDR,
        READ_PACE_STATUS2_ADDR,
        CONFIG_DDS_ADDR    
    };

int32_t afe_dump_registers(void)
{   
    int32_t err = 0;
    uint32_t val = 0;

    for(uint8_t i = 0; i < sizeof(dump_addresses)/sizeof(dump_addresses[0]); i++)
    {
        err = AFE4960_read_reg(dump_addresses[i], &val);
        NRF_LOG_INFO("%02x, %08x", dump_addresses[i], val);
    }

    return err;
}