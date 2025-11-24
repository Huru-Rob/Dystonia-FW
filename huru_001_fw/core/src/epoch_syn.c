#define NRF_LOG_MODULE_NAME syn 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "storage.h"
#include "epoch.h"
#include "epoch_syn.h"
#include "epoch_sto.h"
#include "hw_rtc.h"
#include "app_timer.h"

NRF_LOG_MODULE_REGISTER();

epoch_synthesize_status_t epoch_synthesize_status;
bool next_timestamp_available = false;
uint32_t epoch_syn_timestamp;

APP_TIMER_DEF(_epoch_syn_timer);

void epoch_syn_timer_callback(void *p_context)
{
    next_timestamp_available = true;
}

int32_t epoch_synthesize_init(void)
{
    int32_t err = 0;

    epoch_synthesize_status.state = epoch_synthesize_state_idle;
    epoch_synthesize_status.second_tick = false;
    epoch_synthesize_status.epoch_boundary = false;

    app_timer_create(&_epoch_syn_timer, APP_TIMER_MODE_SINGLE_SHOT, epoch_syn_timer_callback);

    return err;
}

epoch_synthesize_state_e epoch_synthesize_get_state(void)
{
    return epoch_synthesize_status.state;
}

int32_t epoch_synthesize_begin(int32_t num_epochs)
{
    int32_t err = 0;

    if(epoch_synthesize_status.state != epoch_synthesize_state_idle)
    {
        err = SYN_ERROR_MACHINE_NOT_IDLE;
    }
    if(!err)
    {
        err = hw_rtc_get_unix_time(&epoch_syn_timestamp);
    }
    if(!err)
    {
        epoch_synthesize_status.epochs_remaining = num_epochs;
    }

    return err;
}

uint32_t s_eeg = 0;
uint32_t s_acc = 0;
uint32_t s_mag = 0;
uint32_t s_imp = 0;

int32_t epoch_synthesize_sections(epoch_buffer_t *p_buffer)
{
    int32_t err = 0;
    uint32_t i;
    
    i = 0;
    do
    {
        int32_t eeg;
        int32_t ss = s_eeg % P_EEG;       
        if(ss < P_EEG / 2)
        {
            eeg = ss * M_EEG - C_EEG;
        }
        else
        {
            eeg = (ss - P_EEG/2) * -M_EEG + C_EEG;
        }
        p_buffer->epoch.eeg_section.data[i++] = (eeg >> 0) & 0xff;
        p_buffer->epoch.eeg_section.data[i++] = (eeg >> 8) & 0xff;
        p_buffer->epoch.eeg_section.data[i++] = (eeg >> 16) & 0xff;
        p_buffer->epoch.eeg_section.header.section_length += 3;
        s_eeg += 1;
    } while(i<3*EPOCH_DURATION_SECONDS*EEG_SAMPLE_RATE);
    NRF_LOG_DEBUG("EEG %d bytes.", i);
    p_buffer->epoch.eeg_section.header.eeg_sample_rate = EEG_SAMPLE_RATE;
    p_buffer->epoch.eeg_section.header.encoding = encoding_raw;

    i = 0;
    do
    {
        int32_t acc;  
        int32_t ss;
        
        ss = s_acc % P_ACC_X;
        if(ss < P_ACC_X / 2)
        {
            acc = ss * M_ACC_X - C_ACC_X;
        }
        else
        {
            acc = (ss - P_ACC_X/2) * -M_ACC_X + C_ACC_X;
        }
        p_buffer->epoch.acc_x_section.data[i] = (acc >> 0) & 0xff;
        p_buffer->epoch.acc_x_section.data[i+1] = (acc >> 8) & 0xff;
        p_buffer->epoch.acc_x_section.header.section_length += 2;

        ss = s_acc % P_ACC_Y;
        if(ss < P_ACC_Y / 2)
        {
            acc = ss * M_ACC_Y - C_ACC_Y;
        }
        else
        {
            acc = (ss - P_ACC_Y/2) * -M_ACC_Y + C_ACC_Y;
        }
        p_buffer->epoch.acc_y_section.data[i] = (acc >> 0) & 0xff;
        p_buffer->epoch.acc_y_section.data[i+1] = (acc >> 8) & 0xff;
        p_buffer->epoch.acc_y_section.header.section_length += 2;

        ss = s_acc % P_ACC_Z;
        if(ss < P_ACC_Z / 2)
        {
            acc = ss * M_ACC_Z - C_ACC_Z;
        }
        else
        {
            acc = (ss - P_ACC_Z/2) * -M_ACC_Z + C_ACC_Z;
        }
        p_buffer->epoch.acc_z_section.data[i] = (acc >> 0) & 0xff;
        p_buffer->epoch.acc_z_section.data[i+1] = (acc >> 8) & 0xff;
        p_buffer->epoch.acc_z_section.header.section_length += 2;

        i += 2;
        s_acc += 1;

    } while(i<2*EPOCH_DURATION_SECONDS*ACC_SAMPLE_RATE);
    NRF_LOG_DEBUG("ACC %d bytes.", i);
    p_buffer->epoch.acc_x_section.header.acc_sample_rate = ACC_SAMPLE_RATE;
    p_buffer->epoch.acc_x_section.header.encoding = encoding_raw;
    p_buffer->epoch.acc_y_section.header.acc_sample_rate = ACC_SAMPLE_RATE;
    p_buffer->epoch.acc_y_section.header.encoding = encoding_raw;
    p_buffer->epoch.acc_z_section.header.acc_sample_rate = ACC_SAMPLE_RATE;
    p_buffer->epoch.acc_z_section.header.encoding = encoding_raw;

    i = 0;
    do
    {
        int32_t mag;  
        int32_t ss;
        
        ss = s_mag % P_MAG_X;
        if(ss < P_MAG_X / 2)
        {
            mag = ss * M_MAG_X - C_MAG_X;
        }
        else
        {
            mag = (ss - P_MAG_X/2) * -M_MAG_X + C_MAG_X;
        }
        p_buffer->epoch.mag_x_section.data[i] = (mag >> 0) & 0xff;
        p_buffer->epoch.mag_x_section.data[i+1] = (mag >> 8) & 0xff;
        p_buffer->epoch.mag_x_section.header.section_length += 2;

        ss = s_mag % P_MAG_Y;
        if(ss < P_MAG_Y / 2)
        {
            mag = ss * M_MAG_Y - C_MAG_Y;
        }
        else
        {
            mag = (ss - P_MAG_Y/2) * -M_MAG_Y + C_MAG_Y;
        }
        p_buffer->epoch.mag_y_section.data[i] = (mag >> 0) & 0xff;
        p_buffer->epoch.mag_y_section.data[i+1] = (mag >> 8) & 0xff;
        p_buffer->epoch.mag_y_section.header.section_length += 2;

        ss = s_mag % P_MAG_Z;
        if(ss < P_MAG_Z / 2)
        {
            mag = ss * M_MAG_Z - C_MAG_Z;
        }
        else
        {
            mag = (ss - P_MAG_Z/2) * -M_MAG_Z + C_MAG_Z;
        }
        p_buffer->epoch.mag_z_section.data[i] = (mag >> 0) & 0xff;
        p_buffer->epoch.mag_z_section.data[i+1] = (mag >> 8) & 0xff;
        p_buffer->epoch.mag_z_section.header.section_length += 2;

        i += 2;
        s_mag += 1;

    } while(i<2*EPOCH_DURATION_SECONDS*MAG_SAMPLE_RATE);
    NRF_LOG_DEBUG("MAG %d bytes.", i);
    p_buffer->epoch.mag_x_section.header.mag_sample_rate = MAG_SAMPLE_RATE;
    p_buffer->epoch.mag_x_section.header.encoding = encoding_raw;
    p_buffer->epoch.mag_y_section.header.mag_sample_rate = MAG_SAMPLE_RATE;
    p_buffer->epoch.mag_y_section.header.encoding = encoding_raw;
    p_buffer->epoch.mag_z_section.header.mag_sample_rate = MAG_SAMPLE_RATE;
    p_buffer->epoch.mag_z_section.header.encoding = encoding_raw;

    i = 0;
    do
    {
        int32_t ac_amp;
        uint8_t flag = 0x3;
        int32_t ss = s_imp % P_AMP;
        
        if(ss < P_AMP / 2)
        {
            ac_amp = ss * M_AMP - C_AMP;
        }
        else
        {
            ac_amp = (ss - P_AMP/2) * -M_AMP + C_AMP;
        }
        p_buffer->epoch.imp_section.data[i].flags = flag;
        p_buffer->epoch.imp_section.data[i].ac_amplitude[0] = (ac_amp >> 0) & 0xff;
        p_buffer->epoch.imp_section.data[i].ac_amplitude[1] = (ac_amp >> 8) & 0xff;
        p_buffer->epoch.imp_section.data[i].ac_amplitude[2] = (ac_amp >> 16) & 0xff;
        i++;
        p_buffer->epoch.imp_section.header.section_length += 4;

        s_imp += 1;

    } while(i<EPOCH_DURATION_SECONDS*IMP_SAMPLE_RATE);
    NRF_LOG_DEBUG("IMP %d bytes.", i*4);
    p_buffer->epoch.imp_section.header.imp_sample_rate = IMP_SAMPLE_RATE;

    return err;
}

int32_t epoch_synthesize_update(void)
{
    int32_t err = 0;
    static epoch_buffer_t *epoch_building = NULL;
    uint16_t epoch_number;

    epoch_synthesize_state_e next_state = epoch_synthesize_status.state;

    switch(epoch_synthesize_status.state)
    {

    case epoch_synthesize_state_idle:
        if(epoch_synthesize_status.epochs_remaining != 0)
        {
            next_state = epoch_synthesize_state_synthesizing;
        }
        break;

    case epoch_synthesize_state_synthesizing:
        epoch_building = get_free_epoch_buffer();
        if(epoch_building != NULL)
        {
            storage_get_next_epoch_id(&epoch_number);         
            epoch_buffer_init(epoch_building, epoch_number, epoch_syn_timestamp);
            epoch_syn_timestamp += 1;
            app_timer_start(_epoch_syn_timer, APP_TIMER_TICKS(1000), NULL);
            next_timestamp_available = false;
            NRF_LOG_INFO("Beginning epoch %d, timestamp=%d", epoch_number, epoch_building->epoch.header_section.timestamp);  
            epoch_synthesize_sections(epoch_building);
            next_state = epoch_synthesize_state_storing;
        }
        break;

    case epoch_synthesize_state_storing:
            err = epoch_storage_store(epoch_building);
            if(err == 0)
            {
                epoch_synthesize_status.epochs_remaining -= 1;
                if(epoch_synthesize_status.epochs_remaining > 0)
                {
                    next_state = epoch_synthesize_state_wait;
                }
                else
                {
                    next_state = epoch_synthesize_state_idle;
                }
            }
            else
            {
                if(err == STO_ERROR_MACHINE_NOT_READY)
                {
                    // The storage machine is busy storing a previous epoch
                    // so keep trying until it is ready
                    err = 0;
                    next_state = epoch_synthesize_state_storing;
                }
            }

        break;

    case epoch_synthesize_state_wait:
    {       
        if(next_timestamp_available)
        {
            next_state = epoch_synthesize_state_synthesizing;
        }
    }
        break;
    }

    if(epoch_synthesize_status.state != next_state)
    {
        NRF_LOG_DEBUG("State %d -> %d", epoch_synthesize_status.state, next_state);
        epoch_synthesize_status.state = next_state;
    }

    return err;

}