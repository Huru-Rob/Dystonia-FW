#define NRF_LOG_MODULE_NAME rec 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdlib.h>
#include "epoch_sto.h"
#include "epoch_tra.h"
#include "epoch_syn.h"
#include "epoch.h"
#include "storage.h"
#include "bluetooth.h"
#include "hw_rtc.h"
#include "huru_ser.h"
#include "AFE4960.h"
#include "afe.h"
#include "pt10.h"
#include "acc.h"
#include "mag.h"
#include "nrf_log.h"
#include "nrf_queue.h"
#include "app_error.h"
#include "nrf_fstorage_sd.h"
#include "hal.h"
#include "internal_flash.h"
#include "MT25QL256ABA1EW7.h"
#include "app_timer.h"
#include "power.h"

NRF_LOG_MODULE_REGISTER();

int8_t epoch_pp_id = -1;
epoch_buffer_t epoch_buffer_pool[MAX_EPOCH_BUFFERS];

epoch_recording_status_t epoch_status;

uint32_t epoch_timer_ms = 0;
uint32_t epoch_rec_timestamp;

void epoch_buffer_free(epoch_buffer_t *p_epoch_buffer);
int32_t fetch_eeg_epoch_data(epoch_buffer_t *p_epoch_buffer, bool wait_for_watermarks);
int32_t fetch_acc_epoch_data(epoch_buffer_t *p_epoch_buffer, bool wait_for_watermarks);
int32_t fetch_mag_epoch_data(epoch_buffer_t *p_epoch_buffer, bool wait_for_watermarks);

int32_t epoch_data_init(void)
{
    int32_t err = 0;

    epoch_status.recording_state = epoch_recording_state_idle;
    epoch_status.start_recording = false;
    epoch_status.stop_recording = false;

    for (uint8_t i = 0; i < MAX_EPOCH_BUFFERS; i++)
    {
        epoch_buffer_pool[i].status = epoch_buffer_status_free;
    }   

    

    epoch_storage_init();
    epoch_transmission_init();
    epoch_synthesize_init();

    return err;
}

epoch_buffer_t *get_free_epoch_buffer(void)
{
    epoch_buffer_t *p_epoch_buffer = NULL;
    for (uint8_t i = 0; p_epoch_buffer == NULL && i < MAX_EPOCH_BUFFERS; i++)
    {
        if (epoch_buffer_pool[i].status == epoch_buffer_status_free)
        {
            epoch_buffer_pool[i].status = epoch_buffer_status_building;
            p_epoch_buffer = &(epoch_buffer_pool[i]);
        }
    }
    return p_epoch_buffer;
}

void epoch_buffer_free(epoch_buffer_t *p_epoch_buffer)
{
    p_epoch_buffer->status = epoch_buffer_status_free;
}

int32_t epoch_buffer_init(epoch_buffer_t *epoch_buffer, uint16_t epoch_id, uint32_t timestamp)
{
    int32_t err = 0;
    epoch_t *epoch = &(epoch_buffer->epoch);

    // Header Section
    epoch->header_section.section_id = header_section_id;
    epoch->header_section.section_length = sizeof(header_section_t);
    epoch->header_section.protocol_version = HURU_PROTOCOL_VERSION;
    epoch->header_section.epoch_id = epoch_id;
    epoch->header_section.timestamp = timestamp;
    // EEG Data Section
    epoch->eeg_section.header.section_id = eeg_data_section_id;
    epoch->eeg_section.header.section_length = sizeof(eeg_section_header_t); // no data yet
    epoch->eeg_section.header.eeg_sample_rate = (float)EEG_SAMPLE_RATE;    
    epoch->eeg_section.header.eeg_vertical_scale = afe_get_eeg_vertical_scaling();
    epoch->eeg_section.header.encoding = encoding_raw;
    // ACC X Section
    epoch->acc_x_section.header.section_id = acc_x_section_id;
    epoch->acc_x_section.header.section_length = sizeof(acc_section_header_t);
    epoch->acc_x_section.header.acc_sample_rate = (float)ACC_SAMPLE_RATE;
    epoch->acc_x_section.header.acc_vertical_scale = acc_get_vertical_scaling();
    epoch->acc_x_section.header.encoding = encoding_raw;
    // ACC Y Section
    epoch->acc_y_section.header.section_id = acc_y_section_id;
    epoch->acc_y_section.header.section_length = sizeof(acc_section_header_t);
    epoch->acc_y_section.header.acc_sample_rate = (float)ACC_SAMPLE_RATE;
    epoch->acc_y_section.header.acc_vertical_scale = acc_get_vertical_scaling();
    epoch->acc_y_section.header.encoding = encoding_raw;
    // ACC Z Section
    epoch->acc_z_section.header.section_id = acc_z_section_id;
    epoch->acc_z_section.header.section_length = sizeof(acc_section_header_t);
    epoch->acc_z_section.header.acc_sample_rate = (float)ACC_SAMPLE_RATE;
    epoch->acc_z_section.header.acc_vertical_scale = acc_get_vertical_scaling();
    epoch->acc_z_section.header.encoding = encoding_raw;
    // MAG X Section
    epoch->mag_x_section.header.section_id = mag_x_section_id;
    epoch->mag_x_section.header.section_length = sizeof(mag_section_header_t);
    epoch->mag_x_section.header.mag_sample_rate = (float)MAG_SAMPLE_RATE;
    epoch->mag_x_section.header.mag_vertical_scale = mag_get_vertical_scaling();
    epoch->mag_x_section.header.encoding = encoding_raw;
    // MAG Y Section
    epoch->mag_y_section.header.section_id = mag_y_section_id;
    epoch->mag_y_section.header.section_length = sizeof(mag_section_header_t);
    epoch->mag_y_section.header.mag_sample_rate = (float)MAG_SAMPLE_RATE;
    epoch->mag_y_section.header.mag_vertical_scale = mag_get_vertical_scaling();
    epoch->mag_y_section.header.encoding = encoding_raw;
    // MAG Z Section
    epoch->mag_z_section.header.section_id = mag_z_section_id;
    epoch->mag_z_section.header.section_length = sizeof(mag_section_header_t);
    epoch->mag_z_section.header.mag_sample_rate = (float)MAG_SAMPLE_RATE;
    epoch->mag_z_section.header.mag_vertical_scale = mag_get_vertical_scaling();
    epoch->mag_z_section.header.encoding = encoding_raw;
    // IMP Section
    epoch->imp_section.header.section_id = imp_section_id;
    epoch->imp_section.header.section_length = sizeof(imp_section_header_t);
    epoch->imp_section.header.imp_sample_rate = (float)IMP_SAMPLE_RATE;
    epoch->imp_section.header.imp_vertical_scale = afe_get_impedance_vertical_scaling();
    // BAT Section
    epoch->bat_section.header.section_id = bat_section_id;
    epoch->bat_section.header.section_length = sizeof(bat_section_header_t);
    epoch->bat_section.header.bat_sample_rate = 0.0;
    epoch->bat_section.header.bat_vertical_scale = BATTERY_VERTICAL_SCALE;
    bat_data_t *bat_data = (bat_data_t *)((uint8_t *)&(epoch->bat_section) + sizeof(bat_section_header_t));
    bat_data[0].percent = get_battery_percent();
    bat_data[0].mv = get_battery_mv();
    epoch->bat_section.header.section_length += sizeof(bat_data_t);
    // NRF_LOG_HEXDUMP_INFO((uint8_t *)&(epoch->bat_section), 15);
    // NRF_LOG_INFO("Battery: %d %d", bat_data[0].percent, bat_data[0].mv);    
    // TEMP Section
    epoch->temp_section.header.section_id = temp_section_id;
    epoch->temp_section.header.section_length = sizeof(temp_section_header_t);
    epoch->temp_section.header.temp_sample_rate = 0.0;
    epoch->temp_section.header.temp_vertical_scale = ACC_TEMP_VERTICAL_SCALE;
    uint16_t *temp_data = (uint16_t *)((uint8_t *)&(epoch->temp_section) + sizeof(temp_section_header_t));
    err = acc_get_temperature(temp_data);
    epoch->temp_section.header.section_length += sizeof(uint16_t);
    // Footer Sction
    epoch->footer_section.section_id = footer_section_id;
    epoch->footer_section.section_length = 3;

    return err;
}

bool connection_quality_updated = false;
impedance_t impedance;
void epoch_update_connection_quality(void)
{
    uint32_t ac_amplitude = afe_get_offset_corrected_lead_detect_amplitude();
    uint8_t flags = afe_get_lead_detect_flags();
    impedance.ac_amplitude[0] = (uint8_t)(ac_amplitude >> 0) & 0xff;
    impedance.ac_amplitude[1] = (uint8_t)(ac_amplitude >> 8) & 0xff;
    impedance.ac_amplitude[2] = (uint8_t)(ac_amplitude >> 16) & 0xff;
    impedance.flags = flags;
    connection_quality_updated = true;
}

void fetch_connection_quality(epoch_buffer_t *p_epoch_buffer, bool wait_for_new_data)
{
    if (!wait_for_new_data || connection_quality_updated)
    {
        if((p_epoch_buffer->epoch.imp_section.header.section_length - sizeof(imp_section_header_t)) < (IMP_DATA_BUFFER_SAMPLES - 1) * sizeof(impedance_t))
        {
            memcpy(
                (uint8_t *)&(p_epoch_buffer->epoch.imp_section)+p_epoch_buffer->epoch.imp_section.header.section_length, 
                (uint8_t *)&impedance, sizeof(impedance_t)
                );
            p_epoch_buffer->epoch.imp_section.header.section_length += sizeof(impedance_t);
        }
    }
    connection_quality_updated = false;
}

int32_t fetch_eeg_epoch_data(epoch_buffer_t *p_epoch_buffer, bool wait_for_watermarks)
{
    int32_t err = 0;

    uint8_t samples_read;
    int32_t eeg_rx_buf[AFE_FIFO_SIZE*2]; // big enough to hold an entire FIFO of samples

    if (!wait_for_watermarks || afe_samples_available())
    {
        err = afe_samples_fetch(eeg_rx_buf, AFE_COMMON_WATERMARK_LEVEL, &samples_read);
        NRF_LOG_DEBUG("Fetched %d AFE Samples", samples_read);        
        // Only store samples if there's a valid buffer.
        // Otherwise, we're just clearing the FIFO
        if (p_epoch_buffer != NULL)
        {
            eeg_section_t *section;
            section = &(p_epoch_buffer->epoch.eeg_section);

            for (uint8_t i = 0; err == 0 && i < samples_read; i++)
            {
                if (section->header.section_length - sizeof(eeg_section_header_t) > (EEG_DATA_BUFFER_SAMPLES - 1) * 3)
                {
                    NRF_LOG_WARNING("EEG Sample Buffer Overrun");
                    err = EPOCH_ERROR_EEG_BUFFER_OVERRUN;
                }
                else
                {
                    // Copy afe FIFO output to little-endian transmission protocol buffer
                    uint8_t *p_dest;
                    p_dest = (uint8_t *)section + section->header.section_length;
                    *p_dest++ = (uint8_t)((eeg_rx_buf[i] >> 0) & 0x000000ff);
                    *p_dest++ = (uint8_t)((eeg_rx_buf[i] >> 8) & 0x000000ff);
                    *p_dest++ = (uint8_t)((eeg_rx_buf[i] >> 16) & 0x000000ff);
                    section->header.section_length += 3;
                }
            }
        }
    }

    return err;
}

int32_t fetch_acc_epoch_data(epoch_buffer_t *p_epoch_buffer, bool wait_for_watermarks)
{
    int32_t err = 0;

    uint8_t samples_read;
    acc_sample_t acc_rx_buf[ACC_FIFO_SIZE];

    if (!wait_for_watermarks || acc_data_available())
    {
        err = acc_read_fifo(acc_rx_buf, &samples_read);
        // Only store samples if there's a valid buffer.
        // Otherwise, we're just clearing the FIFO
        if (p_epoch_buffer != NULL)
        {
            acc_section_t *x_section = &(p_epoch_buffer->epoch.acc_x_section);
            acc_section_t *y_section = &(p_epoch_buffer->epoch.acc_y_section);
            acc_section_t *z_section = &(p_epoch_buffer->epoch.acc_z_section);

            for (uint8_t i = 0; err == 0 && i < samples_read; i++)
            {
                if (x_section->header.section_length - sizeof(acc_section_header_t) > (ACC_DATA_BUFFER_SAMPLES - 1) * 6)
                {
                    NRF_LOG_WARNING("ACC Sample Buffer Overrun");
                    err = EPOCH_ERROR_ACC_BUFFER_OVERRUN;
                }
                else
                {
                    // Copy little-endian accelerometer output to little-endian transmission protocol buffers
                    uint8_t *p_dest;

                    p_dest = (uint8_t *)x_section + x_section->header.section_length;
                    *((uint16_t *)p_dest) = acc_rx_buf[i].x;
                    x_section->header.section_length += sizeof(int16_t);

                    p_dest = (uint8_t *)y_section + y_section->header.section_length;
                    *((uint16_t *)p_dest) = acc_rx_buf[i].y;
                    y_section->header.section_length += sizeof(int16_t);

                    p_dest = (uint8_t *)z_section + z_section->header.section_length;
                    *((uint16_t *)p_dest) = acc_rx_buf[i].z;
                    z_section->header.section_length += sizeof(int16_t);

                }
            }
        }
    }

    return err;
}

int32_t fetch_mag_epoch_data(epoch_buffer_t *p_epoch_buffer, bool wait_for_watermarks)
{
    int32_t err = 0;

    uint8_t samples_read;
    mag_sample_t mag_rx_buf[MAG_FIFO_SIZE];

    if (!wait_for_watermarks || mag_data_available())
    {
        err = mag_read_fifo(mag_rx_buf, &samples_read);
        // Only store samples if there's a valid buffer.
        // Otherwise, we're just clearing the FIFO
        if (p_epoch_buffer != NULL)
        {
            mag_section_t *x_section = &(p_epoch_buffer->epoch.mag_x_section);
            mag_section_t *y_section = &(p_epoch_buffer->epoch.mag_y_section);
            mag_section_t *z_section = &(p_epoch_buffer->epoch.mag_z_section);

            for (uint8_t i = 0; err == 0 && i < samples_read; i++)
            {
                if ((x_section->header.section_length - sizeof(mag_section_header_t)) > (MAG_DATA_BUFFER_SAMPLES - 1) * 6)
                {
                    NRF_LOG_WARNING("MAG Sample Buffer Overrun");
                    err = EPOCH_ERROR_MAG_BUFFER_OVERRUN;
                }
                else
                {
                    // Copy little-endian magnetometer output to little-endian transmission protocol buffers
                    uint8_t *p_dest;

                    p_dest = (uint8_t *)x_section + x_section->header.section_length;
                    *((uint16_t *)p_dest) = mag_rx_buf[i].x;
                    x_section->header.section_length += sizeof(int16_t);

                    p_dest = (uint8_t *)y_section + y_section->header.section_length;
                    *((uint16_t *)p_dest) = mag_rx_buf[i].y;
                    y_section->header.section_length += sizeof(int16_t);

                    p_dest = (uint8_t *)z_section + z_section->header.section_length;
                    *((uint16_t *)p_dest) = mag_rx_buf[i].z;
                    z_section->header.section_length += sizeof(int16_t);

                }
            }
        }
    }

    return err;
}

epoch_recording_state_e epoch_recording_get_state(void)
{
    return epoch_status.recording_state;
}

int32_t epoch_recording_update(void)
{
    int32_t err = 0;
    uint16_t epoch_number = 0;
    static epoch_buffer_t *epoch_building = NULL;

    epoch_recording_state_e next_state = epoch_status.recording_state;
    switch (epoch_status.recording_state)
    {
    case epoch_recording_state_idle:
        if (epoch_status.start_recording == true)
        {            
            epoch_status.start_recording = false;
            epoch_timer_ms = 0;         
            err = hw_rtc_enable_periodic_update();   
            next_state = epoch_recording_state_wait_for_boundary;
        }
        break;

    case epoch_recording_state_wait_for_boundary:
        if (epoch_status.stop_recording)
        {
            epoch_status.stop_recording = false;
            epoch_status.start_recording = false;
            next_state = epoch_recording_state_idle;
        }
        else if(hal_gpio_pin_read(PIN_RTC_INT) == 0)
        {
            // Clear both FIFOs
            // fetch_eeg_epoch_data(NULL, false);
            afe_clear_fifo();
            fetch_acc_epoch_data(NULL, false);
            fetch_mag_epoch_data(NULL, false); 
            err = hw_rtc_disable_periodic_update();
            hw_rtc_get_unix_time(&epoch_rec_timestamp); 
            epoch_building = get_free_epoch_buffer();            
            storage_get_next_epoch_id(&epoch_number);   
            epoch_buffer_init(epoch_building, epoch_number, epoch_rec_timestamp);
            NRF_LOG_INFO("Beginning epoch %d", epoch_number); 
            epoch_timer_ms = 0;                           
            next_state = epoch_recording_state_in_epoch;            
        }        
        break;

    case epoch_recording_state_in_epoch:
        if (epoch_status.stop_recording)
        {
            epoch_status.stop_recording = false;
            epoch_status.start_recording = false;
            epoch_buffer_free(epoch_building);
            if(!err)
            {
                next_state = epoch_recording_state_idle;
            }
        }
        else
        {
            fetch_acc_epoch_data(epoch_building, true);
            fetch_mag_epoch_data(epoch_building, true);
            if(afe_samples_available())
            {
                fetch_eeg_epoch_data(epoch_building, true);
                epoch_timer_ms += afe_fifo_fill_time_ms();
                if((epoch_timer_ms % 1000) == 0)
                {
                    NRF_LOG_DEBUG("EPOCH: timer = %d", epoch_timer_ms);
                    fetch_connection_quality(epoch_building, false);
                    if(epoch_timer_ms >= 1000 * EPOCH_DURATION_SECONDS)
                    {                        
                        epoch_timer_ms = 0;
                        storage_get_next_epoch_id(&epoch_number);
                        // Flush the acc FIFO
                        fetch_acc_epoch_data(epoch_building, false);
                        fetch_mag_epoch_data(epoch_building, false);
                        
                        err = epoch_storage_store(epoch_building);
                        if(!err)
                        {
                            epoch_building = get_free_epoch_buffer();
                        }
                        if(epoch_building == NULL)
                        {
                            err = EPOCH_NO_FREE_BUFFERS;
                        }
                        if(!err)
                        {
                            NRF_LOG_INFO("Beginning epoch %d", epoch_number);
                            epoch_rec_timestamp += EPOCH_DURATION_SECONDS;
                            epoch_buffer_init(epoch_building, epoch_number, epoch_rec_timestamp);
                            next_state = epoch_recording_state_in_epoch;
                        }
                    }
                }
            }
        }
        
        break;
    }
    if (epoch_status.recording_state != next_state)
    {
        NRF_LOG_DEBUG("State %d -> %d", epoch_status.recording_state, next_state);
        epoch_status.recording_state = next_state;
    }

    return err;
}

int32_t epoch_start_recording(void)
{
    int32_t err = 0;
    NRF_LOG_INFO("Start Recording");
    epoch_status.start_recording = true;
    epoch_status.stop_recording = false;
    return err;
}

int32_t epoch_stop_recording(void)
{
    int32_t err = 0;
    epoch_status.stop_recording = true;
    epoch_status.start_recording = false;
    return err;
}

