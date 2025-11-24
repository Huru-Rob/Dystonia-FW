#define NRF_LOG_MODULE_NAME stream 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include <stdint.h>
#include <stdbool.h>
#include "pt10.h"
#include "epoch.h"
#include "acc.h"
#include "mag.h"
#include "afe.h"
#include "AFE4960.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_queue.h"
#include "stream.h"
#include "huru_ser.h"
#include "hal.h"
#include "serial.h"

NRF_LOG_MODULE_REGISTER();

NRF_QUEUE_DEF(int32_t, m_eeg_debug_queue, EEG_DEBUG_QUEUE_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(acc_sample_t, m_acc_debug_queue, ACC_DEBUG_QUEUE_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(mag_sample_t, m_mag_debug_queue, MAG_DEBUG_QUEUE_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(impedance_t, m_imp_debug_queue, IMP_DEBUG_QUEUE_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);
APP_TIMER_DEF(_impedance_timer);

uint8_t m_debug_output_count = 0;
bool _impedance_timeout = false;

int32_t stream_eeg_rx_buf[AFE_FIFO_SIZE*2]; // big enough to hold an entire FIFO of samples
acc_sample_t stream_acc_rx_buf[ACC_FIFO_SIZE];
mag_sample_t stream_mag_rx_buf[MAG_FIFO_SIZE];
uint8_t debug_output_data[MAX_PAYLOAD];

void impedance_timer_handler(void *p_context);

int32_t fetch_eeg_streaming_data(bool wait_for_watermarks)
{
    int32_t err = 0;
    uint8_t samples_read;

    if (!wait_for_watermarks || afe_samples_available())
    {
        err = afe_samples_fetch(stream_eeg_rx_buf, 0, &samples_read);
        // NRF_LOG_INFO("Read %d samples", samples_read);
        // NRF_LOG_INFO("From the AFE FIFO: %d. err = %d",stream_eeg_rx_buf[0], err);
        for (uint8_t i = 0; err == 0 && i < samples_read; i++)
        {
            err = nrf_queue_write(&m_eeg_debug_queue, &(stream_eeg_rx_buf[i]), 1);
            if(err != NRF_SUCCESS)
            {
                NRF_LOG_INFO("Failed to place AFE data in queue");
            }
        }
    }
    return err;
}

int32_t fetch_acc_streaming_data(bool wait_for_watermarks)
{
    int32_t err = 0;
    uint8_t samples_read;

    if (!wait_for_watermarks || acc_data_available())
    {
        err = acc_read_fifo(stream_acc_rx_buf, &samples_read);
        
        for (uint8_t i = 0; err == 0 && i < samples_read; i++)
        {
            err = nrf_queue_write(&m_acc_debug_queue, &(stream_acc_rx_buf[i]), 1);
        }
    }
    return err;
}

int32_t fetch_mag_streaming_data(bool wait_for_watermarks)
{
    int32_t err = 0;
    uint8_t samples_read;

    if(!wait_for_watermarks || mag_data_available())
    {
        err = mag_read_fifo(stream_mag_rx_buf, &samples_read);

        for(uint8_t i = 0; err == 0 && i < samples_read; i++)
        {
            err = nrf_queue_write(&m_mag_debug_queue, &(stream_mag_rx_buf[i]), 1);
        }
    }
    return err;
}

int32_t fetch_imp_streaming_data(void)
{
    int32_t err = 0;
    impedance_t impedance;

    update_lead_detect_status();
    uint32_t ac_amplitude = afe_get_offset_corrected_lead_detect_amplitude();
    uint8_t flags = afe_get_lead_detect_flags();
    impedance.ac_amplitude[0] = (uint8_t)(ac_amplitude >> 0) & 0xff;
    impedance.ac_amplitude[1] = (uint8_t)(ac_amplitude >> 8) & 0xff;
    impedance.ac_amplitude[2] = (uint8_t)(ac_amplitude >> 16) & 0xff;
    impedance.flags = flags;
    err = nrf_queue_write(&m_imp_debug_queue, &impedance, 1);
    //NRF_LOG_INFO("Fetched 1 impedance sample")

    return err;
}

int32_t streaming_init(void)
{
    int32_t err = 0;
    _impedance_timeout = false;
    err = app_timer_create(&_impedance_timer, APP_TIMER_MODE_REPEATED, impedance_timer_handler);
    return err;
}

int32_t streaming_start(void)
{
    int32_t err = 0;
    err = app_timer_start(_impedance_timer, APP_TIMER_TICKS(IMPEDANCE_STREAMING_PERIOD_MS), NULL);
    return err;
}

int32_t streaming_stop(void)
{
    int32_t err = 0;
    err = app_timer_stop(_impedance_timer);
    return err;
}

void impedance_timer_handler(void *p_context)
{
    //NRF_LOG_INFO("Impedance Timeout");
    _impedance_timeout = true;
}

int32_t streaming_update(void)
{
    int32_t err = 0;
    int32_t eeg_sample;
    acc_sample_t acc_sample;
    mag_sample_t mag_sample;
    impedance_t imp_sample;

    switch (pt10_status.debug_sensor)
    {
    case DEBUG_EEG:

        err = fetch_eeg_streaming_data(true);
        uint16_t n_samples = nrf_queue_utilization_get(&m_eeg_debug_queue);
        // NRF_LOG_INFO("Q: %d", n_samples);
        if ( n_samples >= EEG_DEBUG_SAMPLES_PER_MTU)
        {
            // NRF_LOG_INFO("From the queue: %02x %02x %02x %02x",debug_output_data[1], debug_output_data[2], debug_output_data[3], debug_output_data[4]);
            eeg_section_header_t *p_section = (eeg_section_header_t *)debug_output_data;
            p_section->section_id = eeg_data_section_id;
            p_section->section_length = sizeof(eeg_section_header_t);
            p_section->eeg_sample_rate = (float)EEG_SAMPLE_RATE;
            p_section->eeg_vertical_scale = afe_get_eeg_vertical_scaling();
            p_section->encoding = encoding_raw;
            for(uint8_t i = 0; i < EEG_DEBUG_SAMPLES_PER_MTU; i++)
            {
                err = nrf_queue_read(&m_eeg_debug_queue, &eeg_sample, 1);
                debug_output_data[p_section->section_length++] = eeg_sample & 0xff;
                debug_output_data[p_section->section_length++] = (eeg_sample>>8) & 0xff;
                debug_output_data[p_section->section_length++] = (eeg_sample>>16) & 0xff;
            }
            if (pt10_status.uart_connected)
            {
                err = serial_send_stream_data(debug_output_data, p_section->section_length);

                if(err != 0)
                {
                    NRF_LOG_INFO("err %i when sending UART data", err);
                }
            }
            else if (pt10_status.ble_connected && pt10_status.MTUSet && pt10_status.eeg_stream_notify_en)
            {                
                ble_cus_t m_eeg_cus = get_m_eeg_cus();
                // NRF_LOG_INFO("STR: EEG %d", p_section->section_length);
                err = ble_cus_value_update(&m_eeg_cus, m_eeg_cus.eeg_stream_handles, debug_output_data, MAX_PAYLOAD, true);

                if (err != 0)
                {
                    NRF_LOG_INFO("err: %i when sending data", err);
                }
            }
        }
        break;

    case DEBUG_ACCELEROMETER:

        err = fetch_acc_streaming_data(true);

        if (nrf_queue_utilization_get(&m_acc_debug_queue) >= ACC_DEBUG_SAMPLES_PER_MTU)
        {
            if (pt10_status.uart_connected)
            {

            }
            else if (pt10_status.ble_connected && pt10_status.MTUSet && pt10_status.acc_stream_notify_en)
            {                
                acc_section_header_t *p_section = (acc_section_header_t *)debug_output_data;
                p_section->section_id = acc_xyz_section_id;
                p_section->section_length = sizeof(acc_section_header_t);
                p_section->acc_sample_rate = (float)ACC_SAMPLE_RATE;
                p_section->acc_vertical_scale = acc_get_vertical_scaling();
                p_section->encoding = encoding_raw;
                for(uint8_t i = 0; i < ACC_DEBUG_SAMPLES_PER_MTU; i++)
                {
                    err = nrf_queue_read(&m_acc_debug_queue, &acc_sample, 1);
                    debug_output_data[p_section->section_length++] = acc_sample.x & 0xff;
                    debug_output_data[p_section->section_length++] = (acc_sample.x>>8) & 0xff;
                    debug_output_data[p_section->section_length++] = acc_sample.y & 0xff;
                    debug_output_data[p_section->section_length++] = (acc_sample.y>>8) & 0xff;
                    debug_output_data[p_section->section_length++] = acc_sample.z & 0xff;
                    debug_output_data[p_section->section_length++] = (acc_sample.z>>8) & 0xff;
                }
                ble_cus_t m_eeg_cus = get_m_eeg_cus();
                NRF_LOG_DEBUG("STR: ACC %d", p_section->section_length);
                err = ble_cus_value_update(&m_eeg_cus, m_eeg_cus.acc_stream_handles, debug_output_data, MAX_PAYLOAD, true);

                if (err != 0)
                {
                    NRF_LOG_WARNING("err: %i when sending data", err);
                }



            }
        }
        break;

    case DEBUG_MAGNETOMETER:

        err = fetch_mag_streaming_data(true);

        if (nrf_queue_utilization_get(&m_mag_debug_queue) >= MAG_DEBUG_SAMPLES_PER_MTU)
        {
            if (pt10_status.uart_connected)
            {

            }
            else if (pt10_status.ble_connected && pt10_status.MTUSet && pt10_status.mag_stream_notify_en)
            {                
                mag_section_header_t *p_section = (mag_section_header_t *)debug_output_data;
                p_section->section_id = mag_xyz_section_id;
                p_section->section_length = sizeof(mag_section_header_t);
                p_section->mag_sample_rate = (float)MAG_SAMPLE_RATE;
                p_section->mag_vertical_scale = mag_get_vertical_scaling();
                p_section->encoding = encoding_raw;
                for(uint8_t i = 0; i < MAG_DEBUG_SAMPLES_PER_MTU; i++)
                {
                    err = nrf_queue_read(&m_mag_debug_queue, &mag_sample, 1);
                    debug_output_data[p_section->section_length++] = mag_sample.x & 0xff;
                    debug_output_data[p_section->section_length++] = (mag_sample.x>>8) & 0xff;
                    debug_output_data[p_section->section_length++] = mag_sample.y & 0xff;
                    debug_output_data[p_section->section_length++] = (mag_sample.y>>8) & 0xff;
                    debug_output_data[p_section->section_length++] = mag_sample.z & 0xff;
                    debug_output_data[p_section->section_length++] = (mag_sample.z>>8) & 0xff;
                }
                ble_cus_t m_eeg_cus = get_m_eeg_cus();
                NRF_LOG_DEBUG("STR: ACC %d", p_section->section_length);
                err = ble_cus_value_update(&m_eeg_cus, m_eeg_cus.mag_stream_handles, debug_output_data, MAX_PAYLOAD, true);

                if (err != 0)
                {
                    NRF_LOG_WARNING("err: %i when sending data", err);
                }



            }
        }
        break;

    case DEBUG_IMPEDANCE:
        if(_impedance_timeout == true)
        {
            _impedance_timeout = false;
            fetch_imp_streaming_data();
        }

        if(nrf_queue_utilization_get(&m_imp_debug_queue) >= IMP_DEBUG_SAMPLES_PER_MTU)
        {
            if (pt10_status.uart_connected)
            {

            }
            else if (pt10_status.ble_connected && pt10_status.MTUSet && pt10_status.epoch_data_notify_en)
            {                
                imp_section_header_t *p_section = (imp_section_header_t *)debug_output_data;
                p_section->section_id = imp_section_id;
                p_section->section_length = sizeof(imp_section_header_t);
                p_section->imp_sample_rate = (float)IMP_DEBUG_SAMPLE_RATE;
                p_section->imp_vertical_scale = afe_get_impedance_vertical_scaling();
                for(uint8_t i = 0; i < IMP_DEBUG_SAMPLES_PER_MTU; i++)
                {
                    err = nrf_queue_read(&m_imp_debug_queue, &imp_sample, 1);
                    memcpy(&(debug_output_data[p_section->section_length]), &imp_sample, sizeof(impedance_t));
                    p_section->section_length += sizeof(impedance_t);
                }
                ble_cus_t m_eeg_cus = get_m_eeg_cus();
                NRF_LOG_DEBUG("STR: IMP %d", p_section->section_length);
                err = ble_cus_value_update(&m_eeg_cus, m_eeg_cus.imp_stream_handles, debug_output_data, MAX_PAYLOAD, true);
                NRF_LOG_DEBUG("Pushed 1 impedance MTU");
                if (err != 0)
                {
                    NRF_LOG_WARNING("err: %i when sending data", err);
                }



            }
        }
        break;

    }

    return err;
}