#ifndef EPOCH_H_INCLUDED
#define EPOCH_H_INCLUDED
#include <stdint.h>
#include "board_config.h"
#include "pt10.h"
//#include "epoch_sto.h"

#define EPOCH_REC_ERROR_BASE                  (0x600)
#define EPOCH_NO_FREE_BUFFERS                 (EPOCH_REC_ERROR_BASE + 0x01)
#define EPOCH_ERROR_EEG_BUFFER_OVERRUN        (EPOCH_REC_ERROR_BASE + 0x02)
#define EPOCH_ERROR_ACC_BUFFER_OVERRUN        (EPOCH_REC_ERROR_BASE + 0x03)
#define EPOCH_ERROR_MAG_BUFFER_OVERRUN        (EPOCH_REC_ERROR_BASE + 0x04)


#define EEG_SAMPLES_PER_FETCH_RECORDING (50) // 0.2s of recording
#define ACC_SAMPLES_PER_FETCH_RECORDING 
#define EEG_SAMPLES_PER_FETCH_STREAMING (ACC_DEBUG_SAMPLES_PER_MTU)


#if AFE_CLOCK_SYNC == 1
#define EEG_SAMPLE_RATE (512)
#else
#define EEG_SAMPLE_RATE (250)
#endif
#define ACC_SAMPLE_RATE (50)
#define MAG_SAMPLE_RATE (10)
#define IMP_SAMPLE_RATE (1)
#define EEG_SAMPLES_PER_EPOCH (EPOCH_DURATION_SECONDS * EEG_SAMPLE_RATE)
#define ACC_SAMPLES_PER_EPOCH (EPOCH_DURATION_SECONDS * ACC_SAMPLE_RATE)
#define IMP_SAMPLES_PER_EPOCH (EPOCH_DURATION_SECONDS * IMP_SAMPLE_RATE)
#define MAG_SAMPLES_PER_EPOCH (EPOCH_DURATION_SECONDS * MAG_SAMPLE_RATE)
#define EEG_DATA_BUFFER_SAMPLES (EEG_SAMPLES_PER_EPOCH * 11 / 10)
#define ACC_DATA_BUFFER_SAMPLES (ACC_SAMPLES_PER_EPOCH * 11 / 10)
#define IMP_DATA_BUFFER_SAMPLES (IMP_SAMPLES_PER_EPOCH * 11 / 10)
#define MAG_DATA_BUFFER_SAMPLES (MAG_SAMPLES_PER_EPOCH * 11 / 10)

#define MAX_EPOCH_FILE_SIZE ( \
        sizeof(header_section_t) + \
        sizeof(eeg_section_header_t) + EEG_DATA_BUFFER_SAMPLES * 3 + \
        3 * (sizeof(acc_section_header_t) + ACC_DATA_BUFFER_SAMPLES * 2) + \
        3 * (sizeof(mag_section_header_t) + MAG_DATA_BUFFER_SAMPLES * 2) + \
        sizeof(imp_section_header_t) + IMP_DATA_BUFFER_SAMPLES * sizeof(imp_sample_t) + \
        sizeof(footer_section_t) \
)

#define TRANSMISSION_DATA_SIZE  (243)

#define MAX_EPOCH_BUFFERS (0)

typedef enum
{
  epoch_recording_state_idle = 1,           // 1
  epoch_recording_state_wait_for_boundary,  // 2
  epoch_recording_state_in_epoch            // 3
} epoch_recording_state_e;

typedef enum {
  header_section_id = 1,
  eeg_data_section_id = 2,
  acc_x_section_id = 3,
  acc_y_section_id = 4,
  acc_z_section_id = 5,
  imp_section_id = 6,
  acc_xyz_section_id = 7,
  mag_x_section_id = 8,
  mag_y_section_id = 9,
  mag_z_section_id = 10,
  mag_xyz_section_id = 11,
  bat_section_id = 12,
  temp_section_id = 13,
  footer_section_id = 0xff
} epoch_section_id_e;

typedef enum {
  encoding_raw = 0,
  encoding_flac = 1
} encoding_e;

typedef enum {
  epoch_transmission_state_idle = 0,
  epoch_transmission_state_open_file,
  epoch_transmission_state_read,
  epoch_transmission_state_transmit,
  epoch_transmission_state_close_file,
} epoch_transmission_state_e;

typedef struct
{
  epoch_recording_state_e recording_state;
  
  bool start_recording;
  bool stop_recording;
  uint16_t epoch_to_retransmit;
} epoch_recording_status_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t section_id; // 0
  uint16_t section_length; // 1
  uint16_t protocol_version; // 3
  uint16_t epoch_id; // 5
  uint32_t timestamp; // 7
} header_section_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t section_id; // 0
  uint16_t section_length; // 1-2
  float eeg_sample_rate; // 3-6
  float eeg_vertical_scale; // 7-10
  uint8_t encoding; // 11
} eeg_section_header_t;

typedef struct __attribute__ ((__packed__)) {
  eeg_section_header_t header;
  uint8_t data[EEG_DATA_BUFFER_SAMPLES * 3];
} eeg_section_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t section_id; // 0
  uint16_t section_length; // 1
  float acc_sample_rate; // 3
  float acc_vertical_scale; // 7
  uint8_t encoding; // 11
} acc_section_header_t;

typedef struct __attribute__ ((__packed__)) {
  acc_section_header_t header;
  uint8_t data[ACC_DATA_BUFFER_SAMPLES * 2];
} acc_section_t;

typedef struct __attribute__ ((__packed__)) {
  acc_section_header_t header;
  uint8_t data[ACC_DATA_BUFFER_SAMPLES * 2 * 3];
} acc_xyz_section_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t section_id; // 0
  uint16_t section_length; // 1
  float mag_sample_rate; // 3
  float mag_vertical_scale; // 7
  uint8_t encoding; // 11
} mag_section_header_t;

typedef struct __attribute__ ((__packed__)) {
  mag_section_header_t header;
  uint8_t data[MAG_DATA_BUFFER_SAMPLES * 2];
} mag_section_t;

typedef struct __attribute__ ((__packed__)) {
  mag_section_header_t header;
  uint8_t data[MAG_DATA_BUFFER_SAMPLES * 2 * 3];
} mag_xyz_section_t;

typedef struct __attribute__ ((__packed__)) {
  uint8_t flags;
  uint8_t ac_amplitude[3];
} imp_sample_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t section_id; // 0
  uint16_t section_length; // 1
  float imp_sample_rate; // 3
  float imp_vertical_scale; // 7
} imp_section_header_t;

typedef struct __attribute__ ((__packed__)) {
  imp_section_header_t header;
  imp_sample_t data[IMP_DATA_BUFFER_SAMPLES];
} imp_section_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t section_id; // 0
  uint16_t section_length; // 1
  float bat_sample_rate; // 3
  float bat_vertical_scale; // 7
} bat_section_header_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t percent;
  uint16_t mv;
} bat_data_t;

typedef struct __attribute__ ((__packed__)) {
  bat_section_header_t header;
  bat_data_t bat_data[1];
} bat_section_t;

typedef struct  __attribute__ ((__packed__)) {
  uint8_t section_id; // 0
  uint16_t section_length; // 1
  float temp_sample_rate;
  float temp_vertical_scale;
} temp_section_header_t;

typedef struct __attribute__ ((__packed__)) {
  temp_section_header_t header;
  uint16_t data[1];
} temp_section_t;

typedef struct __attribute__ ((__packed__)) {
  uint8_t section_id;
  uint16_t section_length;
} footer_section_t;

typedef struct {
  header_section_t  header_section;
  eeg_section_t eeg_section;
  acc_section_t acc_x_section;
  acc_section_t acc_y_section;
  acc_section_t acc_z_section;
  mag_section_t mag_x_section;
  mag_section_t mag_y_section;
  mag_section_t mag_z_section;
  imp_section_t imp_section;
  bat_section_t bat_section;
  temp_section_t temp_section;
  footer_section_t footer_section;
} epoch_t;

typedef struct __attribute__ ((__packed__)) {
  uint8_t flags;
  uint8_t ac_amplitude[3];
} impedance_t;

typedef enum {
  epoch_buffer_status_free = 0,
  epoch_buffer_status_building,
  epoch_buffer_status_storing
} epoch_buffer_status_e;

typedef struct {
  epoch_buffer_status_e status;
  epoch_t epoch;
} epoch_buffer_t;



int32_t epoch_data_init();
int32_t epoch_start_recording(void);
int32_t epoch_stop_recording(void);
epoch_recording_state_e epoch_recording_get_state(void);
epoch_buffer_t *get_free_epoch_buffer(void);
int32_t epoch_buffer_init(epoch_buffer_t *epoch_buffer, uint16_t epoch_id, uint32_t timestamp);
int32_t epoch_recording_update(void);
void epoch_buffer_free(epoch_buffer_t *p_epoch_buffer);
void epoch_update_connection_quality(void);

#endif