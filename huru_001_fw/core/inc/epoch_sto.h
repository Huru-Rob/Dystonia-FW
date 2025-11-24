#ifndef EPOCH_STO_H_INCLUDED
#define EPOCH_STO_H_INCLUDED

#include "epoch.h"

#define STO_ERROR_BASE                    (0x300)
#define STO_ERROR_MACHINE_NOT_READY       (STO_ERROR_BASE + 0x01)
#define STO_ERROR_INCORRECT_WRITE_COUNT_1 (STO_ERROR_BASE + 0x02)
#define STO_ERROR_INCORRECT_WRITE_COUNT_2 (STO_ERROR_BASE + 0x03)


typedef enum {
  epoch_storing_state_idle = 0,             // 0
  epoch_storing_state_create_file,          // 1
  epoch_storing_state_write_header_section, // 2
  epoch_storing_state_write_eeg_section,    // 3
  epoch_storing_state_write_acc_x_section,  // 4
  epoch_storing_state_write_acc_y_section,  // 5
  epoch_storing_state_write_acc_z_section,  // 6
  epoch_storing_state_write_mag_x_section,  // 7
  epoch_storing_state_write_mag_y_section,  // 8
  epoch_storing_state_write_mag_z_section,  // 9
  epoch_storing_state_write_imp_section,    // 10
  epoch_storing_state_write_bat_section,    // 11
  epoch_storing_state_write_temp_section,   // 12
  epoch_storing_state_write_footer_section, // 13
  epoch_storing_state_close_file,           // 14
} epoch_storing_state_e;

typedef struct {
  epoch_storing_state_e state;
  epoch_buffer_t *p_epoch_storing;
} epoch_storing_status_t;

int32_t epoch_storage_init(void);
int32_t epoch_storage_update(void);
int32_t epoch_storage_store(epoch_buffer_t *p_buffer);
epoch_storing_state_e epoch_storage_get_state(void);

#endif