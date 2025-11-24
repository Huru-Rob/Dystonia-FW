#ifndef __MT25QL256ABA1EW7_H
#define __MT25QL256ABA1EW7_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define MT25_ERROR_BASE                         (0x900)
#define MT25_ERROR_BANK_PROTECTED               (MT25_ERROR_BASE + 0x01)
#define MT25_ERROR_WRITE_ENABLE_FAILED          (MT25_ERROR_BASE + 0x02)
#define MT25_BULK_ERASE_CMD_FAILED              (MT25_ERROR_BASE + 0x03)
#define MT25_ERROR_BUSY_AFTER_WIP_BIT_CLEAR     (MT25_ERROR_BASE + 0x04)
#define MT25_PREVIOUS_ERASE_SECTOR_FAILED       (MT25_ERROR_BASE + 0x05)
#define MT25_ERROR_PROGRAM_FAILED               (MT25_ERROR_BASE + 0x06)


#define MT25_MICRON_MANUFACTURER_ID             (0x20)

#define MT25_MEMORY_TYPE_ID_3V                  (0xBA)
#define MT25_MEMORY_TYPE_ID_1V8                 (0xBB)

#define MT25_MEMORY_CAPACITY_ID_2GB             (0x22)
#define MT25_MEMORY_CAPACITY_ID_1GB             (0x21)
#define MT25_MEMORY_CAPACITY_ID_512MB           (0x20)
#define MT25_MEMORY_CAPACITY_ID_256MB           (0x19)
#define MT25_MEMORY_CAPACITY_ID_128MB           (0x18)
#define MT25_MEMORY_CAPACITY_ID_64MB            (0x17)

#define FLASH_CMD_READ_ID                       (0x9F)
//#define FLASH_CMD_WRITE_EN                    (0x04)
#define FLASH_CMD_WRITE_EN                      (0x06)
#define FLASH_CMD_WRITE_DIS                     (0x04)
#define FLASH_CMD_READ_STATUS                   (0x05)
#define FLASH_CMD_READ_FLAG_STATUS              (0x70)
#define FLASH_CMD_BULK_ERASE                    (0xc4)
#define FLASH_CMD_RESET_ENABLE                  (0x66)
#define FLASH_CMD_RESET_MEMORY                  (0x99)
#define FLASH_CMD_ENTER_4_BYTE_ADDRESS_MODE     (0xb7)
#define FLASH_CMD_4_BYTE_READ                   (0x13)
#define FLASH_CMD_4_BYTE_PAGE_PROGRAM           (0x12)
#define FLASH_CMD_4_BYTE_4KB_SUBSECTOR_ERASE    (0x21)
#define FLASH_CMD_4_BYTE_32KB_SUBSECTOR_ERASE   (0x5C)
#define FLASH_CMD_4_BYTE_64KB_SECTOR_ERASE      (0xDC)
#define FLASH_CMD_ERASE_SUSPEND                 (0x75)
#define FLASH_CMD_ERASE_RESUME                  (0x7A)
#define FLASH_CMD_ENTER_DEEP_POWER_DOWN         (0xB9)
#define FLASH_CMD_RELEASE_FROM_DEEP_POWER_DOWN  (0xAB)
#define FLASH_CMD_READ_SECTOR_PROTECTION        (0x2D)
#define FLASH_CMD_READ_VOLATILE_LOCK_BITS       (0xE8)
#define FLASH_CMD_READ_NONVOLATILE_LOCK_BITS    (0xE2)

#define MT25_PAGE_SIZE                          (256)
#define MT25_4KB_SUBSECTOR_SIZE                 (4096)
#define MT25_32KB_SUBSECTOR_SIZE                (32768)
#define MT25_64KB_SECTOR_SIZE                   (65536)

// Real Parameters
#define MT25_64KB_SECTOR_COUNT                  (2048)
#define MT25_FLASH_MEMORY_SIZE                  (MT25_64KB_SECTOR_COUNT * MT25_64KB_SECTOR_SIZE)
// // Test parameters for wrap-around hang
// #define MT25_SECTOR_COUNT                       (4)

#define MT25_32KB_SUBSECTOR_COUNT               (2 * MT25_64KB_SECTOR_COUNT)
#define MT25_4KB_SUBSECTOR_COUNT                (16 * MT25_64KB_SECTOR_COUNT)



#define MT25_64KB_SECTOR_ADDRESS_MASK           (MT25_64KB_SECTOR_SIZE - 1)
#define MT25_32KB_SUBSECTOR_ADDRESS_MASK        (MT25_32KB_SUBSECTOR_SIZE - 1)

#define MT25_FLAG_STATUS_READY                  (0x80)

#define FLASH_ERROR_BASE                        (0x200)
#define FLASH_ERROR_INVALID_ADDR                (FLASH_ERROR_BASE + 0x0001)
#define FLASH_ERROR_RESET_WRITE_ENABLE_FAILED   (FLASH_ERROR_BASE + 0x0002)
#define FLASH_ERROR_RESET_WRITE_FAILED          (FLASH_ERROR_BASE + 0x0003)
#define FLASH_ID_CHECK_FAILED                   (FLASH_ERROR_BASE + 0x0004)
#define FLASH_ERROR_4_BYTE_MODE_WRITE_FAILED    (FLASH_ERROR_BASE + 0x0005)
#define FLASH_ERROR_STUCK_IN_3_BYTE_MODE        (FLASH_ERROR_BASE + 0x0006)
#define FLASH_ERROR_STATUS_BYTE_READ_FAILED     (FLASH_ERROR_BASE + 0x0007)
#define FLASH_ERROR_ID_READ_FAILED              (FLASH_ERROR_BASE + 0x0008)
#define FLASH_ERROR_ERASE_SUSPEND_FAILED        (FLASH_ERROR_BASE + 0x0009)
#define FLASH_ERROR_ERASE_SUSPENDED_BUT_NOT_RESUMED (FLASH_ERROR_BASE + 0x000A)
#define FLASH_ERROR_FLAG_STATUS_BYTE_READ_FAILED (FLASH_ERROR_BASE + 0x000B)



#define MT25_STATUS_WRITE_DISABLE_BITPOS                            (7)
#define MT25_STATUS_BP_3_BITPOS                                     (6)
#define MT25_STATUS_BOTTOM_BITPOS                                   (5)
#define MT25_STATUS_BP_2_BITPOS                                     (4)
#define MT25_STATUS_BP_1_BITPOS                                     (3)
#define MT25_STATUS_BP_0_BITPOS                                     (2)
#define MT25_STATUS_WRITE_ENABLE_LATCH_BITPOS                       (1)
#define MT25_STATUS_WRITE_IN_PROGRESS_BITPOS                        (0)

#define MT25_STATUS_WRITE_DISABLE_MASK                              (1<<MT25_STATUS_WRITE_DISABLE_BITPOS)
#define MT25_STATUS_BP_3_MASK                                       (1<<MT25_STATUS_BP_3_BITPOS)
#define MT25_STATUS_BOTTOM_MASK                                     (1<<MT25_STATUS_BOTTOM_BITPOS)
#define MT25_STATUS_BP_2_MASK                                       (1<<MT25_STATUS_BP_2_BITPOS)
#define MT25_STATUS_BP_1_MASK                                       (1<<MT25_STATUS_BP_1_BITPOS)
#define MT25_STATUS_BP_0_MASK                                       (1<<MT25_STATUS_BP_0_BITPOS)
#define MT25_STATUS_WRITE_ENABLE_LATCH_MASK                         (1<<MT25_STATUS_WRITE_ENABLE_LATCH_BITPOS)
#define MT25_STATUS_WRITE_IN_PROGRESS_MASK                          (1<<MT25_STATUS_WRITE_IN_PROGRESS_BITPOS)



#define MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_BITPOS   (7)
#define MT25_STATUS_FLAG_ERASE_SUSPEND_BITPOS                       (6)
#define MT25_STATUS_FLAG_ERASE_BITPOS                               (5)
#define MT25_STATUS_FLAG_PROGRAM_BITPOS                             (4)
#define MT25_STATUS_FLAG_PROGRAM_SUSPEND_BITPOS                     (2)
#define MT25_STATUS_FLAG_PROTECTION_BITPOS                          (1)
#define MT25_STATUS_FLAG_ADDRESSING_BITPOS                          (0)

#define MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_MASK     (1<<MT25_STATUS_FLAG_PROGRAM_OR_ERASE_CONTROLLER_READY_BITPOS)
#define MT25_STATUS_FLAG_ERASE_SUSPEND_MASK                         (1<<MT25_STATUS_FLAG_ERASE_SUSPEND_BITPOS)
#define MT25_STATUS_FLAG_ERASE_MASK                                 (1<<MT25_STATUS_FLAG_ERASE_BITPOS)
#define MT25_STATUS_FLAG_PROGRAM_MASK                               (1<<MT25_STATUS_FLAG_PROGRAM_BITPOS)
#define MT25_STATUS_FLAG_PROGRAM_SUSPEND_MASK                       (1<<MT25_STATUS_FLAG_PROGRAM_SUSPEND_BITPOS)
#define MT25_STATUS_FLAG_PROTECTION_MASK                            (1<<MT25_STATUS_FLAG_PROTECTION_BITPOS)
#define MT25_STATUS_FLAG_ADDRESSING_MASK                            (1<<MT25_STATUS_FLAG_ADDRESSING_BITPOS)






int32_t init_flash(void);
int32_t flash_who_ami(void);
int32_t flash_lower_power_mode(bool set_low_power);
int32_t flash_write_enable(void);
int32_t wait_for_wip_bit(uint16_t delay);

int32_t flash_erase_all(void);
int32_t flash_erase_64(uint32_t address);

int32_t flash_download(uint32_t address, uint32_t size);

int32_t flash_erase_subsector(uint32_t address);
int32_t flash_erase_sector(uint32_t address);
int32_t flash_block_read(uint32_t address, uint8_t *buffer, size_t size);
int32_t flash_block_write(uint32_t address, const uint8_t *buffer, size_t size);
int32_t flash_erase_in_progress(bool *in_progress);

#endif