#ifndef STORAGE_H_INCLUDED
#define STORAGE_H_INCLUDED

typedef enum {
    page_header_erased = 0,
    page_header_start,
    page_header_continuation,
    page_header_used,
    n_page_headers
} page_header_t;

#define PAGE_HEADER_ERASED (0xFF)
#define PAGE_HEADER_EPOCH_START (0xAA)
#define PAGE_HEADER_EPOCH_CONTINUATION (0x55)
#define PAGE_HEADER_USED (0x00)

#define STORAGE_ERROR_BASE (0x400)
#define STORAGE_ERROR_TAIL_FILE_START_HEADER_WRONG          (STORAGE_ERROR_BASE + 0x0001)
#define STORAGE_ERROR_TAIL_FILE_CONTINUATION_HEADER_WRONG   (STORAGE_ERROR_BASE + 0x0002)

typedef struct __attribute__ ((__packed__)){
    uint32_t size;
} epoch_file_header_t;

int32_t storage_blank_check(uint32_t start_address, uint32_t end_address);
int32_t storage_init(bool format);
int32_t storage_open_head(uint32_t size);
int32_t storage_write_head(uint8_t *data, uint32_t length, uint32_t *bytes_written);
int32_t storage_close_head(void);
int32_t storage_open_tail(uint32_t *size);
int32_t storage_read_tail(uint8_t *data, uint32_t length, uint32_t *bytes_read);
int32_t storage_close_tail(void);
int32_t storage_delete_tail_epoch(void);
int32_t storage_blocks_used(uint32_t *blocks_used);
int32_t storage_num_epoch_files(uint32_t *num_epoch_files);
int32_t storage_clear_fs(void);
int32_t storage_can_fit_epoch(bool *can_fit_epoch);
int32_t storage_get_next_epoch_id(uint16_t *epoch_id);
int32_t storage_garbage_collector(void);

#endif