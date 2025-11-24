#define NRF_LOG_MODULE_NAME storage 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL

#include "epoch.h"
#include "storage.h"
#include "MT25QL256ABA1EW7.h"
#include "pt10.h"
#include "internal_flash.h"
#include "hal.h"
#include "board_config.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

NRF_LOG_MODULE_REGISTER();

uint8_t write_buf[MT25_PAGE_SIZE];
uint32_t write_buf_idx;
uint8_t read_buf[MT25_PAGE_SIZE];
uint8_t read_buf_idx;

uint32_t first_data_page_address;
uint32_t first_free_page_address;
uint32_t tail_epoch_start_address;
uint32_t erase_start_address;

uint32_t num_epoch_files;
uint16_t next_epoch_id = 0;

uint32_t page_header_transitions[n_page_headers][n_page_headers];

void storage_format(void)
{
    // // Erase the first 4 64KB sectors to give enough space to record a few epochs.
    // // Then set the erase_start_address just past the erased space.
    // // This will start the process of erasing all sectors up to the first free page.


    write_buf_idx = 0;
    first_data_page_address = 0x00000000;
    first_free_page_address = first_data_page_address;
    flash_erase_sector(first_data_page_address);
    erase_start_address = first_data_page_address + 1 * MT25_64KB_SECTOR_SIZE;
    num_epoch_files = 0;
    next_epoch_id = 0;
}

bool process_header_transition(uint8_t last_page_header, uint8_t page_header, uint32_t page_address)
{
    bool corrupt = false;

    if(page_header == PAGE_HEADER_EPOCH_START)
    {
        header_section_t *p_header = (header_section_t *)(read_buf + 1 + sizeof(epoch_file_header_t));
        NRF_LOG_DEBUG("Found epoch start header at 0x%08x, epoch_id = %d", page_address, p_header->epoch_id);
        num_epoch_files++;
    }            

    switch(last_page_header)
    {

    case PAGE_HEADER_ERASED:
        switch(page_header)
        {
        case PAGE_HEADER_ERASED:
            page_header_transitions[page_header_erased][page_header_erased]++;
            // no transition
            break;
        case PAGE_HEADER_EPOCH_START:
            page_header_transitions[page_header_erased][page_header_start]++;
            first_data_page_address = page_address;
            if(page_header_transitions[page_header_erased][page_header_start] > 1)
            {
                corrupt = true;
                NRF_LOG_WARNING("Illegal second transition erased to epoch start at 0x%08x", page_address);
            }
            break;
        case PAGE_HEADER_EPOCH_CONTINUATION:
            page_header_transitions[page_header_erased][page_header_continuation]++;
            NRF_LOG_WARNING("Illegal transition %d to %d at 0x%08x", last_page_header, page_header, page_address);
            corrupt = true;
            break;
        case PAGE_HEADER_USED:
            page_header_transitions[page_header_erased][page_header_used]++;
            erase_start_address = page_address & ~MT25_64KB_SECTOR_ADDRESS_MASK;
            if(page_header_transitions[page_header_erased][page_header_used] > 1)
            {
                corrupt = true;
                NRF_LOG_WARNING("Illegal second transition erased to used at 0x%08x", page_address);
            }
            break;                            
        }
        break;

    case PAGE_HEADER_EPOCH_START:
        switch(page_header)
        {
            case PAGE_HEADER_ERASED:
            page_header_transitions[page_header_start][page_header_erased]++;
            // This transition could occur if a recorded epoch only occupied one page
            first_free_page_address = page_address;
            if(page_header_transitions[page_header_start][page_header_erased] > 1)
            {
                corrupt = true;
                NRF_LOG_WARNING("Illegal second transition epoch start to erased at 0x%08x", page_address);
            }
            break;
        case PAGE_HEADER_EPOCH_START:
            // This transition could possibly be two consecutive epochs occupying only one page.
            page_header_transitions[page_header_start][page_header_start]++;
            break;
        case PAGE_HEADER_EPOCH_CONTINUATION:
            // This transition is expected at the second page of every epoch
            page_header_transitions[page_header_start][page_header_continuation]++;
            break;                           
        case PAGE_HEADER_USED:
            page_header_transitions[page_header_start][page_header_used]++;
            NRF_LOG_WARNING("Illegal transition %d to %d at 0x%08x", last_page_header, page_header, page_address);
            corrupt = true;
            break;
        }
        break;

    case PAGE_HEADER_EPOCH_CONTINUATION:
        switch(page_header)
        {
        case PAGE_HEADER_ERASED:
            page_header_transitions[page_header_continuation][page_header_erased]++;
            first_free_page_address = page_address;
            if(page_header_transitions[page_header_continuation][page_header_erased] > 1)
            {
                corrupt = true;
                NRF_LOG_WARNING("Illegal second transition epoch continuation to erased at 0x%08x", page_address);
            }
            break;
        case PAGE_HEADER_EPOCH_START:
            // This transition is expected at the end of every epoch except the last one.
            page_header_transitions[page_header_continuation][page_header_start]++;
            break;
        case PAGE_HEADER_EPOCH_CONTINUATION:
            // This transition is expected in the middle of every epoch
            page_header_transitions[page_header_continuation][page_header_continuation]++;
            break;
        case PAGE_HEADER_USED:
            // We don't expect to see this. The next page header after continuation should be erased.
            page_header_transitions[page_header_continuation][page_header_used]++;
            NRF_LOG_WARNING("Illegal transition %d to %d at 0x%08x", last_page_header, page_header, page_address);
            corrupt = true;
            break;
        }                            
        break;

    case PAGE_HEADER_USED:
        switch(page_header)
        {
        case PAGE_HEADER_ERASED:
            page_header_transitions[page_header_used][page_header_erased]++;
            first_free_page_address = page_address;
            if(page_header_transitions[page_header_used][page_header_erased] > 1)
            {
                corrupt = true;
                NRF_LOG_WARNING("Illegal second transition used to erased at 0x%08x", page_address);
            }
            break;
        case PAGE_HEADER_EPOCH_START:
            page_header_transitions[page_header_used][page_header_start]++;
            first_data_page_address = page_address;                                
            if(page_header_transitions[page_header_used][page_header_start] > 1)
            {
                corrupt = true;
                NRF_LOG_WARNING("Illegal second transition used to epoch start at 0x%08x", page_address);
            }
            break;
        case PAGE_HEADER_EPOCH_CONTINUATION:
            page_header_transitions[page_header_used][page_header_continuation]++;
            // This transition is OK.
            // It can happen on the first page header after we've stepped forward by a whole 64KByte sector
            // TODO: check this. I'm not sure that it is correct
            break;
        case PAGE_HEADER_USED:
            page_header_transitions[page_header_used][page_header_used]++;
            // no transition
            break;
        }
        break;
    }

    return corrupt;

}

int32_t storage_parse(bool *corrupt)
{
    int32_t err = 0;
    uint8_t last_page_header;
    uint8_t page_header;
    uint32_t page_address = 0;
    uint32_t address_step;

    err = flash_block_read(MT25_FLASH_MEMORY_SIZE - MT25_PAGE_SIZE, read_buf, 1+sizeof(epoch_file_header_t)+sizeof(header_section_t));
    last_page_header = read_buf[0];
    if(!err)
    {
        do
        {
            err = flash_block_read(page_address, read_buf, 1+sizeof(epoch_file_header_t)+sizeof(header_section_t));
            page_header = read_buf[0];
            if(((page_address % MT25_64KB_SECTOR_SIZE) == 0) && (page_header == PAGE_HEADER_ERASED))
            {
                address_step = MT25_64KB_SECTOR_SIZE;
            }
            else
            {
                address_step = MT25_PAGE_SIZE;
            }
            if( process_header_transition(last_page_header, page_header, page_address) == true)
            {
                *corrupt = true;
            }
            page_address += address_step;
            page_address %= MT25_FLASH_MEMORY_SIZE;
            last_page_header = page_header;
        } while (!err && page_address != 0);
    }
    return err;
}

uint32_t blank_check_pages_erased = 0;
uint32_t blank_check_pages_epoch_start = 0;
uint32_t blank_check_pages_epoch_continuation = 0;
uint32_t blank_check_pages_used = 0;
uint32_t blank_check_read_errors = 0;
uint32_t blank_check_pages_other = 0;
int32_t storage_blank_check(uint32_t start_address, uint32_t end_address)
{
    int32_t err = 0;
    blank_check_pages_erased = 0;    
    blank_check_pages_epoch_start = 0;
    blank_check_pages_epoch_continuation = 0;
    blank_check_pages_used = 0;
    blank_check_pages_other = 0;
    blank_check_read_errors = 0;

    for(uint32_t page_address = start_address; page_address < end_address; page_address += MT25_PAGE_SIZE)
    {
        err = flash_block_read(page_address, read_buf, 1+sizeof(epoch_file_header_t)+sizeof(header_section_t));
        if(err != 0)
        {
            blank_check_read_errors++;
        }
        switch(read_buf[0])
        {
        case PAGE_HEADER_ERASED:
            blank_check_pages_erased++;
            break;
        case PAGE_HEADER_EPOCH_START:
            blank_check_pages_epoch_start++;
            break;
        case PAGE_HEADER_EPOCH_CONTINUATION:
            blank_check_pages_epoch_continuation++;
            break;
        case PAGE_HEADER_USED:
            blank_check_pages_used++;
            break;
        default:
            blank_check_pages_other++;
            break;
        }        
    }
    NRF_LOG_INFO("Blank Check Pages Erased      %d", blank_check_pages_erased); NRF_LOG_PROCESS();
    NRF_LOG_INFO("Blank Check Pages Epoch Start %d", blank_check_pages_epoch_start); NRF_LOG_PROCESS();
    NRF_LOG_INFO("Blank Check Pages Epoch Cont  %d", blank_check_pages_epoch_continuation); NRF_LOG_PROCESS();
    NRF_LOG_INFO("Blank Check Pages Used        %d", blank_check_pages_used); NRF_LOG_PROCESS();
    NRF_LOG_INFO("Blank Check Pages Other       %d", blank_check_pages_other); NRF_LOG_PROCESS();
    NRF_LOG_INFO("Blank Check Read Errors       %d", blank_check_read_errors); NRF_LOG_PROCESS();
    return err;
}

uint32_t incorrect_epoch_start_headers = 0;
uint32_t incorrect_epoch_continuation_headers = 0;
uint32_t incorrect_erased_headers = 0;
uint32_t epoch_start_headers = 0;
int32_t storage_integrity_check(void)
{
    int32_t err = 0;

    uint32_t page_address = first_data_page_address;
    uint32_t next_epoch_start_page = first_data_page_address;
    uint8_t page_header;
    uint8_t pages_in_epoch;
    uint8_t last_page_header = PAGE_HEADER_ERASED;


    for(uint32_t page_number = 0; page_number < MT25_FLASH_MEMORY_SIZE / MT25_PAGE_SIZE; page_number++)
    {
        err = flash_block_read(page_address, read_buf, 1+sizeof(epoch_file_header_t)+sizeof(header_section_t));
        page_header = read_buf[0];
        if(page_address == next_epoch_start_page)
        {
            if(epoch_start_headers == num_epoch_files)
            {
                if(page_header != PAGE_HEADER_ERASED)
                {
                    incorrect_erased_headers++;
                }                
            }
            else
            {
                if(page_header != PAGE_HEADER_EPOCH_START)
                {
                    incorrect_epoch_start_headers++;            
                }
            }  

            if(page_header == PAGE_HEADER_EPOCH_START)
            {
                epoch_start_headers++;
                epoch_file_header_t *header = (epoch_file_header_t *)(read_buf+1);
                uint32_t epoch_size = header->size;
                if((epoch_size + sizeof(epoch_file_header_t)) % 255 == 0)
                {
                    pages_in_epoch = (epoch_size + sizeof(epoch_file_header_t)) / 255;
                }
                else
                {
                    pages_in_epoch = (epoch_size + sizeof(epoch_file_header_t)) / 255 + 1;
                }
                next_epoch_start_page += pages_in_epoch * 256;
                next_epoch_start_page %= MT25_FLASH_MEMORY_SIZE;
            }

            
        }
        else if(last_page_header == PAGE_HEADER_EPOCH_START && page_header != PAGE_HEADER_EPOCH_CONTINUATION)
        {
            incorrect_epoch_continuation_headers++;
        }

        page_address += MT25_PAGE_SIZE;
        page_address %= MT25_FLASH_MEMORY_SIZE;
        last_page_header = page_header;

    }

    return err;
}

int32_t storage_init(bool format)
{
    int32_t err = 0;
    write_buf_idx = 0;
    bool corrupt = false;    

    for(uint8_t i = 0; i < n_page_headers; i++)
    {
        for(uint8_t j = 0; j < n_page_headers; j++)
        {
            page_header_transitions[i][j] = 0;
        }
    }

    if(format)
    {
        storage_format();
    }
    else
    {
        err = storage_parse(&corrupt);        
    }

    if(!err)
    {
        if(num_epoch_files == 0)
        {
            first_data_page_address = first_free_page_address;
        }
        NRF_LOG_INFO("          Erased   Start    Cont    Used"); NRF_LOG_PROCESS();
        NRF_LOG_INFO("========================================"); NRF_LOG_PROCESS();
        NRF_LOG_INFO("Erased   %7d %7d %7d %7d", 
            page_header_transitions[page_header_erased][page_header_erased], 
            page_header_transitions[page_header_erased][page_header_start], 
            page_header_transitions[page_header_erased][page_header_continuation], 
            page_header_transitions[page_header_erased][page_header_used]
        ); NRF_LOG_PROCESS();
        NRF_LOG_INFO("Start    %7d %7d %7d %7d",
            page_header_transitions[page_header_start][page_header_erased], 
            page_header_transitions[page_header_start][page_header_start], 
            page_header_transitions[page_header_start][page_header_continuation], 
            page_header_transitions[page_header_start][page_header_used]
        ); NRF_LOG_PROCESS();
        NRF_LOG_INFO("Cont     %7d %7d %7d %7d",
            page_header_transitions[page_header_continuation][page_header_erased], 
            page_header_transitions[page_header_continuation][page_header_start], 
            page_header_transitions[page_header_continuation][page_header_continuation], 
            page_header_transitions[page_header_continuation][page_header_used]
        ); NRF_LOG_PROCESS();
        NRF_LOG_INFO("Used     %7d %7d %7d %7d", 
            page_header_transitions[page_header_used][page_header_erased], 
            page_header_transitions[page_header_used][page_header_start], 
            page_header_transitions[page_header_used][page_header_continuation], 
            page_header_transitions[page_header_used][page_header_used]
        ); NRF_LOG_PROCESS();
        NRF_LOG_INFO("First data page: 0x%08x", first_data_page_address); NRF_LOG_PROCESS();
        NRF_LOG_INFO("First free page: 0x%08x", first_free_page_address); NRF_LOG_PROCESS();

        // Try to spot an empty filesystem which just has some used pages which need to be erased
        // if(
        //     (page_header_transitions[page_header_erased][page_header_used] == 1) &&
        //     (page_header_transitions[page_header_used][page_header_erased] == 1) &&
        //     storage_num_epoch_files == 0
        // ) 

        if(page_header_transitions[page_header_erased][page_header_used] >= 1 && page_header_transitions[page_header_erased][page_header_start] >= 1)
        {
            NRF_LOG_WARNING("Both erased to used and erased to start transitions found"); NRF_LOG_PROCESS();
            corrupt = true;
        }
        if(page_header_transitions[page_header_continuation][page_header_erased] >= 1 && page_header_transitions[page_header_used][page_header_erased] >= 1)
        {
            NRF_LOG_WARNING("Both continuation to erased and used to erased transitions found"); NRF_LOG_PROCESS();
            corrupt = true;
        }
        
        if(page_header_transitions[page_header_erased][page_header_used] == 0)
        {
            erase_start_address = first_data_page_address;
            NRF_LOG_INFO("No used sectors. Nothing to erase"); NRF_LOG_PROCESS();
        }
        else
        {
            NRF_LOG_INFO("First sector to erase: 0x%08x", erase_start_address); NRF_LOG_PROCESS();
        }

        NRF_LOG_INFO("Found %d epoch files", num_epoch_files); NRF_LOG_PROCESS();

        if(corrupt)
        {
            NRF_LOG_WARNING("Filesystem corrupt"); NRF_LOG_PROCESS();     
            NRF_LOG_WARNING("Formatting Filesystem"); NRF_LOG_PROCESS();
            storage_format();
        }
        else
        {
            if(num_epoch_files != 0)
            {
                uint32_t page_address = first_free_page_address;                
                do
                {
                    page_address += (MT25_FLASH_MEMORY_SIZE - MT25_PAGE_SIZE);
                    page_address %= (MT25_FLASH_MEMORY_SIZE);
                    err = flash_block_read(page_address, read_buf, 1 + sizeof(epoch_file_header_t) + sizeof(header_section_t));
                    if(read_buf[0] == PAGE_HEADER_EPOCH_START)
                    {
                        header_section_t *header = (header_section_t *)(read_buf + 1 + sizeof(epoch_file_header_t));
                        next_epoch_id = header->epoch_id + 1;
                        NRF_LOG_INFO("Next_epoch_id: %d", next_epoch_id); NRF_LOG_PROCESS();
                    }
                    
                } while (read_buf[0] != PAGE_HEADER_EPOCH_START);
                
            }
        }
    }

    // storage_blank_check(0, MT25_FLASH_MEMORY_SIZE);
    // storage_integrity_check();

    return err;

}

int32_t storage_open_head(uint32_t size)
{
    int32_t err = 0;

    epoch_file_header_t header = {
        .size = size
    };

    write_buf_idx = 0;

    write_buf[write_buf_idx++] = PAGE_HEADER_EPOCH_START; 
    memcpy(write_buf + write_buf_idx, (uint8_t *)&header, sizeof(epoch_file_header_t));
    write_buf_idx += sizeof(epoch_file_header_t);
    NRF_LOG_INFO("Opened head file at 0x%08x ", first_free_page_address);  

    return err;
}

int32_t storage_write_head(uint8_t *data, uint32_t length, uint32_t *bytes_written)
{
    int32_t err = 0;

    int32_t remaining = length;
    uint8_t *source = data;
    *bytes_written = 0;

    while(!err && remaining)
    {
        uint32_t tx_length;
        if(remaining >= MT25_PAGE_SIZE - write_buf_idx)
        {
            tx_length = MT25_PAGE_SIZE - write_buf_idx;
            memcpy(write_buf + write_buf_idx, source, tx_length);
            *bytes_written += tx_length; 
            NRF_LOG_DEBUG("Transferred %d bytes", tx_length);           
            source += tx_length;
            err = flash_block_write(first_free_page_address, write_buf, MT25_PAGE_SIZE);
            NRF_LOG_DEBUG("Wrote page to flash at 0x%08x", first_free_page_address);
            write_buf_idx = 0;
            write_buf[write_buf_idx++] = PAGE_HEADER_EPOCH_CONTINUATION;
            first_free_page_address += MT25_PAGE_SIZE;
            first_free_page_address %= (MT25_FLASH_MEMORY_SIZE);
        }
        else
        {
            tx_length = remaining;
            memcpy(write_buf + write_buf_idx, source, tx_length);
            NRF_LOG_DEBUG("Transferred %d bytes", tx_length);
            *bytes_written += tx_length;
            write_buf_idx += tx_length;            
        }
        remaining -= tx_length;        
    }

    return err;
}

int32_t storage_close_head(void)
{
    int32_t err = 0;

    // Check for the case in which write_buf only contains the page header.
    // If this is the case, don't write the page out,
    // as it won't be read back by storage_read_tail()
    if(write_buf_idx > 1)
    {
        err = flash_block_write(first_free_page_address, write_buf, write_buf_idx);
        NRF_LOG_INFO("Wrote final page to flash at 0x%08x, size = %d", first_free_page_address, write_buf_idx);
        first_free_page_address += MT25_PAGE_SIZE;
        first_free_page_address %= (MT25_FLASH_MEMORY_SIZE);
        write_buf_idx = 0;
    }
    if(!err)
    {
        num_epoch_files++;
        NRF_LOG_INFO("num_epoch_files = %d", num_epoch_files);
    }

    return err;
}

int32_t storage_open_tail(uint32_t *size)
{
    int32_t err = 0;

    read_buf_idx = 0;
    epoch_file_header_t header;

    tail_epoch_start_address = first_data_page_address;
    err = flash_block_read(first_data_page_address, read_buf, MT25_PAGE_SIZE);
    tail_epoch_start_address += MT25_PAGE_SIZE;
    tail_epoch_start_address %= (MT25_FLASH_MEMORY_SIZE);
    if(read_buf[0] != PAGE_HEADER_EPOCH_START)
    {
        NRF_LOG_WARNING("Tail file at 0x%08x starts with %d", first_data_page_address, read_buf[0]);
        err = STORAGE_ERROR_TAIL_FILE_START_HEADER_WRONG;
    }
    if(!err)
    {
        NRF_LOG_INFO("Opened tail file at 0x%08x", first_data_page_address);
        read_buf_idx += 1;
        memcpy((uint8_t *)&header, read_buf + read_buf_idx, sizeof(epoch_file_header_t));
        *size = header.size;
        read_buf_idx += sizeof(epoch_file_header_t);
    }

    return err;
}

int32_t storage_read_tail(uint8_t *data, uint32_t length, uint32_t *bytes_read)
{
    int32_t err = 0;

    uint32_t remaining = length;
    uint32_t tx_length;
    uint8_t *dest = data;
    *bytes_read = 0;

    while(remaining && !err)
    {
        if(remaining > MT25_PAGE_SIZE - read_buf_idx)
        {
            tx_length = MT25_PAGE_SIZE - read_buf_idx;            
            memcpy(dest, read_buf + read_buf_idx, tx_length);
            NRF_LOG_DEBUG("Transferred %d bytes to 0x%08x", tx_length, dest);
            *bytes_read += tx_length; 
            err = flash_block_read(tail_epoch_start_address, read_buf, MT25_PAGE_SIZE);
            NRF_LOG_DEBUG("Read %d bytes from 0x%08x", MT25_PAGE_SIZE, tail_epoch_start_address);
            if(read_buf[0] != PAGE_HEADER_EPOCH_CONTINUATION)
            {
                err = STORAGE_ERROR_TAIL_FILE_CONTINUATION_HEADER_WRONG;
                NRF_LOG_WARNING("unexpected page header %d at 0x%08x", read_buf[0], tail_epoch_start_address);
            }                           
            tail_epoch_start_address += MT25_PAGE_SIZE;
            tail_epoch_start_address %= (MT25_FLASH_MEMORY_SIZE);
            read_buf_idx = 1;
            dest += tx_length;                   
        }
        else
        {
            tx_length = remaining;
            memcpy(dest, read_buf + read_buf_idx, tx_length);
            read_buf_idx += tx_length;
            NRF_LOG_DEBUG("Transferred %d bytes to 0x%08x", tx_length, dest);
            *bytes_read += tx_length;
        }
        remaining -= tx_length;            
    }

    return err;
}

int32_t storage_close_tail(void)
{
    int32_t err = 0;
    read_buf_idx = 0;
    return err;
}

int32_t storage_delete_tail_epoch(void)
{
    int32_t err = 0;

    uint8_t page_header = PAGE_HEADER_USED;
    for(
        uint32_t erase_page_address = first_data_page_address;
        !err && erase_page_address != tail_epoch_start_address;
        erase_page_address += MT25_PAGE_SIZE
    )
    {
        erase_page_address %= (MT25_FLASH_MEMORY_SIZE);
        err = flash_block_write(erase_page_address, &page_header, 1);
    }
    if(!err)
    {
        first_data_page_address = tail_epoch_start_address;
        num_epoch_files--;
        NRF_LOG_INFO("num_epoch_files = %d", num_epoch_files);
    }

    return err;
}

bool storage_empty(void)
{
    return first_data_page_address == first_free_page_address;
}

int32_t storage_blocks_used(uint32_t *blocks_used)
{
    int32_t err = 0;
    if(first_free_page_address >= erase_start_address)
    {
        *blocks_used = (first_free_page_address - erase_start_address) / MT25_4KB_SUBSECTOR_SIZE;
    }
    else
    {
        *blocks_used = (first_free_page_address + MT25_FLASH_MEMORY_SIZE - erase_start_address) / MT25_4KB_SUBSECTOR_SIZE;
    }
    return err;
}

int32_t storage_num_epoch_files(uint32_t *num_files)
{
    int32_t err = 0;
    *num_files =  num_epoch_files;
    return err;
}

int32_t storage_clear_fs(void)
{
    int32_t err = 0;
    storage_format();
    return err;
}

int32_t storage_can_fit_epoch(bool *can_fit_epoch)
{
    int32_t err = 0;

    uint32_t num_4kb_subsectors_used;
    err = storage_blocks_used(&num_4kb_subsectors_used);
    uint32_t num_4kb_subsectors_remaining = MT25_4KB_SUBSECTOR_COUNT - num_4kb_subsectors_used;
    *can_fit_epoch = ((num_4kb_subsectors_remaining - 2) * 4 * 1024 > MAX_EPOCH_FILE_SIZE);
    return err;
}

int32_t storage_get_next_epoch_id(uint16_t *epoch_id)
{
    int32_t err = 0;
    *epoch_id = next_epoch_id;
    next_epoch_id += 1;
    return err;
}

int32_t storage_garbage_collector(void)
{
    // TODO: this gets called multiple times for the same sector
    // TODO: there is no way currently to know that the erase has completed
    int32_t err = 0;
    bool in_progress;
#ifdef GARBAGE_BLANK_CHECK
    static uint32_t last_garbage_collection_address = 0;
#endif

    if((first_data_page_address / MT25_64KB_SECTOR_SIZE) != (erase_start_address / MT25_64KB_SECTOR_SIZE))
    {
        err = flash_erase_in_progress(&in_progress);
        if(!err)
        {
            if(!in_progress)
            {
#ifdef GARBAGE_BLANK_CHECK
                NRF_LOG_INFO("Blank check at 0x%08x", last_garbage_collection_address);
                storage_blank_check(last_garbage_collection_address, last_garbage_collection_address + MT25_64KB_SECTOR_SIZE);
                last_garbage_collection_address = erase_start_address;
                PT10_LOG_FLUSH();
                NRF_LOG_INFO("Blank check at 0x%08x", last_garbage_collection_address);
                storage_blank_check(last_garbage_collection_address, last_garbage_collection_address + MT25_64KB_SECTOR_SIZE);
                PT10_LOG_FLUSH();
#endif
                NRF_LOG_INFO("Garbage collecting at 0x%08x", erase_start_address);
#ifdef GARBAGE_BLANK_CHECK
                PT10_LOG_FLUSH();
#endif
                err = flash_erase_sector(erase_start_address);
                erase_start_address += MT25_64KB_SECTOR_SIZE;
                erase_start_address %= (MT25_FLASH_MEMORY_SIZE);
            }
        }       
    }
    return err;
}