#define NRF_LOG_MODULE_NAME esb
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL
// #define NRF_LOG_LEVEL 4 // (4 = Debug)

#include "nrf_esb.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "board_config.h"
#include "hal.h"
#include "esb.h"
#include "app_timer.h"
#include "pt10.h"
#include "internal_flash.h"
#include "oscilltrack.h"

NRF_LOG_MODULE_REGISTER();

APP_TIMER_DEF(esb_timer);

static nrf_esb_payload_t        tx_payload_ping = NRF_ESB_CREATE_PAYLOAD(
    ESB_PIPE_BRIDGE, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00
);
static nrf_esb_payload_t        tx_payload_command_response = NRF_ESB_CREATE_PAYLOAD(
    ESB_PIPE_BRIDGE, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00
);
static nrf_esb_payload_t        tx_payload_oscilltrack = NRF_ESB_CREATE_PAYLOAD(
    ESB_PIPE_BRIDGE, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00
);
static nrf_esb_payload_t        tx_payload_vts_stimulus = NRF_ESB_CREATE_PAYLOAD(
    ESB_PIPE_VTS, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00  
);
static nrf_esb_payload_t        tx_payload_vts_command = NRF_ESB_CREATE_PAYLOAD(
    ESB_PIPE_VTS, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,  
    0x00, 0x00, 0x00, 0x00 
);
static nrf_esb_payload_t        rx_payload;

esb_status_t    esb_status;
esb_payload_type_e payload_written;

void esb_timer_handler(void *p_context)
{
    NRF_LOG_DEBUG("ESB Timer elapsed %d", (uint32_t)p_context);
    esb_status.ping_timeout = true;
}

void esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
    case NRF_ESB_EVENT_TX_SUCCESS:    
#ifdef ESB_PIN_DEBUG    
        hal_gpio_set(PIN_RTC_EVI);
#endif
        NRF_LOG_DEBUG("TX SUCCESS. Attempts: %d", p_event->tx_attempts);
        esb_status.tx_done = true;
        break;
    case NRF_ESB_EVENT_TX_FAILED:
        esb_status.tx_done = true;        
        NRF_LOG_INFO("TX FAILED: type=%d, attempts=%d", payload_written, p_event->tx_attempts);        
        (void) nrf_esb_flush_tx();
        (void) nrf_esb_start_tx();
        break;
    case NRF_ESB_EVENT_RX_RECEIVED:
        // NRF_LOG_INFO("RX RECEIVED EVENT");
        // NRF_LOG_HEXDUMP_INFO(rx_payload.data, rx_payload.length);
        while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
        {
            if (rx_payload.length > 0)
            {
                // NRF_LOG_DEBUG("RX RECEIVED PAYLOAD");
                // NRF_LOG_HEXDUMP_DEBUG(rx_payload.data, rx_payload.length);
                esb_packet_header_t *header = (esb_packet_header_t *)rx_payload.data;
                switch(header->payload_type)
                {
                    case esb_payload_command:

                        esb_packet_header_t *response_header = (esb_packet_header_t *)tx_payload_command_response.data;
                        response_header->payload_type = esb_payload_command_response;
                        response_header->payload_length = 5;                        
                        
                        uint8_t *command = (uint8_t *)rx_payload.data + sizeof(esb_packet_header_t);
                        uint8_t *response = (uint8_t *)tx_payload_command_response.data + sizeof(esb_packet_header_t);
                        
                        // NRF_LOG_INFO("Command");
                        // NRF_LOG_HEXDUMP_INFO(command, 5);
                        pt10_cmd_processor(command, response);
                        tx_payload_command_response.pipe = ESB_PIPE_BRIDGE;
                        // tx_payload_command_response.pid += 1;
                        tx_payload_command_response.length = sizeof(esb_packet_header_t) + 5;

                        // NRF_LOG_INFO("Response");
                        // NRF_LOG_HEXDUMP_INFO(response, 5);
                        esb_status.command_response_ready = true; 
                        
                        break;
                    
                    case esb_payload_ping_response:
                        // NRF_LOG_INFO("Ping Response");
                        esb_status.ping_response = true;

                        break;
                    default:
                        // NRF_LOG_INFO("Other Rx Payload");
                        break;
                }
            }
        }
        break;
    }
}

uint8_t p_base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
uint8_t p_base_addr_1[4] = {0xCC, 0xCC, 0xCC, 0xCC};
uint8_t p_addr_prefix[8] = {0xE7, 0xCC, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
uint8_t channel = 17;

int32_t esb_init(void)
{
    int32_t err_code = 0;

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 250;
    nrf_esb_config.retransmit_count         = 1;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_1MBPS;
    nrf_esb_config.event_handler            = esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = true;
    nrf_esb_config.tx_output_power          = NRF_ESB_TX_POWER_8DBM;

    err_code = nrf_esb_init(&nrf_esb_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_base_address_0(p_base_addr_0);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_base_address_1(p_base_addr_1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_prefixes(p_addr_prefix, 8);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_rf_channel(channel);
    APP_ERROR_CHECK(err_code);

    app_timer_create(&esb_timer, APP_TIMER_MODE_SINGLE_SHOT, esb_timer_handler);    

    esb_status.state = esb_state_reset;
    esb_status.stimulus_ready = false;
    esb_status.oscilltrack_data_ready = false;
    esb_status.vts_command_ready = false;
    esb_status.ping_timeout = false;
    esb_status.ping_response = false;
    esb_status.command_response_ready = false;

    return err_code;
}

int32_t esb_set_stimulus(void)
{
    int32_t err = 0;
    tx_payload_vts_stimulus.pipe = 0;        
    tx_payload_vts_stimulus.length = 1;
    tx_payload_vts_stimulus.data[0] = 0;
    esb_status.stimulus_ready = true;
    return err;
}

int32_t esb_set_oscilltrack_data(uint8_t *data, uint8_t length)
{
    int32_t err = 0;

    if(length > (32 - sizeof(esb_packet_header_t)))
    {
        NRF_LOG_WARNING("Truncating oscilltrack packet");
        length = 32 - sizeof(esb_packet_header_t);
    }

    esb_packet_t *packet = (esb_packet_t *)tx_payload_oscilltrack.data;
    packet->header.payload_type = esb_payload_oscilltrack_data;
    packet->header.payload_length = length;
    memcpy((uint8_t *)(tx_payload_oscilltrack.data + sizeof(esb_packet_header_t)), data, length);
    tx_payload_oscilltrack.length = sizeof(esb_packet_header_t) + length;
    tx_payload_oscilltrack.pipe = ESB_PIPE_BRIDGE;
    // tx_payload_oscilltrack.pid += 1;
    esb_status.oscilltrack_data_ready = true;
    return err;

}

void esb_configure_vts_haptic(uint8_t *haptic_parameters)
{
    tx_payload_vts_command.data[0] = vts_command_configure_haptic;
    memcpy(tx_payload_vts_command.data+1, haptic_parameters, 4);
    tx_payload_vts_command.length = 5;
    esb_status.vts_command_ready = true;
}

void esb_configure_vts_sleep(uint8_t *sleep_parameters)
{
    tx_payload_vts_command.data[0] = vts_command_configure_sleep;
    memcpy(tx_payload_vts_command.data+1, sleep_parameters, 4);
    tx_payload_vts_command.length = 5;
    esb_status.vts_command_ready = true;
}

int32_t esb_create_ping_packet(void)
{
    int32_t err = 0;
    
    esb_packet_t *packet = (esb_packet_t *)tx_payload_ping.data;
    packet->header.payload_type = esb_payload_ping;
    packet->header.payload_length = 0;
    tx_payload_ping.pipe = ESB_PIPE_BRIDGE;
    tx_payload_ping.length = sizeof(esb_packet_header_t);
    

    return err;
}

int32_t err;
int32_t esb_update(void)
{
    int32_t err = 0;

    static uint32_t last_count = 0;

    uint32_t this_count = app_timer_cnt_get();
    uint32_t diff = app_timer_cnt_diff_compute(this_count, last_count);
    if(diff < 1)
    {
        return 0;
    }
    last_count = this_count;

    esb_state_e next_state = esb_status.state;
    switch(esb_status.state)
    {
    case esb_state_reset:
        esb_status.ping_timeout = false;
        app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
        next_state = esb_state_unconnected;
        break;

    case esb_state_unconnected:

        if(esb_status.stimulus_ready == true && nrf_esb_is_idle())
        {
            esb_status.tx_done = false;
            NRF_LOG_DEBUG("Send Stimulus");
            tx_payload_vts_stimulus.pipe = ESB_PIPE_VTS;
            tx_payload_vts_stimulus.data[0] = vts_command_send_haptic;
            tx_payload_vts_stimulus.length = 1;
            // tx_payload_vts_stimulus.pid += 1;
            err = nrf_esb_write_payload(&tx_payload_vts_stimulus);
            payload_written = esb_payload_stimulus;
            APP_ERROR_CHECK(err);
            esb_status.stimulus_ready = false;
            next_state = esb_state_unconnected_wait_tx_done;
        }
        else if(esb_status.ping_timeout == true && nrf_esb_is_idle())
        {
            esb_create_ping_packet();
            esb_status.ping_timeout = false;
            err = app_timer_stop(esb_timer);
            APP_ERROR_CHECK(err);
            err = app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
            APP_ERROR_CHECK(err);

            NRF_LOG_DEBUG("Send Ping");            
            esb_status.tx_done = false;
            err = nrf_esb_write_payload(&tx_payload_ping);
            payload_written = esb_payload_ping;
            APP_ERROR_CHECK(err);
            esb_status.ping_response = false;
            next_state = esb_state_unconnected_wait_tx_done;            
        }
        else if(esb_status.ping_response == true)
        {
            NRF_LOG_INFO("Connected to Bridge");
            esb_status.ping_timeout = false;
            err = app_timer_stop(esb_timer);
            APP_ERROR_CHECK(err);
            err = app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
            APP_ERROR_CHECK(err);
            esb_status.oscilltrack_data_ready = false;
            next_state = esb_state_connected;
        }
        break;

    case esb_state_unconnected_wait_tx_done:
        if(esb_status.tx_done == true)
        {
            esb_status.tx_done = false;
            next_state = esb_state_unconnected;
        }
        if(esb_status.ping_timeout == true)
        {
            // This shouldn't happen. But if we hang here, the ping timeout should save us
            next_state = esb_state_reset;
        }
        break;

    case esb_state_connected:

        // if(esb_status.stimulus_ready == true && nrf_esb_is_idle())
        if(esb_status.stimulus_ready)
        {
            tx_payload_vts_stimulus.pipe = ESB_PIPE_VTS;
            tx_payload_vts_stimulus.data[0] = vts_command_send_haptic;
            tx_payload_vts_stimulus.length = 1;
            // tx_payload_vts_stimulus.pid += 1;
            esb_status.tx_done = false; 
#ifdef ESB_PIN_DEBUG
            hal_gpio_clear(PIN_SPI1_nCS);
#endif
            err = nrf_esb_write_payload(&tx_payload_vts_stimulus);
            payload_written = esb_payload_stimulus;
            APP_ERROR_CHECK(err);
            esb_status.stimulus_ready = false;            
            next_state = esb_state_connected_wait_tx_done;
        }
        // else if(esb_status.command_response_ready == true && nrf_esb_is_idle())
        else if(esb_status.command_response_ready)
        {
            err = app_timer_stop(esb_timer);
            APP_ERROR_CHECK(err);
            err = app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);  
            APP_ERROR_CHECK(err);  

            NRF_LOG_DEBUG("Send Command Response");
            esb_status.tx_done = false;
#ifdef ESB_PIN_DEBUG
            hal_gpio_clear(PIN_SPI1_CLK);
#endif  
            err = nrf_esb_write_payload(&tx_payload_command_response);
            payload_written = esb_payload_command_response;
            APP_ERROR_CHECK(err);
            esb_status.command_response_ready = false;
            next_state = esb_state_connected_wait_tx_done;
        }
        // else if(esb_status.vts_command_ready == true && nrf_esb_is_idle())
        else if(esb_status.vts_command_ready)
        {
            NRF_LOG_DEBUG("Send VTS Command");
            esb_status.tx_done = false;
            err = nrf_esb_write_payload(&tx_payload_vts_command);
            payload_written = esb_payload_vts_command;
            APP_ERROR_CHECK(err);
            esb_status.vts_command_ready = false;
            next_state = esb_state_connected_wait_tx_done;
        }
        // else if(esb_status.oscilltrack_data_ready == true && nrf_esb_is_idle())
        else if(esb_status.oscilltrack_data_ready)
        {
            NRF_LOG_DEBUG("Send Oscilltrack Data");
            esb_status.tx_done = false;
#ifdef ESB_PIN_DEBUG
            hal_gpio_clear(PIN_SPI1_SDO);
#endif
            err = nrf_esb_write_payload(&tx_payload_oscilltrack);
            payload_written = esb_payload_oscilltrack_data;
            APP_ERROR_CHECK(err);
            esb_status.oscilltrack_data_ready = false;
            next_state = esb_state_connected_wait_tx_done;
        }        
        // else if(esb_status.ping_timeout == true && nrf_esb_is_idle())
        else if(esb_status.ping_timeout)
        {
            NRF_LOG_INFO("Disconnected from Bridge");
            NRF_LOG_DEBUG("Send Ping");
            esb_create_ping_packet();            
            esb_status.ping_timeout = false;
            err = app_timer_stop(esb_timer);
            APP_ERROR_CHECK(err);
            err = app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);  
            APP_ERROR_CHECK(err);
            err = nrf_esb_write_payload(&tx_payload_ping);
            payload_written = esb_payload_ping;
            APP_ERROR_CHECK(err);
            
            esb_status.command_response_ready = false;
            next_state = esb_state_unconnected;
        }
        else if(esb_status.ping_response == true)
        {
            NRF_LOG_DEBUG("Ping response");
            esb_status.ping_response = false;
            esb_status.ping_timeout = false;
            err = app_timer_stop(esb_timer);
            APP_ERROR_CHECK(err);
            err = app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);  
            APP_ERROR_CHECK(err);  
        }
        break;

    case esb_state_connected_wait_tx_done:
        if(esb_status.tx_done == true)
        {
            // esb_status.tx_done = false;
#ifdef ESB_PIN_DEBUG
            hal_gpio_clear(PIN_RTC_EVI);
            hal_gpio_set(PIN_SPI1_nCS);
            hal_gpio_set(PIN_SPI1_SDO);
            hal_gpio_set(PIN_SPI1_CLK);
#endif
            next_state = esb_state_connected;
        }
        if(esb_status.ping_timeout == true)
        {
            // This shouldn't happen. But if we hang here, the ping timeout should save us
            next_state = esb_state_reset;
        }
        break;

    }

    
    if(esb_status.state != next_state)
    {
        NRF_LOG_DEBUG("state %d -> %d", esb_status.state, next_state);
        esb_status.state = next_state;        
    }

    return err;
}
