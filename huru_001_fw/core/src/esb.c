#define NRF_LOG_MODULE_NAME esb
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL
// #define NRF_LOG_LEVEL 4 // (4 = Debug)

#include "nrf_esb.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
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
static nrf_esb_payload_t        tx_payload_vts = NRF_ESB_CREATE_PAYLOAD(
    ESB_PIPE_VTS, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00  
);
static nrf_esb_payload_t        rx_payload;

esb_status_t    esb_status;
prx_peripheral_e m_current_peripheral;

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
        if(m_current_peripheral == prx_peripheral_stimulator)
        {
            NRF_LOG_INFO("TX SUCCESS EVENT");
        }
        else
        {
            NRF_LOG_DEBUG("TX SUCCESS EVENT");
        }
        esb_status.tx_done = true;
        break;
    case NRF_ESB_EVENT_TX_FAILED:
        esb_status.tx_done = true;
        if(m_current_peripheral == prx_peripheral_stimulator)
        {
            NRF_LOG_INFO("TX FAILED EVENT");
        }
        else
        {
            NRF_LOG_DEBUG("TX FAILED EVENT");
        }
        (void) nrf_esb_flush_tx();
        (void) nrf_esb_start_tx();
        break;
    case NRF_ESB_EVENT_RX_RECEIVED:
        NRF_LOG_DEBUG("RX RECEIVED EVENT");
        while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
        {
            if (rx_payload.length > 0)
            {
                NRF_LOG_DEBUG("RX RECEIVED PAYLOAD");
                NRF_LOG_HEXDUMP_DEBUG(rx_payload.data, rx_payload.length);
                esb_packet_header_t *header = (esb_packet_header_t *)rx_payload.data;
                switch(header->payload_type)
                {
                    case esb_payload_command:
                    
                        esb_packet_header_t *response_header = (esb_packet_header_t *)tx_payload_command_response.data;
                        response_header->payload_type = esb_payload_command_response;
                        response_header->payload_length = 5;                        
                        
                        uint8_t *command = (uint8_t *)rx_payload.data + sizeof(esb_packet_header_t);
                        uint8_t *response = (uint8_t *)tx_payload_command_response.data + sizeof(esb_packet_header_t);
                        
                        NRF_LOG_INFO("Command");
                        NRF_LOG_HEXDUMP_INFO(command, 5);
                        pt10_cmd_processor(command, response);
                        tx_payload_command_response.length = sizeof(esb_packet_header_t) + 5;

                        NRF_LOG_INFO("Response");
                        NRF_LOG_HEXDUMP_INFO(response, 5);
                        esb_status.command_response_ready = true;                            
                    
                        break;
                    
                    case esb_payload_ping_response:
                        esb_status.ping_response = true;
                        break;
                }
            }
        }
        break;
    }
}

uint8_t p_base_addr_0[NUM_PRX_PERIPHERALS][4] = {
    {0xE7, 0xE7, 0xE7, 0xE7},
    {0x31, 0x31, 0x31, 0x31}
};
uint8_t p_base_addr_1[NUM_PRX_PERIPHERALS][4] = {
    {0xC2, 0xC2, 0xC2, 0xC2},
    {0x32, 0x32, 0x32, 0x32}
};
uint8_t p_addr_prefix[NUM_PRX_PERIPHERALS][8] = {
    {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 },
    {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38 }
};
uint8_t channels[NUM_PRX_PERIPHERALS] = {
    17, 
    17
};


int32_t esb_set_prx(prx_peripheral_e prx_peripheral)
{
    m_current_peripheral = prx_peripheral;

    uint32_t err_code = 0;

    err_code = nrf_esb_set_base_address_0(p_base_addr_0[prx_peripheral]);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_base_address_1(p_base_addr_1[prx_peripheral]);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_prefixes(p_addr_prefix[prx_peripheral], NRF_ESB_PIPE_COUNT);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_esb_set_rf_channel(channels[prx_peripheral]);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

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

    esb_set_prx(prx_peripheral_usb_bridge);

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
    tx_payload_vts.length = 1;
    tx_payload_vts.data[0] = 0;
    esb_status.stimulus_ready = true;
    return err;
}

int32_t esb_set_oscilltrack_data(uint8_t *data, uint8_t length)
{
    int32_t err = 0;

    esb_packet_t *packet = (esb_packet_t *)tx_payload_oscilltrack.data;
    packet->header.payload_type = esb_payload_oscilltrack_data;
    packet->header.payload_length = length;
    memcpy((uint8_t *)(tx_payload_oscilltrack.data + sizeof(esb_packet_header_t)), data, length);
    tx_payload_oscilltrack.length = sizeof(esb_packet_header_t) + length;
    esb_status.oscilltrack_data_ready = true;
    return err;

}

void esb_configure_vts_haptic(uint8_t *haptic_parameters)
{
    tx_payload_vts.data[0] = vts_command_configure_haptic;
    memcpy(tx_payload_vts.data+1, haptic_parameters, 4);
    tx_payload_vts.length = 5;
    esb_status.vts_command_ready = true;
}

void esb_configure_vts_sleep(uint8_t *sleep_parameters)
{
    tx_payload_vts.data[0] = vts_command_configure_sleep;
    memcpy(tx_payload_vts.data+1, sleep_parameters, 4);
    tx_payload_vts.length = 5;
    esb_status.vts_command_ready = true;
}

int32_t esb_send_ping_packet(void)
{
    int32_t err = 0;
    
    esb_packet_t *packet = (esb_packet_t *)tx_payload_ping.data;
    packet->header.payload_type = esb_payload_ping;
    packet->header.payload_length = 0;
    tx_payload_ping.length = sizeof(esb_packet_header_t);
    esb_status.tx_done = false;
    esb_set_prx(prx_peripheral_usb_bridge); 
    nrf_esb_write_payload(&tx_payload_ping);

    return err;
}

int32_t esb_update(void)
{
    int32_t err = 0;

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
            NRF_LOG_INFO("Send Stimulus");             
            esb_set_prx(prx_peripheral_stimulator);
            tx_payload_vts.data[0] = vts_command_send_haptic;
            tx_payload_vts.length = 1;
            nrf_esb_write_payload(&tx_payload_vts);
            esb_status.stimulus_ready = false;
            next_state = esb_state_unconnected_wait_tx_done;
        }
        else if(esb_status.ping_timeout == true && nrf_esb_is_idle())
        {
            NRF_LOG_DEBUG("Send Ping");
            esb_send_ping_packet();
            esb_status.ping_timeout = false;
            app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
            esb_status.ping_response = false;
            next_state = esb_state_unconnected_wait_tx_done;            
        }
        else if(esb_status.ping_response == true)
        {
            NRF_LOG_INFO("Connected to Bridge");
            esb_status.ping_timeout = false;
            app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
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
        if(esb_status.stimulus_ready == true && nrf_esb_is_idle())
        {
            NRF_LOG_INFO("Send Stimulus");
            esb_set_prx(prx_peripheral_stimulator); 
            tx_payload_vts.data[0] = vts_command_send_haptic;
            tx_payload_vts.length = 1;   
            nrf_esb_write_payload(&tx_payload_vts);
            esb_status.stimulus_ready = false;
            next_state = esb_state_connected_wait_tx_done;
        }
        else if(esb_status.command_response_ready == true && nrf_esb_is_idle())
        {
            NRF_LOG_DEBUG("Send Command Response");
            esb_status.tx_done = false;
            esb_set_prx(prx_peripheral_usb_bridge);        
            nrf_esb_write_payload(&tx_payload_command_response);
            esb_status.command_response_ready = false;
            next_state = esb_state_connected_wait_tx_done;
        }
        else if(esb_status.vts_command_ready == true && nrf_esb_is_idle())
        {
            NRF_LOG_DEBUG("Send VTS Command");
            nrf_esb_write_payload(&tx_payload_vts);
            esb_status.vts_command_ready = false;
            next_state = esb_state_connected_wait_tx_done;
        }
        else if(esb_status.oscilltrack_data_ready == true && nrf_esb_is_idle())
        {
            NRF_LOG_DEBUG("Send Oscilltrack Data");
            esb_status.tx_done = false;    
            esb_set_prx(prx_peripheral_usb_bridge);        
            nrf_esb_write_payload(&tx_payload_oscilltrack);
            esb_status.oscilltrack_data_ready = false;
            next_state = esb_state_connected_wait_tx_done;
        }        
        else if(esb_status.ping_timeout == true && nrf_esb_is_idle())
        {
            NRF_LOG_INFO("Disconnected from Bridge");
            NRF_LOG_DEBUG("Send Ping");
            esb_send_ping_packet();            
            esb_status.ping_timeout = false;
            app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
            esb_status.command_response_ready = false;
            next_state = esb_state_unconnected;
        }
        else if(esb_status.ping_response == true)
        {
            NRF_LOG_DEBUG("Ping response");
            esb_status.ping_response = false;
            app_timer_stop(esb_timer);
            esb_status.ping_timeout = false;
            app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
        }
        break;

    case esb_state_connected_wait_tx_done:
        if(esb_status.tx_done == true)
        {
            esb_status.tx_done = false;
            next_state = esb_state_connected;
        }
        if(esb_status.ping_timeout == true)
        {
            // This shouldn't happen. But if we hang here, the ping timeout should save us
            next_state = esb_state_reset;
        }
        break;

    // case esb_state_ping:
        
    //     next_state = esb_state_await_ping_response; 
    //     esb_timeout = false;
    //     esb_ping_response = false;
    //     app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
    //     break;   

    // case esb_state_await_ping_response:
    //     if(esb_ping_response == true)
    //     {
    //         esb_oscilltrack_data_success = true;
    //         esb_status.connected_to_app = true;
    //         next_state = esb_state_connected_to_app;
    //         oscilltrackSetGain();
    //         app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
    //         esb_ping_response = false;
    //     }
    //     else if(esb_timeout == true)
    //     {
    //         next_state = esb_state_ping;
    //     }
    //     break;

    // case esb_state_connected_to_app:
    //     if(esb_ping_response == true)
    //     {
    //         esb_ping_response = false;
    //         app_timer_stop(esb_timer);
    //         app_timer_start(esb_timer, APP_TIMER_TICKS(ESB_PING_MS), (void *)NULL);
    //     }
    //     else if(esb_timeout == true)
    //     {
    //         esb_status.connected_to_app = false;
    //         next_state = esb_state_ping;
    //     }
    //     break;

    }

    
    if(esb_status.state != next_state)
    {
        NRF_LOG_DEBUG("state %d -> %d", esb_status.state, next_state);
        esb_status.state = next_state;        
    }

    return err;
}
