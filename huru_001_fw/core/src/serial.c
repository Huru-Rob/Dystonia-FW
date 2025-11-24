#define NRF_LOG_MODULE_NAME serial 
#define NRF_LOG_LEVEL CLIC_LOG_DEFAULT_LEVEL
// #define NRF_LOG_LEVEL 4

#include "hal.h"
#include "board_config.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrfx_uarte.h"
#include "nrfx_timer.h"
#include "nrf_gpio.h"
#include "nrf_queue.h"
#include "app_timer.h"
#include "nrf_ppi.h"
#include "serial.h"
#include "cobs.h"
#include "internal_flash.h"
#include "crc8.h"

NRF_LOG_MODULE_REGISTER();

static const nrfx_uarte_t m_uart = NRFX_UARTE_INSTANCE(0);
APP_TIMER_DEF(ping_timer);

uint8_t uarte_rx_buffer[UART_BUF_SIZE];
uint8_t uarte_tx_buffer[UART_BUF_SIZE];

// serial_packet_t tx_packet;
NRF_QUEUE_DEF(serial_packet_t, tx_packet_queue, 4, NRF_QUEUE_MODE_NO_OVERFLOW);
serial_packet_t rx_packet;
uint8_t rx_payload_type;
serial_packet_t response_packet;

uint32_t stream_packet_count = 0;

volatile bool rx_done = false;
volatile bool tx_done = false;
volatile bool uart_error = false;
serial_state_e serial_state = serial_state_idle;
bool m_serial_start = false;
bool m_serial_stop = false;

static const nrfx_timer_t m_uart_byte_counter = NRFX_TIMER_INSTANCE(1);

void uart_callback(nrfx_uarte_event_t const * p_event, void * p_context)
{

	// uint32_t err_code;

    switch (p_event->type)
    {
        case NRFX_UARTE_EVT_TX_DONE:
            NRF_LOG_DEBUG("Tx Done");
            tx_done = true;
            break;

        case NRFX_UARTE_EVT_RX_DONE:
            rx_done = true;
            break;

        case NRFX_UARTE_EVT_ERROR:
            NRF_LOG_ERROR("NRFX_UARTE_EVT_ERROR : 0x%x", p_event->data.error.error_mask);
            uart_error = true;
            // err_code = nrfx_uarte_rx(&m_uart, uarte_rx_buffer, 1);
            // APP_ERROR_CHECK(err_code);
            break;
    }
}

uint32_t ping_timeout_reason = 0;
void ping_timer_handler(void *p_context)
{
    NRF_LOG_DEBUG("Ping Timer elapsed %d", (uint32_t)p_context);
    ping_timeout_reason = (uint32_t)p_context;
}

void serial_init(void)
{
    app_timer_create(&ping_timer, APP_TIMER_MODE_SINGLE_SHOT, ping_timer_handler);
}

void serialStopRx(void)
{
    nrf_ppi_channel_disable(NRF_PPI_CHANNEL0);
    // nrfx_uarte_rx_abort(&m_uart);
	nrfx_uarte_uninit(&m_uart);	

    nrfx_timer_disable(&m_uart_byte_counter);
    nrfx_timer_uninit(&m_uart_byte_counter);

    halSetExternalPinFunction(external_pin_function_default);
}

int32_t serialStartRx(void)
{
    int32_t err = 0;

    nrfx_uarte_config_t config = NRFX_UARTE_DEFAULT_CONFIG;
    config.pselrxd = PIN_RESET;
    config.baudrate = 251658240; // 921600 baud
    err = nrfx_uarte_init(&m_uart, &config, uart_callback);
    nrf_gpio_cfg_input(PIN_RESET, NRF_GPIO_PIN_PULLUP);
    
    nrfx_timer_config_t t_config = NRFX_TIMER_DEFAULT_CONFIG;
    t_config.mode = NRF_TIMER_MODE_COUNTER;
    t_config.bit_width = NRF_TIMER_BIT_WIDTH_16;
    nrfx_timer_init(&m_uart_byte_counter, &t_config, NULL);
    nrfx_timer_enable(&m_uart_byte_counter);

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI_CHANNEL0, 
        (uint32_t)nrfx_uarte_event_address_get(&m_uart, NRF_UARTE_EVENT_RXDRDY),
        (uint32_t)nrfx_timer_task_address_get(&m_uart_byte_counter, NRF_TIMER_TASK_COUNT)
    );
    nrf_ppi_channel_enable(NRF_PPI_CHANNEL0);

    if(!err)
    {
        rx_done = false;
        err = nrfx_uarte_rx(&m_uart, uarte_rx_buffer, UART_BUF_SIZE);
    }

    return err;
}

int32_t serialStartTx(void)
{

    int32_t err = 0;

    nrfx_uarte_config_t config = NRFX_UARTE_DEFAULT_CONFIG;
    config.baudrate = 251658240; // 921600 baud
    config.pseltxd = PIN_RESET;
    err = nrfx_uarte_init(&m_uart, &config, uart_callback);

    return err;
}

void serialStopTx(void)
{
    nrfx_uarte_tx_abort(&m_uart);
    nrfx_uarte_uninit(&m_uart);
    halSetExternalPinFunction(external_pin_function_default);
}

void serialStart(void)
{
    NRF_LOG_INFO("Start");
    m_serial_start = true;
}

void serialStop(void)
{
    NRF_LOG_INFO("Stop");
    m_serial_stop = true;
    app_timer_stop(ping_timer);
    pt10_set_uart_connected(false);
}

void serialRxPacket(uint8_t *rx_payload_type)
{
    response_packet.header.payload_type = serial_payload_none;
    *rx_payload_type = serial_payload_none;
    uint32_t device_id;
    get_device_id(&device_id);

    NRF_LOG_DEBUG("Rx Packet: ID: %d Type: %d Length: %d", 
        rx_packet.header.device_id,
        rx_packet.header.payload_type,
        rx_packet.header.payload_length
        );

    if(rx_packet.header.device_id == 0)
    {
        // This is a request to return our device id.
        response_packet.header.device_id = device_id;
        response_packet.header.payload_type = serial_payload_device_id;
        response_packet.header.payload_length = 0;
        pt10_set_uart_connected(true);
        NRF_LOG_INFO("Device ID requested");
        NRF_LOG_INFO("Uart connection: 1");
        *rx_payload_type = serial_payload_device_id;
    }
    else if(rx_packet.header.device_id != device_id)
    {
        NRF_LOG_WARNING("Incorrect ID: %d", rx_packet.header.device_id);
        *rx_payload_type = serial_payload_none;
    }
    else
    {
        uint8_t *rx_payload = (uint8_t *)&rx_packet + sizeof(serial_packet_header_t); 
        response_packet.header.device_id = device_id;
        response_packet.header.payload_type = serial_payload_none;
        response_packet.header.payload_length = 5;
        uint8_t *tx_payload = (uint8_t *)&response_packet + sizeof(serial_packet_header_t);
        memset(tx_payload, 0, 5);            
        tx_payload[0] = rx_payload[0];            
            
        switch(rx_packet.header.payload_type)
        {
            case serial_payload_command:
                NRF_LOG_INFO("Command");
                NRF_LOG_HEXDUMP_INFO(rx_payload, 5);
                pt10_cmd_processor(rx_payload, tx_payload);
                NRF_LOG_INFO("Response");
                NRF_LOG_HEXDUMP_INFO(tx_payload, 5);
                response_packet.header.payload_type = serial_payload_command_response;
                // app_timer_start(ping_timer, APP_TIMER_TICKS(PING_TIMEOUT_MS), (void *)ping_timeout_cmd);
                *rx_payload_type = serial_payload_command_response;
                break;
            case serial_payload_ping:
                NRF_LOG_DEBUG("received ping packet");
                // app_timer_start(ping_timer, APP_TIMER_TICKS(PING_TIMEOUT_MS), (void *)ping_timeout_ping);
                *rx_payload_type = serial_payload_ping;
                break;
            default:
                *rx_payload_type = serial_payload_none;
                break;
        }        
        
    }
}

int32_t serial_send_epoch_data(uint8_t *data, uint32_t length)
{
    int32_t err = 0;

    serial_packet_t packet;

    uint32_t device_id;
    get_device_id(&device_id);    
    packet.header.device_id = device_id;
    packet.header.payload_type = serial_payload_epoch_data;
    packet.header.payload_length = length;
    memcpy(packet.bytes + sizeof(serial_packet_header_t), data, length);
    
    err = nrf_queue_push(&tx_packet_queue, (void *)&packet);
    if(!err)
    {
        NRF_LOG_DEBUG("Queued %d bytes of epoch data", length + sizeof(serial_packet_header_t));
    }
    return err;
}

int32_t serial_send_stream_data(uint8_t *data, uint32_t length)
{
    int32_t err = 0;

    serial_packet_t packet;

    uint32_t device_id;
    get_device_id(&device_id);    
    packet.header.device_id = device_id;
    packet.header.payload_type = serial_payload_sensor_data;
    packet.header.payload_length = length;
    memcpy(packet.bytes + sizeof(serial_packet_header_t), data, length);
    NRF_LOG_DEBUG("Sending %d bytes of stream data", length + sizeof(serial_packet_header_t));
    err = nrf_queue_push(&tx_packet_queue, (void *)&packet);
    return err;
}

void serialUpdate(void)
{
    
    serial_packet_t tx_packet;
    static uint16_t last_rx_byte_count = 0;

    uint16_t rx_byte_count = nrfx_timer_capture(&m_uart_byte_counter, NRF_TIMER_CC_CHANNEL0);

    serial_state_e next_state = serial_state;
    switch(serial_state)
    {

    case serial_state_idle:
        if(m_serial_start)
        {
            m_serial_start = false;
            m_serial_stop = false;            
            halSetExternalPinFunction(external_pin_function_uart_rx);            
            ping_timeout_reason = ping_timeout_none;
            serialStartRx();
            next_state = serial_state_rx;
            last_rx_byte_count = rx_byte_count;
        }
        break;

    case serial_state_rx:
        if(m_serial_stop)
        {
            m_serial_stop = false;
            serialStopRx();
            next_state = serial_state_idle;
        }
        else if(ping_timeout_reason)
        {
            NRF_LOG_INFO("Ping Timeout");
            pt10_set_uart_connected(false);
            ping_timeout_reason = ping_timeout_none;
        }
        if(rx_byte_count != last_rx_byte_count)
        {
            NRF_LOG_DEBUG("Rx %d bytes: %s", rx_byte_count - last_rx_byte_count);
            while(last_rx_byte_count != rx_byte_count)
            {
                if(uarte_rx_buffer[last_rx_byte_count++] == '\0')
                {
                    app_timer_stop(ping_timer);
                    uint32_t rx_packet_length = cobsDecode(uarte_rx_buffer, rx_byte_count, rx_packet.bytes);
                    uint8_t crc = crc8(0, rx_packet.bytes, rx_packet_length);

                    response_packet.header.payload_type = serial_payload_none;
                    if(crc == 0)
                    {                        
                        serialRxPacket(&rx_payload_type);
                        nrfx_uarte_rx_abort(&m_uart);
                        ping_timeout_reason = ping_timeout_none;
                        app_timer_start(ping_timer, APP_TIMER_TICKS(RX_TX_TURNAROUND_MS), (void *)ping_timeout_rxtx_turnaround);
                        next_state = serial_state_wait_rx_abort;
                    }
                    else
                    {
                        rx_payload_type = serial_payload_none;
                        NRF_LOG_WARNING("crc error (%d)", crc);
                        nrfx_uarte_rx_abort(&m_uart);
                        ping_timeout_reason = ping_timeout_none;
                        serialStopRx();
                        next_state = serial_state_idle;
                    }
                    
                }
            }
        }
        break;

    case serial_state_wait_rx_abort:
    if(m_serial_stop)
    {
        m_serial_stop = false;
        serialStopRx();
        next_state = serial_state_idle;
    }
    else if(ping_timeout_reason == ping_timeout_rxtx_turnaround)
        {
            if(rx_done)
            {
                serialStopRx();
                serialStartTx();
            
                ping_timeout_reason = ping_timeout_none;
                uint32_t timeout_type;
                switch(rx_payload_type)
                {
                case serial_payload_ping:
                    timeout_type = ping_timeout_ping;
                    break;
                case serial_payload_command:
                    timeout_type = ping_timeout_cmd;
                    break;
                default:
                    timeout_type = ping_timeout_cmd;
                    break;
                }
                app_timer_start(ping_timer, APP_TIMER_TICKS(PING_TIMEOUT_MS), (void *)timeout_type);
                next_state = serial_state_tx;
            }
            else
            {
                NRF_LOG_WARNING("Timeout waiting for rx_done");
                serialStopRx();
                serialStopTx();
                pt10_set_uart_connected(false);
                next_state = serial_state_idle;
            }
        }
        break;

    case serial_state_tx:

        tx_packet.header.payload_type = serial_payload_none;

        if(m_serial_stop)
        {
            serialStopTx();
            next_state = serial_state_idle;
        }
        else 
        {
            if(response_packet.header.payload_type != serial_payload_none)
            {
                NRF_LOG_DEBUG("Tx packet is command response");
                memcpy((uint8_t *)&tx_packet, (uint8_t *)&response_packet, sizeof(serial_packet_t));
                response_packet.header.payload_type = serial_payload_none;
            }
            else
            {

                if(!nrf_queue_is_empty(&tx_packet_queue))
                {
                    NRF_LOG_DEBUG("Tx packet is from queue");
                    nrf_queue_generic_pop(&tx_packet_queue, &tx_packet, false);
                }        
                else if(ping_timeout_reason)
                {
                    NRF_LOG_DEBUG("Ping timeout reason %d", ping_timeout_reason);
                    switch(ping_timeout_reason)
                    {
                    case ping_timeout_ping:
                    
                        ping_timeout_reason = ping_timeout_none;
                        uint32_t device_id;
                        get_device_id(&device_id);
                        tx_packet.header.device_id = device_id;
                        tx_packet.header.payload_type = serial_payload_ping_response;
                        tx_packet.header.payload_length = 0;
                        NRF_LOG_DEBUG("Constructed a ping response packet");
                        break;

                    case ping_timeout_cmd:
                        ping_timeout_reason = ping_timeout_none;
                        pt10_set_uart_connected(false);
                        next_state = serial_state_idle;
                        NRF_LOG_DEBUG("ping_timeout_cmd");
                        break;

                    default:
                        NRF_LOG_DEBUG("Unhandled ping timeout reason %d", ping_timeout_reason);
                        break;

                    } 
                } 

            }

            if(tx_packet.header.payload_type != serial_payload_none)
            {
                app_timer_stop(ping_timer);

                uint8_t tx_packet_length = sizeof(serial_packet_header_t) + tx_packet.header.payload_length;
                uint8_t crc = crc8(0, tx_packet.bytes, tx_packet_length);
                tx_packet.bytes[tx_packet_length++] = crc;
                if(tx_packet.header.payload_type == serial_payload_sensor_data)
                {
                    stream_packet_count++;
                    NRF_LOG_INFO("Sending sensor data %d", stream_packet_count);    
                }
                uint8_t tx_length = cobsEncode(tx_packet.bytes, tx_packet_length, uarte_tx_buffer);
                uarte_tx_buffer[tx_length++] = '\0';
                tx_done = false;
                // ("sending %d bytes", tx_length);
                nrfx_uarte_tx(&m_uart, uarte_tx_buffer, tx_length);
                next_state = serial_state_wait_tx_done; 
            }
        }
               
        break;

    case serial_state_wait_tx_done:
        if(m_serial_stop)
        {
            serialStopTx();
            next_state = serial_state_idle;
        }    
        else if(tx_done)
        {
            tx_packet.header.payload_type = serial_payload_none;
            serialStopTx();
            last_rx_byte_count = 0;
            serialStartRx();
            next_state = serial_state_rx;
            app_timer_stop(ping_timer);
            ping_timeout_reason = ping_timeout_none;
            app_timer_start(ping_timer, APP_TIMER_TICKS(UART_CONNECTION_TIMEOUT_MS), (void *)ping_timeout_ping);            
        }
        break;
            
    }

    if(next_state != serial_state)
    {
        NRF_LOG_DEBUG("%d -> %d", serial_state, next_state);
        serial_state = next_state;
    }

    

}