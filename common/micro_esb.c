
#include "micro_esb.h"
#include "uesb_error_codes.h"
#include "nrf_gpio.h"
#include <string.h>


static uesb_event_handler_t     m_event_handler;

// RF parameters
static uesb_config_t            m_config_local;

// TX FIFO
static uesb_payload_t           m_tx_fifo_payload[UESB_CORE_TX_FIFO_SIZE];
static uesb_payload_tx_fifo_t   m_tx_fifo;

// RX FIFO
static uesb_payload_t           m_rx_fifo_payload[UESB_CORE_RX_FIFO_SIZE];
static uesb_payload_rx_fifo_t   m_rx_fifo;

static  uint8_t                 m_tx_payload_buffer[UESB_CORE_MAX_PAYLOAD_LENGTH + 2];
static  uint8_t                 m_rx_payload_buffer[UESB_CORE_MAX_PAYLOAD_LENGTH + 2];

// Run time variables
static volatile uint32_t        m_interrupt_flags       = 0;
static uint32_t                 m_pid                   = 0;
static volatile uint32_t        m_retransmits_remaining;
static volatile uint32_t        m_last_tx_retransmit_attempts = 0;

static uesb_payload_t           *current_payload;

static uesb_mainstate_t         m_uesb_mainstate        = UESB_STATE_UNINITIALIZED;

// Constant parameters
#define                         RX_WAIT_FOR_ACK_TIMEOUT_US  60  // Testing shows 43 to be the smallest reliable value, adding some buffer

// Macros
#define                         DISABLE_RF_IRQ      NVIC_DisableIRQ(RADIO_IRQn)
#define                         ENABLE_RF_IRQ       NVIC_EnableIRQ(RADIO_IRQn)

    /*enum uesb_protocol_t    protocol;
    enum uesb_mode_t        mode;
    
    // General RF parameters
    uint8_t                 rf_channel;
    uint8_t                 payload_length;
    enum uesb_bitrate_t     bitrate;
    enum uesb_crc_t         crc;
    
    enum uesb_tx_power_t    tx_output_power;
    uint8_t                 tx_address[5];
    uint8_t                 rx_address_p0[5];
    uint8_t                 rx_address_p1[5];
    uint8_t                 rx_address_p2;
    uint8_t                 rx_address_p3;
    uint8_t                 rx_address_p4;
    uint8_t                 rx_address_p5;  

    // ESB specific features
    uint8_t                 dynamic_payload_length_enabled;
    uint8_t                 dynamic_ack_enabled;
    uint16_t                retransmit_delay;
    uint16_t                retransmit_count;*/

static void on_radio_disabled_esb_dpl_tx_noack(void);
static void on_radio_disabled_esb_dpl_tx(void);
static void on_radio_disabled_esb_dpl_tx_wait_for_ack(void);

static uint32_t swap_bits(uint32_t inp)
{
    inp = (inp & 0x000000F0) >> 4 | (inp & 0x0000000F) << 4;
    inp = (inp & 0x000000CC) >> 2 | (inp & 0x00000033) << 2;
    return (inp & 0x000000AA) >> 1 | (inp & 0x00000055) << 1;
}

static void update_radio_parameters()
{
    // TX power
    NRF_RADIO->TXPOWER   = m_config_local.tx_output_power   << RADIO_TXPOWER_TXPOWER_Pos;
    
    // RF bitrate
    NRF_RADIO->MODE      = m_config_local.bitrate           << RADIO_MODE_MODE_Pos;
    
    // CRC configuration
    NRF_RADIO->CRCCNF    = m_config_local.crc               << RADIO_CRCCNF_LEN_Pos; 
    if(m_config_local.crc == RADIO_CRCCNF_LEN_Two)
    {
        NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value      
        NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
    }
    else if(m_config_local.crc == RADIO_CRCCNF_LEN_One)
    {
        NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
        NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
    }
    
    // Packet format 
    if(m_config_local.protocol == UESB_PROTOCOL_ESB || m_config_local.protocol == UESB_PROTOCOL_ESB_DPL)
    {        
        NRF_RADIO->PCNF0 = (3 << RADIO_PCNF0_S1LEN_Pos) | (0 << RADIO_PCNF0_S0LEN_Pos) | (6 << RADIO_PCNF0_LFLEN_Pos);
        NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled        << RADIO_PCNF1_WHITEEN_Pos) |
                           (RADIO_PCNF1_ENDIAN_Big              << RADIO_PCNF1_ENDIAN_Pos)  |
                           ((m_config_local.rf_addr_length - 1) << RADIO_PCNF1_BALEN_Pos)   |
                           (0                                   << RADIO_PCNF1_STATLEN_Pos) |
                           (UESB_CORE_MAX_PAYLOAD_LENGTH        << RADIO_PCNF1_MAXLEN_Pos); 
    }
    
        // Radio address config
    NRF_RADIO->PREFIX0 = swap_bits(m_config_local.rx_address_p3)    << 24 |
                         swap_bits(m_config_local.rx_address_p2)    << 16 | 
                         swap_bits(m_config_local.rx_address_p1[0]) << 8 | 
                         swap_bits(m_config_local.rx_address_p0[0]);
    
    NRF_RADIO->PREFIX1 = swap_bits(m_config_local.rx_address_p7) << 24 |
                         swap_bits(m_config_local.rx_address_p6) << 16 | 
                         swap_bits(m_config_local.rx_address_p5) << 8  | 
                         swap_bits(m_config_local.rx_address_p4);

    NRF_RADIO->BASE0   = swap_bits(m_config_local.rx_address_p0[1]) << 24 |
                         swap_bits(m_config_local.rx_address_p0[2]) << 16 |
                         swap_bits(m_config_local.rx_address_p0[3]) << 8  |
                         swap_bits(m_config_local.rx_address_p0[4]);  

    NRF_RADIO->BASE1   = swap_bits(m_config_local.rx_address_p1[1]) << 24 |
                         swap_bits(m_config_local.rx_address_p1[2]) << 16 |
                         swap_bits(m_config_local.rx_address_p1[3]) << 8  |
                         swap_bits(m_config_local.rx_address_p1[4]); 
}
    
// FIFO HANDLING --------------------------

static void initialize_fifos()
{
    m_tx_fifo.entry_point   = 0;
    m_tx_fifo.exit_point    = 0;
    m_tx_fifo.count         = 0;
    for(int i = 0; i < UESB_CORE_TX_FIFO_SIZE; i++)
    {
        m_tx_fifo.payload_ptr[i]       = &m_tx_fifo_payload[i];
    }
    
    m_rx_fifo.entry_point   = 0;
    m_rx_fifo.exit_point    = 0;
    m_rx_fifo.count         = 0;
    for(int i = 0; i < UESB_CORE_RX_FIFO_SIZE; i++)
    {
        m_rx_fifo.payload_ptr[i]       = &m_rx_fifo_payload[i];
    }
}

static void tx_fifo_remove_last()
{
    if(m_tx_fifo.count > 0)
    {
        DISABLE_RF_IRQ;
        m_tx_fifo.count--;
        m_tx_fifo.exit_point++;
        if(m_tx_fifo.exit_point >= UESB_CORE_TX_FIFO_SIZE) m_tx_fifo.exit_point = 0;
        ENABLE_RF_IRQ;
    }
}

static bool rx_fifo_push_rfbuf(uint8_t pipe)
{
    if(m_rx_fifo.count < UESB_CORE_RX_FIFO_SIZE && m_rx_payload_buffer[0] <= UESB_CORE_MAX_PAYLOAD_LENGTH)
    {
        DISABLE_RF_IRQ;
        m_rx_fifo.payload_ptr[m_rx_fifo.entry_point]->length = m_rx_payload_buffer[0];
        memcpy(m_rx_fifo.payload_ptr[m_rx_fifo.entry_point]->data, &m_rx_payload_buffer[2], m_rx_payload_buffer[0]); 
        m_rx_fifo.payload_ptr[m_rx_fifo.entry_point]->pipe = pipe;
        if(++m_rx_fifo.entry_point >= UESB_CORE_RX_FIFO_SIZE) m_rx_fifo.entry_point = 0;
        m_rx_fifo.count++;
        ENABLE_RF_IRQ;
        return true;
    }
    return false;
}

// 

static void sys_timer_init()
{
    // Configure the system timer with a 1 MHz base frequency
    UESB_SYS_TIMER->PRESCALER = 4;
    UESB_SYS_TIMER->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    UESB_SYS_TIMER->SHORTS    = TIMER_SHORTS_COMPARE1_CLEAR_Msk | TIMER_SHORTS_COMPARE1_STOP_Msk;
}

static void ppi_init()
{
    NRF_PPI->CH[UESB_PPI_TIMER_START].EEP = (uint32_t)&NRF_RADIO->EVENTS_READY;
    NRF_PPI->CH[UESB_PPI_TIMER_START].TEP = (uint32_t)&UESB_SYS_TIMER->TASKS_START;
    NRF_PPI->CH[UESB_PPI_TIMER_STOP].EEP =  (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[UESB_PPI_TIMER_STOP].TEP =  (uint32_t)&UESB_SYS_TIMER->TASKS_STOP;
    NRF_PPI->CH[UESB_PPI_RX_TIMEOUT].EEP = (uint32_t)&UESB_SYS_TIMER->EVENTS_COMPARE[0];
    NRF_PPI->CH[UESB_PPI_RX_TIMEOUT].TEP = (uint32_t)&NRF_RADIO->TASKS_DISABLE;
    NRF_PPI->CH[UESB_PPI_TX_START].EEP = (uint32_t)&UESB_SYS_TIMER->EVENTS_COMPARE[1];
    NRF_PPI->CH[UESB_PPI_TX_START].TEP = (uint32_t)&NRF_RADIO->TASKS_TXEN;   
}

uint32_t uesb_init(uesb_config_t *parameters, uesb_event_handler_t event_handler)
{
    m_event_handler = event_handler;
    memcpy(&m_config_local, parameters, sizeof(uesb_config_t));
    update_radio_parameters();
    
    initialize_fifos();
    
    sys_timer_init();
    
    ppi_init();
    
    //m_uesb_initialized = true;
    m_uesb_mainstate = UESB_STATE_IDLE;
    
    return UESB_SUCCESS;
}

uint32_t uesb_disable(void)
{
    if(m_uesb_mainstate != UESB_STATE_IDLE) return UESB_ERROR_NOT_IDLE;
    NRF_PPI->CHENCLR = (1 << UESB_PPI_TIMER_START) | (1 << UESB_PPI_TIMER_STOP) | (1 << UESB_PPI_RX_TIMEOUT) | (1 << UESB_PPI_TX_START);
    m_uesb_mainstate = UESB_STATE_UNINITIALIZED;
    return UESB_SUCCESS;
}

static void start_tx_transaction(bool ack)
{
    // Prepare the payload
    current_payload = m_tx_fifo.payload_ptr[m_tx_fifo.exit_point];
    m_pid = (m_pid + 1) % 4;
    switch(m_config_local.protocol)
    {
        case UESB_PROTOCOL_SB:
            break;
        case UESB_PROTOCOL_ESB:
                NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) | (0 << RADIO_PCNF0_LFLEN_Pos) | (1 << RADIO_PCNF0_S1LEN_Pos);
                NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
                   (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                   (4                            << RADIO_PCNF1_BALEN_Pos)   |
                   (current_payload->length      << RADIO_PCNF1_STATLEN_Pos) |
                   (current_payload->length      << RADIO_PCNF1_MAXLEN_Pos);
                m_tx_payload_buffer[0] = 0xCC | m_pid;
                m_tx_payload_buffer[1] = 0;
                
                NRF_RADIO->SHORTS      = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
                NRF_RADIO->INTENSET    = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;  
        
                // Configure the retransmit counter
                m_retransmits_remaining = m_config_local.retransmit_count; 
                m_uesb_mainstate = UESB_STATE_PTX_TX_ACK; 
            break;
        case UESB_PROTOCOL_ESB_DPL:
            m_tx_payload_buffer[0] = current_payload->length;
            m_tx_payload_buffer[1] = m_pid << 1;
            if(ack)
            {
                NRF_RADIO->SHORTS      = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
                NRF_RADIO->INTENSET    = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;  
        
                // Configure the retransmit counter
                m_retransmits_remaining = m_config_local.retransmit_count; 
                m_uesb_mainstate = UESB_STATE_PTX_TX_ACK;                
            }
            else
            {
                NRF_RADIO->SHORTS      = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
                NRF_RADIO->INTENSET    = RADIO_INTENSET_DISABLED_Msk;  
                m_uesb_mainstate = UESB_STATE_PTX_TX;                
            }
            break;
    }


    memcpy(&m_tx_payload_buffer[2], current_payload->data, current_payload->length);
    
    NRF_RADIO->TXADDRESS = current_payload->pipe;
    NRF_RADIO->RXADDRESSES = 1 << current_payload->pipe;
    
    NRF_RADIO->FREQUENCY = m_config_local.rf_channel;
    
    NRF_RADIO->PACKETPTR = (uint32_t)m_tx_payload_buffer;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->EVENTS_ADDRESS = NRF_RADIO->EVENTS_PAYLOAD = 0;

    NRF_RADIO->TASKS_TXEN  = 1;       
}

uint32_t uesb_write_tx_payload(uesb_payload_t *payload)
{
    if(m_uesb_mainstate == UESB_STATE_UNINITIALIZED) return UESB_ERROR_NOT_INITIALIZED;
    if(m_tx_fifo.count >= UESB_CORE_TX_FIFO_SIZE) return UESB_ERROR_TX_FIFO_FULL;
    
    DISABLE_RF_IRQ;
    memcpy(m_tx_fifo.payload_ptr[m_tx_fifo.entry_point], payload, sizeof(uesb_payload_t));
    m_tx_fifo.entry_point++;
    if(m_tx_fifo.entry_point >= UESB_CORE_TX_FIFO_SIZE) m_tx_fifo.entry_point = 0;
    m_tx_fifo.count++;
    ENABLE_RF_IRQ;
    
    if(m_config_local.tx_mode == UESB_TXMODE_AUTO && m_uesb_mainstate == UESB_STATE_IDLE)
    {
        start_tx_transaction(true);
    }
    return UESB_SUCCESS;
}

uint32_t uesb_read_rx_payload(uesb_payload_t *payload)
{
    if(m_uesb_mainstate == UESB_STATE_UNINITIALIZED) return UESB_ERROR_NOT_INITIALIZED;
    if(m_rx_fifo.count == 0) return UESB_ERROR_RX_FIFO_EMPTY; 
    
    DISABLE_RF_IRQ;
    payload->length = m_rx_fifo.payload_ptr[m_rx_fifo.exit_point]->length;
    payload->pipe   = m_rx_fifo.payload_ptr[m_rx_fifo.exit_point]->pipe;
    memcpy(payload->data, m_rx_fifo.payload_ptr[m_rx_fifo.exit_point]->data, payload->length);
    if(++m_rx_fifo.exit_point >= UESB_CORE_RX_FIFO_SIZE) m_rx_fifo.exit_point = 0;
    m_rx_fifo.count--;
    ENABLE_RF_IRQ;
    return UESB_SUCCESS;    
}

uint32_t uesb_start_tx()
{
    if(m_uesb_mainstate != UESB_STATE_IDLE) return UESB_ERROR_NOT_IDLE;
    if(m_tx_fifo.count == 0) return UESB_ERROR_TX_FIFO_EMPTY;
    start_tx_transaction(true);
    return UESB_SUCCESS;    
}

uint32_t uesb_get_tx_attempts(uint32_t *attempts)
{
    if(m_uesb_mainstate == UESB_STATE_UNINITIALIZED) return UESB_ERROR_NOT_INITIALIZED;
    *attempts = m_last_tx_retransmit_attempts;
    return UESB_SUCCESS;
}

uint32_t uesb_flush_tx(void)
{
    if(m_uesb_mainstate != UESB_STATE_IDLE) return UESB_ERROR_NOT_IDLE;
    DISABLE_RF_IRQ;
    m_tx_fifo.count = 0;
    m_tx_fifo.entry_point = m_tx_fifo.exit_point = 0;
    ENABLE_RF_IRQ;
    return UESB_SUCCESS;
}

uint32_t uesb_flush_rx(void)
{
    DISABLE_RF_IRQ;
    m_rx_fifo.count = 0;
    m_rx_fifo.entry_point = 0;
    ENABLE_RF_IRQ;
    return UESB_SUCCESS;
}

uint32_t uesb_get_clear_interrupts(uint32_t *interrupts)
{
    DISABLE_RF_IRQ;
    *interrupts = m_interrupt_flags;
    m_interrupt_flags = 0;
    ENABLE_RF_IRQ;
    return UESB_SUCCESS;
}

uint32_t uesb_set_address(uesb_address_type_t address, const uint8_t *data_ptr)
{
    if(m_uesb_mainstate != UESB_STATE_IDLE) return UESB_ERROR_NOT_IDLE;
    switch(address)
    {
        case UESB_ADDRESS_PIPE0:
            memcpy(m_config_local.rx_address_p0, data_ptr, m_config_local.rf_addr_length);
            break;
        case UESB_ADDRESS_PIPE1:
            memcpy(m_config_local.rx_address_p1, data_ptr, m_config_local.rf_addr_length);
            break;
        case UESB_ADDRESS_PIPE2:
            m_config_local.rx_address_p2 = *data_ptr;
            break;
        case UESB_ADDRESS_PIPE3:
            m_config_local.rx_address_p3 = *data_ptr;
            break;
        case UESB_ADDRESS_PIPE4:
            m_config_local.rx_address_p4 = *data_ptr;
            break;
        case UESB_ADDRESS_PIPE5:
            m_config_local.rx_address_p5 = *data_ptr;
            break;
        case UESB_ADDRESS_PIPE6:
            m_config_local.rx_address_p6 = *data_ptr;
            break;
        case UESB_ADDRESS_PIPE7:
            m_config_local.rx_address_p7 = *data_ptr;
            break;
        default:
            return UESB_ERROR_INVALID_PARAMETERS;
    }
    update_radio_parameters();
    return UESB_SUCCESS;
}

uint32_t uesb_set_rf_channel(uint32_t channel)
{
    if(channel > 125) return UESB_ERROR_INVALID_PARAMETERS;
    m_config_local.rf_channel = channel;
    return UESB_SUCCESS;
}

void RADIO_IRQHandler()
{
    if(NRF_RADIO->EVENTS_READY && (NRF_RADIO->INTENSET & RADIO_INTENSET_READY_Msk))
    {
        NRF_RADIO->EVENTS_READY = 0;
        DEBUG_PIN_SET(DEBUGPIN1);
    }
   
    if(NRF_RADIO->EVENTS_DISABLED && (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk))
    {
        NRF_RADIO->EVENTS_DISABLED = 0;
        DEBUG_PIN_SET(DEBUGPIN3);
        
        if(m_uesb_mainstate == UESB_STATE_PTX_TX)
        {
            on_radio_disabled_esb_dpl_tx_noack();
        }
        else if(m_uesb_mainstate == UESB_STATE_PTX_TX_ACK)
        {
            on_radio_disabled_esb_dpl_tx();
        }
        else if(m_uesb_mainstate == UESB_STATE_PTX_RX_ACK)
        {
            on_radio_disabled_esb_dpl_tx_wait_for_ack();
        }

    }
    DEBUG_PIN_CLR(DEBUGPIN1);
    DEBUG_PIN_CLR(DEBUGPIN2);
    DEBUG_PIN_CLR(DEBUGPIN3);
}

static void on_radio_disabled_esb_dpl_tx_noack()
{
    m_interrupt_flags |= UESB_INT_TX_SUCCESS_MSK;
    tx_fifo_remove_last();
    m_uesb_mainstate = UESB_STATE_IDLE;  
    if(m_event_handler != 0) m_event_handler();
}

static void on_radio_disabled_esb_dpl_tx()
{
    // Remove the DISABLED -> RXEN shortcut, to make sure the radio stays disabled after the RX window
    NRF_RADIO->SHORTS           = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;

    // Make sure the timer is started the next time the radio is ready, 
    // and that it will disable the radio automatically if no packet is received by the time defined in RX_WAIT_FOR_ACK_TIMEOUT_US
    UESB_SYS_TIMER->CC[0]       = RX_WAIT_FOR_ACK_TIMEOUT_US;
    UESB_SYS_TIMER->CC[1]       = m_config_local.retransmit_delay - 130;
    UESB_SYS_TIMER->TASKS_CLEAR = 1;
    NRF_PPI->CHENSET            = (1 << UESB_PPI_TIMER_START) | (1 << UESB_PPI_RX_TIMEOUT) | (1 << UESB_PPI_TIMER_STOP);
    NRF_RADIO->EVENTS_END       = 0;
    if(m_config_local.protocol == UESB_PROTOCOL_ESB)
    {
        NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled << RADIO_PCNF1_WHITEEN_Pos) |
           (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
           (m_config_local.rf_addr_length   << RADIO_PCNF1_BALEN_Pos)   |
           (0                               << RADIO_PCNF1_STATLEN_Pos) |
           (0                               << RADIO_PCNF1_MAXLEN_Pos);        
    }
    NRF_RADIO->PACKETPTR        = (uint32_t)m_rx_payload_buffer;
    m_uesb_mainstate            = UESB_STATE_PTX_RX_ACK;    
}

static void on_radio_disabled_esb_dpl_tx_wait_for_ack()
{
    // This marks the completion of a TX_RX sequence (TX with ACK)

    // Make sure the timer will not deactivate the radio if a packet is received
    NRF_PPI->CHENCLR = (1 << UESB_PPI_TIMER_START) | (1 << UESB_PPI_RX_TIMEOUT) | (1 << UESB_PPI_TIMER_STOP);

    // If the radio has received a packet and the CRC status is OK
    if(NRF_RADIO->EVENTS_END && NRF_RADIO->CRCSTATUS != 0)
    {
        UESB_SYS_TIMER->TASKS_STOP = 1;
        m_interrupt_flags |= UESB_INT_TX_SUCCESS_MSK;
        m_last_tx_retransmit_attempts = m_config_local.retransmit_count - m_retransmits_remaining + 1;
        tx_fifo_remove_last();
        if(m_rx_payload_buffer[0] > 0)
        {
            if(rx_fifo_push_rfbuf((uint8_t)NRF_RADIO->TXADDRESS))
            {
                m_interrupt_flags |= UESB_INT_RX_DR_MSK;
            }         
        }
        m_uesb_mainstate = UESB_STATE_IDLE;   
        if(m_event_handler != 0) m_event_handler();
    }
    else
    {
        if(m_retransmits_remaining-- == 0)
        {
            UESB_SYS_TIMER->TASKS_STOP = 1;
            // All retransmits are expended, and the TX operation is suspended
            m_interrupt_flags |= UESB_INT_TX_FAILED_MSK;
            m_uesb_mainstate = UESB_STATE_IDLE;    
            if(m_event_handler != 0) m_event_handler();
        }
        else
        {
            UESB_SYS_TIMER->TASKS_START = 1;
            // We still have more retransmits left, and we should enter TX mode again as soon as the system timer reaches CC[1]
            NRF_PPI->CHENSET = (1 << UESB_PPI_TX_START);
            NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;
            if(m_config_local.protocol == UESB_PROTOCOL_ESB)
            {
                NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                                   (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                                   (m_config_local.rf_addr_length   << RADIO_PCNF1_BALEN_Pos)   |
                                   (current_payload->length         << RADIO_PCNF1_STATLEN_Pos) |
                                   (current_payload->length         << RADIO_PCNF1_MAXLEN_Pos);
            }
            NRF_RADIO->PACKETPTR = (uint32_t)m_tx_payload_buffer;
            m_uesb_mainstate = UESB_STATE_PTX_TX_ACK;
        }
    }
}

