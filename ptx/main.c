/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "micro_esb.h"
#include "uesb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

static uesb_payload_t tx_payload, rx_payload;

void uesb_event_handler()
{
    static uint32_t rf_interrupts;
    static uint32_t tx_attempts;
    
    uesb_get_clear_interrupts(&rf_interrupts);
    
    if(rf_interrupts & UESB_INT_TX_SUCCESS_MSK)
    {   
    }
    
    if(rf_interrupts & UESB_INT_TX_FAILED_MSK)
    {
        uesb_flush_tx();
    }
    
    if(rf_interrupts & UESB_INT_RX_DR_MSK)
    {
        uesb_read_rx_payload(&rx_payload);
        NRF_GPIO->OUTCLR = 0xFUL << 8;
        NRF_GPIO->OUTSET = (uint32_t)((rx_payload.data[2] & 0x0F) << 8);
    }
    
    uesb_get_tx_attempts(&tx_attempts);
    NRF_GPIO->OUTCLR = 0xFUL << 12;
    NRF_GPIO->OUTSET = (tx_attempts & 0x0F) << 12;
}

int main(void)
{
    uint8_t rx_addr_p0[] = {0x12, 0x34, 0x56, 0x78, 0x9A};
    uint8_t rx_addr_p1[] = {0xBC, 0xDE, 0xF0, 0x12, 0x23};
    uint8_t rx_addr_p2   = 0x66;
    
    nrf_gpio_range_cfg_output(8, 15);
    
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    uesb_config_t uesb_config       = UESB_DEFAULT_CONFIG;
    uesb_config.rf_channel          = 5;
    uesb_config.crc                 = UESB_CRC_16BIT;
    uesb_config.retransmit_count    = 6;
    uesb_config.retransmit_delay    = 500;
    uesb_config.dynamic_ack_enabled = 0;
    uesb_config.protocol            = UESB_PROTOCOL_ESB_DPL;
    uesb_config.bitrate             = UESB_BITRATE_2MBPS;
    uesb_config.event_handler       = uesb_event_handler;
    
    uesb_init(&uesb_config);

    uesb_set_address(UESB_ADDRESS_PIPE0, rx_addr_p0);
    uesb_set_address(UESB_ADDRESS_PIPE1, rx_addr_p1);
    uesb_set_address(UESB_ADDRESS_PIPE2, &rx_addr_p2);

    tx_payload.length  = 8;
    tx_payload.pipe    = 0;
    tx_payload.data[0] = 0x01;
    tx_payload.data[1] = 0x00;
    tx_payload.data[2] = 0x00;
    tx_payload.data[3] = 0x00;
    tx_payload.data[4] = 0x11;
    
    while (true)
    {   
        if(uesb_write_tx_payload(&tx_payload) == UESB_SUCCESS)
        {
            tx_payload.data[1]++;
        }
        nrf_delay_us(10000);
    }
}
