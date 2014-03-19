/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
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

/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
* @{
* @ingroup nrf_examples_nrf6310
*
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "micro_esb.h"
#include "uesb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

static uesb_payload_t tx_payload, rx_payload;

void uesb_event_handler()
{
    static uint32_t rf_interrupts;
    
    uesb_get_clear_interrupts(&rf_interrupts);
    
    if(rf_interrupts & UESB_INT_TX_SUCCESS_MSK)
    {   
        nrf_gpio_pin_set(13);
    }
    if(rf_interrupts & UESB_INT_TX_FAILED_MSK)
    {
        nrf_gpio_pin_set(14);
        uesb_flush_tx();
    }
    if(rf_interrupts & UESB_INT_RX_DR_MSK)
    {
        nrf_gpio_pin_set(15);
        uesb_read_rx_payload(&rx_payload);
        NRF_GPIO->OUTCLR = 0xFF << 16;
        NRF_GPIO->OUTSET = rx_payload.data[1] << 16;
    }

}

int main(void)
{
    uint8_t rx_addr_p0[] = {0x12, 0x34, 0x56, 0x78, 0x9A};
    uint8_t rx_addr_p1[] = {0xBC, 0xDE, 0xF0, 0x12, 0x23};
    uint8_t rx_addr_p2   = 0x66;
    nrf_gpio_range_cfg_output(8, 31);
    
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    uesb_config_t uesb_config       = UESB_DEFAULT_CONFIG;
    uesb_config.rf_channel          = 5;
    uesb_config.crc                 = UESB_CRC_16BIT;
    uesb_config.dynamic_ack_enabled = 0;
    uesb_config.payload_length      = 8;
    uesb_config.protocol            = UESB_PROTOCOL_ESB_DPL;
    uesb_config.bitrate             = UESB_BITRATE_2MBPS;
    uesb_config.mode                = UESB_MODE_PRX;
    
    uesb_init(&uesb_config, uesb_event_handler);

    uesb_set_address(UESB_ADDRESS_PIPE0, rx_addr_p0);
    uesb_set_address(UESB_ADDRESS_PIPE1, rx_addr_p1);
    uesb_set_address(UESB_ADDRESS_PIPE2, &rx_addr_p2);
  
    uesb_start_rx();
  
    while (true)
    {   
        nrf_gpio_pin_toggle(8);
        nrf_delay_us(100000);
    }
}

