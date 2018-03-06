/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_timer.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done = false;  /**< Flag used to indicate that SPI instance completed the transfer. */
const nrf_drv_timer_t TIMER_AUDIO = NRF_DRV_TIMER_INSTANCE(0);

//#define TEST_STRING "Nordic"
static uint8_t			m_tx_buf[2] = {0xFF, 0xFF};
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t			m_rx_buf[2];
//static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t	m_length = 2;
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
		bsp_board_led_invert(1);
//    NRF_LOG_INFO("Transfer completed.");
//    if (m_rx_buf[0] != 0)
//    {
//        NRF_LOG_INFO(" Received: 0x%02x%02x", m_rx_buf[0], m_rx_buf[1]);
//        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
//    }
}

static void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
//	NRF_LOG_DEBUG("Time handler done");
//	if(spi_xfer_done) {
		bsp_board_led_invert(0);
		spi_xfer_done = false;
		nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length);
//		bsp_board_led_invert(0);
//		nrf_drv_timer_clear(&TIMER_AUDIO);
//		bsp_board_led_invert(0);
//		nrf_drv_timer_resume(&TIMER_AUDIO);
//		bsp_board_led_invert(0);
//	}
//	switch(event_type)
//	{
//		case NRF_TIMER_EVENT_COMPARE0:
//			if(spi_xfer_done) {
//				bsp_board_led_invert(0);
//				spi_xfer_done = false;
//				APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
//			}
//			break;
//		
//		default:
//			break;
//	}
}

static void timer_interrupt_init(void)
{
	uint32_t time_us = 20;
	uint32_t time_ticks;
    ret_code_t err_code;
	
	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
	err_code = nrf_drv_timer_init(&TIMER_AUDIO, &timer_cfg, timer_handler);
	APP_ERROR_CHECK(err_code);
	
	time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_AUDIO, time_us);
	nrf_drv_timer_extended_compare(&TIMER_AUDIO, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	nrf_drv_timer_enable(&TIMER_AUDIO);
}


int main(void)
{
    bsp_board_leds_init();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    NRF_LOG_INFO("SPI example.");

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
	spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
	
	spi_xfer_done = true;
	timer_interrupt_init();
	
    while (1)
    {
//		while(!spi_xfer_done) {
			__WFE();
//		}
        // Reset rx buffer and transfer done flag
//        memset(m_rx_buf, 0, m_length);
//        spi_xfer_done = false;

//        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

//        while (!spi_xfer_done)
//        {
//            __WFE();
//        }

//        NRF_LOG_FLUSH();

//        bsp_board_led_invert(BSP_BOARD_LED_0);
//        nrf_delay_ms(200);
    }
}
