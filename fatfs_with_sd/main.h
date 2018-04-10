/*
 * main.h
 *
 * Created: 10.04.2018 16:21:14
 *  Author: schiesser
 */ 


#ifndef MAIN_H__
#define MAIN_H__

/* Standard libraries */
#include <string.h>
#include <stdio.h>
/* NRF libraries*/
#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_queue.h"
#include "nrf_delay.h"
#include "nrf_serial.h"
/* APP libraries */
#include "app_util_platform.h"
#include "app_error.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "app_timer.h"
/* NRF DRV */
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"
/* LOG */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/* SoundingSoil */
#include "wave_header.h"


/* ========================================================================== */
/*                                   MACROS                                   */
/* ========================================================================== */

/*                                   DEBUG                                    */
/* -------------------------------------------------------------------------- */
#define DBG0_PIN						11
#define DBG1_PIN						12
#define DBG_CURRENT_FOLDER				"180322"
#define DBG_CURRENT_FILE				"R123456.wav"
#define DBG_TOGGLE(dbg)					(nrf_drv_gpiote_out_toggle(dbg))

/*                             LED/BUTTON MACROS                              */
/* -------------------------------------------------------------------------- */
#define LED_RECORD						(BSP_LED_0)
#define LED_MONITOR						(BSP_LED_3)
#define LED_ON(led)						(nrf_drv_gpiote_out_clear(led))
#define LED_OFF(led)					(nrf_drv_gpiote_out_set(led))
#define LED_TOGGLE(led)					(nrf_drv_gpiote_out_toggle(led))

#define BUTTON_RECORD					(BSP_BUTTON_0)
#define BUTTON_MONITOR					(BSP_BUTTON_3)

/*                              ADC to SDC FIFO                               */
/* -------------------------------------------------------------------------- */
#define FIFO_DATA_SIZE					8192

/*                                  SD card                                   */
/* -------------------------------------------------------------------------- */
#define FILE_NAME   					"NORDIC.TXT"
#define TEST_STRING 					"SD card example."
#define ROOT_DIR						"0:/"
#define SDC_SCK_PIN     				28  ///< SDC serial clock (SCK) pin.
#define SDC_MISO_PIN    				29  ///< SDC serial data out (DO) pin.
#define SDC_MOSI_PIN    				30  ///< SDC serial data in (DI) pin.
#define SDC_CS_PIN      				31  ///< SDC chip select (CS) pin.
#define SDC_BLOCK_SIZE					(SDC_SECTOR_SIZE)

/*                                    ADC                                     */
/* -------------------------------------------------------------------------- */
// SPI
#define ADC_SPI_CONV_PIN				22
#define ADC_SPI_MOSI_PIN				23
#define ADC_SPI_MISO_PIN				24
#define ADC_SPI_SCK_PIN					25
#define ADC_SPI_INSTANCE				1

// SYNC TIMER
#define ADC_SYNC_TIMER_INSTANCE			0
#define ADC_SYNC_44KHZ_US				23
#define ADC_SYNC_48KHZ_US				20


/*                                   GPS                                      */
/* -------------------------------------------------------------------------- */
#define GPS_NMEA_MAX_SIZE				100
#define GPS_NMEA_START_CHAR				0x24 // $
#define GPS_NMEA_STOP_CHAR				0x0A // LF

#define GPS_GGA_HEADER_STR				"$GPGGA"
#define GPS_GGA_HEADER_TOK				0
#define GPS_GGA_TIME_TOK				1
#define GPS_GGA_LAT_TOK					2
#define GPS_GGA_N_S_TOK					3
#define GPS_GGA_LONG_TOK				4
#define GPS_GGA_E_W_TOK					5
#define GPS_GGA_QUAL_TOK				6
#define GPS_GGA_SAT_TOK					7
#define GPS_GGA_DIL_TOK					8
#define GPS_GGA_ALT_TOK					9
#define GPS_GGA_ALT_UNIT_TOK			10
#define GPS_GGA_GEO_TOK					11
#define GPS_GGA_GEO_UNIT_TOK			12
#define GPS_GGA_DELTA_TOK				13
#define GPS_GGA_ST_ID_TOK				14
#define GPS_GGA_CHKS_TOK				15
#define GPS_GGA_TOKEN_MAX				16

#define GPS_RMC_HEADER_STR				"$GPRMC"
#define GPS_RMC_HEADER_TOK				0
#define GPS_RMC_TIME_TOK				1
#define GPS_RMC_STATUS_TOK				2
#define GPS_RMC_LAT_TOK					3
#define GPS_RMC_N_S_TOK					4
#define GPS_RMC_LONG_TOK				5
#define GPS_RMC_E_W_TOK					6
#define GPS_RMC_SPEED_TOK				7
#define GPS_RMC_TANGLE_TOK				8
#define GPS_RMC_DATE_TOK				9
#define GPS_RMC_MVAR_TOK				10
#define GPS_RMC_MVAR_E_W_TOK			11
#define GPS_RMC_INT_CHKS_TOK			12
#define GPS_RMC_TOKEN_MAX				13

#define GPS_CONV_KNOT_TO_KMH			(1.852)
#define GPS_CONV_KNOT_TO_MPH			(1.15078)
#define GPS_CONV_MPH_TO_KMH				(1.60934)
#define GPS_CONV_KMH_TO_KNOT			(1/GPS_CONV_KNOT_TO_KMH)
#define GPS_CONV_MPH_TO_KNOT			(1/GPS_CONV_KNOT_TO_MPH)
#define GPS_CONV_KMH_TO_MPH				(1/GPS_CONV_MPH_TO_KMH)


#endif /* MAIN_H__ */
