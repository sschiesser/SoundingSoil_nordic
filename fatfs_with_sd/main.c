/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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
/** @file
 * @defgroup fatfs_example_main main.c
 * @{
 * @ingroup fatfs_example
 * @brief FATFS Example Application main file.
 *
 * This file contains the source code for a sample application using FAT filesystem and SD card library.
 *
 */


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
/*                                   DEBUG                                    */
/* ========================================================================== */
#define DBG0_PIN						9
#define DBG1_PIN						10
#define DBG2_PIN						11
#define DBG3_PIN						12
#define DBG_CURRENT_FOLDER				"180322"
#define DBG_CURRENT_FILE				"R123456.wav"
#define DBG_TOGGLE(dbg)					(nrf_drv_gpiote_out_toggle(dbg))

/* ========================================================================== */
/*                             LED/BUTTON MACROS                              */
/* ========================================================================== */
#define LED_RECORD						(BSP_LED_0)
#define LED_MONITOR						(BSP_LED_3)
#define LED_ON(led)						(nrf_drv_gpiote_out_clear(led))
#define LED_OFF(led)					(nrf_drv_gpiote_out_set(led))
#define LED_TOGGLE(led)					(nrf_drv_gpiote_out_toggle(led))

#define BUTTON_RECORD					(BSP_BUTTON_0)
#define BUTTON_MONITOR					(BSP_BUTTON_3)

/* ========================================================================== */
/*                              ADC to SDC FIFO                               */
/* ========================================================================== */
#define FIFO_DATA_SIZE					4096
app_fifo_t								m_adc2sd_fifo;
uint8_t									m_fifo_buffer[FIFO_DATA_SIZE];

/* ========================================================================== */
/*                                  SD card                                   */
/* ========================================================================== */
#define FILE_NAME   					"NORDIC.TXT"
#define TEST_STRING 					"SD card example."
#define ROOT_DIR						"0:/"
#define SDC_SCK_PIN     				28  ///< SDC serial clock (SCK) pin.
#define SDC_MISO_PIN    				29  ///< SDC serial data out (DO) pin.
#define SDC_MOSI_PIN    				30  ///< SDC serial data in (DI) pin.
#define SDC_CS_PIN      				31  ///< SDC chip select (CS) pin.

#define SDC_BLOCK_SIZE					SDC_SECTOR_SIZE
static uint8_t 							data_buffer[FIFO_DATA_SIZE];
static volatile bool 					sdc_init_ok = false;
static volatile bool					sdc_rtw = false;
static volatile bool					sdc_writing = false;
static uint8_t							sdc_block_cnt = 0;
static FIL   							recording_fil;
//static TCHAR							current_folder_name[13] = DBG_CURRENT_FOLDER;

#define ADC_SYNC_TIMER_INSTANCE			0
#define ADC_SYNC_44KHZ_US				23
#define ADC_SYNC_48KHZ_US				20
const nrf_drv_timer_t					ADC_SYNC_TIMER = NRF_DRV_TIMER_INSTANCE(ADC_SYNC_TIMER_INSTANCE);

/**
 * @brief  SDC block device definition                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

/* ========================================================================== */
/*                                    ADC                                     */
/* ========================================================================== */
// SPI
#define ADC_SPI_CONV_PIN				22
#define ADC_SPI_MOSI_PIN				23
#define ADC_SPI_MISO_PIN				24
#define ADC_SPI_SCK_PIN					25
#define ADC_SPI_INSTANCE				1
static const nrf_drv_spi_t				adc_spi = NRF_DRV_SPI_INSTANCE(ADC_SPI_INSTANCE);
static volatile bool 					adc_spi_xfer_done = false;
static volatile uint16_t 				adc_spi_xfer_counter = 0;
static volatile uint32_t				adc_frame_cnt = 0;
static volatile uint32_t				adc_total_samples = 0;

// Audio buffer
static uint8_t							m_tx_buf[2] = {0xFF, 0xFF};
static uint8_t							m_rx_buf[2];
static const uint8_t					m_length = 2;


/* ========================================================================== */
/*                                   GPS                                      */
/* ========================================================================== */
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

struct gps_time {
	uint8_t h;
	uint8_t min;
	uint8_t sec;
	uint16_t msec;
};
struct gps_date {
	uint8_t day;
	uint8_t month;
	uint8_t year;
};
struct gps_lat {
	uint8_t deg;
	uint8_t min;
	uint16_t sec;
	bool north;
};
struct gps_long {
	uint8_t deg;
	uint8_t min;
	uint16_t sec;
	bool east;
};
struct gps_altitude {
	float number;
	bool m_unit;
};
struct gps_geoid {
	float number;
	bool m_unit;
};
struct gps_speed {
	float knots;
	float mph;
	float kmh;
};
struct gps_variation {
	float angle;
	bool east;
};
struct gps_gga_tag {
	struct gps_time time;
	struct gps_lat latitude;
	struct gps_long longitude;
	uint8_t quality;
	uint8_t satellites;
	float dilution;
	struct gps_altitude altitude;
	struct gps_geoid geoid;
	char raw_tag[GPS_NMEA_MAX_SIZE];
	uint8_t length;
};
struct gps_rmc_tag {
	struct gps_time time;
	bool status_active;
	struct gps_lat latitude;
	struct gps_long longitude;
	struct gps_speed speed;
	float track_angle;
	struct gps_date date;
	struct gps_variation mvar;
	char sig_int;
	char raw_tag[GPS_NMEA_MAX_SIZE];
	uint8_t length;
};

enum gps_tag_type {
	GPS_TAG_GGA,
	GPS_TAG_RMC
};

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
	RX_PIN_NUMBER, TX_PIN_NUMBER,
	RTS_PIN_NUMBER, CTS_PIN_NUMBER,
	NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
	NRF_UART_BAUDRATE_9600, UART_DEFAULT_CONFIG_IRQ_PRIORITY);

NRF_SERIAL_CONFIG_DEF(gps_config,
	NRF_SERIAL_MODE_POLLING, NULL,
	NULL, NULL, NULL);

NRF_SERIAL_UART_DEF(gps_uart, 0);

/* ========================================================================== */
/*                                    UI                                      */
/* ========================================================================== */
static volatile bool					ui_rec_start_req = false;
static volatile bool					ui_rec_stop_req = false;
static volatile bool					ui_rec_running = false;
static uint8_t							ui_sdc_init_cnt = 0;


/* ========================================================================== */
/*                              UTIL FUNCTIONS                                */
/* ========================================================================== */
// Slice string into tokens with 1-char delimiter (improvement of strtok());
static char * strslice(char * str, char const * delim)
{
	static char * src = NULL;
	char * p, * ret = 0;
	
	if(str != NULL) src = str;
	
	if (src == NULL) return NULL;
	
	if((p = strpbrk(src, delim)) != NULL) {
		*p = 0;
		ret = src;
		src = ++p;
	}
	else if(*src) {
		ret = src;
		src = NULL;
	}
	
	return ret;
}


// Scan & list files and directories in an SD card
FRESULT scan_files(char *path, uint16_t *nb_items)
{
	FRESULT res;
	FILINFO fno;
	DIR     dir;
	int32_t i;
	uint16_t items = 0;
	char *  pc_fn;
#if _USE_LFN
	char c_lfn[_MAX_LFN + 1];
	fno.lfname = c_lfn;
	fno.lfsize = sizeof(c_lfn);
#endif

	/* Open the directory */
	res = f_opendir(&dir, path);
	if (res == FR_OK) {
		i = strlen(path);
		for (;;) {
			res = f_readdir(&dir, &fno);
//			NRF_LOG_DEBUG("fno[%d]: %s, res %d", items, fno.fname, res);
			if (res != FR_OK || fno.fname[0] == 0) {
				*nb_items = items; // removing the '.' and '..' items
				break;
			}
			items++;


#if _USE_LFN
			pc_fn = *fno.lfname ? fno.lfname : fno.fname;
#else
			pc_fn = fno.fname;
#endif
			if (*pc_fn == '.') {
				continue;
			}

			/* Check if it is a directory type */
			if (fno.fattrib & AM_DIR) {
				uint16_t sub_items;
				sprintf(&path[i], "/%s", pc_fn);
				res = scan_files(path, &sub_items);
				if (res != FR_OK) {
					break;
				}
				items += sub_items;
				path[i] = 0;
			} else {
				NRF_LOG_INFO("%s/%s\n\r", path, pc_fn);
			}
		}
	}

	return res;
}


// Get GGA GPS tag
static struct gps_gga_tag gps_get_gga_geotag(void)
{
	struct gps_gga_tag tag;
	uint8_t cnt = 0;
/* REAL UART */
	char c; // Read UART character
	char uart_buf[GPS_NMEA_MAX_SIZE]; // Read UART buffer
	ret_code_t ret; // Return value of the nrf_serial_read function
	bool reading = true; // Reading flag
	char *p_str; // Pointer on the string comparison result
	while(reading) {
		ret = nrf_serial_read(&gps_uart, &c, sizeof(c), NULL, 1000);
		APP_ERROR_CHECK(ret);
		uart_buf[cnt++] = c;
		if(c == GPS_NMEA_STOP_CHAR) {
			if(uart_buf[0] == GPS_NMEA_START_CHAR) {
				static char comp[7] = "$GPGGA";
				p_str = strstr(uart_buf, comp);
				if(p_str != NULL) {
					strcpy(tag.raw_tag, uart_buf);
					tag.length = strlen(uart_buf);
					reading = false;
				}
			}
		cnt = 0;
		}
	}
/* FAKE UART */
//	strcpy(tag.raw_tag, "$GPGGA,123519.123,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
/* ------------------------ */
//	NRF_LOG_INFO("Raw tag: %s", tag.raw_tag);
	char *tokens[GPS_GGA_TOKEN_MAX];
	cnt = 0;
	const char delim[2] = ",";
	tokens[cnt] = strtok(tag.raw_tag, delim);
	while(tokens[cnt] != NULL) {
		cnt++;
		tokens[cnt] = strtok(NULL, delim);
	}
	
	char temp[12];
	uint8_t len;
	// Time
	len = 2;
	strncpy(temp, tokens[GPS_GGA_TIME_TOK], len);
	temp[len] = '\0';
	tag.time.h = atoi(temp);
	strncpy(temp, tokens[GPS_GGA_TIME_TOK]+2, len);
	temp[len] = '\0';
	tag.time.min = atoi(temp);
	strncpy(temp, tokens[GPS_GGA_TIME_TOK]+4, len);
	temp[len] = '\0';
	tag.time.sec = atoi(temp);
	if(strchr(tokens[GPS_GGA_TIME_TOK], '.') != NULL) {
		len = 3;
		strncpy(temp, tokens[GPS_GGA_TIME_TOK]+7, len);
		temp[len] = '\0';
		tag.time.msec = atoi(temp);
	}
	// Latitude
	len = 2;
	strncpy(temp, tokens[GPS_GGA_LAT_TOK], len);
	temp[len] = '\0';
	tag.latitude.deg = atoi(temp);
	strncpy(temp, tokens[GPS_GGA_LAT_TOK]+2, len);
	temp[len] = '\0';
	tag.latitude.min = atoi(temp);
	len = 3;
	strncpy(temp, tokens[GPS_GGA_LAT_TOK]+5, len);
	temp[len] = '\0';
	tag.latitude.sec = atoi(temp);
	if(strncmp(tokens[GPS_GGA_N_S_TOK], "N", 1) == 0) tag.latitude.north = true;
	else tag.latitude.north = false;
	// Longitude
	len = 3;
	strncpy(temp, tokens[GPS_GGA_LONG_TOK], len);
	temp[len] = '\0';
	tag.longitude.deg = atoi(temp);
	len = 2;
	strncpy(temp, tokens[GPS_GGA_LONG_TOK]+3, len);
	temp[len] = '\0';
	tag.longitude.min = atoi(temp);
	len = 3;
	strncpy(temp, tokens[GPS_GGA_LONG_TOK]+6, len);
	temp[len] = '\0';
	tag.longitude.sec = atoi(temp);
	if(strncmp(tokens[GPS_GGA_E_W_TOK], "E", 1) == 0) tag.longitude.east = true;
	else tag.longitude.east = false;
	// Quality
	strncpy(temp, tokens[GPS_GGA_QUAL_TOK], strlen(tokens[GPS_GGA_QUAL_TOK]));
	temp[1] = '\0';
	tag.quality = atoi(temp);
	// Satellites
	len = strlen(tokens[GPS_GGA_SAT_TOK]);
	strncpy(temp, tokens[GPS_GGA_SAT_TOK], len);
	temp[len] = '\0';
	tag.satellites = atoi(temp);
	// Dilution
	len = strlen(tokens[GPS_GGA_DIL_TOK]);
	strncpy(temp, tokens[GPS_GGA_DIL_TOK], len);
	temp[len] = '\0';
	tag.dilution = atof(temp);
	// Altitude
	len = strlen(tokens[GPS_GGA_ALT_TOK]);
	strncpy(temp, tokens[GPS_GGA_ALT_TOK], len);
	temp[len] = '\0';
	tag.altitude.number = atof(temp);
	if(strncmp(tokens[GPS_GGA_ALT_UNIT_TOK], "M", 1) == 0) tag.altitude.m_unit = true;
	else tag.altitude.m_unit = false;
	// Geoid
	len = strlen(tokens[GPS_GGA_GEO_TOK]);
	strncpy(temp, tokens[GPS_GGA_GEO_TOK], len);
	temp[len] = '\0';
	tag.geoid.number = atof(temp);
	if(strncmp(tokens[GPS_GGA_GEO_UNIT_TOK], "M", 1) == 0) tag.geoid.m_unit = true;
	else tag.geoid.m_unit = false;
	
	return tag;
}


// Get RMC GPS tag
static struct gps_rmc_tag gps_get_rmc_geotag(void)
{
	NRF_LOG_DEBUG("Fetching current date&time from GPS");
	struct gps_rmc_tag tag;
	uint8_t cnt = 0;
	char uart_buf[GPS_NMEA_MAX_SIZE]; // Read UART buffer
/* REAL UART */
//	char c; // Read UART character
//	ret_code_t ret; // Return value of the nrf_serial_read function
//	bool reading = true; // Reading flag
//	char *p_str; // Pointer on the string comparison result
//	while(reading) {
//		ret = nrf_serial_read(&gps_uart, &c, sizeof(c), NULL, 1000);
//		NRF_LOG_DEBUG("Done!");
//		APP_ERROR_CHECK(ret);
////		NRF_LOG_RAW_INFO("%c", c);
//		uart_buf[cnt++] = c;
//		if(c == GPS_NMEA_STOP_CHAR) {
////			NRF_LOG_DEBUG("STOP!")
//			if(uart_buf[0] == GPS_NMEA_START_CHAR) {
////				NRF_LOG_DEBUG("START!");
//				static char comp[7] = "$GPRMC";
//				p_str = strstr(uart_buf, comp);
//				if(p_str != NULL) {
//					strcpy(tag.raw_tag, uart_buf);
//					tag.length = strlen(uart_buf);
////					NRF_LOG_DEBUG("FOUND! %s (%d)", tag.raw_tag, tag.length);
//					reading = false;
//				}
//			}
//		cnt = 0;
//		}
//	}
/* FAKE UART */
	strcpy(tag.raw_tag, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,03.1,W,S*6A");
/* ------------------------ */
	NRF_LOG_INFO("Raw tag: %s", tag.raw_tag);
	char *tokens[GPS_RMC_TOKEN_MAX];
	cnt = 0;
	const char delim[2] = ",";
	strcpy(uart_buf, tag.raw_tag);
	tokens[cnt] = strslice(uart_buf, delim);
	while(tokens[cnt] != NULL) {
		cnt++;
		tokens[cnt] = strslice(NULL, delim);
	}
//	for(uint8_t i = 0; i < GPS_RMC_TOKEN_MAX; i++) {
//		NRF_LOG_INFO("t[%d]: %s", i, tokens[i]);
//	}
	
	char temp[12];
	uint8_t len;
	// Time
	len = 2;
	strncpy(temp, tokens[GPS_RMC_TIME_TOK], len);
	temp[len] = '\0';
	tag.time.h = atoi(temp);
	strncpy(temp, tokens[GPS_RMC_TIME_TOK]+2, len);
	temp[len] = '\0';
	tag.time.min = atoi(temp);
	strncpy(temp, tokens[GPS_RMC_TIME_TOK]+4, len);
	temp[len] = '\0';
	tag.time.sec = atoi(temp);
	if(strchr(tokens[GPS_RMC_TIME_TOK], '.') != NULL) {
		len = 3;
		strncpy(temp, tokens[GPS_RMC_TIME_TOK]+7, len);
		temp[len] = '\0';
		tag.time.msec = atoi(temp);
	}
	else {
		tag.time.msec = 0;
	}
	// Status
	if(strncmp(tokens[GPS_RMC_STATUS_TOK], "A", 1) == 0) tag.status_active = true;
	else tag.status_active = false;
	// Latitude
	len = 2;
	strncpy(temp, tokens[GPS_RMC_LAT_TOK], len);
	temp[len] = '\0';
	tag.latitude.deg = atoi(temp);
	strncpy(temp, tokens[GPS_RMC_LAT_TOK]+2, len);
	temp[len] = '\0';
	tag.latitude.min = atoi(temp);
	len = 3;
	strncpy(temp, tokens[GPS_RMC_LAT_TOK]+5, len);
	temp[len] = '\0';
	tag.latitude.sec = atoi(temp);
	if(strncmp(tokens[GPS_RMC_N_S_TOK], "N", 1) == 0) tag.latitude.north = true;
	else tag.latitude.north = false;
	// Longitude
	len = 3;
	strncpy(temp, tokens[GPS_RMC_LONG_TOK], len);
	temp[len] = '\0';
	tag.longitude.deg = atoi(temp);
	len = 2;
	strncpy(temp, tokens[GPS_RMC_LONG_TOK]+3, len);
	temp[len] = '\0';
	tag.longitude.min = atoi(temp);
	len = 3;
	strncpy(temp, tokens[GPS_RMC_LONG_TOK]+6, len);
	temp[len] = '\0';
	tag.longitude.sec = atoi(temp);
	if(strncmp(tokens[GPS_RMC_E_W_TOK], "E", 1) == 0) tag.longitude.east = true;
	else tag.longitude.east = false;
	// Speed
	len = strlen(tokens[GPS_RMC_SPEED_TOK]);
	strncpy(temp, tokens[GPS_RMC_SPEED_TOK], len);
	temp[len] = '\0';
	tag.speed.knots = atof(temp);
	tag.speed.mph = tag.speed.knots * (float)GPS_CONV_KNOT_TO_MPH;
	tag.speed.kmh = tag.speed.knots * (float)GPS_CONV_KNOT_TO_KMH;
	// Track angle	NRF_LOG_INFO("Tangle len: %d", strlen(tokens[GPS_RMC_TANGLE_TOK]));
	len = strlen(tokens[GPS_RMC_TANGLE_TOK]);
	strncpy(temp, tokens[GPS_RMC_TANGLE_TOK], len);
	temp[len] = '\0';
	tag.track_angle = atof(temp);
	// Date
	len = 2;
	strncpy(temp, tokens[GPS_RMC_DATE_TOK], len);
	temp[len] = '\0';
	tag.date.day = atoi(temp);
	strncpy(temp, tokens[GPS_RMC_DATE_TOK]+2, len);
	temp[len] = '\0';
	tag.date.month = atoi(temp);
	strncpy(temp, tokens[GPS_RMC_DATE_TOK]+4, len);
	temp[len] = '\0';
	tag.date.year = atoi(temp);
	// Magnetic variation
	len = strlen(tokens[GPS_RMC_MVAR_TOK]);
	strncpy(temp, tokens[GPS_RMC_MVAR_TOK], len);
	temp[len] = '\0';
	tag.mvar.angle = atof(temp);
	if(strncmp(tokens[GPS_RMC_MVAR_E_W_TOK], "E", 1) == 0) tag.mvar.east = true;
	else tag.mvar.east = false;
	// Signal integrity
	len = 1;
	strncpy(temp, tokens[GPS_RMC_INT_CHKS_TOK], len);
	temp[len] = '\0';
	tag.sig_int = temp[0];
	
	return tag;
}


// Get geodata
void * gps_get_geodata(enum gps_tag_type tag_type, uint8_t retries)
{
	uint8_t tag_cnt = 0;
	void *ret_tag;
	switch(tag_type) {
		case GPS_TAG_GGA:
		{
			struct gps_gga_tag cur_gga, ggas[retries];
			while(tag_cnt < retries) {
				cur_gga = gps_get_gga_geotag();
				if(cur_gga.quality) ggas[tag_cnt++] = cur_gga;
			}
			ret_tag = (void *)&ggas[0];
			break;
		}
		
		case GPS_TAG_RMC:
		{
			struct gps_rmc_tag cur_rmc, rmcs[retries];
			while(tag_cnt < retries) {
				cur_rmc = gps_get_rmc_geotag();
				if(cur_rmc.status_active) rmcs[tag_cnt++] = cur_rmc;
			}
			ret_tag = (void *)&rmcs[0];
			break;
		}
		
		default:
		{
			ret_tag = (void *)NULL;
			break;
		}
	}
	
	return ret_tag;
}

/* ========================================================================== */
/*                              EVENT HANDLERS                                */
/* ========================================================================== */
// SPI for ADC
void adc_spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	static uint32_t buf_size = 2;
	app_fifo_write(&m_adc2sd_fifo, m_rx_buf, &buf_size);
	if(adc_spi_xfer_counter < (SDC_BLOCK_SIZE-1)) {
		adc_spi_xfer_counter++;
	}
	else {
//		DBG_TOGGLE(DBG1_PIN);
		adc_total_samples += (2*adc_spi_xfer_counter);
		adc_spi_xfer_counter = 0;
		if(!sdc_writing) {
			sdc_rtw = true;
		}
		else {
			sdc_block_cnt++;
		}
	}
	adc_spi_xfer_done = true;
}


// TIMER for ADC
void adc_sync_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
//	DBG_TOGGLE(DBG0_PIN);
	if(adc_spi_xfer_done) {
		adc_spi_xfer_done = false;
		nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);
	}
}


// BUTTONS
void button_rec_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nrf_delay_ms(50);
	if(!nrf_drv_gpiote_in_is_set(BUTTON_RECORD)) {
		NRF_LOG_INFO("Button pressed");
	}
	if(ui_rec_running || ui_rec_start_req) {
		ui_rec_stop_req = true;
		ui_rec_start_req = false;
		ui_rec_running = false;
	}
	else {
		ui_sdc_init_cnt = 0;
		ui_rec_start_req = true;
	}
}
void button_mon_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	LED_TOGGLE(LED_MONITOR);
}


/* ========================================================================== */
/*                                INIT/CONFIG                                 */
/* ========================================================================== */

// GPIO
static void gpio_init(void)
{
	ret_code_t err_code;
	err_code = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
	
	/* Setting board LED as GPIOTE outputs */
	err_code = nrf_drv_gpiote_out_init(LED_RECORD, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(LED_MONITOR, &out_config);
	APP_ERROR_CHECK(err_code);
	
	/* Setting DBG pins as GPIOTE outputs */
	err_code = nrf_drv_gpiote_out_init(DBG0_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(DBG1_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(DBG2_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(DBG3_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	
	/* Setting board buttons as GPIOTE inputs */
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	in_config.pull = NRF_GPIO_PIN_PULLUP;	
	err_code = nrf_drv_gpiote_in_init(BUTTON_RECORD, &in_config, button_rec_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BUTTON_RECORD, true);
	err_code = nrf_drv_gpiote_in_init(BUTTON_MONITOR, &in_config, button_mon_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BUTTON_MONITOR, true);
}


// SPI for ADC
static void adc_config_spi(void)
{
	nrf_drv_spi_config_t adc_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	adc_spi_config.ss_pin = ADC_SPI_CONV_PIN;
	adc_spi_config.miso_pin = ADC_SPI_MISO_PIN;
	adc_spi_config.mosi_pin = ADC_SPI_MOSI_PIN;
	adc_spi_config.sck_pin = ADC_SPI_SCK_PIN;
	adc_spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	adc_spi_config.irq_priority = 4;
	APP_ERROR_CHECK(nrf_drv_spi_init(&adc_spi, &adc_spi_config, adc_spi_event_handler, NULL));
}

// TIMER for ADC
static void adc_config_timer(void)
{
	uint32_t err_code;
	uint32_t time_ticks;
	nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
	timer_config.interrupt_priority = 5;
	err_code = nrf_drv_timer_init(&ADC_SYNC_TIMER, &timer_config, adc_sync_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	time_ticks = nrf_drv_timer_us_to_ticks(&ADC_SYNC_TIMER, ADC_SYNC_44KHZ_US);

	nrf_drv_timer_extended_compare(
		&ADC_SYNC_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, TIMER_SHORTS_COMPARE0_CLEAR_Msk, true);
}

// Init SD card #0
DSTATUS sdc_init(void)
{
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_DEBUG("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed. Disk state: %d", disk_state);
        return disk_state;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_DEBUG("Capacity: %d MB", capacity);
	return RES_OK;
}


// Mount SD card and list content
FRESULT sdc_mount(void)
{
    static FATFS fs;
	static FRESULT res;
    static DIR dir;
    static FILINFO fno;
	
	NRF_LOG_DEBUG("Mounting volume...");
    res = f_mount(&fs, "", 1);
    if (res) {
        NRF_LOG_INFO("Mount failed. Result: %d", res);
        return res;
    }

    NRF_LOG_DEBUG("Listing directory: /");
    res = f_opendir(&dir, "/");
    if (res) {
        NRF_LOG_INFO("Directory listing failed!");
        return res;
    }

    do {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK) {
            NRF_LOG_INFO("Directory read failed.");
            return res;
        }

        if (fno.fname[0]) {
            if (fno.fattrib & AM_DIR) {
                NRF_LOG_DEBUG("  <DIR> %s",(uint32_t)fno.fname);
            }
            else {
                NRF_LOG_DEBUG("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
	
	return FR_OK;
}

FRESULT sdc_init_audio(void)
{	
	FRESULT res;
	TCHAR root_directory[4] = ROOT_DIR;
	TCHAR current_folder_name[12];
	TCHAR current_file_name[12];
	TCHAR current_folder_location[25];
	
	/* ===== GET GPS DATA TO CREATE DIRECTORY ===== */
	struct gps_rmc_tag cur_tag;
	NRF_LOG_DEBUG("Reading...");
	cur_tag = gps_get_rmc_geotag();
	NRF_LOG_DEBUG("Done!");
	if(!cur_tag.status_active) {
		NRF_LOG_INFO("No valid GPS info... creating dummy directory '%s' and file '%s'", DBG_CURRENT_FOLDER, DBG_CURRENT_FILE);
		sprintf(current_folder_name, "%s", DBG_CURRENT_FOLDER);
		sprintf(current_file_name, "%s", DBG_CURRENT_FILE);
	}
	else {
		sprintf(current_folder_name, "%02d%02d%02d", cur_tag.date.year, cur_tag.date.month, cur_tag.date.day);
		sprintf(current_folder_location, "%s%s", root_directory, current_folder_name);
		
		/* ===== CHECK DIRECTORY EXISTENCE ===== */
		FILINFO filinfo;
		bool makedir = false;
		
		res = f_stat(current_folder_location, &filinfo);
		if(res == FR_OK) {
			NRF_LOG_DEBUG("folder '%s' already exist", current_folder_name);
		}
		else if((res == FR_NO_PATH) || (res == FR_NO_FILE)) {
			NRF_LOG_DEBUG("folder '%s' not found... create!", current_folder_name);
			makedir = true;
		}
		else {
			NRF_LOG_INFO("f_stat pb: 0x%X", res);
			return res;
		}
		
		/* ===== CREATE DIRECTORY ===== */
		if(makedir) {
			res = f_mkdir(current_folder_name);
			if (res != FR_OK) {
				NRF_LOG_INFO("f_mkdir pb: 0x%X", res);
				return res;
			}
			else {
				NRF_LOG_DEBUG("directory created");
				makedir = false;
			}
		}

		/* ===== OPEN DIRECTORY ===== */
		NRF_LOG_DEBUG("Opening directory!");
		DIR	dirinfo;
		uint16_t scanned_files;
		res = f_opendir(&dirinfo, current_folder_location);
		if (res == FR_OK) {
			/* Display the file tree */
			NRF_LOG_DEBUG("Display files contained in the memory:");
			strcpy((char *)data_buffer, current_folder_location);
			scan_files((char *)data_buffer, &scanned_files);
			NRF_LOG_DEBUG("Number of found files: %d", scanned_files);
		}

		
		/* ===== CREATE NEW FILE ===== */
		UINT bytes;
		sprintf(current_file_name, "%s/R%02d%02d%02d.wav", current_folder_location, cur_tag.time.h, cur_tag.time.min, cur_tag.time.sec);
		NRF_LOG_DEBUG("Create a file: '%s'", current_file_name);
		res = f_open(&recording_fil, (char const *)current_file_name, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
		if (res != FR_OK) {
			NRF_LOG_INFO("Error while creating new file: #%d", res);
			return res;
		}
			
		res = f_write(&recording_fil, wave_header, 44, (UINT *)&bytes);
		if(res != FR_OK) {
			NRF_LOG_INFO("Error while writing WAV header: #%d", res);
			f_close(&recording_fil);
			return res;
		}
	}

	return FR_OK;
}

FRESULT sdc_close_audio(void)
{
	FRESULT res;
//	UINT bytes;
	
	((uint16_t *)&wave_header)[WAVE_FORMAT_NUM_CHANNEL_OFFSET/2] = AUDIO_NUM_CHANNELS;
	((uint16_t *)&wave_header)[WAVE_FORMAT_BITS_PER_SAMPLE_OFFSET/2] = AUDIO_BITS_PER_SAMPLE;
	((uint16_t *)&wave_header)[WAVE_FORMAT_BLOCK_ALIGN_OFFSET/2] = AUDIO_BITS_PER_SAMPLE/8 * AUDIO_NUM_CHANNELS;
	((uint32_t *)&wave_header)[WAVE_FORMAT_SAMPLE_RATE_OFFSET/4] = AUDIO_SAMPLING_RATE;
	((uint32_t *)&wave_header)[WAVE_FORMAT_BYTE_RATE_OFFSET/4] = AUDIO_SAMPLING_RATE * AUDIO_NUM_CHANNELS * AUDIO_BITS_PER_SAMPLE/8;
	((uint32_t *)&wave_header)[WAVE_FORMAT_SUBCHUNK2_SIZE_OFFSET/4] = adc_total_samples * AUDIO_BITS_PER_SAMPLE/8;
	((uint32_t *)&wave_header)[WAVE_FORMAT_CHUNK_SIZE_OFFSET/4] = (adc_total_samples * AUDIO_BITS_PER_SAMPLE/8) + 36;
	
	res = f_lseek(&recording_fil, 0);
	if(res == FR_OK) {
		res = f_close(&recording_fil);
	}
	
	return res;
}
void gps_config_uart(void)
{
	ret_code_t ret;
	
	// app timer used to generate the serial sequence
    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

	ret = nrf_serial_init(&gps_uart, &m_uart0_drv_config, &gps_config);
	APP_ERROR_CHECK(ret);
}

static void sdc_fill_queue(void)
{
	static FRESULT res;
	static UINT byte_written;
	static uint8_t p_buf[2*SDC_BLOCK_SIZE];
	uint32_t buf_size = 2*SDC_BLOCK_SIZE;
	uint32_t fifo_res = app_fifo_read(&m_adc2sd_fifo, p_buf, &buf_size);
	sdc_writing = true;
	res = f_write(&recording_fil, p_buf, (2*SDC_BLOCK_SIZE), &byte_written);
	nrf_drv_gpiote_out_toggle(DBG3_PIN);
	if(res == FR_OK) {
		res = f_sync(&recording_fil);
		sdc_writing = false;
	}
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
	DSTATUS card_status;
	FRESULT ff_result;

    bsp_board_leds_init();
	
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("=========================")
	NRF_LOG_INFO("FATFS + SD + SPI example.");
    NRF_LOG_INFO("-------------------------")

	adc_config_spi();
	adc_config_timer();
	app_fifo_init(&m_adc2sd_fifo, m_fifo_buffer, FIFO_DATA_SIZE);
	gpio_init();
	gps_config_uart();
//	bool cont = true;
//	while(cont) {
//		NRF_LOG_DEBUG("Reading...");
//		struct gps_rmc_tag cur_tag = gps_get_rmc_geotag();
//		NRF_LOG_DEBUG("Done!");
//		cont = false;
//		if(cur_tag.status_active) {
//			NRF_LOG_INFO("GPS data: %02d.%02d.%02d", cur_tag.date.day, cur_tag.date.month, cur_tag.date.year);
//			NRF_LOG_INFO("GPS time: %02dh%02dm%02d.%03ds", cur_tag.time.h, cur_tag.time.min, cur_tag.time.sec, cur_tag.time.msec);
//			NRF_LOG_INFO("GPS latitude: %02d°%02d'%03d\" %s", 
//				cur_tag.latitude.deg, cur_tag.latitude.min,
//				cur_tag.latitude.sec, (cur_tag.latitude.north) ? "N" : "S");
//			NRF_LOG_INFO("GPS longitude: %03d°%02d'%03d\" %s", 
//				cur_tag.longitude.deg, cur_tag.longitude.min, 
//				cur_tag.longitude.sec, (cur_tag.longitude.east) ? "E" : "W");
//			NRF_LOG_INFO("GPS speed: " NRF_LOG_FLOAT_MARKER " km/h", NRF_LOG_FLOAT(cur_tag.speed.kmh));
//			NRF_LOG_INFO("GPS track angle: " NRF_LOG_FLOAT_MARKER "°", NRF_LOG_FLOAT(cur_tag.track_angle));
//			NRF_LOG_INFO("Mag variation: " NRF_LOG_FLOAT_MARKER " %s", 
//				NRF_LOG_FLOAT(cur_tag.mvar.angle), (cur_tag.mvar.east) ? "E" : "W");
//			NRF_LOG_INFO("Integrity: %c", cur_tag.sig_int);
//		}
//		else {
//			NRF_LOG_INFO("Signal not valid!!");
//		}		
////		nrf_delay_ms(1000);
//	}

//	card_status = sd_card_test();
//	
//	if(card_status != RES_OK) {
//		NRF_LOG_INFO("SD card check failed. Status: %d", card_status);
//	}
//	else {
//		NRF_LOG_INFO("SD card check OK");
//		ff_result = sd_card_init();
//		if(ff_result != FR_OK) {
//			NRF_LOG_INFO("SD card init failed. Result: %d", ff_result);
//		}
//		else {
//			NRF_LOG_INFO("\nSD card init OK");
//			sdc_init_ok = true;
//		}
//	}

//	if(sdc_init_ok) {
//		NRF_LOG_INFO("Starting SPI xfer");
//		adc_spi_xfer_done = true;
//		nrf_drv_timer_enable(&ADC_SYNC_TIMER);
////		nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);
//	}

    while (true)
    {
		
		if(ui_rec_start_req) {
			NRF_LOG_INFO("Start request received");
			card_status = sdc_init();
			if(card_status == RES_OK) {
				NRF_LOG_INFO("SD card init done.");
				ff_result = sdc_mount();
				if(ff_result == FR_OK) {
					NRF_LOG_INFO("SD card mounted.");
					ff_result = sdc_init_audio();
					if(ff_result == FR_OK) {
						NRF_LOG_INFO("Audio file ready to record.");
						LED_ON(BSP_LED_0);
						ui_rec_start_req = false;
						ui_rec_running = true;
						sdc_init_ok = true;
					}
					else {
						NRF_LOG_INFO("Unable to initialize audio file.");
						ui_sdc_init_cnt++;
						nrf_delay_ms(500);
					}
				}
				else {
					NRF_LOG_INFO("SD card init failed. Result: %d", ff_result);
					ui_sdc_init_cnt++;
					nrf_delay_ms(500);
				}
			}
			else {
				NRF_LOG_INFO("SD card check failed. Status: %d", card_status);
				ui_sdc_init_cnt++;
				nrf_delay_ms(500);
			}

			if(ui_sdc_init_cnt >= 10) {
				ui_rec_start_req = false;
				sdc_init_ok = false;
				LED_OFF(BSP_LED_0);
			}
		}
		
		if(ui_rec_stop_req) {
			nrf_drv_timer_disable(&ADC_SYNC_TIMER);
			NRF_LOG_INFO("Stop request received");
			ff_result = sdc_close_audio();
			LED_OFF(BSP_LED_0);
			if(ff_result == FR_OK) {
				NRF_LOG_INFO("Done!");
			}
			else {
				NRF_LOG_INFO("ERROR while closing audio file");
			}
			ui_rec_stop_req = false;
			sdc_init_ok = false;
		}

		
		if(sdc_init_ok) {
			NRF_LOG_INFO("Starting recording");
			sdc_init_ok = false;
			adc_spi_xfer_done = true;
			nrf_drv_timer_enable(&ADC_SYNC_TIMER);
		}
		if(sdc_rtw) {
			bsp_board_led_invert(BSP_BOARD_LED_3);
			sdc_rtw = false;
			sdc_fill_queue();
		}
		else if(sdc_block_cnt > 0) {
			NRF_LOG_INFO("Decounting FIFO...");
			sdc_block_cnt--;
			sdc_fill_queue();
		}
        __WFE();
    }
}

/** @} */
