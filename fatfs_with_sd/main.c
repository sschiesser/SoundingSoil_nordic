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


#include "main.h"
/* ========================================================================== */
/*                                 VARIABLES                                  */
/* ========================================================================== */

/*                              ADC to SDC FIFO                               */
/* -------------------------------------------------------------------------- */
//app_fifo_t								m_adc2sd_fifo;
//uint8_t									m_fifo_buffer[FIFO_DATA_SIZE];

/*                                  SD card                                   */
/* -------------------------------------------------------------------------- */
//static FATFS *							sdc_fs;
//static uint8_t 							data_buffer[FIFO_DATA_SIZE] = {0};
//static volatile bool 					sdc_init_ok = false;
//static volatile bool					sdc_rtw = false;
//static volatile bool					sdc_writing = false;
//static uint8_t							sdc_block_cnt = 0;
//static FIL   							recording_fil;
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

/*                                    ADC                                     */
/* -------------------------------------------------------------------------- */
// SPI
//static const nrf_drv_spi_t				adc_spi = NRF_DRV_SPI_INSTANCE(ADC_SPI_INSTANCE);
//static volatile bool 					adc_spi_xfer_done = false;
//static volatile uint16_t 				adc_spi_xfer_counter = 0;
//static volatile uint32_t				adc_total_samples = 0;
// SYNC TIMER
//const nrf_drv_timer_t					ADC_SYNC_TIMER = NRF_DRV_TIMER_INSTANCE(ADC_SYNC_TIMER_INSTANCE);
// AUDIO BUFFER
//static uint8_t							m_tx_buf[2] = {0xFF, 0xFF};
//static uint8_t							m_rx_buf[2];
//static const uint8_t					m_length = 2;

/*                                    GPS                                     */
/* -------------------------------------------------------------------------- */
//struct gps_time {
//	uint8_t h;
//	uint8_t min;
//	uint8_t sec;
//	uint16_t msec;
//};
//struct gps_date {
//	uint8_t day;
//	uint8_t month;
//	uint8_t year;
//};
//struct gps_lat {
//	uint8_t deg;
//	uint8_t min;
//	uint16_t sec;
//	bool north;
//};
//struct gps_long {
//	uint8_t deg;
//	uint8_t min;
//	uint16_t sec;
//	bool east;
//};
//struct gps_altitude {
//	float number;
//	bool m_unit;
//};
//struct gps_geoid {
//	float number;
//	bool m_unit;
//};
//struct gps_speed {
//	float knots;
//	float mph;
//	float kmh;
//};
//struct gps_variation {
//	float angle;
//	bool east;
//};
//struct gps_gga_tag {
//	struct gps_time time;
//	struct gps_lat latitude;
//	struct gps_long longitude;
//	uint8_t quality;
//	uint8_t satellites;
//	float dilution;
//	struct gps_altitude altitude;
//	struct gps_geoid geoid;
//	char raw_tag[GPS_NMEA_MAX_SIZE];
//	uint8_t length;
//};
//struct gps_rmc_tag {
//	struct gps_time time;
//	bool status_active;
//	struct gps_lat latitude;
//	struct gps_long longitude;
//	struct gps_speed speed;
//	float track_angle;
//	struct gps_date date;
//	struct gps_variation mvar;
//	char sig_int;
//	char raw_tag[GPS_NMEA_MAX_SIZE];
//	uint8_t length;
//};

//enum gps_tag_type {
//	GPS_TAG_GGA,
//	GPS_TAG_RMC
//};

//static volatile bool gps_uart_reading = false;
//static volatile bool gps_uart_timeout = false;

//NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
//	GPS_UART_RX_PIN, NRF_UART_PSEL_DISCONNECTED,
//	NRF_UART_PSEL_DISCONNECTED, NRF_UART_PSEL_DISCONNECTED,
//	NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
//	NRF_UART_BAUDRATE_9600, UART_DEFAULT_CONFIG_IRQ_PRIORITY);

//NRF_SERIAL_CONFIG_DEF(gps_config,
//	NRF_SERIAL_MODE_POLLING, NULL,
//	NULL, NULL, NULL);

//NRF_SERIAL_UART_DEF(gps_uart, GPS_UART_INSTANCE);
//APP_TIMER_DEF(gps_uart_timer);

/*                                    UI                                      */
/* -------------------------------------------------------------------------- */
//static volatile bool					ui_rec_start_req = false;
//static volatile bool					ui_rec_stop_req = false;
//static volatile bool					ui_rec_running = false;
//static volatile bool					ui_mon_start_req = false;
//static volatile bool					ui_mon_stop_req = false;
//static volatile bool					ui_mon_running = false;
//static uint8_t							ui_sdc_init_cnt = 0;
//APP_TIMER_DEF(led_blink_timer);


/*                                    BLE                                     */
/* -------------------------------------------------------------------------- */
//BLE_SSS_DEF(m_sss);																// LED Button Service instance.
//NRF_BLE_GATT_DEF(m_gatt);														// GATT module instance.
//static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        // Handle of the current connection.


/* ========================================================================== */
/*                              UTIL FUNCTIONS                                */
/* ========================================================================== */
/* Slice string into tokens with 1-char delimiter (improvement of strtok()); */
//static char * strslice(char * str, char const * delim)
//{
//	static char * src = NULL;
//	char * p, * ret = 0;
//	
//	if(str != NULL) src = str;
//	
//	if (src == NULL) return NULL;
//	
//	if((p = strpbrk(src, delim)) != NULL) {
//		*p = 0;
//		ret = src;
//		src = ++p;
//	}
//	else if(*src) {
//		ret = src;
//		src = NULL;
//	}
//	
//	return ret;
//}


/* Scan & list files and directories in an SD card */
//FRESULT scan_files(char *path, uint16_t *nb_items)
//{
//	FRESULT res;
//	FILINFO fno;
//	DIR     dir;
//	int32_t i;
//	uint16_t items = 0;
//	char *  pc_fn;
//#if _USE_LFN
//	char c_lfn[_MAX_LFN + 1];
//	fno.lfname = c_lfn;
//	fno.lfsize = sizeof(c_lfn);
//#endif

//	/* Open the directory */
//	res = f_opendir(&dir, path);
//	if (res == FR_OK) {
//		i = strlen(path);
//		for (;;) {
//			res = f_readdir(&dir, &fno);
////			NRF_LOG_DEBUG("fno[%d]: %s, res %d", items, fno.fname, res);
//			if (res != FR_OK || fno.fname[0] == 0) {
//				*nb_items = items; // removing the '.' and '..' items
//				break;
//			}
//			items++;


//#if _USE_LFN
//			pc_fn = *fno.lfname ? fno.lfname : fno.fname;
//#else
//			pc_fn = fno.fname;
//#endif
//			if (*pc_fn == '.') {
//				continue;
//			}

//			/* Check if it is a directory type */
//			if (fno.fattrib & AM_DIR) {
//				uint16_t sub_items;
//				sprintf(&path[i], "/%s", pc_fn);
//				res = scan_files(path, &sub_items);
//				if (res != FR_OK) {
//					break;
//				}
//				items += sub_items;
//				path[i] = 0;
//			} else {
//				NRF_LOG_INFO("%s/%s\n\r", path, pc_fn);
//			}
//		}
//	}

//	return res;
//}


/* Get GGA GPS tag */
//static struct gps_gga_tag gps_get_gga_geotag(void)
//{
//	struct gps_gga_tag tag;
//	uint8_t cnt = 0;
///* REAL UART */
//	char c; // Read UART character
//	char uart_buf[GPS_NMEA_MAX_SIZE]; // Read UART buffer
//	ret_code_t ret; // Return value of the nrf_serial_read function
//	bool reading = true; // Reading flag
//	char *p_str; // Pointer on the string comparison result
//	while(reading) {
//		ret = nrf_serial_read(&gps_uart, &c, sizeof(c), NULL, 1000);
//		APP_ERROR_CHECK(ret);
//		uart_buf[cnt++] = c;
//		if(c == GPS_NMEA_STOP_CHAR) {
//			if(uart_buf[0] == GPS_NMEA_START_CHAR) {
//				static char comp[7] = "$GPGGA";
//				p_str = strstr(uart_buf, comp);
//				if(p_str != NULL) {
//					strcpy(tag.raw_tag, uart_buf);
//					tag.length = strlen(uart_buf);
//					reading = false;
//				}
//			}
//		cnt = 0;
//		}
//	}
///* FAKE UART */
////	strcpy(tag.raw_tag, "$GPGGA,123519.123,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
///* ------------------------ */
////	NRF_LOG_INFO("Raw tag: %s", tag.raw_tag);
//	char *tokens[GPS_GGA_TOKEN_MAX];
//	cnt = 0;
//	const char delim[2] = ",";
//	tokens[cnt] = strtok(tag.raw_tag, delim);
//	while(tokens[cnt] != NULL) {
//		cnt++;
//		tokens[cnt] = strtok(NULL, delim);
//	}
//	
//	char temp[12];
//	uint8_t len;
//	// Time
//	len = 2;
//	strncpy(temp, tokens[GPS_GGA_TIME_TOK], len);
//	temp[len] = '\0';
//	tag.time.h = atoi(temp);
//	strncpy(temp, tokens[GPS_GGA_TIME_TOK]+2, len);
//	temp[len] = '\0';
//	tag.time.min = atoi(temp);
//	strncpy(temp, tokens[GPS_GGA_TIME_TOK]+4, len);
//	temp[len] = '\0';
//	tag.time.sec = atoi(temp);
//	if(strchr(tokens[GPS_GGA_TIME_TOK], '.') != NULL) {
//		len = 3;
//		strncpy(temp, tokens[GPS_GGA_TIME_TOK]+7, len);
//		temp[len] = '\0';
//		tag.time.msec = atoi(temp);
//	}
//	// Latitude
//	len = 2;
//	strncpy(temp, tokens[GPS_GGA_LAT_TOK], len);
//	temp[len] = '\0';
//	tag.latitude.deg = atoi(temp);
//	strncpy(temp, tokens[GPS_GGA_LAT_TOK]+2, len);
//	temp[len] = '\0';
//	tag.latitude.min = atoi(temp);
//	len = 3;
//	strncpy(temp, tokens[GPS_GGA_LAT_TOK]+5, len);
//	temp[len] = '\0';
//	tag.latitude.sec = atoi(temp);
//	if(strncmp(tokens[GPS_GGA_N_S_TOK], "N", 1) == 0) tag.latitude.north = true;
//	else tag.latitude.north = false;
//	// Longitude
//	len = 3;
//	strncpy(temp, tokens[GPS_GGA_LONG_TOK], len);
//	temp[len] = '\0';
//	tag.longitude.deg = atoi(temp);
//	len = 2;
//	strncpy(temp, tokens[GPS_GGA_LONG_TOK]+3, len);
//	temp[len] = '\0';
//	tag.longitude.min = atoi(temp);
//	len = 3;
//	strncpy(temp, tokens[GPS_GGA_LONG_TOK]+6, len);
//	temp[len] = '\0';
//	tag.longitude.sec = atoi(temp);
//	if(strncmp(tokens[GPS_GGA_E_W_TOK], "E", 1) == 0) tag.longitude.east = true;
//	else tag.longitude.east = false;
//	// Quality
//	strncpy(temp, tokens[GPS_GGA_QUAL_TOK], strlen(tokens[GPS_GGA_QUAL_TOK]));
//	temp[1] = '\0';
//	tag.quality = atoi(temp);
//	// Satellites
//	len = strlen(tokens[GPS_GGA_SAT_TOK]);
//	strncpy(temp, tokens[GPS_GGA_SAT_TOK], len);
//	temp[len] = '\0';
//	tag.satellites = atoi(temp);
//	// Dilution
//	len = strlen(tokens[GPS_GGA_DIL_TOK]);
//	strncpy(temp, tokens[GPS_GGA_DIL_TOK], len);
//	temp[len] = '\0';
//	tag.dilution = atof(temp);
//	// Altitude
//	len = strlen(tokens[GPS_GGA_ALT_TOK]);
//	strncpy(temp, tokens[GPS_GGA_ALT_TOK], len);
//	temp[len] = '\0';
//	tag.altitude.number = atof(temp);
//	if(strncmp(tokens[GPS_GGA_ALT_UNIT_TOK], "M", 1) == 0) tag.altitude.m_unit = true;
//	else tag.altitude.m_unit = false;
//	// Geoid
//	len = strlen(tokens[GPS_GGA_GEO_TOK]);
//	strncpy(temp, tokens[GPS_GGA_GEO_TOK], len);
//	temp[len] = '\0';
//	tag.geoid.number = atof(temp);
//	if(strncmp(tokens[GPS_GGA_GEO_UNIT_TOK], "M", 1) == 0) tag.geoid.m_unit = true;
//	else tag.geoid.m_unit = false;
//	
//	return tag;
//}


/* Get RMC GPS tag */
//static struct gps_rmc_tag gps_get_rmc_geotag(void)
//{
//	struct gps_rmc_tag tag;
//	uint8_t cnt = 0;
//	char uart_buf[GPS_NMEA_MAX_SIZE]; // Read UART buffer
///* REAL UART */
//	char c; // Read UART character
//	gps_uart_reading = true; // Reading flag
//	gps_uart_timeout = false; // Timeout flag
//	char *p_str; // Pointer on the string comparison result
////	APP_ERROR_CHECK(app_timer_start(gps_uart_timer, APP_TIMER_TICKS(2000), NULL));
////	DBG_TOGGLE(DBG0_PIN);
//	while(gps_uart_reading) {
//		ret_code_t ret = nrf_serial_read(&gps_uart, &c, sizeof(c), NULL, 2000);
////		DBG_TOGGLE(DBG1_PIN);
//		if(ret == NRF_ERROR_TIMEOUT) {
//			NRF_LOG_DEBUG("UART timeout!");
//			gps_uart_reading = false;
//			gps_uart_timeout = true;
//			break;
//		}
//		uart_buf[cnt++] = c;
//		if(c == GPS_NMEA_STOP_CHAR) { // found STOP char
//			if(uart_buf[0] == GPS_NMEA_START_CHAR) { // START char already stored
//				static char comp[7] = "$GPRMC";
//				p_str = strstr(uart_buf, comp);
//				if(p_str != NULL) {
//					strcpy(tag.raw_tag, uart_buf);
//					tag.length = strlen(uart_buf);
//					app_timer_stop(gps_uart_timer);
////					DBG_TOGGLE(DBG0_PIN);
//					gps_uart_reading = false;
//				}
//			}
//			cnt = 0;
//		}
//	}
//	if(gps_uart_timeout) {
//		strcpy(tag.raw_tag, "$GPRMC, 000000,V,0000.000,N,00000.000,E,000.0,000.0,000000,00.0,W,N*00");
//		gps_uart_timeout = false;
//	}
///* FAKE UART */
////	strcpy(tag.raw_tag, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,03.1,W,S*6A");
///* ------------------------ */
//	NRF_LOG_INFO("Raw tag: %s", tag.raw_tag);
//	char *tokens[GPS_RMC_TOKEN_MAX];
//	cnt = 0;
//	const char delim[2] = ",";
//	strcpy(uart_buf, tag.raw_tag);
//	tokens[cnt] = strslice(uart_buf, delim);
//	while(tokens[cnt] != NULL) {
//		cnt++;
//		tokens[cnt] = strslice(NULL, delim);
//	}
//	
//	char temp[12];
//	uint8_t len;
//	// Time
//	len = 2;
//	strncpy(temp, tokens[GPS_RMC_TIME_TOK], len);
//	temp[len] = '\0';
//	tag.time.h = atoi(temp);
//	strncpy(temp, tokens[GPS_RMC_TIME_TOK]+2, len);
//	temp[len] = '\0';
//	tag.time.min = atoi(temp);
//	strncpy(temp, tokens[GPS_RMC_TIME_TOK]+4, len);
//	temp[len] = '\0';
//	tag.time.sec = atoi(temp);
//	if(strchr(tokens[GPS_RMC_TIME_TOK], '.') != NULL) {
//		len = 3;
//		strncpy(temp, tokens[GPS_RMC_TIME_TOK]+7, len);
//		temp[len] = '\0';
//		tag.time.msec = atoi(temp);
//	}
//	else {
//		tag.time.msec = 0;
//	}
//	// Status
//	if(strncmp(tokens[GPS_RMC_STATUS_TOK], "A", 1) == 0) tag.status_active = true;
//	else tag.status_active = false;
//	// Latitude
//	len = 2;
//	strncpy(temp, tokens[GPS_RMC_LAT_TOK], len);
//	temp[len] = '\0';
//	tag.latitude.deg = atoi(temp);
//	strncpy(temp, tokens[GPS_RMC_LAT_TOK]+2, len);
//	temp[len] = '\0';
//	tag.latitude.min = atoi(temp);
//	len = 3;
//	strncpy(temp, tokens[GPS_RMC_LAT_TOK]+5, len);
//	temp[len] = '\0';
//	tag.latitude.sec = atoi(temp);
//	if(strncmp(tokens[GPS_RMC_N_S_TOK], "N", 1) == 0) tag.latitude.north = true;
//	else tag.latitude.north = false;
//	// Longitude
//	len = 3;
//	strncpy(temp, tokens[GPS_RMC_LONG_TOK], len);
//	temp[len] = '\0';
//	tag.longitude.deg = atoi(temp);
//	len = 2;
//	strncpy(temp, tokens[GPS_RMC_LONG_TOK]+3, len);
//	temp[len] = '\0';
//	tag.longitude.min = atoi(temp);
//	len = 3;
//	strncpy(temp, tokens[GPS_RMC_LONG_TOK]+6, len);
//	temp[len] = '\0';
//	tag.longitude.sec = atoi(temp);
//	if(strncmp(tokens[GPS_RMC_E_W_TOK], "E", 1) == 0) tag.longitude.east = true;
//	else tag.longitude.east = false;
//	// Speed
//	len = strlen(tokens[GPS_RMC_SPEED_TOK]);
//	strncpy(temp, tokens[GPS_RMC_SPEED_TOK], len);
//	temp[len] = '\0';
//	tag.speed.knots = atof(temp);
//	tag.speed.mph = tag.speed.knots * (float)GPS_CONV_KNOT_TO_MPH;
//	tag.speed.kmh = tag.speed.knots * (float)GPS_CONV_KNOT_TO_KMH;
//	// Track angle	NRF_LOG_INFO("Tangle len: %d", strlen(tokens[GPS_RMC_TANGLE_TOK]));
//	len = strlen(tokens[GPS_RMC_TANGLE_TOK]);
//	strncpy(temp, tokens[GPS_RMC_TANGLE_TOK], len);
//	temp[len] = '\0';
//	tag.track_angle = atof(temp);
//	// Date
//	len = 2;
//	strncpy(temp, tokens[GPS_RMC_DATE_TOK], len);
//	temp[len] = '\0';
//	tag.date.day = atoi(temp);
//	strncpy(temp, tokens[GPS_RMC_DATE_TOK]+2, len);
//	temp[len] = '\0';
//	tag.date.month = atoi(temp);
//	strncpy(temp, tokens[GPS_RMC_DATE_TOK]+4, len);
//	temp[len] = '\0';
//	tag.date.year = atoi(temp);
//	// Magnetic variation
//	len = strlen(tokens[GPS_RMC_MVAR_TOK]);
//	strncpy(temp, tokens[GPS_RMC_MVAR_TOK], len);
//	temp[len] = '\0';
//	tag.mvar.angle = atof(temp);
//	if(strncmp(tokens[GPS_RMC_MVAR_E_W_TOK], "E", 1) == 0) tag.mvar.east = true;
//	else tag.mvar.east = false;
//	// Signal integrity
//	len = 1;
//	strncpy(temp, tokens[GPS_RMC_INT_CHKS_TOK], len);
//	temp[len] = '\0';
//	tag.sig_int = temp[0];
//	
//	return tag;
//}


/* Get geodata */
//void * gps_get_geodata(enum gps_tag_type tag_type, uint8_t retries)
//{
//	uint8_t tag_cnt = 0;
//	void *ret_tag;
//	switch(tag_type) {
//		case GPS_TAG_GGA:
//		{
//			struct gps_gga_tag cur_gga, ggas[retries];
//			while(tag_cnt < retries) {
//				cur_gga = gps_get_gga_geotag();
//				if(cur_gga.quality) ggas[tag_cnt++] = cur_gga;
//			}
//			ret_tag = (void *)&ggas[0];
//			break;
//		}
//		
//		case GPS_TAG_RMC:
//		{
//			struct gps_rmc_tag cur_rmc, rmcs[retries];
//			while(tag_cnt < retries) {
//				cur_rmc = gps_get_rmc_geotag();
//				if(cur_rmc.status_active) rmcs[tag_cnt++] = cur_rmc;
//			}
//			ret_tag = (void *)&rmcs[0];
//			break;
//		}
//		
//		default:
//		{
//			ret_tag = (void *)NULL;
//			break;
//		}
//	}
//	
//	return ret_tag;
//}

/* ========================================================================== */
/*                               APP FUNCTIONS                                */
/* ========================================================================== */
/* Start advertising */
//static void advertising_start(void)
//{
//    ret_code_t           err_code;
//    ble_gap_adv_params_t adv_params;

//    // Start advertising
//    memset(&adv_params, 0, sizeof(adv_params));

//    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
//    adv_params.p_peer_addr = NULL;
//    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
//    adv_params.interval    = APP_ADV_INTERVAL;
//    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

//    err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
//    APP_ERROR_CHECK(err_code);
//    bsp_board_led_on(ADVERTISING_LED);
//}
/* Fill SDC write queue */
//static void sdc_fill_queue(void)
//{
//	static FRESULT res;
//	static UINT byte_written;
//	static uint8_t p_buf[2*SDC_BLOCK_SIZE];
//	uint32_t buf_size = 2*SDC_BLOCK_SIZE;
//	uint32_t fifo_res = app_fifo_read(&m_adc2sd_fifo, p_buf, &buf_size);
//	sdc_writing = true;
//	res = f_write(&recording_fil, p_buf, (2*SDC_BLOCK_SIZE), &byte_written);
//	NRF_LOG_DEBUG("res: %d");
//	if(res == FR_OK) {
////		DBG_TOGGLE(DBG1_PIN);
//		res = f_sync(&recording_fil);
//		sdc_writing = false;
//		sdc_block_cnt--;
//	}
//}



//static bool gps_get_tag(void)
//{
//	bool retval = false;
//	
//	for(uint8_t i = 0; i < 3; i++) {
//		NRF_LOG_DEBUG("Reading...");
//		struct gps_rmc_tag cur_tag = gps_get_rmc_geotag();
//		NRF_LOG_DEBUG("GPS tag read!");
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
//			retval = true;
//		}
//		else {
//			NRF_LOG_INFO("Signal not valid!!");
//			retval = false;
//		}
//	}
//	return retval;
//}
/* ========================================================================== */
/*                              EVENT HANDLERS                                */
/* ========================================================================== */
/* SPI for ADC */
//void adc_spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
//{
//	static uint32_t buf_size = 2;
//	app_fifo_write(&m_adc2sd_fifo, m_rx_buf, &buf_size);
//	
//	if(adc_spi_xfer_counter < (SDC_BLOCK_SIZE-1)) {
//		adc_spi_xfer_counter++;
//	}
//	else {
////		DBG_TOGGLE(DBG0_PIN);
//		adc_total_samples += (2*adc_spi_xfer_counter);
//		sdc_rtw = true;
////		if(!sdc_writing) {
////			sdc_rtw = true;
////		}
////		else {
////			sdc_block_cnt++;
////		}
//		adc_spi_xfer_counter = 0;
//	}
//	adc_spi_xfer_done = true;
//}


/* TIMER for ADC */
//void adc_sync_timer_handler(nrf_timer_event_t event_type, void * p_context)
//{
////	DBG_TOGGLE(DBG0_PIN);
//	if(adc_spi_xfer_done) {
//		adc_spi_xfer_done = false;
//		nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);
//	}
//}


/* BLE event handler */
//static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
//{
//    ret_code_t err_code;

//    switch (p_ble_evt->header.evt_id)
//    {
//        case BLE_GAP_EVT_CONNECTED:
//            NRF_LOG_INFO("Connected");
//            bsp_board_led_on(CONNECTED_LED);
//            bsp_board_led_off(ADVERTISING_LED);
//            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

////            err_code = app_button_enable();
//            APP_ERROR_CHECK(err_code);
//            break;

//        case BLE_GAP_EVT_DISCONNECTED:
//            NRF_LOG_INFO("Disconnected");
//            bsp_board_led_off(CONNECTED_LED);
//            m_conn_handle = BLE_CONN_HANDLE_INVALID;
////            err_code = app_button_disable();
////            APP_ERROR_CHECK(err_code);
//            advertising_start();
//            break;

//        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
//            // Pairing not supported
//            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
//                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
//                                                   NULL,
//                                                   NULL);
//            APP_ERROR_CHECK(err_code);
//            break;

//#ifndef S140
//        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
//        {
//            NRF_LOG_DEBUG("PHY update request.");
//            ble_gap_phys_t const phys =
//            {
//                .rx_phys = BLE_GAP_PHY_AUTO,
//                .tx_phys = BLE_GAP_PHY_AUTO,
//            };
//            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
//            APP_ERROR_CHECK(err_code);
//        } break;
//#endif

//        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
//            // No system attributes have been stored.
//            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
//            APP_ERROR_CHECK(err_code);
//            break;

//        case BLE_GATTC_EVT_TIMEOUT:
//            // Disconnect on GATT Client timeout event.
//            NRF_LOG_DEBUG("GATT Client Timeout.");
//            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
//                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            APP_ERROR_CHECK(err_code);
//            break;

//        case BLE_GATTS_EVT_TIMEOUT:
//            // Disconnect on GATT Server timeout event.
//            NRF_LOG_DEBUG("GATT Server Timeout.");
//            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
//                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            APP_ERROR_CHECK(err_code);
//            break;

//        case BLE_EVT_USER_MEM_REQUEST:
//            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
//            APP_ERROR_CHECK(err_code);
//            break;

//        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
//        {
//            ble_gatts_evt_rw_authorize_request_t  req;
//            ble_gatts_rw_authorize_reply_params_t auth_reply;

//            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

//            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
//            {
//                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
//                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
//                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
//                {
//                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
//                    {
//                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
//                    }
//                    else
//                    {
//                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
//                    }
//                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
//                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
//                                                               &auth_reply);
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
//        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

//        default:
//            // No implementation needed.
//            break;
//    }
//}

/* LED */
//static void led_blink_handler(void * p_context)
//{
//	LED_TOGGLE(LED_RECORD);
//}
/* BUTTONS */
//static void button_event_handler(uint8_t pin_no, uint8_t button_action)
//{
////    ret_code_t err_code;

//	if(button_action) {
//		switch (pin_no)
//		{
//			case BUTTON_RECORD:
//				NRF_LOG_DEBUG("REC: rec_running = %d", ui_rec_running);
//				if(ui_rec_running || ui_rec_start_req) {
//					ui_rec_stop_req = true;
//					ui_rec_start_req = false;
//				}
//				else {
//					ui_sdc_init_cnt = 0;
//					ui_rec_start_req = true;
//					APP_ERROR_CHECK(app_timer_start(led_blink_timer, APP_TIMER_TICKS(200), NULL));
//				}
//				break;
//			case BUTTON_MONITOR:
//				NRF_LOG_DEBUG("MON: mon_running = %d", ui_mon_running);
//				if(ui_mon_running || ui_mon_start_req) {
//					ui_mon_stop_req = true;
//					ui_mon_start_req = false;
//				}
//				else {
//					ui_mon_start_req = true;
//				}
//				break;

//			default:
//				APP_ERROR_HANDLER(pin_no);
//				break;
//		}
//	}
//}
/* TIMER for GPS */
//static void gps_timeout_handler(void * p_context)
//{
//	NRF_LOG_INFO("GPS TIMEOUT");
////	DBG_TOGGLE(DBG1_PIN);
//	gps_uart_reading = false;
//	gps_uart_timeout = true;
//}
/* BLE connection parameter event reception */
//static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
//{
//    ret_code_t err_code;

//    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
//    {
//        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//        APP_ERROR_CHECK(err_code);
//    }
//}
/* BLE connection parameters error */
//static void conn_params_error_handler(uint32_t nrf_error)
//{
//    APP_ERROR_HANDLER(nrf_error);
//}

/* BLE LED REC */
//static void led_rec_handler(uint16_t conn_handle, ble_sss_t * p_sss, uint8_t led_state)
//{
//	if(led_state) {
//		NRF_LOG_DEBUG("REC START requested");
//		if(!ui_rec_running && !ui_rec_start_req) {
//			ui_sdc_init_cnt = 0;
//			ui_rec_start_req = true;
//		}
//		else {
//			NRF_LOG_DEBUG("REC already running...");
//		}
//	}
//	else {
//		NRF_LOG_DEBUG("REC STOP requested");
//		if(ui_rec_running || ui_rec_start_req) {
//			ui_rec_stop_req = true;
//			ui_rec_start_req = false;
//			ui_rec_running = false;
//		}
//		else {
//			NRF_LOG_DEBUG("No REC running...");
//		}
//	}
//}
/* BLE LED MON */
//static void led_mon_handler(uint16_t conn_handle, ble_sss_t * p_sss, uint8_t led_state)
//{
//	if(led_state) {
//		NRF_LOG_DEBUG("MON START requested");
//		if(!ui_mon_running && !ui_mon_start_req) {
//			ui_mon_start_req = true;
//		}
//		else {
//			NRF_LOG_DEBUG("MON already running...");
//		}
//	}
//	else {
//		NRF_LOG_DEBUG("MON STOP requested");
//		if(ui_mon_running || ui_mon_start_req) {
//			ui_mon_stop_req = true;
//			ui_mon_start_req = false;
//			ui_mon_running = false;
//		}
//		else {
//			NRF_LOG_DEBUG("No MON running...");
//		}
//	}
//}

/* ========================================================================== */
/*                                INIT/CONFIG                                 */
/* ========================================================================== */
// LFCLK (only debug mode without SoftDevice)
//static void lfclk_init(void)
//{
//    uint32_t err_code;
//    err_code = nrf_drv_clock_init();
//    APP_ERROR_CHECK(err_code);

//    nrf_drv_clock_lfclk_request(NULL);
//}
// TIMER
//static void timer_init(void)
//{
//    ret_code_t ret = app_timer_init();
//    APP_ERROR_CHECK(ret);
//}

// LED
static void leds_init(void)
{
	bsp_board_leds_init();
//	ret_code_t ret = app_timer_create(&led_blink_timer, APP_TIMER_MODE_REPEATED, led_blink_handler);
}

// BUTTON
//static void buttons_init(void)
//{
//    ret_code_t err_code;

//    //The array must be static because a pointer to it will be saved in the button handler module.
//    static app_button_cfg_t buttons[] =
//    {
//        {BUTTON_RECORD, false, BUTTON_PULL, button_event_handler},
//		{BUTTON_MONITOR, false, BUTTON_PULL, button_event_handler}
//    };

//    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY);
//    APP_ERROR_CHECK(err_code);
//}

// LOG
static void log_init(void)
{
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

// GPIO OUT for DBG
//static void gpio_dbg_init(void)
//{
//	ret_code_t err_code;
//	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
//	err_code = nrf_drv_gpiote_out_init(DBG0_PIN, &out_config);
//	APP_ERROR_CHECK(err_code);
//	err_code = nrf_drv_gpiote_out_init(DBG1_PIN, &out_config);
//	APP_ERROR_CHECK(err_code);
//}
// SPI for ADC
//static void adc_config_spi(void)
//{
//	nrf_drv_spi_config_t adc_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
//	adc_spi_config.ss_pin = ADC_SPI_CONV_PIN;
//	adc_spi_config.miso_pin = ADC_SPI_MISO_PIN;
//	adc_spi_config.mosi_pin = ADC_SPI_MOSI_PIN;
//	adc_spi_config.sck_pin = ADC_SPI_SCK_PIN;
//	adc_spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
//	adc_spi_config.irq_priority = 2;
//	APP_ERROR_CHECK(nrf_drv_spi_init(&adc_spi, &adc_spi_config, adc_spi_event_handler, NULL));
//}

// TIMER for ADC
//static void adc_config_timer(void)
//{
//	uint32_t err_code;
//	uint32_t time_ticks;
//	nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
//	timer_config.interrupt_priority = 3;
//	err_code = nrf_drv_timer_init(&ADC_SYNC_TIMER, &timer_config, adc_sync_timer_handler);
//	APP_ERROR_CHECK(err_code);
//	
//	time_ticks = nrf_drv_timer_us_to_ticks(&ADC_SYNC_TIMER, ADC_SYNC_44KHZ_US);

//	nrf_drv_timer_extended_compare(
//		&ADC_SYNC_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
//}

// BLE stack
//static void ble_stack_init(void)
//{
//    ret_code_t err_code;

//    err_code = nrf_sdh_enable_request();
//    APP_ERROR_CHECK(err_code);

//    // Configure the BLE stack using the default settings.
//    // Fetch the start address of the application RAM.
//    uint32_t ram_start = 0;
//    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
//    APP_ERROR_CHECK(err_code);

//    // Enable BLE stack.
//    err_code = nrf_sdh_ble_enable(&ram_start);
//    APP_ERROR_CHECK(err_code);

//    // Register a handler for BLE events.
//    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
//}

// Init SD card #0
//DSTATUS sdc_init(void)
//{
//    volatile DSTATUS disk_state = STA_NOINIT;

//    // Initialize FATFS disk I/O interface by providing the block device.
//    static diskio_blkdev_t drives[] =
//    {
//            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
//    };

//    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

//    NRF_LOG_DEBUG("Initializing disk 0 (SDC)...");
//    for (uint32_t retries = 3; retries && disk_state; --retries)
//    {
//        disk_state = disk_initialize(0);
//    }
//    if (disk_state)
//    {
//        NRF_LOG_INFO("Disk initialization failed. Disk state: %d", disk_state);
//        return disk_state;
//    }

//    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
//    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
//    NRF_LOG_DEBUG("Capacity: %d MB", capacity);
//	return RES_OK;
//}
// Un-init SD card #0
//DSTATUS sdc_uninit(void)
//{
//	volatile DSTATUS disk_state = STA_NOINIT;
//	disk_state = disk_uninitialize(0);
//	return disk_state;
//}


// Mount SD card and list content
//FRESULT sdc_mount(void)
//{
//	static FRESULT res;
//    static DIR dir;
//    static FILINFO fno;
//	
//	NRF_LOG_DEBUG("Mounting volume...");
// 	sdc_fs = malloc(sizeof(FATFS));
//	res = f_mount(sdc_fs, "", 1);
//    if (res) {
//        NRF_LOG_INFO("Mount failed. Result: %d", res);
//        return res;
//    }

//    NRF_LOG_DEBUG("Listing directory: /");
//    res = f_opendir(&dir, "/");
//    if (res) {
//        NRF_LOG_INFO("Directory listing failed!");
//        return res;
//    }

//    do {
//        res = f_readdir(&dir, &fno);
//        if (res != FR_OK) {
//            NRF_LOG_INFO("Directory read failed.");
//            return res;
//        }

//        if (fno.fname[0]) {
//            if (fno.fattrib & AM_DIR) {
//                NRF_LOG_DEBUG("  <DIR> %s",(uint32_t)fno.fname);
//            }
//            else {
//                NRF_LOG_DEBUG("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
//            }
//        }
//    }
//    while (fno.fname[0]);
//	
//	return FR_OK;
//}

// Init audio recording
//FRESULT sdc_init_audio(void)
//{	
//	FRESULT res;
//	TCHAR root_directory[4] = ROOT_DIR;
//	TCHAR current_folder_name[12];
//	TCHAR current_file_name[12];
//	TCHAR current_folder_location[25];
//	
//	/* ===== GET GPS DATA TO CREATE DIRECTORY ===== */
//	struct gps_rmc_tag cur_tag;
//	NRF_LOG_DEBUG("Reading...");
////	if(!gps_get_tag()) {
////		NRF_LOG_INFO("Failed to get GPS tag");
////		nrf_serial_uninit(&gps_uart);
////		nrf_delay_ms(1000);
////		gps_config_uart();
////	}
////	else {
////		NRF_LOG_INFO("Success with GPS tag");
////		nrf_delay_ms(1000);
////	}
//	cur_tag = gps_get_rmc_geotag();
//	NRF_LOG_DEBUG("Done!");
//	if(!cur_tag.status_active) {
//		NRF_LOG_INFO("No valid GPS info... creating dummy directory '%s' and file '%s'", DBG_CURRENT_FOLDER, DBG_CURRENT_FILE);
//		sprintf(current_folder_name, "%s", DBG_CURRENT_FOLDER);
//		sprintf(current_file_name, "%s", DBG_CURRENT_FILE);
//	}
//	else {
//		sprintf(current_folder_name, "%02d%02d%02d", cur_tag.date.year, cur_tag.date.month, cur_tag.date.day);
//		sprintf(current_folder_location, "%s%s", root_directory, current_folder_name);
//		
//		/* ===== CHECK DIRECTORY EXISTENCE ===== */
//		FILINFO filinfo;
//		bool makedir = false;
//		
//		res = f_stat(current_folder_location, &filinfo);
//		if(res == FR_OK) {
//			NRF_LOG_DEBUG("folder '%s' already exist", current_folder_name);
//		}
//		else if((res == FR_NO_PATH) || (res == FR_NO_FILE)) {
//			NRF_LOG_DEBUG("folder '%s' not found... create!", current_folder_name);
//			makedir = true;
//		}
//		else {
//			NRF_LOG_INFO("f_stat pb: 0x%X", res);
//			return res;
//		}
//		
//		/* ===== CREATE DIRECTORY ===== */
//		if(makedir) {
//			res = f_mkdir(current_folder_name);
//			if (res != FR_OK) {
//				NRF_LOG_INFO("f_mkdir pb: 0x%X", res);
//				return res;
//			}
//			else {
//				NRF_LOG_DEBUG("directory created");
//				makedir = false;
//			}
//		}

//		/* ===== OPEN DIRECTORY ===== */
//		NRF_LOG_DEBUG("Opening directory!");
//		DIR	dirinfo;
//		uint16_t scanned_files;
//		res = f_opendir(&dirinfo, current_folder_location);
//		if (res == FR_OK) {
//			/* Display the file tree */
//			NRF_LOG_DEBUG("Display files contained in the memory:");
//			strcpy((char *)data_buffer, current_folder_location);
//			scan_files((char *)data_buffer, &scanned_files);
//			NRF_LOG_DEBUG("Number of found files: %d", scanned_files);
//		}

//		
//		/* ===== CREATE NEW FILE ===== */
//		UINT bytes;
//		sprintf(current_file_name, "%s/R%02d%02d%02d.wav", current_folder_location, cur_tag.time.h, cur_tag.time.min, cur_tag.time.sec);
//		NRF_LOG_DEBUG("Create a file: '%s'", current_file_name);
//		res = f_open(&recording_fil, (char const *)current_file_name, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
//		if (res != FR_OK) {
//			NRF_LOG_INFO("Error while creating new file: #%d", res);
//			return res;
//		}
//		NRF_LOG_INFO("Writing 0s...");
//		for(uint32_t i = 0; i < (16 * SDC_BLOCK_SIZE); i++) {
//			f_write(&recording_fil, data_buffer, SDC_BLOCK_SIZE, (UINT *)&bytes);
//		}
//		NRF_LOG_INFO("Done!");
//			
//		f_lseek(&recording_fil, 0);
//		res = f_write(&recording_fil, wave_header, 44, (UINT *)&bytes);
//		if(res != FR_OK) {
//			NRF_LOG_INFO("Error while writing WAV header: #%d", res);
//			f_close(&recording_fil);
//			return res;
//		}
//	}

//	return FR_OK;
//}

// Close audio recording
//FRESULT sdc_close_audio(void)
//{
//	FRESULT res;
//	UINT bytes;

//	nrf_drv_timer_disable(&ADC_SYNC_TIMER);

//	((uint16_t *)&wave_header)[WAVE_FORMAT_NUM_CHANNEL_OFFSET/2] = AUDIO_NUM_CHANNELS;
//	((uint16_t *)&wave_header)[WAVE_FORMAT_BITS_PER_SAMPLE_OFFSET/2] = AUDIO_BITS_PER_SAMPLE;
//	((uint16_t *)&wave_header)[WAVE_FORMAT_BLOCK_ALIGN_OFFSET/2] = AUDIO_BITS_PER_SAMPLE/8 * AUDIO_NUM_CHANNELS;
//	((uint32_t *)&wave_header)[WAVE_FORMAT_SAMPLE_RATE_OFFSET/4] = AUDIO_SAMPLING_RATE;
//	((uint32_t *)&wave_header)[WAVE_FORMAT_BYTE_RATE_OFFSET/4] = AUDIO_SAMPLING_RATE * AUDIO_NUM_CHANNELS * AUDIO_BITS_PER_SAMPLE/8;
//	((uint32_t *)&wave_header)[WAVE_FORMAT_SUBCHUNK2_SIZE_OFFSET/4] = adc_total_samples * AUDIO_BITS_PER_SAMPLE/8;
//	((uint32_t *)&wave_header)[WAVE_FORMAT_CHUNK_SIZE_OFFSET/4] = (adc_total_samples * AUDIO_BITS_PER_SAMPLE/8) + 36;
//	
//	res = f_lseek(&recording_fil, 0);
//	res = f_write(&recording_fil, wave_header, 44, &bytes);
//	if(res == FR_OK) {
//		res = f_close(&recording_fil);
//	}
//	
//	return res;
//}

// UART & APP TIMER for GPS
//void gps_config_uart(void)
//{
//	ret_code_t ret = nrf_serial_init(&gps_uart, &m_uart0_drv_config, &gps_config);
//	APP_ERROR_CHECK(ret);
//	
//	ret = app_timer_create(&gps_uart_timer, APP_TIMER_MODE_SINGLE_SHOT, gps_timeout_handler);
//}



// BLE GAP parameters
//static void gap_params_init(void)
//{
//    ret_code_t              err_code;
//    ble_gap_conn_params_t   gap_conn_params;
//    ble_gap_conn_sec_mode_t sec_mode;

//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

//    err_code = sd_ble_gap_device_name_set(&sec_mode,
//                                          (const uint8_t *)DEVICE_NAME,
//                                          strlen(DEVICE_NAME));
//    APP_ERROR_CHECK(err_code);

//    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

//    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
//    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
//    gap_conn_params.slave_latency     = SLAVE_LATENCY;
//    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

//    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
//    APP_ERROR_CHECK(err_code);
//}


// BLE GATT
//static void gatt_init(void)
//{
//    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
//    APP_ERROR_CHECK(err_code);
//}

// BLE services
//static void services_init(void)
//{
//    ret_code_t     err_code;
//    ble_sss_init_t init;

//    init.led1_write_handler = led_rec_handler;
//	init.led2_write_handler = led_mon_handler;
//    err_code = ble_sss_init(&m_sss, &init);
//    APP_ERROR_CHECK(err_code);
//}
// BLE connection parameters
//static void conn_params_init(void)
//{
//    ret_code_t             err_code;
//    ble_conn_params_init_t cp_init;

//    memset(&cp_init, 0, sizeof(cp_init));

//    cp_init.p_conn_params                  = NULL;
//    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
//    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
//    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
//    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
//    cp_init.disconnect_on_fail             = false;
//    cp_init.evt_handler                    = on_conn_params_evt;
//    cp_init.error_handler                  = conn_params_error_handler;

//    err_code = ble_conn_params_init(&cp_init);
//    APP_ERROR_CHECK(err_code);
//}
// BLE advertisement
//static void advertising_init(void)
//{
//    ret_code_t    err_code;
//    ble_advdata_t advdata;
//    ble_advdata_t srdata;

//    ble_uuid_t adv_uuids[] = {{SSS_UUID_SERVICE, m_sss.uuid_type}};

//    // Build and set advertising data
//    memset(&advdata, 0, sizeof(advdata));

//    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
//    advdata.include_appearance = true;
//    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


//    memset(&srdata, 0, sizeof(srdata));
//    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
//    srdata.uuids_complete.p_uuids  = adv_uuids;

//    err_code = ble_advdata_set(&advdata, &srdata);
//    APP_ERROR_CHECK(err_code);
//}

/* ========================================================================== */
/*                                    MAIN                                    */
/* ========================================================================== */

static void fatfs_example()
{
    static FATFS fs;
    static DIR dir;
    static FILINFO fno;
    static FIL file;

    uint32_t bytes_written;
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
        return;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed.");
        return;
    }

    NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return;
    }

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }

        if (fno.fname[0])
        {
            if (fno.fattrib & AM_DIR)
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("");

    NRF_LOG_INFO("Writing to file " FILE_NAME "...");
    ff_result = f_open(&file, FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
        return;
    }

    ff_result = f_write(&file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_INFO("%d bytes written.", bytes_written);
    }

    (void) f_close(&file);
    return;
}




int main(void)
{
	DSTATUS card_status;
	FRESULT ff_result;
	ret_code_t err_code;
	
	// Board settings
//	timer_init(); 		// app_timer_init()
//	leds_init(); 		// bsp_board_leds_init() + app_timer_create((&led_blink_timer,...)
//	buttons_init(); 	// config + app_button_init()
//	log_init();			// NRF_LOG_INIT(NULL) + NRF_LOG_DEFAULT_BACKENDS_INIT()
//#ifdef DEBUG
//	gpio_dbg_init();	// config + nrf_drv_gpiote_out_init() for two debug pins
//#endif
	
	// SPI, DRV_TIMER, FIFO for LTC1864L ADC
//	adc_config_spi();
//	adc_config_timer();
//	app_fifo_init(&m_adc2sd_fifo, m_fifo_buffer, FIFO_DATA_SIZE);

	// UART for GPS
//	gps_config_uart();	// nrf_serial_init()
	
	// BLE
//	lfclk_init(); 		// nrf_drv_clock_init() + nrf_drv_clock_lfclk_request(NULL)... comment when ble stack active
//	ble_stack_init();
//	gap_params_init();
//	gatt_init();
//	services_init();
//	advertising_init();
//	conn_params_init();
	
	bsp_board_leds_init();
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();
	/* Starting application */
	/* -------------------- */
    NRF_LOG_INFO("=========================")
	NRF_LOG_INFO("FATFS + SD + SPI example.");
    NRF_LOG_INFO("-------------------------")
//	app_button_enable();
//	advertising_start();
	
	fatfs_example();
	while(true) {
		__WFE();
	}
//	for(;;) {
//		/* REC button pressed! (starting) */
//		if(ui_rec_start_req) {
//			NRF_LOG_INFO("REC Start request received");
//			card_status = sdc_init();
//			if(card_status == RES_OK) {
//				NRF_LOG_INFO("SD card init done.");
//				nrf_delay_ms(100);
//				ff_result = sdc_mount();
//				if(ff_result == FR_OK) {
//					NRF_LOG_INFO("SD card mounted.");
//					nrf_delay_ms(100);
//					ff_result = sdc_init_audio();
//					if(ff_result == FR_OK) {
//						NRF_LOG_INFO("Audio file ready to record.");
//						app_timer_stop(led_blink_timer);
//						LED_ON(LED_RECORD);
//						DBG_TOGGLE(DBG0_PIN);
//						ui_rec_start_req = false;
//						ui_rec_running = true;
//						sdc_init_ok = true;
//						err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 1);
//						if (err_code != NRF_SUCCESS &&
//							err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//							err_code != NRF_ERROR_INVALID_STATE &&
//							err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
//							APP_ERROR_CHECK(err_code);
//						}
//					}
//					else {
//						NRF_LOG_INFO("Unable to initialize audio file.");
//						f_mount(0, "", 1);
//						free(sdc_fs);
//						ui_sdc_init_cnt++;
//						nrf_delay_ms(500);
//					}
//				}
//				else {
//					NRF_LOG_INFO("SD card mounting failed. Result: %d", ff_result);
//					f_mount(0, "", 1);
//					free(sdc_fs);
//					ui_sdc_init_cnt++;
//					nrf_delay_ms(500);
//				}
//			}
//			else {
//				NRF_LOG_INFO("SD card check failed. Status: %d, Init cnt: %d", card_status, ui_sdc_init_cnt);
//				sdc_uninit();
//				ui_sdc_init_cnt++;
//				nrf_delay_ms(500);
//			}
//			
//			/* If SDC init failed, retry some times then fail */
//			if(!sdc_init_ok) {
//				if(ui_sdc_init_cnt < 10) {
//					NRF_LOG_INFO("Retrying...");
//					ui_rec_start_req = true;
//					sdc_init_ok = false;
//				}
//				else {
//					NRF_LOG_INFO("SDC init failed!");
//					app_timer_stop(led_blink_timer);
//					ui_rec_start_req = false;
//					sdc_init_ok = false;
//					LED_OFF(LED_RECORD);
//				}
//			}
//		}
//		
//		/* REC button pressed! (stopping) */
//		if(ui_rec_stop_req) {
//			NRF_LOG_INFO("REC Stop request received");
//			ff_result = sdc_close_audio();
//			if(ff_result == FR_OK) {
//				NRF_LOG_INFO("Done!");
//			}
//			else {
//				NRF_LOG_INFO("ERROR while closing audio file");
//			}
//			LED_OFF(LED_RECORD);
//			ui_rec_running = false;
//			ui_rec_start_req = false;
//			ui_rec_stop_req = false;
//			sdc_init_ok = false;
//			err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 0);
//			if (err_code != NRF_SUCCESS &&
//				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//				err_code != NRF_ERROR_INVALID_STATE &&
//				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
//				APP_ERROR_CHECK(err_code);
//			}
//		}
//		
//		/* MON button pressedè (starting) */
//		if(ui_mon_start_req) {
//			NRF_LOG_INFO("MON Start request received");
//			LED_ON(LED_MONITOR);
//			ui_mon_running = true;
//			ui_mon_start_req = false;
//			err_code = ble_sss_on_button2_change(m_conn_handle, &m_sss, 1);
//			if (err_code != NRF_SUCCESS &&
//				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//				err_code != NRF_ERROR_INVALID_STATE &&
//				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
//				APP_ERROR_CHECK(err_code);
//			}
//		}
//		
//		/* MON button pressed! (stopping) */
//		if(ui_mon_stop_req) {
//			NRF_LOG_INFO("MON Stop request received");
//			LED_OFF(LED_MONITOR);
//			ui_mon_running = false;
//			ui_mon_start_req = false;
//			ui_mon_stop_req = false;
//			err_code = ble_sss_on_button2_change(m_conn_handle, &m_sss, 0);
//			if( err_code != NRF_SUCCESS &&
//				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
//				err_code != NRF_ERROR_INVALID_STATE &&
//				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
//				APP_ERROR_CHECK(err_code);
//			}
//		}

//		/* SD card initialization done */
//		if(sdc_init_ok) {
//			NRF_LOG_INFO("Starting recording");
//			sdc_init_ok = false;
//			adc_spi_xfer_done = true;
//			nrf_drv_timer_enable(&ADC_SYNC_TIMER);
//		}
//		
//		if(sdc_rtw) {
//			DBG_TOGGLE(DBG1_PIN);
//			sdc_rtw = false;
//			if(!sdc_writing) {
//				sdc_fill_queue();
//			}
//			else {
//				sdc_block_cnt++;
////				NRF_LOG_INFO("Decounting FIFO...");
////				sdc_block_cnt--;
////				sdc_fill_queue();
//			}
//		}
//		
//        __WFE();
//    }
}

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

//    while (true)

//}

/** @} */
