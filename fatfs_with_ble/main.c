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
app_fifo_t								audio_fifo;
static uint8_t							fifo_buffer[FIFO_DATA_SIZE];

/*                                  SD card                                   */
/* -------------------------------------------------------------------------- */
// SDC block device definition                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);
static TCHAR							sdc_foldername[13] = "180424";
static TCHAR							sdc_folderpath[14] = "/180424";
static TCHAR							sdc_filename[12] = "R123456.wav";
static FATFS 							sdc_fs;
static DIR 								sdc_dir;
static FILINFO 							sdc_fno;
static FIL 								sdc_file;
static volatile bool 					sdc_init_ok = false;
static volatile bool					sdc_rtw = false;
static volatile bool					sdc_writing = false;
static volatile uint8_t					sdc_chunk_counter = 0;
static uint8_t							sdc_buffer[SDC_BLOCK_SIZE];


/*                                    ADC                                     */
/* -------------------------------------------------------------------------- */
static const nrf_drv_spi_t				adc_spi = NRF_DRV_SPI_INSTANCE(ADC_SPI_INSTANCE);
static volatile bool					adc_spi_xfer_done = false;
static uint8_t							adc_spi_txbuf[2] = {0xFF, 0xFF};
static uint8_t							adc_spi_rxbuf[2];
static const uint8_t					adc_spi_len = 2;

const nrf_drv_timer_t					ADC_SYNC_TIMER = NRF_DRV_TIMER_INSTANCE(ADC_SYNC_TIMER_INSTANCE);

static volatile uint32_t				adc_samples_counter = 0;
static volatile uint32_t				adc_total_samples = 0;

/*                                    GPS                                     */
/* -------------------------------------------------------------------------- */
static volatile bool 					gps_uart_reading = false;
static volatile bool					gps_uart_timeout = false;

NRF_SERIAL_DRV_UART_CONFIG_DEF(m_gps_uart_config,
	GPS_UART_RX_PIN, NRF_UART_PSEL_DISCONNECTED,
	NRF_UART_PSEL_DISCONNECTED, NRF_UART_PSEL_DISCONNECTED,
	NRF_UART_HWFC_DISABLED, NRF_UART_PARITY_EXCLUDED,
	NRF_UART_BAUDRATE_9600, UART_DEFAULT_CONFIG_IRQ_PRIORITY);
	
NRF_SERIAL_CONFIG_DEF(gps_config,
	NRF_SERIAL_MODE_POLLING, NULL,
	NULL, NULL, NULL);
	
NRF_SERIAL_UART_DEF(gps_uart, GPS_UART_INSTANCE);

APP_TIMER_DEF(gps_uart_timer);

/*                                    UI                                      */
/* -------------------------------------------------------------------------- */
static volatile bool 					ui_rec_start_req = false;
static volatile bool 					ui_rec_stop_req = false;
static volatile bool 					ui_rec_running = false;
static volatile bool					ui_mon_start_req = false;
static volatile bool 					ui_mon_stop_req = false;
static volatile bool 					ui_mon_running = false;
APP_TIMER_DEF(led_blink_timer);

/*                                    BLE                                     */
/* -------------------------------------------------------------------------- */
BLE_SSS_DEF(m_sss);																// LED Button Service instance.
NRF_BLE_GATT_DEF(m_gatt);														// GATT module instance.
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        // Handle of the current connection.

/* ========================================================================== */
/*                              APP FUNCTIONS                                 */
/* ========================================================================== */
// Slice string into tokens with 1-char delimiter (improvement of strtok())
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
// Init & mount SD card, create new recording file
static uint32_t sdc_start()
{
    FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;
    uint32_t bytes_written;

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
        NRF_LOG_DEBUG("Disk initialization failed. Disk state: %d", disk_state);
        return (uint32_t)disk_state;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_DEBUG("Capacity: %d MB", capacity);

    NRF_LOG_DEBUG("Mounting volume...");
    ff_result = f_mount(&sdc_fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_DEBUG("Mount failed. Result: %d", ff_result);
        return (uint32_t)ff_result;
    }

    NRF_LOG_DEBUG("\r\n Listing directory: /");
    ff_result = f_opendir(&sdc_dir, "/");
    if (ff_result)
    {
        NRF_LOG_DEBUG("Directory listing failed!");
        return (uint32_t)ff_result;
    }

 	bool dir_found = false;
	do {
        ff_result = f_readdir(&sdc_dir, &sdc_fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return (uint32_t)ff_result;
        }

        if (sdc_fno.fname[0])
        {
            if (sdc_fno.fattrib & AM_DIR)
            {
                NRF_LOG_DEBUG("   <DIR>   %s",(uint32_t)sdc_fno.fname);
				uint8_t res = strcmp(sdc_fno.fname, sdc_foldername);
				NRF_LOG_DEBUG("Comp result: %d", res);
				if(res == 0) {
					dir_found = true;
					NRF_LOG_DEBUG("DIR FOUND! Path: %s", sdc_folderpath);
				}
            }
            else
            {
                NRF_LOG_DEBUG("%9lu  %s", sdc_fno.fsize, (uint32_t)sdc_fno.fname);
            }
        }
    } while (sdc_fno.fname[0]);
	
	
	if(!dir_found) {
		NRF_LOG_DEBUG("DIR not found... creating");
		ff_result = f_mkdir(sdc_folderpath);
		if(ff_result != FR_OK) {
			NRF_LOG_DEBUG("Unable to create directory");
			return (uint32_t)ff_result;
		}
		ff_result = f_opendir(&sdc_dir, sdc_folderpath);
		if(ff_result != FR_OK) {
			NRF_LOG_DEBUG("Unable to open directory");
			return (uint32_t)ff_result;
		}
	}
	else {
		NRF_LOG_DEBUG("DIR found... opening");
		ff_result = f_chdir(sdc_folderpath);
		if(ff_result != FR_OK) {
			NRF_LOG_DEBUG("Unable to change directory");
			return (uint32_t)ff_result;
		}
	}
	
    NRF_LOG_DEBUG("Creating file %s...", sdc_filename);
    ff_result = f_open(&sdc_file, sdc_filename, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if (ff_result != FR_OK)
    {
        NRF_LOG_DEBUG("Unable to open or create file: %s", sdc_filename);
        return (uint32_t)ff_result;
    }
	
	NRF_LOG_DEBUG("Writing WAV header...");
	ff_result = f_write(&sdc_file, wave_header, 44, (UINT *) &bytes_written);
	if (ff_result != FR_OK) {
		NRF_LOG_DEBUG("Unable to write WAV header");
		return (uint32_t)ff_result;
	}
	
	return (uint32_t)0;
}

// Write audio chunk to opened file
static FRESULT sdc_write(void)
{
    uint32_t bytes_written;
    FRESULT ff_result;
	uint32_t buf_size = SDC_BLOCK_SIZE;
	
	sdc_writing = true;
	NRF_LOG_DEBUG("Reading fifo...");
	uint32_t fifo_res = app_fifo_read(&audio_fifo, sdc_buffer, &buf_size);

	DBG_TOGGLE(DBG1_PIN);
	
    NRF_LOG_DEBUG("Writing to file %s...", sdc_filename);
    ff_result = f_write(&sdc_file, sdc_buffer, buf_size, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_DEBUG("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_DEBUG("%d bytes written.", bytes_written);
    }
	
//	DBG_TOGGLE(DBG1_PIN);

//	f_sync(&sdc_file);
	sdc_writing = false;

//	DBG_TOGGLE(DBG1_PIN);

	if(sdc_chunk_counter > 0) {
		sdc_rtw = true;
		sdc_chunk_counter--;
	}
	return ff_result;
}

// Write values to WAV header & close recording file
static FRESULT sdc_close(void)
{
	FRESULT ff_result;
	UINT bytes;
	
	((uint16_t *)&wave_header)[WAVE_FORMAT_NUM_CHANNEL_OFFSET/2] = AUDIO_NUM_CHANNELS;
	((uint16_t *)&wave_header)[WAVE_FORMAT_BITS_PER_SAMPLE_OFFSET/2] = AUDIO_BITS_PER_SAMPLE;
	((uint16_t *)&wave_header)[WAVE_FORMAT_BLOCK_ALIGN_OFFSET/2] = AUDIO_BITS_PER_SAMPLE/8 * AUDIO_NUM_CHANNELS;
	((uint32_t *)&wave_header)[WAVE_FORMAT_SAMPLE_RATE_OFFSET/4] = AUDIO_SAMPLING_RATE;
	((uint32_t *)&wave_header)[WAVE_FORMAT_BYTE_RATE_OFFSET/4] = AUDIO_SAMPLING_RATE * AUDIO_NUM_CHANNELS * AUDIO_BITS_PER_SAMPLE/8;
	((uint32_t *)&wave_header)[WAVE_FORMAT_SUBCHUNK2_SIZE_OFFSET/4] = adc_total_samples * AUDIO_BITS_PER_SAMPLE/8;
	((uint32_t *)&wave_header)[WAVE_FORMAT_CHUNK_SIZE_OFFSET/4] = (adc_total_samples * AUDIO_BITS_PER_SAMPLE/8) + 36;
	
	ff_result = f_lseek(&sdc_file, 0);
	if(ff_result != FR_OK) {
		NRF_LOG_DEBUG("Error while seeking for beginning of file");
		return ff_result;
	}
	
	ff_result = f_write(&sdc_file, wave_header, 44, &bytes);
	if(ff_result != FR_OK) {
		NRF_LOG_DEBUG("Error while updating the WAV header");
		return ff_result;
	}
	
    (void)f_close(&sdc_file);
    return ff_result;
}



// Start advertising
static void advertising_start(void)
{
    ret_code_t           err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
    LED_ON(LED_ADVERTISING);
}

static struct gps_rmc_tag gps_get_rmc_geotag(void)
{
	struct gps_rmc_tag tag;
	uint8_t cnt = 0;
	char uart_buf[GPS_NMEA_MAX_SIZE]; // Read UART buffer
/* REAL UART */
	char c; // Read UART character
	gps_uart_reading = true; // Reading flag
	gps_uart_timeout = false; // Timeout flag
	char *p_str; // Pointer on the string comparison result
	while(gps_uart_reading) {
		ret_code_t ret = nrf_serial_read(&gps_uart, &c, sizeof(c), NULL, 2000);
		if(ret == NRF_ERROR_TIMEOUT) {
			NRF_LOG_DEBUG("UART timeout!");
			gps_uart_reading = false;
			gps_uart_timeout = true;
			break;
		}
		uart_buf[cnt++] = c;
		if(c == GPS_NMEA_STOP_CHAR) { // found STOP char
			if(uart_buf[0] == GPS_NMEA_START_CHAR) { // START char already stored
				static char comp[7] = "$GPRMC";
				p_str = strstr(uart_buf, comp);
				if(p_str != NULL) {
					strcpy(tag.raw_tag, uart_buf);
					tag.length = strlen(uart_buf);
					app_timer_stop(gps_uart_timer);
					gps_uart_reading = false;
				}
			}
			cnt = 0;
		}
	}
	if(gps_uart_timeout) {
		strcpy(tag.raw_tag, "$GPRMC, 000000,V,0000.000,N,00000.000,E,000.0,000.0,000000,00.0,W,N*00");
		gps_uart_timeout = false;
	}
/* FAKE UART */
//	strcpy(tag.raw_tag, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,03.1,W,S*6A");
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
static void gps_poll_data(void)
{
	struct gps_rmc_tag gps_cur_tag = gps_get_rmc_geotag();
	char temp[6];
	
	sprintf(temp, "%02d%02d%02d", gps_cur_tag.date.year, gps_cur_tag.date.month, gps_cur_tag.date.day);
	memcpy(sdc_foldername, temp, 6);
	sprintf(sdc_folderpath, "/%s", sdc_foldername);
	NRF_LOG_DEBUG("Folder name %s, folder path %s", sdc_foldername, sdc_folderpath);
	
	sprintf(temp, "%02d%02d%02d", gps_cur_tag.time.h, gps_cur_tag.time.min, gps_cur_tag.time.sec);
	memcpy(&sdc_filename[1], temp, 6);
	NRF_LOG_DEBUG("sdc_filename: %s", sdc_filename);
}
/* ========================================================================== */
/*                              EVENT HANDLERS                                */
/* ========================================================================== */
// ADC SPI
void adc_spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	static uint32_t size = adc_spi_len;
	app_fifo_write(&audio_fifo, adc_spi_rxbuf, &size);
	adc_samples_counter += 2;
	adc_spi_xfer_done = true;
}

// ADC timer
void adc_sync_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	DBG_TOGGLE(DBG0_PIN);
	if(adc_spi_xfer_done) {
		adc_spi_xfer_done = false;
		nrf_drv_spi_transfer(&adc_spi, adc_spi_txbuf, adc_spi_len, adc_spi_rxbuf, adc_spi_len);
		if(adc_samples_counter >= SDC_BLOCK_SIZE) {
			adc_total_samples += SDC_BLOCK_SIZE;
			adc_samples_counter = 0;
			sdc_rtw = true;
		}
	}
}
// BLE
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            LED_ON(LED_CONNECTED);
            LED_OFF(LED_ADVERTISING);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            LED_OFF(LED_CONNECTED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}



// BLE connection parameter event reception
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

// BLE connection parameters error
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

// BUTTONS
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
	if(button_action) {
		switch (pin_no)
		{
			case BUTTON_RECORD:
				NRF_LOG_DEBUG("REC! state: %d", ui_rec_running);
				if(ui_rec_running || ui_rec_start_req) {
					ui_rec_stop_req = true;
					ui_rec_start_req = false;
					ui_rec_running = false;
				}
				else {
					ui_rec_start_req = true;
				}
				break;
				
			case BUTTON_MONITOR:
				NRF_LOG_DEBUG("MON: state = %d", ui_mon_running);
				if(ui_mon_running || ui_mon_start_req) {
					ui_mon_stop_req = true;
					ui_mon_start_req = false;
					ui_mon_running = false;
				}
				else {
					ui_mon_start_req = true;
				}
				break;

			default:
				APP_ERROR_HANDLER(pin_no);
				break;
		}
	}
}


// LED blink
static void led_blink_handler(void * p_context)
{
	LED_TOGGLE(LED_RECORD);
}

// LED REC
static void led_rec_handler(uint16_t conn_handle, ble_sss_t * p_sss, uint8_t led_state)
{
	if(led_state) {
		NRF_LOG_DEBUG("REC START requested");
		if(!ui_rec_running && !ui_rec_start_req) {
			ui_rec_start_req = true;
		}
		else {
			NRF_LOG_DEBUG("REC already running...");
		}
	}
	else {
		NRF_LOG_DEBUG("REC STOP requested");
		if(ui_rec_running || ui_rec_start_req) {
			ui_rec_stop_req = true;
			ui_rec_start_req = false;
			ui_rec_running = false;
		}
		else {
			NRF_LOG_DEBUG("No REC running...");
		}
	}
}
// LED MON
static void led_mon_handler(uint16_t conn_handle, ble_sss_t * p_sss, uint8_t led_state)
{
	if(led_state) {
		NRF_LOG_DEBUG("MON START requested");
		if(!ui_mon_running && !ui_mon_start_req) {
			ui_mon_start_req = true;
		}
		else {
			NRF_LOG_DEBUG("MON already running...");
		}
	}
	else {
		NRF_LOG_DEBUG("MON STOP requested");
		if(ui_mon_running || ui_mon_start_req) {
			ui_mon_stop_req = true;
			ui_mon_start_req = false;
			ui_mon_running = false;
		}
		else {
			NRF_LOG_DEBUG("No MON running...");
		}
	}
}

/* ========================================================================== */
/*                                INIT/CONFIG                                 */
/* ========================================================================== */
// ADC timer
static void adc_config_timer(void)
{
	uint32_t time_ticks;
	nrf_drv_timer_config_t timer_config = NRF_DRV_TIMER_DEFAULT_CONFIG;
	timer_config.interrupt_priority = 3;
	ret_code_t err_code = nrf_drv_timer_init(&ADC_SYNC_TIMER, &timer_config, adc_sync_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	time_ticks = nrf_drv_timer_us_to_ticks(&ADC_SYNC_TIMER, ADC_SYNC_44KHZ_US);

	nrf_drv_timer_extended_compare(
		&ADC_SYNC_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}
	
// ADC SPI
static void adc_config_spi(void)
{
	nrf_drv_spi_config_t adc_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	adc_spi_config.ss_pin = ADC_SPI_CONV_PIN;
	adc_spi_config.miso_pin = ADC_SPI_MISO_PIN;
	adc_spi_config.mosi_pin = ADC_SPI_MOSI_PIN;
	adc_spi_config.sck_pin = ADC_SPI_SCK_PIN;
	adc_spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	adc_spi_config.irq_priority = 2;
	APP_ERROR_CHECK(nrf_drv_spi_init(&adc_spi, &adc_spi_config, adc_spi_event_handler, NULL));
}

// BLE advertisement
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{SSS_UUID_SERVICE, m_sss.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &srdata);
    APP_ERROR_CHECK(err_code);
}
// BLE connection parameters
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
// BLE GAP parameters
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}
// BLE GATT
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

// BLE services
static void services_init(void)
{
    ret_code_t     err_code;
    ble_sss_init_t init;

    init.led1_write_handler = led_rec_handler;
	init.led2_write_handler = led_mon_handler;
    err_code = ble_sss_init(&m_sss, &init);
    APP_ERROR_CHECK(err_code);
}

// BLE stack
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

// BUTTONS
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BUTTON_RECORD, false, BUTTON_PULL, button_event_handler},
		{BUTTON_MONITOR, false, BUTTON_PULL, button_event_handler}
    };

	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}


// DEBUG GPIO out
static void gpio_dbg_init(void)
{
	ret_code_t err_code;
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
	err_code = nrf_drv_gpiote_out_init(DBG0_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(DBG1_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
}

// GPS UART & TIMER
void gps_init(void)
{
	ret_code_t err_code = nrf_serial_init(&gps_uart, &m_gps_uart_config, &gps_config);
	APP_ERROR_CHECK(err_code);
	
//	err_code = app_timer_create(&gps_uart_timer, APP_TIMER_MODE_SINGLE_SHOT, gps_timeout_handler);
}

// LEDS
static void leds_init(void)
{
	bsp_board_leds_init();
	ret_code_t err_code = app_timer_create(&led_blink_timer, APP_TIMER_MODE_REPEATED, led_blink_handler);
}

// LOG
static void log_init(void)
{
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
	ret_code_t err_code;
	
	leds_init();
	log_init();
	buttons_init();
#ifdef DEBUG
	gpio_dbg_init();
#endif
	
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();
	
	adc_config_spi();
	adc_config_timer();
	app_fifo_init(&audio_fifo, fifo_buffer, FIFO_DATA_SIZE);
	gps_init();
	
	/* Starting application */
	/* -------------------- */
    NRF_LOG_INFO("===========")
	NRF_LOG_INFO("FATFS + BLE");
    NRF_LOG_INFO("-----------")

 	app_button_enable();
	advertising_start();
	
	for(;;)
    {
		// REC START request
		if(ui_rec_start_req) {
			NRF_LOG_INFO("Starting REC");
			app_timer_start(led_blink_timer, APP_TIMER_TICKS(200), NULL);
			gps_poll_data();
			if(sdc_start() == 0) {
				sdc_init_ok = true;
			}
			ui_rec_start_req = false;
		}
		// REC STOP request
		if(ui_rec_stop_req) {
			NRF_LOG_INFO("Stopping REC");
			nrf_drv_timer_disable(&ADC_SYNC_TIMER);
			sdc_close();
			LED_OFF(LED_RECORD);
			err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 0);
			if (err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
				APP_ERROR_CHECK(err_code);
			}
			ui_rec_stop_req = false;
			ui_rec_start_req = false;
			ui_rec_running = false;
		}
		// MON START request
		if(ui_mon_start_req) {
			NRF_LOG_INFO("Starting MON");
			LED_ON(LED_MONITOR);
			err_code = ble_sss_on_button2_change(m_conn_handle, &m_sss, 1);
			if (err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
				APP_ERROR_CHECK(err_code);
			}
			ui_mon_start_req = false;
			ui_mon_running = true;
		}
		// MON STOP request
		if(ui_mon_stop_req) {
			NRF_LOG_INFO("Stopping MON");
			LED_OFF(LED_MONITOR);
			err_code = ble_sss_on_button2_change(m_conn_handle, &m_sss, 0);
			if( err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
				APP_ERROR_CHECK(err_code);
			}
			ui_mon_stop_req = false;
			ui_mon_start_req = false;
			ui_mon_running = false;
		}
		// SDC INIT OK
		if(sdc_init_ok) {
			NRF_LOG_INFO("SDC init OK");
			app_timer_stop(led_blink_timer);
			err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 1);
			if (err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
				APP_ERROR_CHECK(err_code);
			}
			LED_ON(LED_RECORD);
			nrf_drv_spi_transfer(&adc_spi, adc_spi_txbuf, adc_spi_len, adc_spi_rxbuf, adc_spi_len);
			nrf_drv_timer_enable(&ADC_SYNC_TIMER);
			ui_rec_running = true;
			sdc_init_ok = false;
		}
		
        // SDC ready-to-write
		if(sdc_rtw) {
			DBG_TOGGLE(DBG1_PIN);
			sdc_rtw = false;
//			NRF_LOG_INFO("Ready to write");
			if(sdc_writing) {
				sdc_chunk_counter++;
				NRF_LOG_DEBUG("Increased counter: %d", sdc_chunk_counter);
			}
			else {
				NRF_LOG_DEBUG("Writing to SDC. Cnt: %d", sdc_chunk_counter);
				sdc_write();
			}
		}
		
		__WFE();
    }
}

/** @} */
