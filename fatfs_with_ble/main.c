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
#include "ble_advertising.h"
/* ========================================================================== */
/*                                 VARIABLES                                  */
/* ========================================================================== */

/*                                   FIFO                                     */
/* -------------------------------------------------------------------------- */
app_fifo_t								sdc_fifo;
uint8_t									sdc_buffer[SDC_FIFO_SIZE];
app_fifo_t								ble_fifo;
uint8_t									ble_buffer[BLE_FIFO_SIZE];

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
TCHAR									sdc_foldername[7] = "000000";
TCHAR									sdc_folderpath[8] = "/000000";
TCHAR									sdc_filename[12] = "R000000.WAV";
FATFS 									sdc_fs;
DIR 									sdc_dir;
FILINFO 								sdc_fno;
FIL 									sdc_file;
static volatile bool 					sdc_init_ok = false;
static volatile bool					sdc_rtw = false;
static volatile bool					sdc_writing = false;
static volatile uint8_t					sdc_chunk_counter = 0;
//APP_TIMER_DEF(sdc_init_timer);


/*                                    ADC                                     */
/* -------------------------------------------------------------------------- */
static const nrf_drv_spi_t				adc_spi = NRF_DRV_SPI_INSTANCE(ADC_SPI_INSTANCE);
static volatile bool					adc_spi_xfer_done = false;
uint8_t									adc_spi_txbuf[2] = {0xFF, 0xFF};
uint8_t									adc_spi_rxbuf[2];
static const uint8_t					adc_spi_len = 2;

const nrf_drv_timer_t					ADC_SYNC_TIMER = NRF_DRV_TIMER_INSTANCE(ADC_SYNC_TIMER_INSTANCE);

static volatile uint32_t				adc_samples_counter = 0;
static volatile uint32_t				adc_total_samples = 0;

/*                                    GPS                                     */
/* -------------------------------------------------------------------------- */
static volatile bool 					gps_uart_reading = false;
static volatile bool					gps_uart_timeout = false;
static struct gps_rmc_tag				gps_backup_tag;

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
static volatile bool					ui_rec_stop_restart = false;
static volatile bool 					ui_rec_running = false;
static volatile bool					ui_mon_start_req = false;
static volatile bool 					ui_mon_stop_req = false;
static volatile bool 					ui_mon_running = false;
static volatile bool					ui_ble_advertising = false;
static volatile bool					ui_ble_connected = false;
APP_TIMER_DEF(led_blink_timer);


/*                                 TIMESTAMP                                  */
/* -------------------------------------------------------------------------- */
struct timestamp_tag					current_ts = {0};
static bool 							first_rec = true;
enum timestamp_source					ts_source = TS_SOURCE_NONE;
APP_TIMER_DEF(rec_window_timer);
static volatile uint8_t					rec_run_cnt = 0;

/*                                    BLE                                     */
/* -------------------------------------------------------------------------- */
static ble_uuid_t m_adv_uuids[] =
{
	{BLE_SSS_UUID_SERVICE, BLE_UUID_TYPE_BLE},
	{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE}
};
BLE_SSS_DEF(m_sss);																// LED Button Service instance
BLE_NUS_DEF(m_nus);																// Nordic UART Service instance (for audio streaming)
NRF_BLE_GATT_DEF(m_gatt);														// GATT module instance
BLE_ADVERTISING_DEF(m_advertising);                         					// Advertising module instance
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        // Handle of the current connection
static volatile uint32_t				ble_samples_counter = 0;
static volatile uint8_t					ble_chunk_counter = 0;
APP_TIMER_DEF(led_advertising_timer);


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
	uint32_t file_cnt;


    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };

    diskio_blockdev_register(drives, ARRAY_SIZE(drives));

	// Initialize disk & read capacity
//    NRF_LOG_DEBUG("Initializing disk 0 (SDC)...");
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
//    NRF_LOG_DEBUG("Capacity: %d MB", capacity);

	// Mount disk
    NRF_LOG_DEBUG("Mounting volume...");
    ff_result = f_mount(&sdc_fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_DEBUG("Mount failed. Result: %d", ff_result);
        return (uint32_t)ff_result;
    }

	// Looking for directories in root
//    NRF_LOG_DEBUG("Listing directory: /");
    ff_result = f_opendir(&sdc_dir, "/");
    if (ff_result)
    {
        NRF_LOG_DEBUG("Directory listing failed!");
        return (uint32_t)ff_result;
    }

 	bool dir_found = false;
	do 
	{
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
                NRF_LOG_DEBUG("<DIR>   %s",(uint32_t)sdc_fno.fname);
				uint8_t res = strcmp(sdc_fno.fname, sdc_foldername);
				if(res == 0) 
				{
					NRF_LOG_DEBUG("        DIR FOUND! Path: %s", sdc_folderpath);
					if((ts_source == TS_SOURCE_NONE) && (first_rec)) 
					{
						uint32_t dir_cnt = strtol((const char *)sdc_foldername, NULL, 10) + 1;
						sprintf(sdc_foldername, "%06d", dir_cnt);
						sprintf(sdc_folderpath, "/%s", sdc_foldername);
						NRF_LOG_DEBUG("Increasing dir name to %s", sdc_foldername);
					}
					else 
					{
						dir_found = true;
					}
				}
            }
        }
    } while (sdc_fno.fname[0]);
	NRF_LOG_INFO("Folder name: %s", sdc_foldername);
	first_rec = false;
	
	// Creating current date directory if no exists
	if(!dir_found) 
	{
		NRF_LOG_DEBUG("DIR not found... creating & opening");
		ff_result = f_mkdir(sdc_folderpath);
		if(ff_result != FR_OK) 
		{
			NRF_LOG_DEBUG("Unable to create directory");
			return (uint32_t)ff_result;
		}
		ff_result = f_chdir(sdc_folderpath);
		if(ff_result != FR_OK) 
		{
			NRF_LOG_DEBUG("Unable to change directory");
			return (uint32_t)ff_result;
		}
		ff_result = f_opendir(&sdc_dir, sdc_folderpath);
		if(ff_result != FR_OK) 
		{
			NRF_LOG_DEBUG("Unable to open directory");
			return (uint32_t)ff_result;
		}
	}
	// Entering existing directory (with current date)
	else 
	{
		NRF_LOG_DEBUG("Dir found... opening & searching");
		ff_result = f_chdir(sdc_folderpath);
		if(ff_result != FR_OK) 
		{
			NRF_LOG_DEBUG("Unable to change directory");
			return (uint32_t)ff_result;
		}
		ff_result = f_opendir(&sdc_dir, sdc_folderpath);
		if(ff_result != FR_OK) 
		{
			NRF_LOG_DEBUG("Unable to open directory");
			return (uint32_t)ff_result;
		}
		
		// Exploring current date directory
		do 
		{
			ff_result = f_readdir(&sdc_dir, &sdc_fno);
//			NRF_LOG_DEBUG("Reading... ");
			if (ff_result != FR_OK)
			{
				NRF_LOG_INFO("Directory read failed.");
				return (uint32_t)ff_result;
			}

			if (sdc_fno.fname[0])
			{
				if (sdc_fno.fattrib & AM_DIR)
				{
					NRF_LOG_DEBUG("<DIR>   %s",(uint32_t)sdc_fno.fname);
				}
				else
				{
					NRF_LOG_DEBUG("        %s (%lu)", (uint32_t)sdc_fno.fname, sdc_fno.fsize);
					uint8_t res = strncmp(sdc_fno.fname, sdc_filename, 12);
					NRF_LOG_DEBUG("Comparing %s to %s... res %d", sdc_fno.fname, sdc_filename, res);
					if(res == 0) {
						if(ts_source == TS_SOURCE_NONE) 
						{
							char temp_str[7] = "000000";
							strncpy(temp_str, &sdc_filename[1], 6);
							file_cnt = strtol((const char *)temp_str, NULL, 10);
							NRF_LOG_DEBUG("        temp str: %s, file_cnt: %ld", temp_str, file_cnt);
							file_cnt++;
							sprintf(sdc_filename, "R%06d.WAV", file_cnt);
							NRF_LOG_INFO("        FILE FOUND! Increasing filename to %s", sdc_filename);
						}
					}
				}
			}
		} while (sdc_fno.fname[0]);
	}
	
    NRF_LOG_INFO("Creating file %s...", sdc_filename);
    ff_result = f_open(&sdc_file, sdc_filename, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if (ff_result != FR_OK)
    {
        NRF_LOG_DEBUG("Unable to open or create file: %s", sdc_filename);
        return (uint32_t)ff_result;
    }
	
	NRF_LOG_DEBUG("Writing WAV header...");
	ff_result = f_write(&sdc_file, wave_header, 44, (UINT *) &bytes_written);
	if (ff_result != FR_OK) 
	{
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
	uint8_t temp_buf[SDC_BLOCK_SIZE];

	DBG_TOGGLE(DBG0_PIN);
	sdc_writing = true;
	uint32_t fifo_res = app_fifo_read(&sdc_fifo, temp_buf, &buf_size);

    ff_result = f_write(&sdc_file, temp_buf, SDC_BLOCK_SIZE, (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_DEBUG("Write failed.");
    }
	
	sdc_chunk_counter--;
	sdc_writing = false;

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
	((uint32_t *)&wave_header)[WAVE_FORMAT_SUBCHUNK2_SIZE_OFFSET/4] = (adc_total_samples/2) * (AUDIO_BITS_PER_SAMPLE/8);//adc_total_samples * AUDIO_BITS_PER_SAMPLE/8;
	((uint32_t *)&wave_header)[WAVE_FORMAT_CHUNK_SIZE_OFFSET/4] = ((adc_total_samples/2) * AUDIO_BITS_PER_SAMPLE/8) + 36;//(adc_total_samples * AUDIO_BITS_PER_SAMPLE/8) + 36;
	
	
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
	if(ff_result != FR_OK) {
		NRF_LOG_DEBUG("Error while closing file");
		return ff_result;
	}
	
	
	// Writing metadata
	char meta_filename[13];
	char temp[7] = "000000";
	strncpy(temp, (const char *)&sdc_filename[1], 6);
	sprintf(meta_filename, "T%06s.TXT", temp);
	
	ff_result = f_opendir(&sdc_dir, sdc_folderpath);
	if(ff_result != FR_OK) {
		NRF_LOG_DEBUG("Unable to open directory");
		return ff_result;
	}

    NRF_LOG_DEBUG("Creating meta-file %s...", meta_filename);
    ff_result = f_open(&sdc_file, meta_filename, FA_READ | FA_WRITE | FA_CREATE_NEW);
    if (ff_result != FR_OK) {
        NRF_LOG_DEBUG("Unable to open or create file: %s", meta_filename);
        return ff_result;
    }
	
	NRF_LOG_DEBUG("Writing GPS header...");
	int res = f_printf(&sdc_file, "GPS information:\n\r----------------\n\r" \
									"Folder name: %s\n\r" \
									"Recording\n\r" \
									"   name: %s\n\r" \
									"   sampling rate: %ld\n\r" \
									"   bit depth: %ld\n\r" \
									"GPS\n\r" \
									"   date: %02d.%02d.%04d\n\r" \
									"   time: %02dh%02dm%02ds\n\r" \
									"   longitude: %d�%d'%d'' %c\n\r" \
									"   latitude: %d�%d'%d'' %c\n\r",
									sdc_foldername, sdc_filename, AUDIO_SAMPLING_RATE, AUDIO_BITS_PER_SAMPLE,
									gps_backup_tag.date.day, gps_backup_tag.date.month, gps_backup_tag.date.year,
									gps_backup_tag.time.h, gps_backup_tag.time.min, gps_backup_tag.time.sec,
									gps_backup_tag.longitude.deg, gps_backup_tag.longitude.min,
									gps_backup_tag.longitude.sec, (gps_backup_tag.longitude.east ? 'E' : 'W'),
									gps_backup_tag.latitude.deg, gps_backup_tag.latitude.min,
									gps_backup_tag.latitude.sec, (gps_backup_tag.latitude.north ? 'N' : 'S'));
	NRF_LOG_DEBUG("File written. res = %d", res);

	ff_result = f_close(&sdc_file);
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
	if(err_code != NRF_ERROR_INVALID_STATE) {
		APP_ERROR_CHECK(err_code);
	}
	app_timer_start(led_advertising_timer, APP_TIMER_TICKS(500), NULL);
	ui_ble_advertising = true;
}

static struct gps_rmc_tag gps_get_rmc_geotag(void)
{
	struct gps_rmc_tag tag;
	bool do_slicing = true;
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
	if(gps_uart_timeout) { // GPS timeout! Couldn't read any raw tag
		tag.status_active = false;
		do_slicing = false;
		gps_uart_timeout = false;
	}
/* FAKE UART */
//	strcpy(tag.raw_tag, "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,03.1,W,S*6A");
/* ------------------------ */
	if(do_slicing) {
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
	}
	
	if(tag.status_active) {
		NRF_LOG_DEBUG("Valid GPS tag");
		ts_source = TS_SOURCE_GPS;
	}
	else {
		NRF_LOG_DEBUG("No valid GPS tag... trying with other sources");
		switch(ts_source) {
			case TS_SOURCE_BLE:
				NRF_LOG_INFO("TS valid... filling date/time");
				memcpy(&tag.date, &current_ts.date, sizeof(struct gps_date));
				memcpy(&tag.time, &current_ts.time, sizeof(struct gps_time));
				memcpy(&tag.latitude, &gps_backup_tag.latitude, sizeof(struct gps_long));
				memcpy(&tag.longitude, &gps_backup_tag.longitude, sizeof(struct gps_lat));
				break;
			
			case TS_SOURCE_BACKUP:
				NRF_LOG_INFO("Fetching backup TS value");
				memcpy(&tag.date, &gps_backup_tag.date, sizeof(struct gps_date));
				memcpy(&tag.time, &gps_backup_tag.time, sizeof(struct gps_time));
				memcpy(&tag.latitude, &gps_backup_tag.latitude, sizeof(struct gps_long));
				memcpy(&tag.longitude, &gps_backup_tag.longitude, sizeof(struct gps_lat));
				break;
			
			case TS_SOURCE_NONE:
				NRF_LOG_INFO("No valid source... setting to 0");
				memset(&tag, 0, sizeof(struct gps_rmc_tag));
				break;
			
			default:
				break;
		}
//		if(current_ts.ts_valid) {
//			NRF_LOG_DEBUG("TS valid... filling date/time");
//			memcpy(&tag.date, &current_ts.date, sizeof(struct gps_date));
//			memcpy(&tag.time, &current_ts.time, sizeof(struct gps_time));
//			memcpy(&tag.latitude, &gps_backup_tag.latitude, sizeof(struct gps_long));
//			memcpy(&tag.longitude, &gps_backup_tag.longitude, sizeof(struct gps_lat));
////			memcpy(&gps_backup_tag.date, &current_ts.date, sizeof(struct gps_date));
////			memcpy(&gps_backup_tag.time, &current_ts.time, sizeof(struct gps_time));
//			ts_source = TS_SOURCE_BLE;
//		}
//		else{
//			NRF_LOG_DEBUG("No valid TS!");
//			if(gps_backup_tag.date.month != 0) {
//				memcpy(&tag.date, &gps_backup_tag.date, sizeof(struct gps_date));
//				memcpy(&tag.time, &gps_backup_tag.time, sizeof(struct gps_time));
//				memcpy(&tag.latitude, &gps_backup_tag.latitude, sizeof(struct gps_long));
//				memcpy(&tag.longitude, &gps_backup_tag.longitude, sizeof(struct gps_lat));
//				ts_source = TS_SOURCE_BACKUP;
//			}
//			else {
//				memset(&tag, 0, sizeof(struct gps_rmc_tag));
//				ts_source = TS_SOURCE_NONE;
//			}				
//		}
	}
	
	return tag;
}
static void gps_poll_data(void)
{
	struct tm temp_tm;
	time_t temp_time;
	char temp_dir[6];
	char temp_file[6];

	struct gps_rmc_tag gps_cur_tag;
	memset(&gps_cur_tag, 0, sizeof(struct gps_rmc_tag));
	gps_cur_tag = gps_get_rmc_geotag();
	
	NRF_LOG_INFO("TS source: %d", ts_source);
	switch(ts_source) {
		case TS_SOURCE_BACKUP:
			// Write the GPS backup time to time_t value
			temp_tm.tm_year = gps_cur_tag.date.year - 1900;
			temp_tm.tm_mon = gps_cur_tag.date.month - 1;
			temp_tm.tm_mday = gps_cur_tag.date.day;
			temp_tm.tm_hour = gps_cur_tag.time.h;
			temp_tm.tm_min = gps_cur_tag.time.min;
			temp_tm.tm_sec = gps_cur_tag.time.sec;
			temp_time = mktime(&temp_tm);
			// Add seconds to the time_t
			temp_time += REC_PERIODE_IN_S;
			// Convert the time_t to tm
			temp_tm = *localtime(&temp_time);
			// Write the tm to the current GPS tag
			gps_cur_tag.date.year = temp_tm.tm_year + 1900;
			gps_cur_tag.date.month = temp_tm.tm_mon + 1;
			gps_cur_tag.date.day = temp_tm.tm_mday;
			gps_cur_tag.time.h = temp_tm.tm_hour;
			gps_cur_tag.time.min = temp_tm.tm_min;
			gps_cur_tag.time.sec = temp_tm.tm_sec;
		case TS_SOURCE_GPS:
		case TS_SOURCE_BLE:
			// Generate 'sdc_foldername' and 'sdc_filename' from the GPS current tag
			// The modulo-100 trick is used to get only the last 2 digits of the year value
			sprintf(temp_dir, "%02d%02d%02d", (gps_cur_tag.date.year%100), gps_cur_tag.date.month, gps_cur_tag.date.day);
			sprintf(temp_file, "%02d%02d%02d", gps_cur_tag.time.h, gps_cur_tag.time.min, gps_cur_tag.time.sec);
			NRF_LOG_INFO("temp_dir: %s", temp_dir);
			NRF_LOG_INFO("temp_file: %s", temp_file);
			// Copy generated string to 'sdc_foldername'
			memcpy(sdc_foldername, temp_dir, 6);
			sprintf(sdc_folderpath, "/%s", sdc_foldername);
			// Copy generated string to 'sdc_filename'
			memcpy(&sdc_filename[1], temp_file, 6);
			break;
		
		case TS_SOURCE_NONE:
			if(first_rec) {
				sprintf(temp_dir, "%02d%02d%02d", (gps_cur_tag.date.year%100), gps_cur_tag.date.month, gps_cur_tag.date.day);
				sprintf(temp_file, "%02d%02d%02d", gps_cur_tag.time.h, gps_cur_tag.time.min, gps_cur_tag.time.sec);
				memcpy(sdc_foldername, temp_dir, 6);
				sprintf(sdc_folderpath, "/%s", sdc_foldername);
				memcpy(&sdc_filename[1], temp_file, 6);
			}
			break;
		
		default:
			break;
	}

	// Backup the current GPS tag
	memcpy(&gps_backup_tag, &gps_cur_tag, sizeof(struct gps_rmc_tag));

	NRF_LOG_INFO("Folder name %s, folder path %s", sdc_foldername, sdc_folderpath);
	NRF_LOG_INFO("File name: %s", sdc_filename);
}
/* ========================================================================== */
/*                              EVENT HANDLERS                                */
/* ========================================================================== */
// ADC SPI
void adc_spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	if(ui_rec_running) {
		static uint32_t size = 1;
		app_fifo_write(&sdc_fifo, &adc_spi_rxbuf[1], &size);
		size = 1;
		app_fifo_write(&sdc_fifo, &adc_spi_rxbuf[0], &size);
	}
	if((ui_mon_running) && ((adc_samples_counter % (2*MON_DOWNSAMPLE_FACTOR)) == 0)) {
		static uint32_t size = adc_spi_len;
		app_fifo_write(&ble_fifo, adc_spi_rxbuf, &size);
		ble_samples_counter += 2;
	}
	adc_samples_counter += 2;
	adc_spi_xfer_done = true;
}

// ADC timer
void adc_sync_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	if(adc_spi_xfer_done) {
		adc_spi_xfer_done = false;
		if(ui_rec_running && (adc_samples_counter >= SDC_BLOCK_SIZE)) 
		{
			adc_total_samples += SDC_BLOCK_SIZE;
			adc_samples_counter = 0;
			sdc_chunk_counter++;
			if(ui_rec_stop_req) 
			{
				sdc_chunk_counter = 0;
				ui_rec_stop_req = false;
			}
		}
		if(ui_mon_running && (ble_samples_counter >= BLE_MAX_MTU_SIZE)) 
		{
			ble_samples_counter = 0;
			ble_chunk_counter++;
		}
		nrf_drv_spi_transfer(&adc_spi, adc_spi_txbuf, 2, adc_spi_rxbuf, 2);
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
			app_timer_stop(led_advertising_timer);
            LED_ON(LED_BLE);
			ui_ble_advertising = false;
			ui_ble_connected = true;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
			ui_ble_connected = false;
			ui_ble_advertising = false;
			LED_OFF(LED_BLE);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			// Not advertising here: function called in ble_advertising->on_disconnected(), LED notification in on_adv_event()
            break;

		case BLE_GAP_EVT_TIMEOUT:
			app_timer_stop(led_advertising_timer);
			LED_OFF(LED_BLE);
			err_code = sd_ble_gap_adv_stop();
			NRF_LOG_INFO("Adv. timeout. err code: 0x%04x", err_code);
			if((err_code != NRF_ERROR_INVALID_STATE) && 
				(err_code != NRF_SUCCESS)) {
				APP_ERROR_CHECK(err_code);
			}
			ui_ble_advertising = false;
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



// BLE advertising event
static void on_adv_event(ble_adv_evt_t ble_adv_evt)
{
//	ret_code_t err_code;
	
	switch(ble_adv_evt)
	{
		case BLE_ADV_EVT_FAST:
			ui_ble_advertising = true;
			app_timer_start(led_advertising_timer, APP_TIMER_TICKS(500), NULL);
			break;
		
		case BLE_ADV_EVT_IDLE:
			break;
		
		default:
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
	ret_code_t err_code;
	
	if(button_action) {
		switch (pin_no)
		{
			// REC button pressed
			case BUTTON_RECORD:
				// REC running or starting -> stop REC and disable restart
				if(ui_rec_running || ui_rec_start_req) {
					ui_rec_stop_req = true;
					ui_rec_start_req = false;
					ui_rec_stop_restart = false;
				}
				// REC stopped and restarting -> stop REC and disable restart
				else if(ui_rec_stop_restart) {
					ui_rec_stop_req = true;
					ui_rec_stop_restart = false;
				}
				// REC stopped -> start REC
				else {
					ui_rec_start_req = true;
				}
				break;
				
			case BUTTON_MONITOR:
				// MON running or starting -> stop MON
				if(ui_mon_running || ui_mon_start_req) {
					ui_mon_stop_req = true;
					ui_mon_start_req = false;
					ui_mon_running = false;
				}
				// MON stopped -> start MON
				else {
					ui_mon_start_req = true;
				}
				break;

			case BUTTON_BLE:
				// BLE stopped -> start advertising
				if(!ui_ble_advertising && !ui_ble_connected) {
					advertising_start();
				}
				// BLE advertising -> stop advertising
				else if(ui_ble_advertising) {
					err_code = sd_ble_gap_adv_stop();
					if(err_code != NRF_ERROR_INVALID_STATE) {
						APP_ERROR_CHECK(err_code);
					}
					app_timer_stop(led_advertising_timer);
					LED_OFF(LED_BLE);
					ui_ble_advertising = false;
				}
				// BLE connected -> disconnect BLE
				else if(ui_ble_connected) {
					err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
					APP_ERROR_CHECK(err_code);
					LED_OFF(LED_BLE);
					ui_ble_connected = false;
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

// LED ADVERTISING
static void led_advertising_handler(void * p_context)
{
	LED_TOGGLE(LED_BLE);
}

// LED REC
static void led_rec_handler(uint16_t conn_handle, ble_sss_t * p_sss, uint8_t led_state)
{
	// LED ON -> REC start request
	if(led_state) {
		// REC stopped -> start REC
		if(!ui_rec_running && !ui_rec_start_req) {
			ui_rec_start_req = true;
		}
		// REC running -> do nothing
		else {
//			NRF_LOG_DEBUG("REC already running...");
		}
	}
	// LED OFF -> REC stop request
	else {
		// REC running or starting -> stop REC
		if(ui_rec_running || ui_rec_start_req) {
			ui_rec_stop_req = true;
			ui_rec_stop_restart = false;
		}
		// REC stopped -> do nothing
		else {
//			NRF_LOG_DEBUG("No REC running...");
		}
	}
}
// LED MON
static void led_mon_handler(uint16_t conn_handle, ble_sss_t * p_sss, uint8_t led_state)
{
	// LED ON -> MON start request
	if(led_state) {
		// MON stopped -> start MON
		if(!ui_mon_running && !ui_mon_start_req) {
			ui_mon_start_req = true;
		}
		// MON running -> do nothing
		else {
//			NRF_LOG_DEBUG("MON already running...");
		}
	}
	// LED OFF -> MON stop request
	else {
		// MON running or starting -> stop MON
		if(ui_mon_running || ui_mon_start_req) {
			ui_mon_stop_req = true;
		}
		// MON stopped -> do nothing
		else {
//			NRF_LOG_DEBUG("No MON running...");
		}
	}
}

// NUS
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
}

// TS WRITE
static void ts_write_handler(uint16_t conn_handle, ble_sss_t * p_sss, uint8_t timestamp)
{
	static time_t cur_time;
	struct tm cur_tm;
	static uint8_t byte_cnt = 0;
	switch(byte_cnt) {
		case 0:
			cur_time = (time_t)timestamp << 24;
			current_ts.ts_valid = false;
			byte_cnt = 1;
			break;
		case 1:
			cur_time |= (time_t)timestamp << 16;
			current_ts.ts_valid = false;
			byte_cnt = 2;
			break;
		case 2:
			cur_time |= (time_t)timestamp << 8;
			current_ts.ts_valid = false;
			byte_cnt = 3;
			break;
		case 3:
			cur_time |= (time_t)timestamp;
			byte_cnt = 0;
			cur_tm = *localtime(&cur_time);
//			c_time_string = asctime(&p_read_time);
//			NRF_LOG_DEBUG("Current time: %s", c_time_string);
			current_ts.date.year = cur_tm.tm_year + 1900;
			current_ts.date.month = cur_tm.tm_mon + 1;
			current_ts.date.day = cur_tm.tm_mday;
			current_ts.time.h = cur_tm.tm_hour;
			current_ts.time.min = cur_tm.tm_min;
			current_ts.time.sec = cur_tm.tm_sec;
			current_ts.ts_valid = true;
			ts_source = TS_SOURCE_BLE;
			break;
		default:
			break;
	}
}

// REC WINDOW
static void rec_window_handler(void * p_context)
{
	if(ui_rec_running) {
		rec_run_cnt++;
		ui_rec_stop_req = true;
		if(rec_run_cnt < REC_OCCURENCE_MAX) {
			ui_rec_stop_restart = true;
		}
		else {
			ui_rec_stop_restart = false;
		}
	}
	else {
		ui_rec_start_req = true;
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
	
	time_ticks = nrf_drv_timer_us_to_ticks(&ADC_SYNC_TIMER, ADC_SYNC_US);

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
	ble_advertising_init_t adv_init;

	memset(&adv_init, 0, sizeof(adv_init));
	
	adv_init.advdata.name_type					= BLE_ADVDATA_FULL_NAME;
	adv_init.advdata.include_appearance			= true;
	adv_init.advdata.flags						= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
	adv_init.advdata.uuids_complete.uuid_cnt	= sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	adv_init.advdata.uuids_complete.p_uuids		= m_adv_uuids;
	
	adv_init.config.ble_adv_fast_enabled		= true;
	adv_init.config.ble_adv_fast_interval		= APP_ADV_INTERVAL;
	adv_init.config.ble_adv_fast_timeout		= APP_ADV_TIMEOUT_IN_SECONDS;
	
	adv_init.evt_handler						= on_adv_event;
	
	err_code = ble_advertising_init(&m_advertising, &adv_init);
	APP_ERROR_CHECK(err_code);
	
	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
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

	err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
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
    ble_sss_init_t sss_init;
	ble_nus_init_t nus_init;
	
	memset(&sss_init, 0, sizeof(sss_init));

    sss_init.led1_write_handler = led_rec_handler;
	sss_init.led2_write_handler = led_mon_handler;
	sss_init.ts_write_handler = ts_write_handler;
    err_code = ble_sss_init(&m_sss, &sss_init);
    APP_ERROR_CHECK(err_code);
	
	memset(&nus_init, 0, sizeof(nus_init));
	
	nus_init.data_handler = nus_data_handler;
	err_code = ble_nus_init(&m_nus, &nus_init);
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
		{BUTTON_MONITOR, false, BUTTON_PULL, button_event_handler},
		{BUTTON_BLE, false, BUTTON_PULL, button_event_handler}
    };

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
	ret_code_t err_code;
	bsp_board_leds_init();
	err_code = app_timer_create(&led_blink_timer, APP_TIMER_MODE_REPEATED, led_blink_handler);
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_create(&led_advertising_timer, APP_TIMER_MODE_REPEATED, led_advertising_handler);
	APP_ERROR_CHECK(err_code);
}


// MON
static void mon_init(void)
{
	ret_code_t err_code;
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true);
	err_code = nrf_drv_gpiote_out_init(MON_EN_PIN, &out_config);
	APP_ERROR_CHECK(err_code);
	MON_DISABLE();
}


// REC configuration
static void rec_window_init(void)
{
	ret_code_t err_code;
	err_code = app_timer_create(&rec_window_timer, APP_TIMER_MODE_SINGLE_SHOT, rec_window_handler);
	APP_ERROR_CHECK(err_code);
}

// LOG
static void log_init(void)
{
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}





/* ========================================================================== */
/*                               DUMMY FUNCTIONS                              */
/* ========================================================================== */
#ifdef DUMMY_MODE 
APP_TIMER_DEF(dummy_mon_timer);

static void dummy_mon_handler(void * p_context)
{
    uint8_t b[2];
    for(uint8_t i = 0; i < BLE_MAX_MTU_SIZE; i += 2) {
        static uint32_t size = 2;
        b[0] = 0x01;
        b[1] = (rand() % 0xFF);
//        NRF_LOG_DEBUG("b[0]: 0x%02x, b[1]: 0x%02x", b[0], b[1]);
        app_fifo_write(&ble_fifo, b, &size);
    }
    ble_chunk_counter++;
}
#endif




/**
 * @brief Function for main application entry.
 */
int main(void)
{
	static ret_code_t err_code;
	current_ts.ts_valid = false;
	memset(&gps_backup_tag, 0, sizeof(struct gps_rmc_tag));

	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
	
	leds_init();
	log_init();
	buttons_init();
	gpio_dbg_init();
	mon_init();
	
	ble_stack_init();
	gap_params_init();
	gatt_init();
	advertising_init();
	services_init();
	conn_params_init();
	
	adc_config_spi();
	adc_config_timer();
	err_code = app_fifo_init(&sdc_fifo, sdc_buffer, SDC_FIFO_SIZE);
	err_code = app_fifo_init(&ble_fifo, ble_buffer, BLE_FIFO_SIZE);
	gps_init();
	
	rec_window_init();
	
	/* Starting application */
	/* -------------------- */
    NRF_LOG_INFO("=============")
	NRF_LOG_INFO("SOUNDING SOIL");
    NRF_LOG_INFO("-------------")
	NRF_LOG_INFO("Audio shield ");
	NRF_LOG_INFO("V0.3, R001   ");
    NRF_LOG_INFO("-------------")
#ifdef DUMMY_MODE
	NRF_LOG_INFO("SIMULATION!");
	NRF_LOG_INFO("-----------");
#endif

 	app_button_enable();
	advertising_start();
	
	for(;;)
    {
		/* REC START request
		 * ----------------- */
		if(ui_rec_start_req) 
		{
			// Clear flags
			ui_rec_start_req = false;
			// Comment (deferred)
			NRF_LOG_INFO("REC start requested: MON - %d, BLE - %d/%d", ui_mon_running, ui_ble_connected, ui_ble_advertising);
			// REC can start if NO BLE MON is currently running
			if((ui_ble_connected && ui_mon_running)) {
				NRF_LOG_INFO("Cannot start REC when monitoring over BLE!");
			}
			else {
				/* Initialize REC state:
				   - get GPS tag for date, time, location
				   - test & mount SD card
				   - open/create dir & file for audio record */
				app_timer_start(led_blink_timer, APP_TIMER_TICKS(200), NULL);
#ifdef DUMMY_MODE
				nrf_delay_ms(1000);
				sdc_init_ok = true;
#else
				gps_poll_data();
				if(sdc_start() == 0) {
					nrf_delay_ms(1000);
					// Set flags
					sdc_init_ok = true;
				}
				else {
					app_timer_stop(led_blink_timer);
					sdc_init_ok = false;
					NRF_LOG_DEBUG("SDC init failed, stopping");
				}
#endif
			}
		}
		
		/* REC STOP request
		 * ---------------- */
		if(ui_rec_stop_req) 
		{
			// Clear flags
//			ui_rec_start_req = false;
			// Comment (deferred)
			NRF_LOG_INFO("REC stop requested: MON - %d, BLE - %d/%d", ui_mon_running, ui_ble_connected, ui_ble_advertising);
			// Clear TS flag
			current_ts.ts_valid = false;
			sdc_chunk_counter = 0;
#ifdef DUMMY_MODE
			//
#else
			// Disable audio syncronisation IF NO MON STILL RUNNING!!
			if(!ui_mon_running) {
				// Waiting for next audio sync. ui_rec_stop_rec will be cleared there.
				while(ui_rec_stop_req) {};
				ui_rec_running = false;
				nrf_drv_timer_disable(&ADC_SYNC_TIMER);
			}
			else {
				ui_rec_stop_req = false;
				ui_rec_running = false;
			}
			// Write WAV header & close SD card file
			sdc_close();
#endif
			// Notify REC STOP
			LED_OFF(LED_RECORD);
			if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
				err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 0);
				if (err_code != NRF_SUCCESS &&
					err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
					err_code != NRF_ERROR_INVALID_STATE &&
					err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
						APP_ERROR_CHECK(err_code);
				}
			}
			if(ui_rec_stop_restart) {
				// On REC restart, set TS source to backup or none. Can be overwritten by GPS.
				ts_source = (ts_source != TS_SOURCE_NONE) ? TS_SOURCE_BACKUP : TS_SOURCE_NONE;
//				ui_rec_stop_restart = false;
				app_timer_start(rec_window_timer, APP_TIMER_TICKS((REC_PERIODE_IN_S - REC_DURATION_IN_S) * 1000), NULL);
				NRF_LOG_INFO("REC stopped and restarting in %d seconds", (REC_PERIODE_IN_S - REC_DURATION_IN_S));
			}
			else {
				// On REC stop, reset TS source.
				ts_source = TS_SOURCE_NONE;
				app_timer_stop(rec_window_timer);
				rec_run_cnt = 0;
				first_rec = true;
//				current_ts.ts_valid = false;
				NRF_LOG_DEBUG("REC stopped definetly");
			}
		}
		
		/* MON START request
		 * ----------------- */
		if(ui_mon_start_req) 
		{
			// Clear flags
			ui_mon_start_req = false;
			// Comment (deferred)
			NRF_LOG_INFO("MON start requested: REC - %d, BLE - %d/%d", ui_rec_running, ui_ble_connected, ui_ble_advertising);
#ifdef DUMMY_MODE
			app_timer_create(&dummy_mon_timer, APP_TIMER_MODE_REPEATED, dummy_mon_handler);
			app_timer_start(dummy_mon_timer, APP_TIMER_TICKS(100), NULL);
#else 
			// Enable audio synchronisation IF NO REC ALREADY RUNNING!!
			if(!ui_rec_running) {
				nrf_drv_spi_transfer(&adc_spi, adc_spi_txbuf, adc_spi_len, adc_spi_rxbuf, adc_spi_len);
				nrf_drv_timer_enable(&ADC_SYNC_TIMER);
			}
#endif
			// Enable MON outupt
			MON_ENABLE();
				
			// Notify MON START
			LED_ON(LED_MONITOR);
			if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
				err_code = ble_sss_on_button2_change(m_conn_handle, &m_sss, 1);
				if (err_code != NRF_SUCCESS &&
					err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
					err_code != NRF_ERROR_INVALID_STATE &&
					err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
						APP_ERROR_CHECK(err_code);
				}
			}
			// Set flags
			ui_mon_running = true;
		}
		
		/* MON STOP request
		 * ---------------- */
		if(ui_mon_stop_req) 
		{
			// Clear flags
			ui_mon_stop_req = false;
			ui_mon_start_req = false;
			ui_mon_running = false;
			NRF_LOG_INFO("MON stop requested: REC - %d, BLE - %d/%d", ui_rec_running, ui_ble_connected, ui_ble_advertising);
			ble_chunk_counter = 0;
#ifdef DUMMY_MODE
			app_timer_stop(dummy_mon_timer);
			app_fifo_flush(&ble_fifo);
#else
			// Disable audio synchronization IF NO REC STILL RUNNING!!
			if(!ui_rec_running) {
				nrf_drv_timer_disable(&ADC_SYNC_TIMER);
			}
#endif
			// Disable MON output
			MON_DISABLE();
			
			// Notify MON STOP
			LED_OFF(LED_MONITOR);
			if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
				err_code = ble_sss_on_button2_change(m_conn_handle, &m_sss, 0);
				if( err_code != NRF_SUCCESS &&
					err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
					err_code != NRF_ERROR_INVALID_STATE &&
					err_code != NRF_ERROR_RESOURCES &&
					err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
						APP_ERROR_CHECK(err_code);
				}
			}
//			NRF_LOG_DEBUG("MON stopped: mon_running %d, chunk_counter %d, fifo pos %d", ui_mon_running, ble_chunk_counter, (ble_fifo.read_pos - ble_fifo.write_pos));
//			NRF_LOG_INFO("MON stopped");
			app_fifo_flush(&ble_fifo);
		}
		
		/* SDC INIT OK (REC READY)
		 * ----------------------- */
		if(sdc_init_ok) 
		{
			sdc_init_ok = false;
			NRF_LOG_INFO("SDC OK... starting REC: MON - %d, BLE - %d/%d", ui_mon_running, ui_ble_connected, ui_ble_advertising);
			app_timer_stop(led_blink_timer);
#ifdef DUMMY_MODE
			//
#else
			// Clear the audio samples counters
			adc_samples_counter = 0;
			adc_total_samples = 0;
			// Enable audio synchronisation IF NO MON ALREADY RUNNING!!
			if(!ui_mon_running) {
				nrf_drv_spi_transfer(&adc_spi, adc_spi_txbuf, adc_spi_len, adc_spi_rxbuf, adc_spi_len);
				nrf_drv_timer_enable(&ADC_SYNC_TIMER);
			}
#endif
			// Notify REC START
			LED_ON(LED_RECORD);
//			if(m_conn_handle != BLE_CONN_HANDLE_INVALID) {
			if(ui_ble_connected) {
				err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 1);
				if (err_code != NRF_SUCCESS &&
					err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
					err_code != NRF_ERROR_INVALID_STATE &&
					err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
						APP_ERROR_CHECK(err_code);
				}
			}
			// Start window timer
			err_code = app_timer_start(rec_window_timer, APP_TIMER_TICKS(REC_DURATION_IN_S * 1000), NULL);
			// Set flags
			ui_rec_running = true;
			NRF_LOG_DEBUG("REC #%d started for %d seconds", rec_run_cnt, REC_DURATION_IN_S);
		}
		
		/* CHUNKS TO SDC
		 * --------------- */
		if((sdc_chunk_counter > 0) && (!sdc_writing)) 
		{
			sdc_write();
		}
		
		/* CHUNKS TO BLE
		 * ------------- */
		if((ble_chunk_counter > 0) && (ui_ble_connected)) 
		{
			if(!ui_rec_running) {
				uint32_t len = (uint32_t)BLE_MAX_MTU_SIZE;
				uint8_t temp_buf[BLE_MAX_MTU_SIZE];
				app_fifo_read(&ble_fifo, temp_buf, &len);
				uint32_t err_code = ble_nus_string_send(&m_nus, temp_buf, (uint16_t*)&len);
				ble_chunk_counter--;
			}
		}

#if (NRF_LOG_DEFERRED == 1)
		NRF_LOG_PROCESS();
#endif
		
		__WFE();
    }
}

/** @} */
