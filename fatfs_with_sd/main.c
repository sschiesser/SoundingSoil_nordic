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

/* ========================================================================== */
/*                                   DEBUG                                    */
/* ========================================================================== */
#define DBG0_PIN						9
#define DBG1_PIN						10
#define DBG2_PIN						11
#define DBG3_PIN						12
#define DBG_CURRENT_FOLDER				"180322"

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
static TCHAR							current_folder_name[13] = DBG_CURRENT_FOLDER;

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

// Audio buffer
static uint8_t							m_tx_buf[2] = {0xFF, 0xFF};
static uint8_t							m_rx_buf[2];
static const uint8_t					m_length = 2;


/* ========================================================================== */
/*                                   GPS                                      */
/* ========================================================================== */
#define GPS_NMEA_MAX_SIZE				74
#define GPS_NMEA_START_CHAR				0x24
#define GPS_NMEA_STOP_CHAR				0x0A
#define GPS_GGA_SIZE					74
#define GPS_GGA_TIME_POS				7
#define GPS_GGA_TIME_LEN				10
#define GPS_GGA_LAT_POS					(GPS_GGA_TIME_POS + \
										GPS_GGA_TIME_LEN + 1) 			// 18
#define GPS_GGA_LAT_LEN					9
#define GPS_GGA_LAT_N_S_POS				(GPS_GGA_LAT_POS + \
										GPS_GGA_LAT_LEN + 1) 			// 28
#define GPS_GGA_LONG_POS				(GPS_GGA_LAT_N_S_POS + 2) 		// 30
#define GPS_GGA_LONG_LEN				10
#define GPS_GGA_LONG_E_W_POS			(GPS_GGA_LONG_POS + \
										GPS_GGA_LONG_LEN + 1)			// 41
#define GPS_GGA_QUAL_POS				(GPS_GGA_LONG_E_W_POS + 2)		// 43
#define GPS_GGA_DIL_POS					(GPS_GGA_QUAL_POS + 5)			// 48
#define GPS_GGA_DIL_LEN					3
#define GPS_GGA_ALT_POS					(GPS_GGA_DIL_POS + \
										GPS_GGA_DIL_LEN + 1)			// 52
#define GPS_GGA_ALT_LEN					5
struct gps_time {
	uint8_t h;
	uint8_t min;
	uint8_t sec;
	uint16_t msec;
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
struct gps_tag {
	struct gps_time time;
	struct gps_lat latitude;
	struct gps_long longitude;
	uint8_t quality;
	float dilution;
	int16_t altitude;
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
//		bsp_board_led_invert(1);
		nrf_drv_gpiote_out_toggle(DBG1_PIN);
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
//	bsp_board_led_invert(0);
	nrf_drv_gpiote_out_toggle(DBG0_PIN);
	if(adc_spi_xfer_done) {
		adc_spi_xfer_done = false;
		nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);
	}
}


// BUTTONS
void button0_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nrf_drv_gpiote_out_toggle(BSP_LED_0);
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
void button1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nrf_drv_gpiote_out_toggle(BSP_LED_1);
}
void button2_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nrf_drv_gpiote_out_toggle(BSP_LED_2);
}
void button3_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	nrf_drv_gpiote_out_toggle(BSP_LED_3);
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
	err_code = nrf_drv_gpiote_out_init(BSP_LED_0, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(BSP_LED_1, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(BSP_LED_2, &out_config);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(BSP_LED_3, &out_config);
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
	err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_0, &in_config, button0_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BSP_BUTTON_0, true);
	err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_1, &in_config, button1_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BSP_BUTTON_1, true);
	err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_2, &in_config, button2_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BSP_BUTTON_2, true);
	err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_3, &in_config, button3_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_gpiote_in_event_enable(BSP_BUTTON_3, true);
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
	
//	nrf_drv_timer_enable(&ADC_SYNC_TIMER);
}

DSTATUS sdc_init(void)
{
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
        NRF_LOG_INFO("Disk initialization failed. Disk state: %d", disk_state);
        return disk_state;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);
	return RES_OK;
}


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
			if (res != FR_OK || fno.fname[0] == 0) {
				*nb_items = items-2; // removing the '.' and '..' items
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


FRESULT sdc_mount(void)
{
    static FATFS fs;
	static FRESULT res;
    static DIR dir;
    static FILINFO fno;

//	TCHAR root_directory[4];
//	TCHAR file_name[18];
//	TCHAR test_folder_name[13];
//	TCHAR test_folder_location[25];
//	char disk_str_num[2];
//	sprintf(disk_str_num, "%d", 0);
//	sprintf(test_folder_name, "%s", (const char *)"180316");
//	sprintf(root_directory, "%s:/", disk_str_num);
//	sprintf(test_folder_location, "%s%s", root_directory, test_folder_name);
	
	NRF_LOG_INFO("Mounting volume...");
    res = f_mount(&fs, "", 1);
    if (res) {
        NRF_LOG_INFO("Mount failed. Result: %d", res);
        return res;
    }

    NRF_LOG_INFO("\r\n Listing directory: /");
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
                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fno.fname);
            }
            else {
                NRF_LOG_RAW_INFO("%9lu  %s", fno.fsize, (uint32_t)fno.fname);
            }
        }
    }
    while (fno.fname[0]);
    NRF_LOG_RAW_INFO("");
	
	return FR_OK;
}

FRESULT sdc_init_audio(void)
{	
	FRESULT res;

	TCHAR root_directory[4] = ROOT_DIR;
	TCHAR file_name[18];
	TCHAR current_folder_location[25];
	sprintf(current_folder_location, "%s%s", root_directory, current_folder_name);
	/* ===== CHECK DIRECTORY EXISTENCE ===== */
	FILINFO filinfo;
	bool makedir = false;
	
	res = f_stat(current_folder_location, &filinfo);
	if(res == FR_OK) {
		NRF_LOG_INFO("-I- folder already exist\n\r");
	}
	else if((res == FR_NO_PATH) || (res == FR_NO_FILE)) {
		NRF_LOG_INFO("-W- folder not found... create!\n\r");
		makedir = true;
	}
	else {
		NRF_LOG_INFO("-E- f_stat pb: 0x%X\n\r", res);
		return res;
	}
	
	/* ===== CREATE DIRECTORY ===== */
	if(makedir) {
		res = f_mkdir(current_folder_name);
		if (res != FR_OK) {
			NRF_LOG_INFO("-E- f_mkdir pb: 0x%X\n\r", res);
			return res;
		}
		else {
			NRF_LOG_INFO("-I- directory created\n\r");
			makedir = false;
		}
	}

	/* ===== OPEN DIRECTORY ===== */
	NRF_LOG_INFO("-I- Opening directory !\r");
	DIR	dirinfo;
	uint16_t scanned_files;
	res = f_opendir(&dirinfo, current_folder_location);
	if (res == FR_OK) {
		/* Display the file tree */
		NRF_LOG_INFO("-I- Display files contained in the memory :\r");
		strcpy((char *)data_buffer, current_folder_location);
		scan_files((char *)data_buffer, &scanned_files);
		NRF_LOG_INFO("Number of found files: %d\n\r", scanned_files);
	}

	
	/* ===== CREATE NEW FILE ===== */
//	static FIL   file_object;
	TCHAR new_file[13];
	for(;;) {
		sprintf(new_file, "R%04d", scanned_files);
		sprintf(file_name, "%s/%s.wav", current_folder_location, new_file); /*File path*/
		NRF_LOG_INFO("-I- Create a file : \"%s\"\n\r", file_name);
		res = f_open(&recording_fil, (char const *)file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
		if (res == FR_OK) {
			break;
		}
		else if (res != FR_EXIST) {
			NRF_LOG_INFO("-E- Filename already exist... increasing iteration\n\r");
			scanned_files++;
		}
		else {
			NRF_LOG_INFO("-E- f_open create pb: 0x%X\n\r", res);
			return res;
		}
	}

	return FR_OK;
}

static struct gps_tag gps_get_geotag(void)
{
	struct gps_tag tag;
	ret_code_t ret;
	char c;
	uint8_t buffer[GPS_NMEA_MAX_SIZE];
	uint8_t temp[12];
	uint8_t cnt = 0;
	bool reading = true;
	
	while(reading) {
		ret = nrf_serial_read(&gps_uart, &c, sizeof(c), NULL, 1000);
		APP_ERROR_CHECK(ret);
		buffer[cnt++] = c;
		if(c == GPS_NMEA_STOP_CHAR) {
			if ((cnt == GPS_GGA_SIZE)) {
				if(buffer[0] == GPS_NMEA_START_CHAR) {
					reading = false;
				}
			}
			cnt = 0;
		}
	}
	// Time
	temp[0] = buffer[GPS_GGA_TIME_POS];
	temp[1] = buffer[GPS_GGA_TIME_POS + 1];
	temp[2] = '\0';
	tag.time.h = atoi(temp);
	temp[0] = buffer[GPS_GGA_TIME_POS + 2];
	temp[1] = buffer[GPS_GGA_TIME_POS + 3];
	tag.time.min = atoi(temp);
	temp[0] = buffer[GPS_GGA_TIME_POS + 4];
	temp[1] = buffer[GPS_GGA_TIME_POS + 5];
	tag.time.sec = atoi(temp);
	temp[0] = buffer[GPS_GGA_TIME_POS + 7];
	temp[1] = buffer[GPS_GGA_TIME_POS + 8];
	temp[2] = buffer[GPS_GGA_TIME_POS + 9];
	temp[3] = '\0';
	tag.time.msec = atoi(temp);
	// Latitude
	temp[0] = buffer[GPS_GGA_LAT_POS];
	temp[1] = buffer[GPS_GGA_LAT_POS + 1];
	temp[2] = '\0';
	tag.latitude.deg = atoi(temp);
	temp[0] = buffer[GPS_GGA_LAT_POS + 2];
	temp[1] = buffer[GPS_GGA_LAT_POS + 3];
	tag.latitude.min = atoi(temp);
	temp[0] = buffer[GPS_GGA_LAT_POS + 5];
	temp[1] = buffer[GPS_GGA_LAT_POS + 6];
	temp[2] = buffer[GPS_GGA_LAT_POS + 7];
	temp[3] = buffer[GPS_GGA_LAT_POS + 8];
	temp[4] = '\0';
	tag.latitude.sec = atoi(temp);
	if(buffer[GPS_GGA_LAT_N_S_POS] == 'N') tag.latitude.north = true;
	else tag.latitude.north = false;
	// Longitude
	
	// Quality
	
	// Dilution
	
	// Altitude
	
	return tag;
}
static struct gps_tag gps_get_geodata(uint8_t retries)
{
	struct gps_tag cur_tag, avg_tag;
	cur_tag = gps_get_geotag();
	avg_tag = cur_tag;
	return avg_tag;
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
	ret_code_t ret;

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
	
	uint8_t buf[256];
	uint8_t cnt = 0;
	while(true) {
		struct gps_tag cur_tag = gps_get_geodata(1);
		NRF_LOG_INFO("GPS time: %dh%dm%ds", cur_tag.time.h, cur_tag.time.min, cur_tag.time.sec);
		NRF_LOG_INFO("GPS latitude: %ddeg %d'%d\", %s", 
			cur_tag.latitude.deg, cur_tag.latitude.min,
			cur_tag.latitude.sec, (cur_tag.latitude.north) ? "N" : "S");
		NRF_LOG_INFO("GPS longitude: %ddeg %d'%d\", %s", 
			cur_tag.longitude.deg, cur_tag.longitude.min, 
			cur_tag.longitude.sec, (cur_tag.longitude.east) ? "E" : "W");
//		char c;
//		struct gps_tag cur_tag;
//		ret = nrf_serial_read(&gps_uart, &c, sizeof(c), NULL, 1000);
//		APP_ERROR_CHECK(ret);
//		buf[cnt] = c;
//		cnt++;
//		NRF_LOG_RAW_INFO("%02x ", c);
//		if((c == 0x0A)) {
//			NRF_LOG_INFO("Counter: %d", cnt)
//			if((cnt == 74) && (buf[0] == 0x24)) {
//				NRF_LOG_INFO("GPGGA found!");
//				char temp[8];
//				temp[0] = buf[GPS_GGA_TIME_POS];
//				temp[1] = buf[GPS_GGA_TIME_POS + 1];
//				temp[2] = '\0';
//				cur_tag.time.h = atoi(temp);
//				temp[0] = buf[GPS_GGA_TIME_POS + 2];
//				temp[1] = buf[GPS_GGA_TIME_POS + 3];
//				temp[2] = '\0';
//				cur_tag.time.min = atoi(temp);
//				temp[0] = buf[GPS_GGA_TIME_POS + 4];
//				temp[1] = buf[GPS_GGA_TIME_POS + 5];
//				temp[2] = '\0';
//				cur_tag.time.sec = atoi(temp);
//				NRF_LOG_INFO("Current time: %dh%dm%ds", cur_tag.time.h, cur_tag.time.min, cur_tag.time.sec);
//			}
//			cnt = 0;
//		}
	}

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
			card_status = sdc_init();
			if(card_status == RES_OK) {
				NRF_LOG_INFO("SD card init done.");
				ff_result = sdc_mount();
				if(ff_result == FR_OK) {
					NRF_LOG_INFO("SD card mounted.");
					ff_result = sdc_init_audio();
					if(ff_result == FR_OK) {
						NRF_LOG_INFO("Audio file ready to record.");
						ui_rec_start_req = false;
						sdc_init_ok = true;
					}
					else {
						NRF_LOG_INFO("Unable to initialize audio file.");
						ui_sdc_init_cnt++;
					}
				}
				else {
					NRF_LOG_INFO("SD card init failed. Result: %d", ff_result);
					ui_sdc_init_cnt++;
				}
			}
			else {
				NRF_LOG_INFO("SD card check failed. Status: %d", card_status);
				ui_sdc_init_cnt++;
			}
			if(ui_sdc_init_cnt >= 3) {
				ui_rec_start_req = false;
			}
		}
//		if(sdc_rtw) {
//			sdc_rtw = false;
//			sdc_fill_queue();
//		}
//		else if(sdc_block_cnt > 0) {
//			NRF_LOG_INFO("Decounting FIFO...");
//			sdc_block_cnt--;
//			sdc_fill_queue();
//		}
//        __WFE();
    }
}

/** @} */
