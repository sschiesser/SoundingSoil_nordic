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


/* NRF libraries*/
#include "nrf.h"
#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include <string.h>
#include <stdio.h>
/* APP libraries */
#include "app_util_platform.h"
#include "app_error.h"

#include "nrf_drv_spi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/* SD card definitions */
#define FILE_NAME   "NORDIC.TXT"
#define TEST_STRING "SD card example."
#define SDC_SCK_PIN     28  ///< SDC serial clock (SCK) pin.
#define SDC_MISO_PIN    29  ///< SDC serial data out (DO) pin.
#define SDC_MOSI_PIN    30  ///< SDC serial data in (DI) pin.
#define SDC_CS_PIN      31  ///< SDC chip select (CS) pin.
//#define SDC_CD_PIN		04  ///< SCD card detect (CD) pin.

#define DATA_SIZE		2048
static uint8_t data_buffer[DATA_SIZE];

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

/* ADC defintions */
#define ADC_SPI_CONV_PIN				22
#define ADC_SPI_MOSI_PIN				23
#define ADC_SPI_MISO_PIN				24
#define ADC_SPI_SCK_PIN					25

#define ADC_SPI_INSTANCE				1
static const nrf_drv_spi_t adc_spi = NRF_DRV_SPI_INSTANCE(ADC_SPI_INSTANCE);
static volatile bool adc_spi_xfer_done = false;

static uint8_t							m_tx_buf[2] = {0xFF, 0xFF};
static uint8_t							m_rx_buf[2];
static const uint8_t					m_length = 2;

void adc_spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	adc_spi_xfer_done = true;
	bsp_board_led_invert(0);
}

DSTATUS sd_card_test(void)
{
//    static FATFS fs;
//    static DIR dir;
//    static FILINFO fno;
//    static FIL file;

//    uint32_t bytes_written;
//    FRESULT ff_result;
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


FRESULT sd_card_init(void)
{
    static FATFS fs;
	static FRESULT ff_result;
    static DIR dir;
    static FILINFO fno;
	
	NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed. Result: %d", ff_result);
        return ff_result;
    }

    NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return ff_result;
    }

    do
    {
        ff_result = f_readdir(&dir, &fno);
        if (ff_result != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return ff_result;
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
	return FR_OK;
}


static uint8_t ss_example(uint32_t disk_dev_num)
{
	FRESULT res;
//    static FATFS fs;
//    static DIR dir;
//    static FILINFO fno;
//    static FIL file;

//    uint32_t bytes_written;
//    DSTATUS disk_state = STA_NOINIT;

	TCHAR   root_directory[4];
	TCHAR file_name[18];
	//UINT file_number;
	TCHAR test_folder_name[13];
	TCHAR test_folder_location[25];

	
	/* Declare these as static to avoid stack usage.
	 * They each contain an array of maximum sector size.
	 */
	char         disk_str_num[2];

	sprintf(disk_str_num, "%d", disk_dev_num);
	sprintf(test_folder_name, "%s", (const char *)"180314");
	sprintf(root_directory, "%s:/", disk_str_num);
	sprintf(test_folder_location, "%s%s", root_directory, test_folder_name);

	
	/* ===== CHECK DIRECTORY EXISTENCE ===== */
	FILINFO filinfo;
	bool makedir = false;
	
	res = f_stat(test_folder_location, &filinfo);
	if(res == FR_OK) {
		NRF_LOG_INFO("-I- folder already exist\n\r");
	}
	else if((res == FR_NO_PATH) || (res == FR_NO_FILE)) {
		NRF_LOG_INFO("-W- folder not found... create!\n\r");
		makedir = true;
	}
	else {
		NRF_LOG_INFO("-E- f_stat pb: 0x%X\n\r", res);
		return 1;
	}
	
	/* ===== CREATE DIRECTORY ===== */
	if(makedir) {
		res = f_mkdir(test_folder_name);
		if (res != FR_OK) {
			NRF_LOG_INFO("-E- f_mkdir pb: 0x%X\n\r", res);
			return 1;
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
	res = f_opendir(&dirinfo, test_folder_location);
	if (res == FR_OK) {
		/* Display the file tree */
		NRF_LOG_INFO("-I- Display files contained in the memory :\r");
		strcpy((char *)data_buffer, test_folder_location);
		scan_files((char *)data_buffer, &scanned_files);
		NRF_LOG_INFO("Number of found files: %d\n\r", scanned_files);
	}

	
	/* ===== CREATE NEW FILE ===== */
	static FIL   file_object;
	TCHAR new_file[13];
	for(;;) {
		sprintf(new_file, "R%04d", scanned_files);
		sprintf(file_name, "%s/%s.wav", test_folder_location, new_file); /*File path*/
		NRF_LOG_INFO("-I- Create a file : \"%s\"\n\r", file_name);
		res = f_open(&file_object, (char const *)file_name, FA_CREATE_NEW | FA_READ | FA_WRITE);
		if (res == FR_OK) {
			break;
		}
		else if (res != FR_EXIST) {
			NRF_LOG_INFO("-E- Filename already exist... increasing iteration\n\r");
			scanned_files++;
		}
		else {
			NRF_LOG_INFO("-E- f_open create pb: 0x%X\n\r", res);
			return 1;
		}
	}

	/* ===== READ FILE TO COPY ===== */
	static FIL file2;
	static FSIZE_t f2_size;
	int i;
	NRF_LOG_INFO("-I- Opening read-only file\n\r");
	res = f_open(&file2, "0:/song.mp3", FA_READ);
	if(res != FR_OK) {
		NRF_LOG_INFO("Opening failed");
		return 1;
	}

	f2_size = f_size(&file2);
	UINT byte_read, byte_written;
	for(i = 0; i < f2_size; i += DATA_SIZE) {
//		NRF_LOG_INFO(".");
		res = f_read(&file2, (char*)data_buffer, DATA_SIZE, &byte_read);
		if(res == FR_OK) {
//			NRF_LOG_INFO(".");
			res = f_write(&file_object, (const char *)data_buffer, DATA_SIZE, &byte_written);
			if(res == FR_OK) {
				NRF_LOG_RAW_INFO(".");
				res = f_sync(&file_object);
			}
		}
	}
	NRF_LOG_INFO("\n\r");
	///* Write a checkerboard pattern in the buffer */
	//int i;
//
	//for (i = 0; i < sizeof(data_buffer); i++) {
		//if ((i & 1) == 0) {
			//data_buffer[i] = (i & 0x55);
		//} else {
			//data_buffer[i] = (i & 0xAA);
		//}
	//}
	//puts("-I- Write file\r");
	//for (i = 0; i < TEST_SIZE; i += DATA_SIZE) {
		//res = f_write(&file_object, data_buffer, DATA_SIZE, &byte_written);
//
		//if (res != FR_OK) {
			//printf("-E- f_write pb: 0x%X\n\r", res);
			//return 0;
		//}
	//}

	/* Flush after writing */
	NRF_LOG_INFO("-I- Syncing file");
	res = f_sync(&file_object);
	if (res != FR_OK) {
		NRF_LOG_INFO("-E- f_sync pb: 0x%X", res);
		return 1;
	}
	/* Close the file */
	NRF_LOG_INFO("-I- Close file");
	res = f_close(&file_object);
	if (res != FR_OK) {
		NRF_LOG_INFO("-E- f_close pb: 0x%X", res);
		return 1;
	}
	return 0;
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

    NRF_LOG_INFO("=========================\n\rFATFS + SD + SPI example.");

	nrf_drv_spi_config_t adc_spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	adc_spi_config.ss_pin = ADC_SPI_CONV_PIN;
	adc_spi_config.miso_pin = ADC_SPI_MISO_PIN;
	adc_spi_config.mosi_pin = ADC_SPI_MOSI_PIN;
	adc_spi_config.sck_pin = ADC_SPI_SCK_PIN;
	adc_spi_config.frequency = NRF_DRV_SPI_FREQ_8M;
	APP_ERROR_CHECK(nrf_drv_spi_init(&adc_spi, &adc_spi_config, adc_spi_event_handler, NULL));
	

	card_status = sd_card_test();
	if(card_status != RES_OK) {
		NRF_LOG_INFO("SD card check failed. Status: %d", card_status);
	}
	else {
		NRF_LOG_INFO("SD card check OK");
		ff_result = sd_card_init();
		if(ff_result != FR_OK) {
			NRF_LOG_INFO("SD card init failed. Result: %d", ff_result);
		}
		else {
			NRF_LOG_INFO("\nSD card init OK");
			if(!ss_example(0)) {
				NRF_LOG_INFO("SS test passed!");
			}
			else {
				NRF_LOG_INFO("SS test failed!");
			}
		}
	}

	NRF_LOG_INFO("Starting SPI xfer");
	adc_spi_xfer_done = false;
	nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);

    while (true)
    {
		if(adc_spi_xfer_done) {
			adc_spi_xfer_done = false;
			nrf_delay_us(200);
			nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);
		}
        __WFE();
    }
}

/** @} */
