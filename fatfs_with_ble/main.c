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
static uint8_t							audio_buffer[SDC_BLOCK_SIZE];

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
static FATFS 							sdc_fs;
static DIR 								sdc_dir;
static FILINFO 							sdc_fno;
static FIL 								sdc_file;
static volatile bool 					sdc_init_ok = false;
static volatile bool					sdc_rtw = false;

/*                                    ADC                                     */
/* -------------------------------------------------------------------------- */
const nrf_drv_timer_t					ADC_SYNC_TIMER = NRF_DRV_TIMER_INSTANCE(ADC_SYNC_TIMER_INSTANCE);
static volatile uint32_t				adc_samples_counter = 0;
static volatile uint32_t				adc_total_samples = 0;

/*                                    GPS                                     */
/* -------------------------------------------------------------------------- */

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
/*                                  UTILS                                     */
/* ========================================================================== */
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

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed. Disk state: %d", disk_state);
        return (uint32_t)disk_state;
    }

    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&sdc_fs, "", 1);
    if (ff_result)
    {
        NRF_LOG_INFO("Mount failed. Result: %d", ff_result);
        return (uint32_t)ff_result;
    }

    NRF_LOG_INFO("\r\n Listing directory: /");
    ff_result = f_opendir(&sdc_dir, "/");
    if (ff_result)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return (uint32_t)ff_result;
    }

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
                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)sdc_fno.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s", sdc_fno.fsize, (uint32_t)sdc_fno.fname);
            }
        }
    }
    while (sdc_fno.fname[0]);
    NRF_LOG_INFO("");
    NRF_LOG_INFO("Opening file " FILE_NAME "...");
    ff_result = f_open(&sdc_file, FILE_NAME, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Unable to open or create file: " FILE_NAME ".");
        return (uint32_t)ff_result;
    }
	
	NRF_LOG_INFO("Writing WAV header...");
	ff_result = f_write(&sdc_file, wave_header, 44, (UINT *) &bytes_written);
	if (ff_result != FR_OK) {
		NRF_LOG_INFO("Unable to write WAV header");
		return (uint32_t)ff_result;
	}
	
	return (uint32_t)0;
}

static FRESULT sdc_write(void)
{
    uint32_t bytes_written;
    FRESULT ff_result;

    NRF_LOG_INFO("Writing to file " FILE_NAME "...");
//    ff_result = f_write(&sdc_file, TEST_STRING, sizeof(TEST_STRING) - 1, (UINT *) &bytes_written);
    ff_result = f_write(&sdc_file, audio_buffer, sizeof(audio_buffer), (UINT *) &bytes_written);
    if (ff_result != FR_OK)
    {
        NRF_LOG_INFO("Write failed\r\n.");
    }
    else
    {
        NRF_LOG_INFO("%d bytes written.", bytes_written);
    }
	f_sync(&sdc_file);
	return ff_result;
}

static void sdc_close(void)
{
    (void) f_close(&sdc_file);
    return;
}


/* ========================================================================== */
/*                              APP FUNCTIONS                                 */
/* ========================================================================== */
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

/* ========================================================================== */
/*                              EVENT HANDLERS                                */
/* ========================================================================== */
// ADC timer
void adc_sync_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	DBG_TOGGLE(DBG0_PIN);
	audio_buffer[adc_samples_counter] = 0xAA;
	audio_buffer[adc_samples_counter+1] = 0x55;
	adc_samples_counter += 2;
	if(adc_samples_counter >= SDC_BLOCK_SIZE) {
		adc_total_samples += SDC_BLOCK_SIZE;
		adc_samples_counter = 0;
		sdc_rtw = true;
	}
//	if(adc_spi_xfer_done) {
//		adc_spi_xfer_done = false;
//		nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);
//	}
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
	
	adc_config_timer();
	
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
			NRF_LOG_DEBUG("Starting REC");
			app_timer_start(led_blink_timer, APP_TIMER_TICKS(200), NULL);
			if(sdc_start() == 0) {
				sdc_init_ok = true;
			}
			ui_rec_start_req = false;
		}
		// REC STOP request
		if(ui_rec_stop_req) {
			NRF_LOG_DEBUG("Stopping REC");
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
			NRF_LOG_DEBUG("Starting MON");
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
			NRF_LOG_DEBUG("Stopping MON");
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
			NRF_LOG_DEBUG("SDC init OK");
			app_timer_stop(led_blink_timer);
			err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 1);
			if (err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
				APP_ERROR_CHECK(err_code);
			}
			LED_ON(LED_RECORD);
//			sdc_write();
			nrf_drv_timer_enable(&ADC_SYNC_TIMER);
			ui_rec_running = true;
			sdc_init_ok = false;
		}
		
        // SDC ready-to-write
		if(sdc_rtw) {
			sdc_write();
			sdc_rtw = false;
		}
		
		__WFE();
    }
}

/** @} */
