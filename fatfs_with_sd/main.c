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
//app_fifo_t								audio_fifo;
//static uint8_t							fifo_buffer[FIFO_DATA_SIZE];
app_fifo_t								m_adc2sd_fifo;
uint8_t									m_fifo_buffer[FIFO_DATA_SIZE];

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
//static uint8_t							sdc_buffer[SDC_BLOCK_SIZE];

static uint8_t 							data_buffer[FIFO_DATA_SIZE];
static uint8_t							sdc_block_cnt = 0;
static FIL   							recording_fil;


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

static volatile uint16_t 				adc_spi_xfer_counter = 0;

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




static uint8_t							m_tx_buf[2] = {0xFF, 0xFF};
static uint8_t							m_rx_buf[2];
static const uint8_t					m_length = 2;


/* ========================================================================== */
/*                              APP FUNCTIONS                                 */
/* ========================================================================== */
static void sdc_fill_queue(void)
{
	static FRESULT res;
	static UINT byte_written;
	static uint8_t p_buf[2*SDC_BLOCK_SIZE];
//	uint32_t fifo_size = m_adc2sd_fifo.write_pos - m_adc2sd_fifo.read_pos;
//	NRF_LOG_INFO("FIFO W->%d, R->%d, size: %d", m_adc2sd_fifo.write_pos, m_adc2sd_fifo.read_pos, fifo_size);
	uint32_t buf_size = 2*SDC_BLOCK_SIZE;
	DBG_TOGGLE(DBG2_PIN);
	NRF_LOG_INFO("FIFO READ @ %d", m_adc2sd_fifo.read_pos);
	uint32_t fifo_res = app_fifo_read(&m_adc2sd_fifo, p_buf, &buf_size);
//
	sdc_writing = true;
	res = f_write(&recording_fil, p_buf, (2*SDC_BLOCK_SIZE), &byte_written);
	if(res == FR_OK) {
		res = f_sync(&recording_fil);
		sdc_writing = false;
	}
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
	
	ff_result = f_lseek(&recording_fil, 0);
	if(ff_result != FR_OK) {
		NRF_LOG_DEBUG("Error while seeking for beginning of file");
		return ff_result;
	}
	
	ff_result = f_write(&recording_fil, wave_header, 44, &bytes);
	if(ff_result != FR_OK) {
		NRF_LOG_DEBUG("Error while updating the WAV header");
		return ff_result;
	}
	
    (void)f_close(&recording_fil);
    return ff_result;
}

/* ========================================================================== */
/*                              EVENT HANDLERS                                */
/* ========================================================================== */

void adc_spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
	static uint32_t buf_size = 2;
	app_fifo_write(&m_adc2sd_fifo, m_rx_buf, &buf_size);
	if(adc_spi_xfer_counter < (SDC_BLOCK_SIZE-1)) {
		adc_spi_xfer_counter++;
	}
	else {
		DBG_TOGGLE(DBG1_PIN);
		NRF_LOG_INFO("FIFO WROTE to %d", m_adc2sd_fifo.write_pos);
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

void adc_sync_timer_handler(nrf_timer_event_t event_type, void * p_context)
{
	DBG_TOGGLE(DBG0_PIN);
	if(adc_spi_xfer_done) {
		adc_spi_xfer_done = false;
		nrf_drv_spi_transfer(&adc_spi, m_tx_buf, m_length, m_rx_buf, m_length);
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
		&ADC_SYNC_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	
//	nrf_drv_timer_enable(&ADC_SYNC_TIMER);
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
	static FRESULT res;
    static DIR dir;
    static FILINFO fno;

	TCHAR root_directory[4];
	TCHAR file_name[18];
	TCHAR test_folder_name[13];
	TCHAR test_folder_location[25];
	char disk_str_num[2];
	sprintf(disk_str_num, "%d", 0);
	sprintf(test_folder_name, "%s", (const char *)"180316");
	sprintf(root_directory, "%s:/", disk_str_num);
	sprintf(test_folder_location, "%s%s", root_directory, test_folder_name);
	
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
		return res;
	}
	
	/* ===== CREATE DIRECTORY ===== */
	if(makedir) {
		res = f_mkdir(test_folder_name);
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
	res = f_opendir(&dirinfo, test_folder_location);
	if (res == FR_OK) {
		/* Display the file tree */
		NRF_LOG_INFO("-I- Display files contained in the memory :\r");
		strcpy((char *)data_buffer, test_folder_location);
		scan_files((char *)data_buffer, &scanned_files);
		NRF_LOG_INFO("Number of found files: %d\n\r", scanned_files);
	}

	
	/* ===== CREATE NEW FILE ===== */
//	static FIL   file_object;
	TCHAR new_file[13];
	for(;;) {
		sprintf(new_file, "R%04d", scanned_files);
		sprintf(file_name, "%s/%s.wav", test_folder_location, new_file); /*File path*/
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
	
	uint32_t bytes_written;
	NRF_LOG_DEBUG("Writing WAV header...");
	res = f_write(&recording_fil, wave_header, 44, (UINT *) &bytes_written);
	if (res != FR_OK) {
		NRF_LOG_DEBUG("Unable to write WAV header");
		return (uint32_t)res;
	}


	return FR_OK;
}






/**
 * @brief Function for main application entry.
 */
int main(void)
{
	DSTATUS card_status;
	FRESULT ff_result;
	ret_code_t err_code;
	
	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
	
    bsp_board_leds_init();
	buttons_init();
#ifdef DEBUG
	gpio_dbg_init();
#endif
	
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("=========================")
	NRF_LOG_INFO("FATFS + SD + SPI example.");
    NRF_LOG_INFO("-------------------------")

	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();

	adc_config_spi();
	adc_config_timer();
	app_fifo_init(&m_adc2sd_fifo, m_fifo_buffer, FIFO_DATA_SIZE);
	
	app_button_enable();
	advertising_start();

	for(;;)
	{
		if(ui_rec_start_req) {
			ui_rec_start_req = false;
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
					sdc_init_ok = true;
				}
			}
		}
		
		if(ui_rec_stop_req) {
			ui_rec_stop_req = false;
			nrf_drv_timer_disable(&ADC_SYNC_TIMER);
			sdc_close();
			err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 0);
			if(err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
				APP_ERROR_CHECK(err_code);
			}
			ui_rec_start_req = false;
			ui_rec_running = false;
		}
				

		if(sdc_init_ok) {
			sdc_init_ok = false;
			NRF_LOG_INFO("Starting SPI xfer");
			err_code = ble_sss_on_button1_change(m_conn_handle, &m_sss, 1);
			if (err_code != NRF_SUCCESS &&
				err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
				err_code != NRF_ERROR_INVALID_STATE &&
				err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) {
				APP_ERROR_CHECK(err_code);
			}
			adc_spi_xfer_done = true;
			nrf_drv_timer_enable(&ADC_SYNC_TIMER);
			ui_rec_running = true;
		}

		if(sdc_rtw) {
			sdc_rtw = false;
			sdc_fill_queue();
		}
		else if(sdc_block_cnt > 0) {
//			NRF_LOG_INFO("Decounting FIFO...");
			sdc_block_cnt--;
			sdc_fill_queue();
		}

		__WFE();

	}
}

/** @} */
