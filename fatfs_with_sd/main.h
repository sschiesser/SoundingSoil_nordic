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
#include <stdint.h>
/* NRF libraries*/
#include "nrf.h"
#include "nordic_common.h"
#include "bsp.h"
#include "boards.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_error.h"
#include "nrf_queue.h"
#include "nrf_delay.h"
#include "nrf_serial.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
/* APP libraries */
#include "app_util_platform.h"
#include "app_error.h"
#include "app_fifo.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_button.h"
/* NRF DRV */
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"
#include "nrf_drv_clock.h"
/* BLE */
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_sss.h"
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
#define LED_RECORD						(BSP_LED_2)
#define LED_MONITOR						(BSP_LED_3)
#define LED_ADVERTISE					(BSP_LED_0)
#define LED_ON(led)						(nrf_drv_gpiote_out_clear(led))
#define LED_OFF(led)					(nrf_drv_gpiote_out_set(led))
#define LED_TOGGLE(led)					(nrf_drv_gpiote_out_toggle(led))

#define BUTTON_RECORD					(BSP_BUTTON_2)
#define BUTTON_MONITOR					(BSP_BUTTON_3)

/*                              ADC to SDC FIFO                               */
/* -------------------------------------------------------------------------- */
#define FIFO_DATA_SIZE					8192

/*                                  SD card                                   */
/* -------------------------------------------------------------------------- */
#define FILE_NAME   					"NORDIC.TXT"
#define TEST_STRING 					"SD card example."
#define ROOT_DIR						"0:/"
#define SDC_SCK_PIN     				23//28  ///< SDC serial clock (SCK) pin.
#define SDC_MOSI_PIN    				24//30  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    				22//29  ///< SDC serial data out (DO) pin.
#define SDC_CS_PIN      				25//31  ///< SDC chip select (CS) pin.
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
#define ADC_SYNC_TIMER_INSTANCE			1
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

#define GPS_UART_INSTANCE				0
#define GPS_UART_RX_PIN					27


/*                                   BLE                                      */
/* -------------------------------------------------------------------------- */
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "Nordic_Blinky"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



//void gps_config_uart(void);

#endif /* MAIN_H__ */
