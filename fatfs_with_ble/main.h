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
#include <time.h>
/* NRF libraries*/
#include "nrf.h"
#include "nordic_common.h"
#include "bsp.h"
#include "boards.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "mem_manager.h"
#include "nrf_block_dev_sdc.h"
#include "nrf_serial.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
/* APP libraries */
#include "app_fifo.h"
#include "app_timer.h"
#include "app_button.h"
#include "nrf_delay.h"
/* NRF DRV */
#include "nrf_drv_spi.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"
/* SoftDevice header (SDH) */
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
/* BLE */
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_sss.h"
#include "ble_nus.h"
#include "ble_advertising.h"
/* LOG */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/* SoundingSoil */
#include "wave_header.h"


/* ========================================================================== */
/*                                   MACROS                                   */
/* ========================================================================== */

/*                                REC PRESET                                  */
/* -------------------------------------------------------------------------- */
#define REC_DURATION_IN_S				10//300
#define REC_PERIODE_IN_S				12//3600
#if (REC_PERIODE_IN_S <= REC_DURATION_IN_S)
#error "Cannot use this recording preset!"
#endif
#define REC_OCCURENCE_MAX				2//24

/*                                   DEBUG                                    */
/* -------------------------------------------------------------------------- */
#define DBG0_PIN						11
#define DBG1_PIN						12
#define DBG2_PIN						18
#define DBG_TOGGLE(dbg)					(nrf_drv_gpiote_out_toggle(dbg))

/*                                  MONITOR                                   */
/* -------------------------------------------------------------------------- */
#define MON_EN_PIN						3
#define MON_ENABLE()					(nrf_drv_gpiote_out_set(MON_EN_PIN))
#define MON_DISABLE()					(nrf_drv_gpiote_out_clear(MON_EN_PIN))

/*                                 LED/BUTTON                                 */
/* -------------------------------------------------------------------------- */
#define LED_BLE							(BSP_LED_3)
#define LED_RECORD						(BSP_LED_2)
#define LED_MONITOR						(BSP_LED_1)
#define LED_ON(led)						(nrf_drv_gpiote_out_clear(led))
#define LED_OFF(led)					(nrf_drv_gpiote_out_set(led))
#define LED_TOGGLE(led)					(nrf_drv_gpiote_out_toggle(led))

#define BUTTON_BLE						(BSP_BUTTON_3)
#define BUTTON_RECORD					(BSP_BUTTON_2)
#define BUTTON_MONITOR					(BSP_BUTTON_1)
#define BUTTON_DETECTION_DELAY			APP_TIMER_TICKS(50) // Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks).

/*                                  SD card                                   */
/* -------------------------------------------------------------------------- */
//#define FILE_NAME   					"R123456.wav"
#define TEST_STRING 					"SD card example."
//#define ROOT_DIR						"0:/"
#define SDC_CS_PIN      				31  ///< SDC chip select (CS) pin.
#define SDC_MOSI_PIN    				30  ///< SDC serial data in (DI) pin.
#define SDC_MISO_PIN    				29  ///< SDC serial data out (DO) pin.
#define SDC_SCK_PIN     				28  ///< SDC serial clock (SCK) pin.
#define SDC_CD_PIN						4	///< SDC card detect (CD) pin.
#define SDC_BLOCK_SIZE					(16*SDC_SECTOR_SIZE) // 2x -> 19-39ms, 4x -> 24-44ms, 8x ->38-55ms, 16x -> 74ms, 32x -> 110ms

/*                                    ADC                                     */
/* -------------------------------------------------------------------------- */
// SPI
#define ADC_SPI_CONV_PIN				23
#define ADC_SPI_MOSI_PIN				22
#define ADC_SPI_MISO_PIN				24
#define ADC_SPI_SCK_PIN					25
#define ADC_SPI_INSTANCE				1

// SYNC TIMER
#define ADC_SYNC_TIMER_INSTANCE			1	// TIMER0 is blocked by SoftDevice
#define ADC_SYNC_16KHZ_US				60
#define ADC_SYNC_28KHZ_US				36 // <-- currently max usable frequency
#define ADC_SYNC_44KHZ_US				23
#define ADC_SYNC_48KHZ_US				20
#ifdef DEBUG
	#define ADC_SYNC_US					ADC_SYNC_16KHZ_US
#else
	#define ADC_SYNC_US					ADC_SYNC_28KHZ_US
#endif


/*                                    FIFO                                    */
/* -------------------------------------------------------------------------- */
#define SDC_FIFO_SIZE					(32768)
#define BLE_MAX_MTU_SIZE				(NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3)
#define BLE_FIFO_SIZE					256


/*                                   GPS                                      */
/* -------------------------------------------------------------------------- */
#define GPS_NMEA_MAX_SIZE				100
#define GPS_NMEA_START_CHAR				0x24 // $
#define GPS_NMEA_STOP_CHAR				0x0A // LF

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

struct gps_time {
	uint8_t h;
	uint8_t min;
	uint8_t sec;
	uint16_t msec;
};
struct gps_date {
	uint8_t day;
	uint8_t month;
	uint16_t year;
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


/*                                TIMESTAMP                                   */
/* -------------------------------------------------------------------------- */
struct timestamp_tag {
	bool ts_valid;
	struct gps_time time;
	struct gps_date date;
};
enum timestamp_source {
	TS_SOURCE_GPS = 0x0,
	TS_SOURCE_PHONE,
	TS_SOURCE_BACKUP,
	TS_SOURCE_NONE
};

/*                                   BLE                                      */
/* -------------------------------------------------------------------------- */
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2    /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "Nordic_Blinky"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      5//BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (30 ms). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(2000, UNIT_10_MS)         /**< Connection supervisory time-out (2 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(4000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (4 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define MON_DOWNSAMPLE_FACTOR			8



#endif /* MAIN_H__ */
