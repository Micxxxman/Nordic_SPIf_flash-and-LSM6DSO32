/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
//#include "ble_nus.h"
//#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_drv_usbd.h"
#include "nrfx_power.h"
#include "nrf_drv_power.h"
#include "nrfx_saadc.h"

#if (NRF_QUEUE_ENABLED)
#include "nrf_queue.h"
#endif

#include "ble_bas.h"
#include "ble_collar_data.h"
#include "ble_dis.h"
#include "myspi.h"
#include "lsm6dso32x.h"
#include "w25q64jv.h"

#define TEST_ONLY						1
#define USE_TX_8DBM						0
#define USE_POWER_DOWN_ON_FLASH			0
#define SAVE_DATA_TO_FLASH				1
#define VERIFY_SAVE_DATA				0

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

//#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
//#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

//#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
//#define APP_ADV_INTERVAL                1120
#define APP_ADV_INTERVAL				320

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

//#define FIFO_THRESHOLD					100
#define SAMPLING_FREQ					(double)12.5
#define PERIOD_OF_DATA_RETRIEVE			8	// 4 seconds
#define GET_DATA_THRESHOLD				(uint32_t)(SAMPLING_FREQ * PERIOD_OF_DATA_RETRIEVE)
#define FIFO_THRESHOLD					100
#define NUM_OF_CHUNKS					600
#define NUM_OF_RESULTANT				64
#define RESULTANT_THRESHOLD				6
#define ACTIVE_THRESHOLD				25

#define SENSOR_TIMER_INTERVAL			100 //ms
#define SENSOR_APP_TIMER_INTERVAL		APP_TIMER_TICKS(SENSOR_TIMER_INTERVAL)
#define SENSOR_ACT_TO_PASS_INTERVAL		30000 // ms
#define SENSOR_ACT_TO_PASS_CNT			(SENSOR_ACT_TO_PASS_INTERVAL / SENSOR_TIME_INTERVAL)

#define NUM_OF_TMP_DATA_PACKS			10

#define MANUFACTURER_NAME				"Acoustic Arc Int. Ltd."
#define MODEL_NO_NAME					"HT2227"
#define SERIAL_NO_NAME					"SERIAL_NO"
#define HW_REVISION_NAME				"1.0.0"
#define SW_REVISION_NAME				"1.0.0"
#define FW_REVISION_NAME				"1.0.0"

#define DIS_INFO_IDX_MANUF				0
#define DIS_INFO_IDX_MODEL				1
#define DIS_INFO_IDX_SERIAL				2
#define DIS_INFO_IDX_HW					3
#define DIS_INFO_IDX_FW					4
#define DIS_INFO_IDX_SW					5

typedef enum
{
	SENSOR_STATE_IDLE,		// this is the state when the application just first start
	SENSOR_STATE_SETUP,
	SENSOR_STATE_READY,	// this is the default after setup
	SENSOR_STATE_UNKNOWN,	// this is the case when no LSM6DS032X,
}SENSOR_STATE_t;

typedef enum
{
	FLASH_CHECK_STAGE_IDLE,
	FLASH_CHECK_STAGE_HEADER,
	FLASH_CHECK_STAGE_LAST_DATA_LOC,
//	FLASH_CHECK_STAGE_LAST_DATA_LOC2,
	FLASH_CHECK_STAGE_DONE
}FLASH_CHECK_STAGE_t;

typedef struct __attribute__((packed))
{
	uint8_t version;
	uint8_t activation;
	uint32_t timestamp;
}FlashHeader_t;

typedef struct __attribute__((packed))
{
	uint16_t year:7;	// 0 ~ 99
	uint16_t month:4;	// 1 ~ 12
	uint16_t day:5;		// 1 ~ 31
	uint16_t hour: 5;	// 0 ~ 23
	uint16_t minute: 6;	// 0 ~ 59
	uint16_t second: 6;	// 0 ~ 59
	uint16_t millis: 7;	// 0 ~ 1000
}MyDate_t;

typedef union __attribute__((packed))
{
	uint32_t val;
	struct __attribute__((packed))
	{
		uint32_t millis: 16;	// 0 ~ 59999
		uint32_t minute: 6;		// 0 ~ 60
		uint32_t hours: 5;		// 0 ~ 23
		uint32_t days: 5;		// 0 ~ 29
	};
}MyTicks_t;

typedef struct __attribute__((packed))
{
	uint8_t data[255];
}FlashPage_t;	//!< This represents a page of the flash data.

typedef struct __attribute__((packed))
{
	SENSOR_STATE_t sensorState;
	struct __attribute__((packed))
	{
		volatile uint32_t timeForSensor: 1;
		uint32_t hasGyro: 1;				//!< 0 -> no sensor is detected
		uint32_t hasFlash: 1;				//!< 0 -> no flash is detected
		uint32_t isActivated: 1;			//!< 0 -> not activated, 1 -> activated.

		uint32_t is5V: 1;					//!< has 5V plugged in
		uint32_t isCharging: 1;				//!< charge pin is '0' ( only valid when is5V=1 )
		uint32_t isFull: 1;					//!< standyby pin is '0' ( only valid when is5V=1 )
		uint32_t isCollarActive: 1;			//!< 0 -> passive ( no motion ), 1 -> active

		uint32_t isAdvertising: 1;			//!< 0 -> not advertising, 1 -> advertising
		uint32_t is1stData: 1;				//!< 1 -> 1st time retrieving the data.
		uint32_t isNoPrev: 1;				//!< 1 -> no previous data at hand.
		uint32_t isConnected: 1;			//!< 1 -> BLE is connected

		uint32_t is1Sec: 1;
		uint32_t is4Sec: 1;					//!< Assume we get data at 12.5Hz, we should retrieve data every 4 seconds
		uint32_t isDataCountReached: 1;		//!< int1 trigger data
		uint32_t hasCheckBatt:1;			//!< This is set after the 1st battery level measurement is done.

		uint32_t isAdcDone: 1;

		uint32_t : 15;
	};
}SysFlag_t;		//!< This is my main system flag

typedef union __attribute__((packed))
{
	uint8_t val;
	struct __attribute__((packed))
	{
		uint8_t isDiff:	1;		//!< 0 -> full int16_t x, y, z, data. 1 -> diff of int8_t x, y, z
		uint8_t cnt: 7;			//!< This means the no. of successive data with diff only content from previous value.
	};
}DataFlag_t;

typedef struct __attribute__((packed))
{
	uint8_t pData[7];

	struct __attribute__((packed))
	{
		union __attribute__((packed))
		{
			uint8_t val;
			struct __attribute__((packed))
			{
				uint8_t isActive: 1;
				uint8_t :7;
			};
		};
		Acc3Data_t acc;
	};
}Acc3DataWithFlag_t;

//typedef struct __attribute__((packed))
//{
//	Acc3DataWithFlag_t data[FIFO_THRESHOLD];
//	uint32_t timestamp;		//!< timestamp for the 1st data
////	uint16_t idx;
//	uint8_t idx;			//!< this is used to mark which data is not processed
//}TmpData_t;	//!< This stands for one block of FIFO data

typedef struct __attribute__((packed))
{
	uint8_t getData;		//!< This is the time counter to when the time is enough for retrieve data.
							// This is implemented as the frequencies of the sensor and nRF52 are not the same,
							// We may end up like having 52 data in 4 seconds under 12.5Hz sampling rate.
							// As a result, we cannot rely solely on the int1 for when the data is ready.
	uint8_t toPassive;		//!< This is used to count of no. of interval passive data
	uint16_t passiveData;	//!< This is used to count the no. of passive data retrieve
	uint8_t keepTime;		//!< This is used to count the 1 second for RTC
	uint8_t for1Sec;
}MyCnt_t;

static void startADC(void);

uint32_t prevAvg, prevSum, curAvg;
MyTicks_t lastTick;

//BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
BLE_COLLAR_DEF(m_collar);
BLE_BAS_DEF(m_batt);

#if (NRF_QUEUE_ENABLED)
// Setup the queue for temporary storing the data retrieve from the sensor.
NRF_QUEUE_DEF(Acc3DataWithTime_t, rawDataQueue, NUM_OF_CHUNKS, NRF_QUEUE_MODE_OVERFLOW);
#endif

APP_TIMER_DEF(m_sensor_id);
APP_TIMER_DEF(mTimerConnUpd);

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
//static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
//{
//    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
//};

static FlashPage_t flashData[4];
static uint8_t flashDataPage = 0;

static SysFlag_t flag;
static uint32_t startAddr = 0x001000;

static uint8_t lastDayOfMonth = 0;
static MyTicks_t myTicks;		//!< this is used as a time stamp, it should only be added inside the app timer routine
static MyDate_t myDate;
static MyCnt_t myCnt;	//!< this variables groups all sorts of counters
//static MyBuf_t myBuf;	//!< This is to hold the data received from sensor

static uint32_t epochTime = 0;

//static TmpData_t tmpData[NUM_OF_TMP_DATA_PACKS];
//static uint8_t tmpStart = 0;
//static uint8_t tmpIdx = 0;

#define LENGTH_OF_COLLAR_NAME						sizeof(collarName)/sizeof(uint8_t)
static uint8_t collarName[] =
{
	'm', 'o', 'g', 'g', 'i', 'e',
	'c', 'o', 'l', 'l', 'a', 'r', '-',
	'0', '0',
	'0', '0',
	'0', '0',
	'0', '0',
};

static const ble_srv_utf8_str_t disInfo[] =
{
	// manufacturer name
	{sizeof(MANUFACTURER_NAME)/sizeof(uint8_t), MANUFACTURER_NAME},
	{sizeof(MODEL_NO_NAME)/sizeof(uint8_t), MODEL_NO_NAME},
	{sizeof(SERIAL_NO_NAME)/sizeof(uint8_t), SERIAL_NO_NAME},
	{sizeof(HW_REVISION_NAME)/sizeof(uint8_t), HW_REVISION_NAME},
	{sizeof(FW_REVISION_NAME)/sizeof(uint8_t), FW_REVISION_NAME},
	{sizeof(SW_REVISION_NAME)/sizeof(uint8_t), SW_REVISION_NAME}
};

static nrf_saadc_value_t samples[1];
static nrfx_saadc_channel_t channels[1] =
{
	NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0)	// PIN 0.02 -> AIN0
};
static uint8_t adcCnt = 0;
static uint32_t adcSum = 0;

static uint8_t battLevel = 0;
static uint8_t battCnt = 0xFF;

static uint32_t getDataSize = 0;

// Function prototypes
static void advertising_start(void);


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//// ========================================================================= //
//static uint8_t readFlashData(uint32_t address, uint8_t* pData, uint8_t datasize)
//// ========================================================================= //
//{
//	Cmd_t* pCmd;
//	uint8_t len, tmp[4], ok;
//
//	nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
//	pCmd = W25Q64JVGetCmds(FLASH_CMD_READ_DATA);
//	len = pCmd->dataSize;
//	memcpy(tmp, pCmd->pData, len);
//	tmp[1] = (address >> 16) & 0xFF;
//	tmp[2] = (address >> 8) & 0xFF;
//	tmp[3] = address & 0xFF;
//	ok = MySpiWriteOnly(tmp, len);
//	if(!ok)
//	{
//		nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
//		return NO;
//	}
//	ok = MySpiReadOnly(pData, datasize);
//	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
//	NRF_LOG_INFO("Read address: %06X", address);
//	NRF_LOG_HEXDUMP_INFO(tmp, MIN(datasize, 16));
//
//	return ok;
//}

//// ========================================================================= //
//static uint8_t writeFlashData(uint32_t address, uint8_t* pData, uint8_t datasize)
//// ========================================================================= //
//{
//	Cmd_t* pCmd;
//	uint8_t ok;
//	uint8_t firstByte = 0x06, tmp[4];
//
//
//	// 1. Write enable.
//	nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
//	ok = MySpiWriteOnly(&firstByte, 1);
//	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
//	if(!ok)
//	{
//		NRF_LOG_INFO("Write enable fail...");
//		return NO;
//	}
//
//	// 2.
//	pCmd = W25Q64JVGetCmds(FLASH_CMD_PAGE_WRITE);
//	memcpy(tmp, pCmd->pData, pCmd->dataSize);
//	tmp[1] = (address >> 16) & 0xFF;
//	tmp[2] = (address >> 8) & 0xFF;
//	tmp[3] = address & 0xFF;
//	nrf_delay_us(50);
//
//	// 3.
//	nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
//	ok = MySpiWriteOnly(tmp, pCmd->dataSize);
//	if(!ok)
//	{
//		nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
//		NRF_LOG_INFO("Fail to send write command");
//		return NO;
//	}
//
//	ok = MySpiWriteOnly(pData, datasize);
//	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
////	nrf_delay_ms(3);
//
//	return ok;
//}


// ========================================================================= //
/**
 * @brief This function is used t to prepare the data and build the structure
 *        of the data to be saved.
 *        Using the flag, we can limit to just use one full 7 bytes data
 *        block for flag (1 byte), x (2 bytes), y (2 bytes), z (2 bytes).
 *        For the others, we can use flag to use diff-data, which only
 *        contains the difference of the previous data. The flag can also
 *        indicates how many diff-data is placed so that we don't need to
 *        include the flag for every diff-data.
 *
 *  For example,
 *  data 1 : x = 100, y = 200, z = 300
 *  data 2 : x = 101, y = 199, z = 299
 *
 *  If we use diff-data, we have rewrite into,
 *  diff-data: x = 1, y = -1, z = -1
 *  In this way, we use 3 bytes less for each data.
 *  For one page of data, we can have up to 80 diff-data.
 */
// 255 bytes for one page
//
// +------+--------+--------+--------+--------+--------+--------+--------+
// | flag | data-x | data-x | data-y | data-y | data-z | data-z | .......|
// +------+--------+--------+--------+--------+--------+--------+--------+
// + flag + diff-x | diff-y | diff-z | diff-x + diff-y + diff-z | .......|
// +------+--------+--------+--------+--------+--------+--------+--------+
// .                                 .                          .        .
// .                                 .                          .        .
// .                                 .                          .        .
// +------+--------+--------+--------+--------+--------+--------+--------+
// + flag | diff-x | diff-y | diff-z |             timestamp             |
// +------+--------+--------+--------+--------+--------+--------+--------+
// ========================================================================= //
static void prepareDataToSave(void)
// ========================================================================= //
{
#if (NRF_QUEUE_ENABLED)
	FlashPage_t *pPage;
	Acc3DataWithTime_t val;
	Acc3Data_t *pData, prev, diff;
	uint8_t idx, useDiff = NO, fullCnt = 0, shortCnt = 0;
	DataFlag_t* pFlag;
	ret_code_t err;

#if (SAVE_DATA_TO_FLASH)
	Cmd_t* pCmd;
	uint8_t cmdBytes[16];
#endif

	// Precaution
	uint16_t spaceLeft = nrf_queue_available_get(&rawDataQueue);
	spaceLeft = NUM_OF_CHUNKS - spaceLeft;
	if(spaceLeft < 82) return;	// since the maximum no. of data can be stored is 81 and we use peek, we need to have max no. + 1.

	// 0. Check that the flash is not busy first
	if(W25Q64JVIsBusy()) return;

	// 1. Get first data
	pPage = flashData + flashDataPage;
	memset(pPage->data, 0xFF, sizeof(FlashPage_t));
//	err = nrf_queue_read(&rawDataQueue, pPage->data + 1, 1);
	err = nrf_queue_read(&rawDataQueue, &val, 1);
	APP_ERROR_CHECK(err);
	pData = (Acc3Data_t*)(val.pData + 4);
	pPage->data[0] = 0;
	prev = *pData;
	*((Acc3Data_t*)(pPage->data + 1)) = prev;
	idx = 7;

	fullCnt += 1;

	// 2. Continue to get new data
	for(;;)
	{
		// a. get the data first
		err = nrf_queue_peek(&rawDataQueue, &val);
		APP_ERROR_CHECK(err);

		diff.x = val.x - prev.x;
		diff.y = val.y - prev.y;
		diff.z = val.z - prev.z;

		// b. check if we can use diff-data to compress data
		if ((ABS(diff.x) <= 127) ||
			(ABS(diff.y) <= 127) ||
			(ABS(diff.z) <= 127))
		{
			// Precaution: make sure we do have space for that
			if((idx + 4) >= 251) break;

			if(!useDiff)
			{
				useDiff = YES;

				pFlag = (DataFlag_t*)(pPage->data + idx);
				pFlag->cnt = 1;
				pFlag->isDiff = YES;

				idx += 1;
			}
			else
			{
				pFlag->cnt += 1;
			}

			*(pPage->data + idx) = (uint8_t)(diff.x & 0xFF); idx += 1;
			*(pPage->data + idx) = (uint8_t)(diff.y & 0xFF); idx += 1;
			*(pPage->data + idx) = (uint8_t)(diff.z & 0xFF); idx += 1;

			shortCnt += 1;
		}
		else
		{
			// Precaution: make sure we do have space for that
			if((idx + 7) >= 251) break;

			// copy the full data

			// a. flag
			*(pPage->data + idx) = 0; idx += 1;

			// b. data
//			*((Acc3Data_t*)(pPage->data + idx)) = val;
			pData = (Acc3Data_t*)(pPage->data + idx);
			pData->x = val.x;
			pData->y = val.y;
			pData->z = val.z;

			idx += 6;

			useDiff = NO;
			fullCnt += 1;

			NRF_LOG_INFO("Prev: %6d, %6d, %6d", prev.x, prev.y, prev.z);
			NRF_LOG_INFO("Curr: %6d, %6d, %6d", val.x, val.y, val.z);
			NRF_LOG_INFO("[%3d] Diff: %d, %d, %d",
			             idx, diff.x, diff.y, diff.z);
		}

		// c. pop the data out
		err = nrf_queue_read(&rawDataQueue, &val, 1);
		APP_ERROR_CHECK(err);

		prev.x = val.x;
		prev.y = val.y;
		prev.z = val.z;
	}

	NRF_LOG_INFO("last block time: %d", val.timestamp);

	MyTicks_t tmp;
	tmp.val = val.timestamp;
	NRF_LOG_INFO("Minute: %d, %d", tmp.minute, tmp.millis);

	// 3. write down the timestamp....
	*((uint32_t*)(pPage->data + 251)) = tmp.val;

#if (SAVE_DATA_TO_FLASH)

#if (USE_POWER_DOWN_ON_FLASH)
	// TODO: maybe we need to wake up the flash
	nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
	pCmd = W25Q64JVGetCmds(FLASH_CMD_WAKEUP);
	MySpiWriteOnly(pCmd->pData, pCmd->dataSize);
	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
#endif

	W25Q64JVWriteData(startAddr, pPage->data, 255);
	nrf_delay_ms(4);

	// TODO: maybe we need to power down the flash

	flashDataPage += 1;
	if(flashDataPage >= 4) flashDataPage = 0;

#if (USE_POWER_DOWN_ON_FLASH)
	pCmd = W25Q64JVGetCmds(FLASH_CMD_POWER_DOWN);
	nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
	MySpiWriteOnly(pCmd->pData, pCmd->dataSize);
	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
#endif

#if (VERIFY_SAVE_DATA)
	memset(cmdBytes, 0, 16);
	W25Q64JVReadData(startAddr, cmdBytes, 16);
	uint8_t match = memcmp(cmdBytes, pPage->data, 16);

	NRF_LOG_INFO("Verifying address - %6X");
	NRF_LOG_HEXDUMP_INFO(cmdBytes, 16);
	NRF_LOG_HEXDUMP_INFO(pPage->data, 16);
	if(match == 0)
	{
		NRF_LOG_INFO("It matches!");
	}
	else
	{
		NRF_LOG_INFO("Does not match ...");
	}
	// verify write correct
//	nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
//	pCmd = W25Q64JVGetCmds(FLASH_CMD_READ_DATA);
//	memcpy(cmdBytes, pCmd->pData, pCmd->dataSize);
//	cmdBytes[1] = (startAddr >> 16) & 0xFF;
//	cmdBytes[2] = (startAddr >> 8) & 0xFF;
//	cmdBytes[3] = startAddr & 0xFF;
////	cmdBytes[1] = 0;
////	cmdBytes[2] = 0;
////	cmdBytes[3] = 0;
//	NRF_LOG_INFO("Send: %02X, %02X, %02X, %02X - %02X",
//	             cmdBytes[0], cmdBytes[1], cmdBytes[2], cmdBytes[3], pCmd->dataSize);
//	MySpiWriteOnly(cmdBytes, pCmd->dataSize);
//	memset(cmdBytes, 0, sizeof(cmdBytes));
//	MySpiReadOnly(cmdBytes, 16);
//	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
//	NRF_LOG_HEXDUMP_INFO(cmdBytes, 16);
//	NRF_LOG_HEXDUMP_INFO(pPage->data, 16);
//	uint8_t match = memcmp(cmdBytes, pPage->data, 16);
//	if(match == 0)
//	{
//		NRF_LOG_INFO("It matches!");
//	}
//	else
//	{
//		NRF_LOG_INFO("Does not match ...");
//	}
#endif

#endif

	// advance flash page
#if (SAVE_DATA_TO_FLASH)
	uint32_t prevAddr = startAddr;
#endif
	startAddr += 0x100;	// add page

#if (SAVE_DATA_TO_FLASH)
	if (((prevAddr >> 12) ^ (startAddr >> 12)) != 0)
	{
		NRF_LOG_INFO("Prepare for next sector\nErase address: %08X", startAddr);

		nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
		pCmd = W25Q64JVGetCmds(FLASH_CMD_WRITE_ENABLE);
		MySpiWriteOnly(pCmd->pData, pCmd->dataSize);
		nrf_gpio_pin_set(PIN_SPI_SS_FLASH);

		// need to erase 4K area first
		nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
		pCmd = W25Q64JVGetCmds(FLASH_CMD_ERASE_4K);
		memcpy(cmdBytes, pCmd->pData, pCmd->dataSize);
		cmdBytes[pCmd->dataSize - 1] = startAddr & 0xFF;
		cmdBytes[pCmd->dataSize - 2] = (startAddr >> 8) & 0xFF;
		cmdBytes[pCmd->dataSize - 3] = (startAddr >> 16) & 0xFF;
		MySpiWriteOnly(cmdBytes, pCmd->dataSize);
		nrf_gpio_pin_set(PIN_SPI_SS_FLASH);

		// TODO: make sure we have at least 400ms delay before we access the flash again
	}
#endif

	if(flag.isConnected)
	{
		ble_collar_update_datasize(&m_collar, startAddr);
	}

//	uint16_t before = spaceLeft;
//	spaceLeft = nrf_queue_available_get(&rawDataQueue);
//	NRF_LOG_INFO("Before: %d, After: %d data left", before, NUM_OF_CHUNKS - spaceLeft);

	NRF_LOG_INFO("Full: %d, short: %d, idx: %d", fullCnt, shortCnt, idx);
	NRF_LOG_INFO("Next address: %08X", startAddr);
#endif
}

// ========================================================================= //
/**
 * @brief This function is used to calculate the integer square root.
 * @param val - The value of interest.
 * @return It returns the square root of the given value.
 */
// ========================================================================= //
static uint32_t mySqrt(uint32_t val)
// ========================================================================= //
{
	uint32_t root = 0, bit = (1 << 30);
	while (bit > val)
	{
		bit >>= 2;
	}

	while (bit != 0)
	{
		if(val >= (root + bit))
		{
			val -= (root + bit);
			root += (bit << 1);
		}
		root >>= 1;
		bit >>= 2;
	}
	return root;
}

// ========================================================================= //
/**
 * @brief This function is used to calculate the resultant for a given
 *        linear accelerometer x, y, z.
 * @param x
 * @param y
 * @param z
 * @return It results the calculated resultant vector.
 */
// ========================================================================= //
static uint32_t calResultant(int16_t x, int16_t y, int16_t z)
// ========================================================================= //
{
	uint32_t sum = 0;
	x = ABS(x);
	y = ABS(y);
	z = ABS(z);

	if(x >= 1024) x -= 1024;
	else x = 1024 - x;

	if(y >= 1024) y -= 1024;
	else y = 1024 - y;

	if(z >= 1024) z -= 1024;
	else z = 1024 - z;

	sum += x*x;
	sum += y*y;
	sum += z*z;

	return (uint16_t)mySqrt(sum);
}

// ========================================================================= //
/**
 * @brief This function is used to calculate the myticks variables plus a
 *        certain amount of time
 *
 * @param now
 * @param millis
 * @return
 */
// ========================================================================= //
static MyTicks_t myTicksPlus(MyTicks_t now, uint16_t millis)
// ========================================================================= //
{
	uint32_t val = now.millis;
	val += millis;
	if(val >= 60000)
	{
		val -= 60000;
		now.minute += 1;
		if(now.minute >= 60)
		{
			now.minute = 0;
			now.hours += 1;
			if(now.hours >= 24)
			{
				now.hours = 0;
				now.days += 1;
				if(now.days >= 30)
				{
					now.days = 0;
				}
			}
		}
	}
	now.millis = (uint16_t)val;

	return now;
}

// ========================================================================= //
/**
 * @brief This function is used to perform a minus action on my timing
 *        reference.
 *
 * @param now    - the current ticks.
 * @param millis - the amount of time to be subtracted from.
 *
 * @return It returns the subtracted result.
 */
// ========================================================================= //
static MyTicks_t myTicksMinus(MyTicks_t now, uint16_t millis)
// ========================================================================= //
{
	if(now.millis >= millis)
	{
		now.millis -= millis;
		return now;
	}

	now.millis = (60000) - millis;
	if(now.minute > 0)
	{
		now.minute -= 1;
		return now;
	}

	now.minute = 59;
	if(now.hours > 0)
	{
		now.hours -= 1;
		return now;
	}

	now.hours = 23;
	if(now.days > 0)
	{
		now.days -= 1;
		return now;
	}

	now.days = 29;
	return now;
}

// ========================================================================== //
/**
 * This function is used to update connection interval to my desired settings.
 * @param pCtx - Not used.
 */
// ========================================================================== //
static void updConnParam(void* pCtx)
// ========================================================================== //
{
	ret_code_t errCode;
	ble_gap_conn_params_t const config =
	{
		.min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS),
		.max_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS),
		.slave_latency = 25,
		.conn_sup_timeout = MSEC_TO_UNITS(6000, UNIT_10_MS)
	};

	errCode = sd_ble_gap_conn_param_update(m_conn_handle, &config);
	if(errCode != NRF_SUCCESS)
	{
		NRF_LOG_INFO("Error Code: %d", errCode);
	}
}

// ========================================================================= //
/**
 * @brief This function is used to retrieve data when enough data is gathered.
 *
 */
// ========================================================================= //
static void retrieveData(void)
// ========================================================================= //
{
	uint16_t start;
	Acc3DataWithTime_t accWt;
//	uint8_t tmp[7];
//	Acc3Data_t* pAcc = (Acc3Data_t*)(tmp + 1);
	FIFOStatus_t status;
//	MyTicks_t timestamp = lastTick;
	uint32_t sum, diff;

	// 1. Check how many data to retrieve
	LSM6DSOGetReg16(LSM6DSO32_REG_FIFO_STATUS1, status.pData);

//	timestamp = myTicksMinus(myTicks, (uint16_t)4000);	// since 50 data at 12.5Hz is 4 seconds

	// 2. take all data out from the FIFO first
	start = status.unreadSize - FIFO_THRESHOLD;
	for(uint16_t i = 0; i < status.unreadSize; i++)
	{
//		LSM6DSOGetFifo(tmp);
//		sum = calResultant(pAcc->x, pAcc->y, pAcc->z);

		LSM6DSOGetFifo(accWt.pData + 3);	// remark: this function reads 7 bytes
		sum = calResultant(accWt.x, accWt.y, accWt.z);

		curAvg = (prevAvg << 1);
		curAvg += (prevSum + sum);
		curAvg >>= 2;

		prevSum = sum;
		prevAvg = curAvg;

		// a. If this is the 1st time to retrieve data, filter out the noisy one
		if(flag.is1stData) continue;

		// TODO: if USB is plugged, don't save any data
		// ...

		if(curAvg >= sum) diff = curAvg - sum;
		else diff = sum - curAvg;

		if(diff >= ACTIVE_THRESHOLD)
		{
			NRF_LOG_INFO("[%2d] Sum: %d, Avg: %d, Diff: %d",
						 i, sum, curAvg, diff)
			flag.isCollarActive = YES;
			myCnt.passiveData = 0;
		}

		// skip unneeded data
		if(i < start) continue;

		accWt.timestamp = lastTick.val;

#if (NRF_QUEUE_ENABLED)

		// TODO: We need to skip storing data if USB is plugged.
		//       We also need to reset the queue, so that the old data
		//       would not be retained.
		// ..
		nrf_queue_push(&rawDataQueue, &accWt);
#endif

		// advance time
		lastTick = myTicksPlus(lastTick, 80);	// since 12.5Hz -> 80ms
	}

	// if this is the 1st time to retrieve data, filter out the noisy one
	if(flag.is1stData)
	{
		flag.is1stData = NO;
//		lastTick = timestamp;
		lastTick.val = 0;
		return;
	}

	// update the counters
	NRF_LOG_INFO("\nFIFO size: %d", status.unreadSize);
}

// ========================================================================= //
/**
 * @brief This function is used to calculate the last day of month for a
 *        given year and month.
 *
 * @param year  - the year index. 0 ~ 99
 * @param month - the month
 * @return It returns the last day of month.
 */
// ========================================================================= //
static uint8_t getLastDayOfMonth(uint8_t year, uint8_t month)
// ========================================================================= //
{
	switch(month)
	{
		case 1:
		case 3:
		case 5:
		case 7:
		case 8:
		case 10:
		case 12:
			return 31;

		case 4:
		case 6:
		case 9:
		case 11:
		default:	// not very logical, but I don't want the app to crash.
			return 30;

		case 2:
			break;
	}

	// assume that we are not going to have 2000 for year
	// we can skip the divisible by 400 check

	if((year & 0x03) != 0) return 28;

	// because 2100 is divisible by 100
	if(year != 100) return 28;

	return 29;
}

// ========================================================================= //
/**
 * @brief This is the ADC callback. It is called when an ADC conversion is
 *        finished.
 *
 * @param pEvt - The event of the ADC conversion.
 */
// ========================================================================= //
static void adcEvtCB(nrfx_saadc_evt_t const* pEvt)
// ========================================================================= //
{
	uint32_t tmp;

	if(pEvt->type == NRFX_SAADC_EVT_DONE)
	{
//		NRF_LOG_INFO("ADC: %d", samples[0]);

		adcSum += samples[0];	// 35 is the calculated offset, further investigation is needed
		adcCnt += 1;

		flag.isAdcDone = YES;

		if(adcCnt >= 8)
		{
			// TODO: calculate the battery level
			tmp = (adcSum >> 3) * 75;
			tmp >>= 6;

			tmp -= 27;

			battLevel = (uint8_t)((tmp * 100) / 4200);
			battLevel = MIN(battLevel, 100);

			NRF_LOG_INFO("Val: %d, Cal: %d, Batt: %d%%", adcSum >> 3, tmp, battLevel);

			nrf_gpio_pin_clear(PIN_BATT_DETECT_EN);
		}
	}
}

//static uint8_t tmpCnt = 0;
// ========================================================================= //
static void sensorCB(void* param)
// ========================================================================= //
{
	uint8_t ok, val;
//	Acc3Data_t acc;

	myCnt.for1Sec += 1;	// assume that this callback function is called every 100ms
	if(myCnt.for1Sec >= 10)
	{
		myCnt.for1Sec = 0;
		flag.is1Sec = YES;
	}

//	if(spiCtrl.isActive) return;

	switch(flag.sensorState)
	{
		case SENSOR_STATE_IDLE:
		{
			ok = LSM6DSOGetReg8(LSM6DSO32_REG_WHO_AM_I, &val);
			if(ok)
			{
				NRF_LOG_INFO("WHO-AM-I: %02X", val);
				if(val == 0x6C)
				{
					flag.sensorState = SENSOR_STATE_SETUP;
				}
			}
		}
			break;

		case SENSOR_STATE_SETUP:
		{
			ok = LSM6DSOInit(FIFO_THRESHOLD);
			if(ok)
			{
				flag.sensorState = SENSOR_STATE_READY;

				nrf_drv_gpiote_in_event_enable(PIN_SPI_INT1, true);
			}
		}
			break;

		case SENSOR_STATE_READY:
		{
			// Since the initialization is done, we will use this as a timer counter.
			if(myCnt.toPassive < 0xF0)
			{
				myCnt.toPassive += 1;
			}

			myCnt.getData += 1;
			if(!flag.is4Sec)
			{
				if(flag.is1stData && (myCnt.getData >= 10))
				{
					// this is to eliminate the 1st batch of data.
					// which could be subject to noise
					flag.is4Sec = YES;
					flag.isDataCountReached = YES;
					myCnt.getData = 0;
				}
				else if(myCnt.getData >= 40)
				{
					myCnt.getData = 0;
					flag.is4Sec = YES;
				}
			}

			// update the RTC
			myCnt.keepTime += 1;
			if(myCnt.keepTime >= 10)
			{
				epochTime += 1;
			}
		}
			break;

		default: break;
	}

	// advance timing related counters
//	myTicks.millis += SENSOR_TIMER_INTERVAL;
//	if(myTicks.millis >= 60000)
//	{
//		myTicks.millis -= 60000;
//		myTicks.minute += 1;
//		if(myTicks.minute >= 60)
//		{
//			myTicks.minute = 0;
//			myTicks.hours += 1;
//			if(myTicks.hours >= 24)
//			{
//				myTicks.hours = 0;
//				myTicks.days += 1;
//				if(myTicks.days >= 30)
//				{
//					myTicks.days = 0;
//				}
//			}
//		}
//	}

	myDate.millis += SENSOR_TIMER_INTERVAL;
	if(myDate.millis >= 1000)
	{
		myDate.millis -= 1000;
		myDate.second += 1;
		if(myDate.second >= 60)
		{
			myDate.second -= 60;
			myDate.minute += 1;
			if(myDate.minute >= 60)
			{
				myDate.minute -= 60;
				myDate.hour += 1;
				if(myDate.hour >= 24)
				{
					myDate.hour = 0;
					myDate.day += 1;
					if(myDate.day >= lastDayOfMonth)
					{
						myDate.day = 1;
						myDate.month += 1;
						if(myDate.month >= 12)
						{
							myDate.month = 1;
							myDate.year += 1;

							lastDayOfMonth = getLastDayOfMonth(myDate.year, myDate.month);
						}
					}
				}
			}
		}
	}
}

// ========================================================================= //
/**
 * @brief This function is the callback function when a collar data event
 *        changes.
 *
 * @param evt     - The corresponding event related to the collar service.
 * @param pCollar - The collar instance.
 */
// ========================================================================= //
static void collarCB(COLLAR_UPDATE_EVT_t evt, ble_collar_t *pCollar)
// ========================================================================= //
{
	uint32_t errCode;

	switch(evt)
	{
		case COLLAR_UPDATE_NOTIFICATION_ENABLED:
		{
			NRF_LOG_INFO("Notification enabled!");

			// TODO: need to update connection interval
			app_timer_start(mTimerConnUpd, APP_TIMER_TICKS(100), NULL);
		}
			break;

		case COLLAR_UPDATE_NOTIFICATION_DISABLED:
		{
			NRF_LOG_INFO("Notification disabled!");
		}
			break;

		case COLLAR_UPDATE_TIMESTAMP:
		{
			// TODO: we need to update our own RTC
			// ...
			NRF_LOG_INFO("Timestamp: %08X", pCollar->timestamp.value);
		}
			break;

		case COLLAR_UPDATE_GET_SIZE:
		{
			// TODO: the no. of items to get
			// ...
		}
			break;

		case COLLAR_UPDATE_GET_FROM_TIMESTAMP:
		{
			// TODO: compute how many data to retrieve from.
			// ...
		}
			break;


		default: break;
	}
}

// ========================================================================= //
/**
 * @brief This is the battery service event callback.
 *
 * @param pBas - the battery service instance.
 * @param pEvt - the corresponding event.
 */
// ========================================================================= //
static void battEvtCB(ble_bas_t *pBas, ble_bas_evt_t* pEvt)
// ========================================================================= //
{
	NRF_LOG_INFO("Battery event handler is called!");
}

// ========================================================================= //
/**
 * @brief This function is used to trigger an ADC conversion start.
 *
 */
// ========================================================================= //
static void startADC(void)
// ========================================================================= //
{
	ret_code_t err;

	nrf_gpio_pin_set(PIN_BATT_DETECT_EN);

	nrf_delay_ms(1);

	flag.isAdcDone = NO;

	err = nrfx_saadc_simple_mode_set(1 << channels[0].channel_index,
	                                 NRF_SAADC_RESOLUTION_12BIT,
	                                 NRF_SAADC_OVERSAMPLE_DISABLED,
	                                 adcEvtCB);
	APP_ERROR_CHECK(err);

	samples[0] = 0;
	err = nrfx_saadc_buffer_set(samples, 1);
	APP_ERROR_CHECK(err);

	err = nrfx_saadc_mode_trigger();
	APP_ERROR_CHECK(err);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
	ret_code_t err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_create(&m_sensor_id, APP_TIMER_MODE_REPEATED, sensorCB);
	APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&mTimerConnUpd, APP_TIMER_MODE_SINGLE_SHOT, updConnParam);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
	uint32_t err_code;
	ble_gap_conn_params_t gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

	ble_gap_addr_t ble_addr;
	err_code = sd_ble_gap_addr_get(&ble_addr);
	APP_ERROR_CHECK(err_code);

	uint8_t idx = 13, tmp;
	uint8_t *pAddr = ble_addr.addr;
	for(int8_t i = BLE_GAP_ADDR_LEN - 1; i >= BLE_GAP_ADDR_LEN - 4; i--)
	{
		tmp = (*pAddr >> 4) & 0x0F;
		collarName[idx] = ((tmp < 10) ? (0x30 + tmp):(87 + tmp));
		idx += 1;

		tmp = *pAddr & 0x0F;
		collarName[idx] = ((tmp < 10) ? (0x30 + tmp):(87 + tmp));
		idx += 1;

		pAddr += 1;
	}

//	err_code = sd_ble_gap_device_name_set(&sec_mode,
//	                                      (const uint8_t*) DEVICE_NAME,
//	                                      strlen(DEVICE_NAME));
	err_code = sd_ble_gap_device_name_set(&sec_mode,
	                                      (const uint8_t*) collarName,
	                                      LENGTH_OF_COLLAR_NAME);
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
	uint32_t err_code;
	nrf_ble_qwr_init_t qwr_init =
	{ 0 };

	// Initialize Queued Write Module.
	qwr_init.error_handler = nrf_qwr_error_handler;

	err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);

	ble_collar_init_t collarInit;
	memset(&collarInit, 0, sizeof(collarInit));

	collarInit.collar_write_handler = collarCB;
	ble_collar_init(&m_collar, &collarInit);

	ble_dis_init_t disInit =
	{
		.manufact_name_str = disInfo[DIS_INFO_IDX_MANUF],
		.model_num_str = disInfo[DIS_INFO_IDX_MODEL],
		.serial_num_str = disInfo[DIS_INFO_IDX_SERIAL],
		.hw_rev_str = disInfo[DIS_INFO_IDX_HW],
		.fw_rev_str = disInfo[DIS_INFO_IDX_FW],
		.sw_rev_str = disInfo[DIS_INFO_IDX_SW],
		.dis_char_rd_sec = SEC_OPEN
	};

	// Initialize the device information profile
	err_code = ble_dis_init(&disInit);
	APP_ERROR_CHECK(err_code);

	ble_bas_init_t bas_init =
	{
		.evt_handler = battEvtCB,
		.support_notification = false,
		.initial_batt_level = 0,
		.bl_rd_sec = SEC_OPEN
	};
	// Initialize the battery profile
	bas_init.evt_handler = battEvtCB;
	err_code = ble_bas_init(&m_batt, &bas_init);
	APP_ERROR_CHECK(err_code);

#if (BLE_DFU_ENABLED)
    // Initialize the DFU service
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_buttonless_evt_handler
    };

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
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


///**@brief Function for putting the chip into sleep mode.
// *
// * @note This function will not return.
// */
//static void sleep_mode_enter(void)
//{
//	uint32_t err_code;
//
////    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
////    APP_ERROR_CHECK(err_code);
//
//    // Prepare wakeup buttons.
////    err_code = bsp_btn_ble_sleep_mode_prepare();
////    APP_ERROR_CHECK(err_code);
//
//    // Go to system-off mode (this function will not return; wakeup will cause a reset).
//    err_code = sd_power_system_off();
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
//    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
        	NRF_LOG_INFO("Advertising fast started!");
        	flag.isAdvertising = YES;
            break;
        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
//        	ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        	NRF_LOG_INFO("Advertising stopped.");

        	flag.isAdvertising = NO;
        	sd_ble_gap_adv_stop(m_advertising.adv_handle);
//        	sd_power_system_off();
            break;
        default:
        	NRF_LOG_INFO("Advertising un-tackled State: %d", ble_adv_evt);
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
	uint32_t err_code;

	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
		{
			NRF_LOG_INFO("Connected");
			err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
			APP_ERROR_CHECK(err_code);
			m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
			APP_ERROR_CHECK(err_code);

			// TODO: we need to update the values in the service
			// ...
			flag.isConnected = YES;
			flag.isAdvertising = NO;
			getDataSize = 0;

			ble_bas_battery_level_update(&m_batt, battLevel, m_conn_handle);

			ble_collar_update_datasize(&m_collar, startAddr);
		}
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			NRF_LOG_INFO("Disconnected");
			// LED indication will be changed when advertising starts.
			m_conn_handle = BLE_CONN_HANDLE_INVALID;

			flag.isConnected = NO;

			// If we have already activated, nevermind, we can keep advertising
			if (flag.isActivated || flag.is5V)
			{
				advertising_start();
			}
			break;

		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
		{
//			ble_gap_evt_conn_param_update_t* pParam = (ble_gap_evt_conn_param_update_t*)p_context;
//
//			NRF_LOG_INFO("Update connection interval\nInterval: %d\nLatency: %d\nTimeout: %d\n",
//			               pParam->conn_params.max_conn_interval,
//			               pParam->conn_params.slave_latency,
//			               pParam->conn_params.conn_sup_timeout);

			NRF_LOG_INFO("Update interval: %d", p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval);

		}
			break;

		case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
		{
			NRF_LOG_DEBUG("PHY update request.");
			ble_gap_phys_t const phys =
			{ .rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO, };
			err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle,
			                                 &phys);
			APP_ERROR_CHECK(err_code);
		}
			break;

		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
			// Pairing not supported
			err_code = sd_ble_gap_sec_params_reply(
			                m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
			                NULL, NULL);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTS_EVT_SYS_ATTR_MISSING:
			// No system attributes have been stored.
			err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTC_EVT_TIMEOUT:
			// Disconnect on GATT Client timeout event.
			err_code = sd_ble_gap_disconnect(
			                p_ble_evt->evt.gattc_evt.conn_handle,
			                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;

		case BLE_GATTS_EVT_TIMEOUT:
			// Disconnect on GATT Server timeout event.
			err_code = sd_ble_gap_disconnect(
			                p_ble_evt->evt.gatts_evt.conn_handle,
			                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
			APP_ERROR_CHECK(err_code);
			break;

		default:
			// No implementation needed.
			break;
	}
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
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


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
//        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
//        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

//
///**@brief Function for handling events from the BSP module.
// *
// * @param[in]   event   Event generated by button press.
// */
//void bsp_event_handler(bsp_event_t event)
//{
//    uint32_t err_code;
//    switch (event)
//    {
//        case BSP_EVENT_SLEEP:
//            sleep_mode_enter();
//            break;
//
//        case BSP_EVENT_DISCONNECT:
//            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
//            if (err_code != NRF_ERROR_INVALID_STATE)
//            {
//                APP_ERROR_CHECK(err_code);
//            }
//            break;
//
//        case BSP_EVENT_WHITELIST_OFF:
//            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//            {
//                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//                if (err_code != NRF_ERROR_INVALID_STATE)
//                {
//                    APP_ERROR_CHECK(err_code);
//                }
//            }
//            break;
//
//        default:
//            break;
//    }
//}
//
//
///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
///**@snippet [Handling the data received over UART] */
//void uart_event_handle(app_uart_evt_t * p_event)
//{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;
//    uint32_t       err_code;
//
//    switch (p_event->evt_type)
//    {
//        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;
//
//            if ((data_array[index - 1] == '\n') ||
//                (data_array[index - 1] == '\r') ||
//                (index >= m_ble_nus_max_data_len))
//            {
//                if (index > 1)
//                {
//                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);
//
//                    do
//                    {
//                        uint16_t length = (uint16_t)index;
//                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
//                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
//                            (err_code != NRF_ERROR_RESOURCES) &&
//                            (err_code != NRF_ERROR_NOT_FOUND))
//                        {
//                            APP_ERROR_CHECK(err_code);
//                        }
//                    } while (err_code == NRF_ERROR_RESOURCES);
//                }
//
//                index = 0;
//            }
//            break;
//
//        case APP_UART_COMMUNICATION_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;
//
//        case APP_UART_FIFO_ERROR:
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;
//
//        default:
//            break;
//    }
//}
///**@snippet [Handling the data received over UART] */
//
//
///**@brief  Function for initializing the UART module.
// */
///**@snippet [UART Initialization] */
//static void uart_init(void)
//{
//    uint32_t                     err_code;
//    app_uart_comm_params_t const comm_params =
//    {
//        .rx_pin_no    = RX_PIN_NUMBER,
//        .tx_pin_no    = TX_PIN_NUMBER,
//        .rts_pin_no   = RTS_PIN_NUMBER,
//        .cts_pin_no   = CTS_PIN_NUMBER,
//        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
//        .use_parity   = false,
//#if defined (UART_PRESENT)
//        .baud_rate    = NRF_UART_BAUDRATE_115200
//#else
//        .baud_rate    = NRF_UARTE_BAUDRATE_115200
//#endif
//    };
//
//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUF_SIZE,
//                       UART_TX_BUF_SIZE,
//                       uart_event_handle,
//                       APP_IRQ_PRIORITY_LOWEST,
//                       err_code);
//    APP_ERROR_CHECK(err_code);
//}
///**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
	uint32_t err_code;
	ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = false;
//    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
	init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

//    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;
	init.srdata.uuids_complete.uuid_cnt = 0;

	init.config.ble_adv_fast_enabled = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout = 0;
	init.config.ble_adv_slow_enabled = true;
	init.config.ble_adv_slow_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_slow_timeout = 100;
    init.config.ble_adv_on_disconnect_disabled = YES;
    init.evt_handler = on_adv_evt;
//	init.evt_handler = NULL;

	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

//
///**@brief Function for initializing buttons and leds.
// *
// * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
// */
//static void buttons_leds_init(bool * p_erase_bonds)
//{
//    bsp_event_t startup_event;
//
//    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);
//
//    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
//}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


static void dcdcTo3V(void)
{
	if (NRF_POWER->MAINREGSTATUS & (POWER_MAINREGSTATUS_MAINREGSTATUS_High << POWER_MAINREGSTATUS_MAINREGSTATUS_Pos))
	{

		// Configure UICR_REGOUT0 register only if it is set to default value.
		if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) != (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos))
		{
			NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

			NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
								(UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

			NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

			// System reset is needed to update UICR registers.
			NVIC_SystemReset();
		}
	}
}

// ========================================================================= //
/**@brief Function for starting advertising.
 */
// ========================================================================= //
static void advertising_start(void)
// ========================================================================= //
{
	// Precaution
	if(flag.isAdvertising) return;

    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
//    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_SLOW);
    APP_ERROR_CHECK(err_code);
}

// ========================================================================= //
/**
 * @brief This function is used to stop issue an advertising stop event to
 *        the backend.
 */
// ========================================================================= //
static void advertising_stop(void)
// ========================================================================= //
{
	if(!flag.isAdvertising) return;

	uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_IDLE);
	APP_ERROR_CHECK(err_code);
}

// ========================================================================= //
/**
 * @brief This function is the interrupt pin detect for the gyroscope
 *        gpiote callback function. It should be set to be triggered upon
 *        the INT1 pin changes from '0'->'1'.
 * @param pin    - the pin that triggers the callback event.
 * @param action - the corresponding action ( e.g. lo-to-hi event )
 */
// ========================================================================= //
static void sensorIntCB(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
// ========================================================================= //
{
	flag.isDataCountReached = YES;
}

// ========================================================================= //
/**
 * @brief This is the callback function when an USB plug-in or plug-out
 *        action is made.
 * @param event - The corresponding USB event occurs.
 */
// ========================================================================= //
static void usbEvtCB(nrf_drv_power_usb_evt_t event)
// ========================================================================= //
{
	switch (event)
	{
		case NRF_DRV_POWER_USB_EVT_DETECTED:
		{
			NRF_LOG_INFO("USB power detected");
			flag.is5V = YES;

			uint8_t isCharging = nrf_gpio_pin_read(PIN_CHARGE);

			if(isCharging)
			{
				flag.isCharging = YES;
				flag.isFull = NO;
			}

			if(!flag.isAdvertising)
			{
				advertising_start();
			}
		}
			break;

		case NRF_DRV_POWER_USB_EVT_REMOVED:
		{
			NRF_LOG_INFO("USB power removed");
			flag.is5V = NO;
			flag.isCharging = NO;
			flag.isFull = NO;

			if(!flag.isActivated)
			{
				advertising_stop();
			}
		}
			break;

		default: break;
	}
}

// ========================================================================= //
/**
 * @brief This function is used at the beginning when the program starts.
 *        It checks whether the header of the flash is intact. Otherwise, it
 *        initialize it and prepare it for use in the future.
 */
// ========================================================================= //
static void checkFlashRegion(void)
// ========================================================================= //
{
//	uint8_t len, val, i, ok;
	uint8_t isDone = NO, ok, val;
	uint8_t tmp[16];
	uint32_t address = startAddr;
	uint32_t mask = 0x100000;

	FLASH_CHECK_STAGE_t stage = FLASH_CHECK_STAGE_IDLE;

	NRF_LOG_INFO("Check flash...");
	while (!isDone)
	{
		switch(stage)
		{
			case FLASH_CHECK_STAGE_IDLE:
				stage = FLASH_CHECK_STAGE_HEADER;

				// Wake up the flash - just in case
				nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
				Cmd_t* pCmd = W25Q64JVGetCmds(FLASH_CMD_WAKEUP);
				MySpiWriteOnly(pCmd->pData, pCmd->dataSize);
				nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
				break;

			case FLASH_CHECK_STAGE_HEADER:
			{
//				ok = readFlashData(0x000000, tmp, sizeof(FlashHeader_t));
				ok = W25Q64JVReadData(0x000000, tmp, sizeof(FlashHeader_t));
				if(!ok)
				{
					NRF_LOG_INFO("Fail to read flash...");
					break;
				}

				FlashHeader_t* header = (FlashHeader_t*)tmp;

				if(header->version != 1)
				{
					// either this is the first time using the flash or it is corrupted.
					// we need to initialize it
					NRF_LOG_INFO("No header found!!\nErase and build!");

					// a. make sure the region is erased first
					ok = W25Q64JVEraseData(0x000000, FLASH_ERASE_SIZE_4K);

					// b. Write header
					header = (FlashHeader_t*)tmp;
					header->version = 1;
					header->activation = 0xFF;
					header->timestamp = 0xFFFFFFFF;

					ok = W25Q64JVWriteData(0x000000, tmp, sizeof(FlashHeader_t));

					stage = FLASH_CHECK_STAGE_IDLE;

				}
				else
				{
					NRF_LOG_INFO("Header Version: %d\n", header->version);
					NRF_LOG_INFO("Is activated? %s\n", (header->activation == 0xFF ? ("NO"):("YES")));

					switch(header->activation)
					{
						case 0x01:
							flag.isActivated = YES;
							break;

						case 0xFF:
#if (TEST_ONLY)
							flag.isActivated = YES;
#endif
							break;

						default: break;
					}

					address = 0x001000;
					stage = FLASH_CHECK_STAGE_LAST_DATA_LOC;				}
			}
				break;

			case FLASH_CHECK_STAGE_LAST_DATA_LOC:
			{
				// TODO: we miss the check that address could be larger than the 8Mbit area
				ok = W25Q64JVReadData(address, tmp, 16);
				NRF_LOG_INFO("Read ok? %d", ok);
				NRF_LOG_INFO("Read address: %06X", address);
				NRF_LOG_HEXDUMP_INFO(tmp, 16);
				NRF_LOG_FLUSH();
				val = tmp[0] & tmp[1] & tmp[2] & tmp[3];
				if(val != 0xFF)
				{
					address += mask;
					break;
				}

				// Precaution
				if((address == 0x001000) || (mask == 0x100))
				{
					stage = FLASH_CHECK_STAGE_DONE;
					startAddr = address;
					isDone = YES;
					NRF_LOG_INFO("Start from %06X", startAddr);
					break;
				}

				address -= mask;
				mask >>= 4;
				if(address == 0) address = 0x001000;
			}
				break;

			default: break;
		}

		nrf_delay_ms(10);
	}

	// TODO: maybe we need to power down the flash too
	// ...
}

// ========================================================================= //
/**
 * @brief This function is used to add in the USB 5V action check.
 */
// ========================================================================= //
static void addUSBCheck(void)
// ========================================================================= //
{
	static const nrf_drv_power_usbevt_config_t config =
	{
		.handler = usbEvtCB
	};
	ret_code_t err = nrf_drv_power_usbevt_init(&config);
	APP_ERROR_CHECK(err);
}

// ========================================================================= //
/**
 * @brief This is my initialization method used at the beginning of the code.
 *        It should help to initialize a lot of things at the start, including
 *        (1) Setting the pins for LEDs correctly.
 *        (2) Setting the charging IC's pins as input pins.
 *        (3) Initialize the variables.
 *        (4) Run flash check function.
 *        (5) Setup gpio interrupt callback events.
 */
// ========================================================================= //
static void myInit(void)
// ========================================================================= //
{
	MySpiInit();

	nrf_gpio_cfg_output(PIN_LED_RED);
	nrf_gpio_cfg_output(PIN_LED_GREEN);

	nrf_gpio_pin_clear(PIN_LED_GREEN);
	nrf_gpio_pin_clear(PIN_LED_RED);

	nrf_gpio_cfg_output(PIN_BATT_DETECT_EN);
	nrf_gpio_pin_clear(PIN_BATT_DETECT_EN);

	memset(&flag, 0, sizeof(SysFlag_t));
	flag.sensorState = SENSOR_STATE_IDLE;
	flag.isCollarActive = YES;// always assume the collar is active first - actively moving
	flag.is1stData = YES;

	memset(&myDate, 0, sizeof(MyDate_t));
	memset(&myTicks, 0, sizeof(MyTicks_t));

//	myDate.month = 1;
//	myDate.day = 1;
//	lastDayOfMonth = 31;

//	rawDataBlock.start = 0;
//	rawDataBlock.end = 0;
	myCnt.getData = 0;
	myCnt.passiveData = 0;

	// setup the charge detection pin
	// remark: cannot use NO_PULL, otherwise there will be current leakage
	//         not sure why it would be like this.
	nrf_gpio_cfg_input(PIN_CHARGE, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(PIN_STANDBY, NRF_GPIO_PIN_PULLUP);

	// setup pin interrupt
	ret_code_t err = nrf_drv_gpiote_init();
	APP_ERROR_CHECK(err);

	nrf_drv_gpiote_in_config_t cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	cfg.pull = NRF_GPIO_PIN_NOPULL;

	err = nrf_drv_gpiote_in_init(PIN_SPI_INT1, &cfg, sensorIntCB);
	APP_ERROR_CHECK(err);

	// Check out the flash region
	// a. find out to which page of flash should read to
	// b. find out if the flash header is correct
	// c. ??
    checkFlashRegion();

    int8_t val;
    LSM6DSOGetReg8(LSM6DSO32_REG_INTERNAL_FREQ_FINE, &val);
    NRF_LOG_INFO("Freq: %d", val);

#if (USE_POWER_DOWN_ON_FLASH)
    // Power down the flash
    nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
	Cmd_t* pCmd = W25Q64JVGetCmds(FLASH_CMD_POWER_DOWN);
	MySpiWriteOnly(pCmd->pData, pCmd->dataSize);
	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
#endif

	// setup ADC
	err = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
	APP_ERROR_CHECK(err);

	channels[0].channel_config.gain = NRF_SAADC_GAIN1_2;
	channels[0].channel_config.acq_time = NRF_SAADC_ACQTIME_40US;
	err = nrfx_saadc_channels_config(channels, 1);
	APP_ERROR_CHECK(err);

	nrf_gpio_pin_set(PIN_LED_RED);
	nrf_delay_ms(500);
	nrf_gpio_pin_set(PIN_LED_GREEN);
	nrf_delay_ms(500);
	nrf_gpio_pin_clear(PIN_LED_RED);
	nrf_delay_ms(500);
	nrf_gpio_pin_clear(PIN_LED_GREEN);
	nrf_delay_ms(500);
}

//static void eraseFlash4K(uint32_t address)
//{
//	Cmd_t* pCmd;
//	uint8_t len, tmp[16];
//
//	address &= 0x00FFFF00;
//
//	nrf_gpio_pin_clear(PIN_SPI_SS_FLASH);
//	pCmd = W25Q64JVGetCmds(FLASH_CMD_READ_DATA);
//	len = pCmd->dataSize;
//	memcpy(tmp, pCmd->pData, len);
//	tmp[1] = (address >> 16) & 0xFF;
//	tmp[2] = (address >> 8) & 0xFF;
//	tmp[3] = address & 0xFF;
//	MySpiWriteOnly(tmp, len);
//	nrf_delay_ms(1);
//	nrf_gpio_pin_set(PIN_SPI_SS_FLASH);
//	NRF_LOG_INFO("Erase 4k address: %06X", address);
//}

/**@brief Application main function.
 */
int main(void)
{
//    bool erase_bonds;

	// Initialize.
	//uart_init();
	log_init();

	dcdcTo3V();

	timers_init();
	//buttons_leds_init(&erase_bonds);
	power_management_init();
	// enable use of dc-dc
	sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
//	sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();

#if (USE_TX_8DBM)
	uint32_t errCode = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
	                                           m_advertising.adv_handle,
	                                           RADIO_TXPOWER_TXPOWER_Pos8dBm);
	APP_ERROR_CHECK(errCode);
#else
	uint32_t errCode = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,
		                                           m_advertising.adv_handle,
		                                           RADIO_TXPOWER_TXPOWER_0dBm);
		APP_ERROR_CHECK(errCode);
#endif

	myInit();

	// Start execution.
//    printf("\r\nUART started.\r\n");
	NRF_LOG_INFO("Collar started.\n");
	NRF_LOG_INFO("Get data threshold: %d\n", GET_DATA_THRESHOLD);
	if (flag.isActivated)
	{
		advertising_start();

		// We try to gather data in a periodic manner
		app_timer_start(m_sensor_id, SENSOR_APP_TIMER_INTERVAL, NULL);
	}
	else
	{
		LSM6DSO32X_Common_t comm;
		comm.pData[0] = 0;
		comm.ctrl3.swReset = 1;
		LSM6DSOSetReg8(LSM6DSO32_REG_CTRL3_C, comm.pData[0]);

		LSM6DSOGetReg8(LSM6DSO32_REG_CTRL1_XL, comm.pData);

		NRF_LOG_INFO("Ctrl1: %02X", comm.pData[0]);
	}

	addUSBCheck();



//	eraseFlash();

//	uint32_t myAddr = 0x001000;
	// Enter main loop.
	adcCnt = 8;

	for (;;)
	{
		idle_state_handle();

		// TODO: in actual code, we need to also check that the device is activated.
		// ...

		//if ((flag.timeForSensor) && (!spiCtrl.isActive))
		if(flag.isActivated)
		{
			if(flag.is4Sec && flag.isDataCountReached)
			{
				retrieveData();
				flag.is4Sec = NO;
				flag.isDataCountReached = NO;
			}

			prepareDataToSave();
		}

		if(flag.is1Sec)
		{
			flag.is1Sec = NO;
			NRF_LOG_FLUSH();
//			if(myAddr == 0x001000)
//			{
//				eraseFlash4K(myAddr);
//			}
//			else
//			{
//				readFlashData(myAddr);
//			}
//			myAddr += 16;

			if(battCnt >= 60)
			{
				adcCnt = 0;
				adcSum = 0;
				battCnt = 0;
				startADC();
			}
			else
			{
				battCnt += 1;
			}
		}

		if((adcCnt < 8) && flag.isAdcDone)
		{
			startADC();
		}
	}
}


/**
 * @}
 */
