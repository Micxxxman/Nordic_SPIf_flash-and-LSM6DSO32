/**
 * Copyright (c) 2015 - 2020, Nordic Semiconductor ASA
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
 * @defgroup ble_lbs LED Button Service Server
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief LED Button Service Server module.
 *
 * @details This module implements a custom LED Button Service with an LED and Button Characteristics.
 *          During initialization, the module adds the LED Button Service and Characteristics
 *          to the BLE stack database.
 *
 *          The application must supply an event handler for receiving LED Button Service
 *          events. Using this handler, the service notifies the application when the
 *          LED value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of the Button Characteristic to connected peers.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_hids_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_HIDS_BLE_OBSERVER_PRIO,
 *                                   ble_hids_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef BLE_COLLAR_H__
#define BLE_COLLAR_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_lbs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */

#ifndef BLE_COLLAR_BLE_OBSERVER_PRIO
#define BLE_COLLAR_BLE_OBSERVER_PRIO			2
#endif

#define BLE_COLLAR_DEF(_name)                                                                          \
static ble_collar_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_COLLAR_BLE_OBSERVER_PRIO,                                                     \
                     ble_collar_on_ble_evt, &_name)

#define COLLAR_UUID_BASE				{0x17, 0x8B, 0xC3, 0x42, 0x7F, 0x07, 0x41, 0x73, 0x86, 0xDC, 0xC2, 0x96, 0x00, 0x00, 0xCC, 0xDE}
#define COLLAR_UUID_SERVICE				0xFA00
#define COLLAR_UUID_TIMESTAMP			0xFA01	//!< this is for retreiving the timestamp from base.
#define COLLAR_UUID_DATASIZE			0xFA02	//!< this is for no. of data to retreive 
#define COLLAR_UUID_LAST_MARKER			0xFA03
#define COLLAR_UUID_DATA_GET			0xFA04	//!< write the no. of data to retrieve
#define COLLAR_UUID_DATA_GET_WITH_TIME	0xFA05	//!< send over the timestamp and let to collar to retrieve data later than the provide datetime.
#define COLLAR_UUID_DATA				0xFA06	//!< this is the notification character.

typedef enum
{
	COLLAR_UPDATE_NONE,
	COLLAR_UPDATE_TIMESTAMP,
	COLLAR_UPDATE_GET_SIZE,
	COLLAR_UPDATE_GET_FROM_TIMESTAMP,
	COLLAR_UPDATE_NOTIFICATION_ENABLED,
	COLLAR_UPDATE_NOTIFICATION_DISABLED
} COLLAR_UPDATE_EVT_t;

// Forward declaration of the ble_lbs_t type.
typedef struct ble_collar_s ble_collar_t;

typedef struct ble_collar_timestamp
{
	uint8_t year;	// 0 ~ 99
	uint8_t month;	// 1 ~ 12
	uint8_t day;	// 1 ~ 31
	uint8_t hour;	// 0 ~ 23
	uint8_t minute;	// 0 ~ 59
	uint8_t sec;	// 0 ~ 59
} BLECollarTimestamp_t;

typedef union __attribute__((packed))
{
	uint32_t value;
	uint8_t data[4];
} FourBytes_t;

typedef union __attribute__((packed))
{
	struct __attribute__((packed))
	{
		uint32_t value;
		uint16_t value2;
	};
	uint8_t data[6];
} SixBytes_t;

typedef void (*ble_collar_write_handler_t)(COLLAR_UPDATE_EVT_t event,
				ble_collar_t *p_collar);

typedef void (*ble_collar_timestamp_handler_t)(uint16_t conn_handle,
				ble_collar_t *pCollar);

/** @brief LED Button Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
	ble_collar_write_handler_t collar_write_handler; /**< Event handler to be called when characteristic is written. */
	ble_collar_timestamp_handler_t timestamp_updater;//!< This is used to update the timestamp.
} ble_collar_init_t;

/**@brief LED Button Service structure. This structure contains various status information for the service. */
struct ble_collar_s
{
	uint16_t service_handle; /**< Handle of LED Button Service (as provided by the BLE stack). */
	ble_gatts_char_handles_t timestamp_handles;
	ble_gatts_char_handles_t datasize_handles;
	ble_gatts_char_handles_t last_marker_handles;
	ble_gatts_char_handles_t req_data_from_size_handles;
	ble_gatts_char_handles_t req_data_from_time_handles;
	ble_gatts_char_handles_t send_data_handles;
	uint8_t uuid_type; /**< UUID type for the LED Button Service. */
	
	ble_collar_write_handler_t collar_write_handler; /**< Event handler to be called new data is written. */
	
	FourBytes_t datasize;
	FourBytes_t req_datasize;
	FourBytes_t lastMarker;
	SixBytes_t timestamp;
	SixBytes_t from_timestamp;
	uint8_t buffer[140];
	uint16_t outLen;
	uint8_t isNotified;
};

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_lbs      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_lbs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_collar_init(ble_collar_t *p_collar,
				const ble_collar_init_t *p_collar_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Collar Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  LED Button Service structure.
 */
void ble_collar_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for sending a button state notification.
 *
 ' @param[in] conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_lbs         LED Button Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
//uint32_t ble_lbs_on_button_change(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t button_state);
uint32_t ble_collar_send_out(uint16_t conn_handle, ble_collar_t *p_collar,
				uint8_t *pData, uint8_t datasize);

uint32_t ble_collar_update_datasize(ble_collar_t* pCollar, uint32_t datasize);

#ifdef __cplusplus
}
#endif

#endif // BLE_LBS_H__

/** @} */
