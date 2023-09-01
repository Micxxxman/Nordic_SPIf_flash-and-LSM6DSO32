/**
 * Copyright (c) 2013 - 2020, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(BLE_COLLAR)
#include "ble_collar_data.h"
#include "ble_srv_common.h"

#define MAX_NOTIFY_PACKET_SIZE				20

#include "nrf_log.h"
//NRF_LOG_MODULE_REGISTER();


static uint16_t handles[] = 
{
	0,	// timestamp handle
	0,	// request data using datasize handle
	0,	// request data using timestamp handle
};

static void on_notify_done(ble_collar_t* p_collar, ble_evt_t const* p_ble_evt)
{

}

/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_collar_t * p_collar, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	// Precaution - if we have no callback function, we can simply skip it.
	if(p_collar->collar_write_handler == NULL) return;

	// Determine what has been updated.
	uint16_t len = p_evt_write->len;
	uint16_t updHandle = p_evt_write->handle;
	COLLAR_UPDATE_EVT_t event = COLLAR_UPDATE_NONE;

	if(updHandle == handles[0])
	{
		if(len <= 6) event = COLLAR_UPDATE_TIMESTAMP;
	}
	else if(updHandle == handles[1])
	{
		event = COLLAR_UPDATE_GET_SIZE;
	}
	else if(updHandle == handles[2])
	{
		event = COLLAR_UPDATE_GET_FROM_TIMESTAMP;
	}
	else if(updHandle == p_collar->send_data_handles.cccd_handle)
	{
		if(p_evt_write->len == 2)
		{
			if(ble_srv_is_notification_enabled(p_evt_write->data))
			{
				event = COLLAR_UPDATE_NOTIFICATION_ENABLED;
			}
			else
			{
				event = COLLAR_UPDATE_NOTIFICATION_DISABLED;
			}
		}
	}

	// If the matching item is found, we notify using the callback function.
	if(event != COLLAR_UPDATE_NONE)
	{
		p_collar->collar_write_handler(event, p_collar);
	}
}


void ble_collar_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_collar_t * p_collar = (ble_collar_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
		case BLE_GAP_EVT_CONNECTED:
			//on_connect(p_cus, p_ble_evt);
			handles[0] = p_collar->timestamp_handles.value_handle;
			handles[1] = p_collar->req_data_from_size_handles.value_handle;
			handles[2] = p_collar->req_data_from_time_handles.value_handle;
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			//on_disconnect(p_cus, p_ble_evt);
			handles[0] = 0;
			handles[1] = 0;
			handles[2] = 0;
			break;

		case BLE_GATTS_EVT_WRITE:
			on_write(p_collar, p_ble_evt);
			break;

//		case BLE_GATTS_EVT_HVN_TX_COMPLETE:
//			on_notify_done(p_collar, p_ble_evt);
//			break;

		default:
			// No implementation needed.
			break;
	}
}

uint32_t ble_collar_init(ble_collar_t * p_collar, const ble_collar_init_t * p_collar_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
	ble_gatts_char_pf_t   pf;	// presentation format

    // Initialize service structure.
    p_collar->collar_write_handler = p_collar_init->collar_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {COLLAR_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_collar->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_collar->uuid_type;
    ble_uuid.uuid = COLLAR_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
										&ble_uuid,
										&p_collar->service_handle);
    VERIFY_SUCCESS(err_code);
	
	// (0xFA01) Add timestamp characteristic - write only
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = COLLAR_UUID_TIMESTAMP;
    add_char_params.uuid_type        = p_collar->uuid_type;
    add_char_params.init_len         = sizeof(uint8_t) * 6;
    add_char_params.max_len          = sizeof(uint8_t) * 6;
	add_char_params.p_init_value     = p_collar->timestamp.data;
 	add_char_params.char_props.write_wo_resp = 1;
	//add_char_params.char_props.read = 1;
	add_char_params.is_value_user = 1;

    //add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
	
	err_code = characteristic_add(p_collar->service_handle,
                                  &add_char_params,
                                  &p_collar->timestamp_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	
	// (0xFA02) Add datasize characteristic - read only ( no. of data available )
	memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = COLLAR_UUID_DATASIZE;
    add_char_params.uuid_type        = p_collar->uuid_type;
    add_char_params.init_len         = sizeof(uint8_t) * 4;
    add_char_params.max_len          = sizeof(uint8_t) * 4;
	add_char_params.p_init_value     = p_collar->datasize.data;
    add_char_params.char_props.read  = 1;
    //add_char_params.char_props.write = 0;
	add_char_params.is_value_user = 1;

    add_char_params.read_access  = SEC_OPEN;
    //add_char_params.write_access = SEC_OPEN;
	
	err_code = characteristic_add(p_collar->service_handle,
                                  &add_char_params,
                                  &p_collar->datasize_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	// (0xFA03) Add last marker characteristic - read only ( the last data retrival marker )
	memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = COLLAR_UUID_LAST_MARKER;
    add_char_params.uuid_type        = p_collar->uuid_type;
    add_char_params.init_len         = sizeof(uint8_t) * 4;
    add_char_params.max_len          = sizeof(uint8_t) * 4;
    add_char_params.char_props.read  = 1;
	add_char_params.p_init_value     = p_collar->lastMarker.data;
    //add_char_params.char_props.write = 0;
	add_char_params.is_value_user    = 1;

    add_char_params.read_access  = SEC_OPEN;
    //add_char_params.write_access = SEC_OPEN;
	
	err_code = characteristic_add(p_collar->service_handle,
                                  &add_char_params,
                                  &p_collar->last_marker_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	// (0xFA04) Add data get characteristic - write only ( get request on the no. of data to retrieve )
	memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = COLLAR_UUID_DATA_GET;
    add_char_params.uuid_type        = p_collar->uuid_type;
    add_char_params.init_len         = sizeof(uint8_t) * 4;
    add_char_params.max_len          = sizeof(uint8_t) * 4;
	add_char_params.p_init_value     = p_collar->req_datasize.data;
    //add_char_params.char_props.read  = 1;
    add_char_params.char_props.write_wo_resp = 1;
	add_char_params.is_value_user    = 1;

    //add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
	
	err_code = characteristic_add(p_collar->service_handle,
                                  &add_char_params,
                                  &p_collar->req_data_from_size_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	// (0xFA05) Add data get with time characteristic - write only ( get request on data based on timestamp )
	memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid             = COLLAR_UUID_DATA_GET_WITH_TIME;
    add_char_params.uuid_type        = p_collar->uuid_type;
    add_char_params.init_len         = sizeof(uint8_t) * 6;
    add_char_params.max_len          = sizeof(uint8_t) * 6;
	add_char_params.p_init_value     = p_collar->from_timestamp.data;
    //add_char_params.char_props.read  = 1;
    add_char_params.char_props.write_wo_resp = 1;
	add_char_params.is_value_user = 1;

    //add_char_params.read_access  = SEC_OPEN;
    add_char_params.write_access = SEC_OPEN;
	
	err_code = characteristic_add(p_collar->service_handle,
                                  &add_char_params,
                                  &p_collar->req_data_from_time_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // (0xFA06) Add data send out characteristic.
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid              = COLLAR_UUID_DATA;
    add_char_params.uuid_type         = p_collar->uuid_type;
    add_char_params.init_len          = sizeof(uint8_t) * 1;
    add_char_params.max_len           = sizeof(uint8_t) * MAX_NOTIFY_PACKET_SIZE;
    add_char_params.char_props.read   = 1;
    add_char_params.char_props.notify = 1;
	add_char_params.is_var_len = 1;

    add_char_params.read_access       = SEC_OPEN;
    add_char_params.cccd_write_access = SEC_OPEN;

    err_code = characteristic_add(p_collar->service_handle,
                                  &add_char_params,
                                  &p_collar->send_data_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

	return NRF_SUCCESS;
}

uint32_t ble_collar_update_datasize(ble_collar_t* pCollar, uint32_t datasize)
{
	// Precaution
	if(pCollar == NULL) return NRF_ERROR_NULL;

	ret_code_t errCode = NRF_SUCCESS;
	ble_gatts_value_t gatts_value;

	memset(&gatts_value, 0, sizeof(gatts_value));

	pCollar->datasize.value = datasize;

	gatts_value.len = 4;
	gatts_value.offset = 0;
	gatts_value.p_value = pCollar->datasize.data;

	errCode = sd_ble_gatts_value_set(BLE_CONN_HANDLE_INVALID,
	                                 pCollar->datasize_handles.value_handle,
	                                 &gatts_value);

	if(errCode == NRF_SUCCESS)
	{

		NRF_LOG_INFO("Update datasize to %08X", datasize);
	}
	else
	{
		NRF_LOG_INFO("Fail to update datasize...%d", errCode);
	}

	return errCode;
}

uint32_t ble_collar_send_out(uint16_t conn_handle, ble_collar_t * p_collar, uint8_t* pData, uint8_t datasize)
{
	uint32_t err_code = NRF_SUCCESS;
	ble_gatts_hvx_params_t params;
	uint16_t len = (uint16_t)datasize;

	// 1. Check that it is already notified.
	//bool enabled = gatt_characteristic_check_notify_enable( conn_handle,
	//														p_collar->send_data_handles.cccd_handle);
	//if(!enabled) return;

	for(uint8_t i = 0; i < datasize; i++)
	{
		p_collar->buffer[i] = *(pData + i);
	}
	p_collar->outLen = len;

	memset(&params, 0, sizeof(params));
	params.type   = BLE_GATT_HVX_NOTIFICATION;
	params.handle = p_collar->send_data_handles.value_handle;
	params.offset = 0;

	uint16_t offset = 0;
	uint16_t pktLength;
	for(uint8_t i = 0; i < 4; i++)
	{
		pktLength = MIN(len - offset, MAX_NOTIFY_PACKET_SIZE);

		params.p_data = pData;
		params.p_len = &pktLength;
		params.offset = 0;

		err_code = sd_ble_gatts_hvx(conn_handle, &params);
		if(err_code != NRF_SUCCESS) break;

		offset += pktLength;
		pData += pktLength;

		if(offset >= len) break;
	}
	
	return err_code;

	uint16_t onePktMaxLength = 10;

	//memset(&gatts_value, 0, sizeof(gatts_value));
	//gatts_value.len     = sizeof(uint8_t);
 //   gatts_value.offset  = 0;
 //   gatts_value.p_value = pData;

	//err_code = sd_ble_gatts_value_set(conn_handle, 
	//								  p_collar->send_data_handles.value_handle,
	//								  &gatts_value);
	//if(err_code != NRF_SUCCESS)
	//{
	//	return err_code;
	//}

	//uint8_t no_of_bytes_sent = 0;
	//uint16_t offset = 0;
	//uint16_t actualSize;
	//while (no_of_bytes_sent < len)
	//{
	//	actualSize = MIN(onePktMaxLength, len);

	//	memset(&params, 0, sizeof(params));
	//	params.type   = BLE_GATT_HVX_NOTIFICATION;
	//	params.handle = p_collar->send_data_handles.value_handle;
	//	params.p_data = pData + offset;
	//	params.p_len = &actualSize;
	//	params.offset = offset;

	//	err_code = sd_ble_gatts_hvx(conn_handle, &params);
	//	if(err_code != NRF_SUCCESS) break;

	//	len -= actualSize;
	//	offset += actualSize;
	//	no_of_bytes_sent += actualSize;
	//}
	//return err_code;
}
#endif // NRF_MODULE_ENABLED(BLE_LBS)
