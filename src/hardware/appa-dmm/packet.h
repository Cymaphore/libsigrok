/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2020 Martin Eitzenberger <x@cymaphore.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file
 * @version 1
 *
 * APPA B Interface
 *
 * Based on APPA Communication Protocol v2.8
 *
 * Driver for modern APPA meters (handheld, bench, clamp). Communication is
 * done over a serial interface using the known APPA-Frames, see below. The
 * base protocol is always the same and deviates only where the models have
 * differences in ablities, range and features.
 *
 */

#ifndef LIBSIGROK_HARDWARE_APPA_DMM_PACKET_H
#define LIBSIGROK_HARDWARE_APPA_DMM_PACKET_H

#include <config.h>
#include "protocol.h"

#include <math.h>

/* ********************************* */
/* ****** Encoding / decoding ****** */
/* ********************************* */


/**
 * Get frame size of request command
 *
 * @param arg_command Command
 * @return Size in bytes of frame
 */
static int appadmm_get_request_size(enum appadmm_command_e arg_command)
{
	switch (arg_command) {
	case APPADMM_COMMAND_READ_INFORMATION:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_READ_INFORMATION;
	case APPADMM_COMMAND_READ_DISPLAY:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_READ_DISPLAY;
	case APPADMM_COMMAND_READ_PROTOCOL_VERSION:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_READ_PROTOCOL_VERSION;
	case APPADMM_COMMAND_READ_BATTERY_LIFE:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_READ_BATTERY_LIFE;
	case APPADMM_COMMAND_WRITE_UART_CONFIGURATION:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_WRITE_UART_CONFIGURATION;
	case APPADMM_COMMAND_CAL_READING:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_READING;
	case APPADMM_COMMAND_READ_MEMORY:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_READ_MEMORY;
	case APPADMM_COMMAND_READ_HARMONICS_DATA:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_READ_HARMONICS_DATA;
	case APPADMM_COMMAND_CAL_ENTER:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_ENTER;
	case APPADMM_COMMAND_CAL_WRITE_FUNCTION_CODE:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_WRITE_FUNCTION_CODE;
	case APPADMM_COMMAND_CAL_WRITE_RANGE_CODE:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_WRITE_RANGE_CODE;
	case APPADMM_COMMAND_CAL_WRITE_MEMORY:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_WRITE_MEMORY;
	case APPADMM_COMMAND_CAL_EXIT:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_EXIT;
	case APPADMM_COMMAND_OTA_ENTER:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_ENTER;
	case APPADMM_COMMAND_OTA_SEND_INFORMATION:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_SEND_INFORMATION;
	case APPADMM_COMMAND_OTA_SEND_FIRMWARE_PACKAGE:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_SEND_FIRMWARE_PACKAGE;
	case APPADMM_COMMAND_OTA_START_UPGRADE_PROCEDURE:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_START_UPGRADE_PROCEDURE;

		/* these are responses only */
	case APPADMM_COMMAND_FAILURE:
	case APPADMM_COMMAND_SUCCESS:

		/* safe default */
	default:
		return SR_ERR_DATA;
	}
	return SR_ERR_BUG;
}

/**
 * Get frame size of response command
 *
 * @param arg_command Command
 * @return Size in bytes of frame
 */
static int appadmm_get_response_size(enum appadmm_command_e arg_command)
{
	switch (arg_command) {
	case APPADMM_COMMAND_READ_INFORMATION:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_INFORMATION;
	case APPADMM_COMMAND_READ_DISPLAY:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_DISPLAY;
	case APPADMM_COMMAND_READ_PROTOCOL_VERSION:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_PROTOCOL_VERSION;
	case APPADMM_COMMAND_READ_BATTERY_LIFE:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_BATTERY_LIFE;
	case APPADMM_COMMAND_CAL_READING:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_CAL_READING;
	case APPADMM_COMMAND_READ_MEMORY:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_MEMORY;
	case APPADMM_COMMAND_READ_HARMONICS_DATA:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_HARMONICS_DATA;
	case APPADMM_COMMAND_FAILURE:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_FAILURE;
	case APPADMM_COMMAND_SUCCESS:
		return APPADMM_FRAME_DATA_SIZE_RESPONSE_SUCCESS;

		/* these respond with APPADMM_FRAME_DATA_SIZE_RESPONSE_SUCCESS
		 * or APPADMM_FRAME_DATA_SIZE_RESPONSE_FAILURE */
	case APPADMM_COMMAND_WRITE_UART_CONFIGURATION:
	case APPADMM_COMMAND_CAL_ENTER:
	case APPADMM_COMMAND_CAL_WRITE_FUNCTION_CODE:
	case APPADMM_COMMAND_CAL_WRITE_RANGE_CODE:
	case APPADMM_COMMAND_CAL_WRITE_MEMORY:
	case APPADMM_COMMAND_CAL_EXIT:
	case APPADMM_COMMAND_OTA_ENTER:
	case APPADMM_COMMAND_OTA_SEND_INFORMATION:
	case APPADMM_COMMAND_OTA_SEND_FIRMWARE_PACKAGE:
	case APPADMM_COMMAND_OTA_START_UPGRADE_PROCEDURE:

		/* safe default */
	default:
		return SR_ERR_DATA;
	}
	return SR_ERR_BUG;
}

/**
 * Check, if response size is valid
 *
 * @param arg_command Command
 * @return SR_OK if size is valid, otherwise SR_ERR_...
 */
static int appadmm_is_response_size_valid(enum appadmm_command_e arg_command,
	int arg_size)
{
	int size;

	size = appadmm_get_response_size(arg_command);

	if (size < SR_OK)
		return size;

	if (arg_command == APPADMM_COMMAND_READ_MEMORY
		&& arg_size <= size)
		return SR_OK;

	if (size == arg_size)
		return SR_OK;

	return SR_ERR_DATA;
}

/**
 * Callback-Function for data acquisition
 * will call appadmm_receive() to process received data
 *
 * @param arg_fd Unused file descripter
 * @param arg_revents Kind of event
 * @param arg_cb_data Unused CB data
 * @return SR_OK if successfull, otherweise SR_ERR_...
 */
static int appadmm_enc_read_information(const struct appadmm_request_data_read_information_s *arg_read_information,
	struct sr_tp_appa_packet *arg_packet)
{
	if (arg_packet == NULL
		|| arg_read_information == NULL)
		return SR_ERR_ARG;
	
	arg_packet->command = APPADMM_COMMAND_READ_INFORMATION;
	arg_packet->length = appadmm_get_request_size(APPADMM_COMMAND_READ_INFORMATION);
	
	return SR_OK;
}

/**
 * Decode raw data of COMMAND_READ_INFORMATION
 *
 * @param arg_rdptr Pointer to read from
 * @param arg_data Data structure to decode into
 * @return SR_OK if successfull, otherweise SR_ERR_...
 */
static int appadmm_dec_read_information(const struct sr_tp_appa_packet *arg_packet,
	struct appadmm_response_data_read_information_s *arg_read_information)
{
	int xloop;
	char *ltr;
	const uint8_t *rdptr;
	

	if (arg_packet == NULL
		|| arg_read_information == NULL)
		return SR_ERR_ARG;

	if (sizeof(arg_read_information->model_name) == 0
		|| sizeof(arg_read_information->serial_number) == 0)
		return SR_ERR_BUG;

	if (arg_packet->command != APPADMM_COMMAND_READ_INFORMATION)
		return SR_ERR_DATA;
	
	if (appadmm_is_response_size_valid(APPADMM_COMMAND_READ_INFORMATION, arg_packet->length))
		return SR_ERR_DATA;
	
	rdptr = &arg_packet->data[0];
	
	arg_read_information->model_name[0] = 0;
	arg_read_information->serial_number[0] = 0;
	arg_read_information->firmware_version = 0;
	arg_read_information->model_id = 0;

	ltr = &arg_read_information->model_name[0];
	for (xloop = 0; xloop < 32; xloop++) {
		*ltr = read_u8_inc(&rdptr);
		ltr++;
	}
	arg_read_information->model_name[sizeof(arg_read_information->model_name) - 1] = 0;

	/* Strip spaces from model name */
	g_strstrip(arg_read_information->model_name);

	ltr = &arg_read_information->serial_number[0];
	for (xloop = 0; xloop < 16; xloop++) {
		*ltr = read_u8_inc(&rdptr);
		ltr++;
	}
	arg_read_information->serial_number[sizeof(arg_read_information->serial_number) - 1] = 0;

	/* Strip spaces from serial number */
	g_strstrip(arg_read_information->serial_number);

	arg_read_information->model_id = read_u16le_inc(&rdptr);
	arg_read_information->firmware_version = read_u16le_inc(&rdptr);

	return SR_OK;
}

static int appadmm_rere_read_information(struct sr_tp_appa_inst* arg_tpai,
	const struct appadmm_request_data_read_information_s *arg_request,
	struct appadmm_response_data_read_information_s *arg_response)
{
	struct sr_tp_appa_packet packet_request;
	struct sr_tp_appa_packet packet_response;
	
	int retr;
	
	if (arg_tpai == NULL
		|| arg_request == NULL
		|| arg_response == NULL)
		return SR_ERR_ARG;
	
	if ((retr = appadmm_enc_read_information(arg_request, &packet_request))
		< SR_OK)
		return retr;
	if ((retr = sr_tp_appa_send_receive(arg_tpai, &packet_request,
		&packet_response)) < TRUE)
		return retr;
	if ((retr = appadmm_dec_read_information(&packet_response, arg_response))
		< SR_OK)
		return retr;
	
	return TRUE;
}

static int appadmm_enc_read_display(const struct appadmm_request_data_read_display_s *arg_read_display,
	struct sr_tp_appa_packet *arg_packet)
{
	if (arg_packet == NULL
		|| arg_read_display == NULL)
		return SR_ERR_ARG;
	
	arg_packet->command = APPADMM_COMMAND_READ_DISPLAY;
	arg_packet->length = appadmm_get_request_size(APPADMM_COMMAND_READ_DISPLAY);
	
	return SR_OK;
}

/**
 * Decode raw data of COMMAND_READ_DISPLAY
 *
 * @param arg_rdptr Pointer to read from
 * @param arg_data Data structure to decode into
 * @return SR_OK if successfull, otherweise SR_ERR_...
 */
static int appadmm_dec_read_display(const struct sr_tp_appa_packet *arg_packet,
	struct appadmm_response_data_read_display_s *arg_read_display)
{
	const uint8_t *rdptr;
	uint8_t u8;

	if (arg_packet == NULL
		|| arg_read_display == NULL)
		return SR_ERR_ARG;

	if (arg_packet->command != APPADMM_COMMAND_READ_DISPLAY)
		return SR_ERR_DATA;
	
	if (appadmm_is_response_size_valid(APPADMM_COMMAND_READ_DISPLAY, arg_packet->length))
		return SR_ERR_DATA;
	
	rdptr = &arg_packet->data[0];

	u8 = read_u8_inc(&rdptr);
	arg_read_display->function_code = u8 & 0x7f;
	arg_read_display->auto_test = u8 >> 7;

	u8 = read_u8_inc(&rdptr);
	arg_read_display->range_code = u8 & 0x7f;
	arg_read_display->auto_range = u8 >> 7;

	arg_read_display->main_display_data.reading = read_i24le_inc(&rdptr);

	u8 = read_u8_inc(&rdptr);
	arg_read_display->main_display_data.dot = u8 & 0x7;
	arg_read_display->main_display_data.unit = u8 >> 3;

	u8 = read_u8_inc(&rdptr);
	arg_read_display->main_display_data.data_content = u8 & 0x7f;
	arg_read_display->main_display_data.overload = u8 >> 7;

	arg_read_display->sub_display_data.reading = read_i24le_inc(&rdptr);

	u8 = read_u8_inc(&rdptr);
	arg_read_display->sub_display_data.dot = u8 & 0x7;
	arg_read_display->sub_display_data.unit = u8 >> 3;

	u8 = read_u8_inc(&rdptr);
	arg_read_display->sub_display_data.data_content = u8 & 0x7f;
	arg_read_display->sub_display_data.overload = u8 >> 7;

	return SR_OK;
}

static int appadmm_request_read_display(struct sr_tp_appa_inst* arg_tpai,
	const struct appadmm_request_data_read_display_s *arg_request)
{
	struct sr_tp_appa_packet packet_request;
	
	int retr;
	
	if (arg_tpai == NULL
		|| arg_request == NULL)
		return SR_ERR_ARG;
	
	if ((retr = appadmm_enc_read_display(arg_request, &packet_request))
		< SR_OK)
		return retr;
	if ((retr = sr_tp_appa_send(arg_tpai, &packet_request, FALSE)) < SR_OK)
		return retr;
	
	return retr;
}

static int appadmm_response_read_display(struct sr_tp_appa_inst *arg_tpai,
	struct appadmm_response_data_read_display_s *arg_response)
{
	struct sr_tp_appa_packet packet_response;
	
	int retr;
	
	if (arg_tpai == NULL
		|| arg_response == NULL)
		return SR_ERR_ARG;
	
	if ((retr = sr_tp_appa_receive(arg_tpai, &packet_response, FALSE))
		< TRUE)
		return retr;
	
	if ((retr = appadmm_dec_read_display(&packet_response, arg_response))
		< SR_OK)
		return retr;
	
	return TRUE;
}

/**
 * Encode raw data of COMMAND_READ_MEMORY
 * 
 * @param arg_wrptr Pointer to write to
 * @param arg_data Data structure to encode from
 * @return  SR_OK if successfull, otherwise SR_ERR_...
 */
static int appadmm_enc_read_memory(const struct appadmm_request_data_read_memory_s *arg_read_memory,
	struct sr_tp_appa_packet *arg_packet)
{
	uint8_t *wrptr;
	
	if (arg_packet == NULL
		|| arg_read_memory == NULL)
		return SR_ERR_ARG;
	
	arg_packet->command = APPADMM_COMMAND_READ_MEMORY;
	arg_packet->length = appadmm_get_request_size(APPADMM_COMMAND_READ_MEMORY);
	
	wrptr = &arg_packet->data[0];
	
	write_u8_inc(&wrptr, arg_read_memory->device_number);
	write_u16le_inc(&wrptr, arg_read_memory->memory_address);
	write_u8_inc(&wrptr, arg_read_memory->data_length);
	
	return SR_OK;
}

/**
 * Decode raw data of COMMAND_READ_MEMORY
 *
 * @param arg_rdptr Pointer to read from
 * @param arg_data Data structure to decode into
 * @return SR_OK if successfull, otherweise SR_ERR_...
 */
static int appadmm_dec_read_memory(const struct sr_tp_appa_packet *arg_packet,
	struct appadmm_response_data_read_memory_s *arg_read_memory)
{
	const uint8_t *rdptr;
	int xloop;
	
	if (arg_packet == NULL
		|| arg_read_memory == NULL)
		return SR_ERR_ARG;
	
	if (arg_packet->command != APPADMM_COMMAND_READ_MEMORY)
		return SR_ERR_DATA;
	
	if (appadmm_is_response_size_valid(APPADMM_COMMAND_READ_MEMORY, arg_packet->length))
		return SR_ERR_DATA;
	
	rdptr = &arg_packet->data[0];
	
	if(arg_packet->length > sizeof(arg_read_memory->data))
		return SR_ERR_DATA;

	/* redundent, for future compatibility with older models */
	arg_read_memory->data_length = arg_packet->length;
	
	for(xloop = 0; xloop < arg_packet->length; xloop++)
		arg_read_memory->data[xloop] = read_u8_inc(&rdptr);
	
	return SR_OK;
}

static int appadmm_rere_read_memory(struct sr_tp_appa_inst* arg_tpai,
	const struct appadmm_request_data_read_memory_s *arg_request,
	struct appadmm_response_data_read_memory_s *arg_response)
{
	struct sr_tp_appa_packet packet_request;
	struct sr_tp_appa_packet packet_response;
	
	int retr;
	
	if (arg_tpai == NULL
		|| arg_request == NULL
		|| arg_response == NULL)
		return SR_ERR_ARG;
	
	if ((retr = appadmm_enc_read_memory(arg_request, &packet_request))
		< SR_OK)
		return retr;
	
	if ((retr = sr_tp_appa_send_receive(arg_tpai, &packet_request,
		&packet_response)) < TRUE)
		return retr;
	
	if ((retr = appadmm_dec_read_memory(&packet_response, arg_response))
		< SR_OK)
		return retr;
	
	return TRUE;
}

static int appadmm_request_read_memory(struct sr_tp_appa_inst* arg_tpai,
	const struct appadmm_request_data_read_memory_s *arg_request)
{
	struct sr_tp_appa_packet packet_request;
	
	int retr;
	
	if (arg_tpai == NULL
		|| arg_request == NULL)
		return SR_ERR_ARG;
	
	if ((retr = appadmm_enc_read_memory(arg_request, &packet_request))
		< SR_OK)
		return retr;
	if ((retr = sr_tp_appa_send(arg_tpai, &packet_request, FALSE)) < SR_OK)
		return retr;
	
	return retr;
}

static int appadmm_response_read_memory(struct sr_tp_appa_inst *arg_tpai,
	struct appadmm_response_data_read_memory_s *arg_response)
{
	struct sr_tp_appa_packet packet_response;
	
	int retr;
	
	if (arg_tpai == NULL
		|| arg_response == NULL)
		return SR_ERR_ARG;
	
	if ((retr = sr_tp_appa_receive(arg_tpai, &packet_response, FALSE))
		< TRUE)
		return retr;
	
	if ((retr = appadmm_dec_read_memory(&packet_response, arg_response))
		< SR_OK)
		return retr;
	
	return TRUE;
}

static int appadmm_dec_storage_info(const struct appadmm_response_data_read_memory_s
*arg_read_memory, struct appadmm_storage_info_s *arg_storage_info)
{
	const uint8_t *rdptr;
	
	if (arg_read_memory == NULL
		|| arg_storage_info == NULL)
		return SR_ERR_ARG;
	
	if (arg_read_memory->data_length != 6)
		return SR_ERR_DATA;
	
	rdptr = &arg_read_memory->data[0];
	
	arg_storage_info[APPADMM_STORAGE_LOG].rate = read_u16be_inc(&rdptr);
	arg_storage_info[APPADMM_STORAGE_LOG].amount = read_u16be_inc(&rdptr);
	arg_storage_info[APPADMM_STORAGE_MEM].amount = read_u16be_inc(&rdptr);
	
	/** @TODO do device detection */
	arg_storage_info[APPADMM_STORAGE_MEM].entry_size = 5;
	arg_storage_info[APPADMM_STORAGE_MEM].entry_count = 500;
	arg_storage_info[APPADMM_STORAGE_MEM].mem_offset = 0x500;
	arg_storage_info[APPADMM_STORAGE_MEM].mem_count = 2;
	
	arg_storage_info[APPADMM_STORAGE_LOG].entry_size = 5;
	arg_storage_info[APPADMM_STORAGE_LOG].entry_count = 10000;
	arg_storage_info[APPADMM_STORAGE_LOG].mem_offset = 0x1000;
	arg_storage_info[APPADMM_STORAGE_LOG].mem_count = 4;
	
	
	return SR_OK;
}

static int appadmm_enc_read_storage(struct appadmm_request_data_read_memory_s *arg_read_memory,
	struct appadmm_storage_info_s *arg_storage_info, int arg_start_entry,
	int arg_entry_count)
{
	int address_position;
	
	if (arg_read_memory == NULL
		|| arg_storage_info == NULL)
		return SR_ERR_ARG;
	
	if (arg_start_entry >
		arg_storage_info->mem_count * arg_storage_info->entry_count)
		return SR_ERR_ARG;
	
	/*
	sr_err("**** get log input: %d, %d",
		arg_start_entry, arg_entry_count);
	*/
	
	address_position = (arg_start_entry % arg_storage_info->entry_count);
	
	if(arg_entry_count > (SR_TP_APPA_MAX_DATA_SIZE / arg_storage_info->entry_size))
		arg_entry_count = (SR_TP_APPA_MAX_DATA_SIZE / arg_storage_info->entry_size);
	
	if(address_position + arg_entry_count > arg_storage_info->entry_count)
		arg_entry_count = arg_storage_info->entry_count - address_position;
	
	arg_read_memory->device_number = arg_start_entry /
		(arg_storage_info->entry_count);
	arg_read_memory->memory_address = arg_storage_info->mem_offset
		+ address_position * arg_storage_info->entry_size;
	arg_read_memory->data_length = arg_entry_count * arg_storage_info->entry_size;
	
	while (arg_read_memory->data_length > SR_TP_APPA_MAX_DATA_SIZE)
		arg_read_memory->data_length -= arg_storage_info->entry_size;
	
	if (arg_read_memory->device_number > arg_storage_info->mem_count)
		return SR_ERR_BUG;

	/* I don't want to know why I need to do this to avoid data to become garbage */
	arg_read_memory->data_length = 64;
	
	/*
	sr_err("**** get log output: %d, %d, %d",
		arg_read_memory->device_number,
		arg_read_memory->memory_address,
		arg_read_memory->data_length);
	*/
	
	return SR_OK;
}

static int appadmm_dec_read_storage(const struct appadmm_response_data_read_memory_s
	*arg_read_memory, struct appadmm_storage_info_s *arg_storage_info,
	struct appadmm_display_data_s *arg_display_data)
{
	const uint8_t *rdptr;
	uint8_t u8;
	int xloop;
	
	if (arg_read_memory == NULL
		|| arg_storage_info == NULL)
		return SR_ERR_ARG;

	rdptr = &arg_read_memory->data[0];

	for(xloop = 0; xloop < arg_read_memory->data_length /
		arg_storage_info->entry_size; xloop++) {
		arg_display_data[xloop].reading = read_i24le_inc(&rdptr);

		u8 = read_u8_inc(&rdptr);
		arg_display_data[xloop].dot = u8 & 0x7;
		arg_display_data[xloop].unit = u8 >> 3;

		u8 = read_u8_inc(&rdptr);
		arg_display_data[xloop].data_content = u8 & 0x7f;
		arg_display_data[xloop].overload = u8 >> 7;
	}
	
	return SR_OK;
}

#endif/*LIBSIGROK_HARDWARE_APPA_DMM_PACKET_H*/
