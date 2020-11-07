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
 * @file appa_dmm.c
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

#include <config.h>
#include "protocol.h"

/* *************************************** */
/* ****** Internal Helper Functions ****** */
/* *************************************** */

static gboolean appadmm_is_wordcode(const int arg_wordcode)
{
	return arg_wordcode >= APPADMM_WORDCODE_TABLE_MIN;
}

static gboolean appadmm_is_wordcode_dash(const int arg_wordcode)
{
	return
		arg_wordcode == APPADMM_WORDCODE_DASH
		|| arg_wordcode == APPADMM_WORDCODE_DASH1
		|| arg_wordcode == APPADMM_WORDCODE_DASH2;
}

static u_int8_t appadmm_checksum(const u_int8_t *arg_data, int arg_size)
{
	u_int8_t checksum;

	if (arg_data == NULL) {
		sr_err("appadmm_checksum(): checksum data error, NULL provided. returning 0");
		return 0;
	}

	checksum = 0;
	while (arg_size-- > 0)
		checksum += arg_data[arg_size];

	return checksum;
}

static int appadmm_frame_request_size(enum appadmm_command_e arg_command)
{
	switch(arg_command) {
	case APPADMM_COMMAND_READ_DISPLAY:
		return APPADMM_FRAME_DATA_SIZE_REQUEST_READ_DISPLAY;
	}
	return SR_ERR_DATA;
}

static int appadmm_frame_encode(const struct appadmm_frame_s *arg_frame, u_int8_t *arg_out_data, int arg_size)
{
	uint8_t *wrptr;
	
	int size;
	
	if (arg_frame == NULL
		|| arg_out_data == NULL) {
		sr_err("appadmm_frame_encode(): invalid arguments");
		return SR_ERR_ARG;
	}
	
	size = appadmm_frame_request_size(arg_frame->command);
	
	if (size < 0)
		return SR_ERR_DATA;
	
	if (size + APPADMM_FRAME_HEADER_SIZE + APPADMM_FRAME_CHECKSUM_SIZE
		> arg_size)
		return SR_ERR_DATA;
	
	wrptr = &arg_out_data[0];
	
	write_u16le_inc(&wrptr, APPADMM_FRAME_START_VALUE);
	
	write_u8_inc(&wrptr, arg_frame->command);
	
	write_u8_inc(&wrptr, size);
	
	switch (arg_frame->command) {
	
	case APPADMM_COMMAND_READ_DISPLAY:
		break;
	
	default:
		return SR_ERR_DATA;
		
	}
	
	write_u8_inc(&wrptr, appadmm_checksum(arg_out_data, size + APPADMM_FRAME_HEADER_SIZE));
	
	return SR_OK;
}

static int appadmm_frame_decode(const u_int8_t *arg_data, int arg_size, struct appadmm_frame_s *arg_out_frame)
{
	const uint8_t *rdptr;
	uint8_t u8;
	uint16_t u16;
	int32_t i32;
	
	int size;
	uint8_t checksum;
	
	if (arg_out_frame == NULL
		|| arg_data == NULL) {
		sr_err("appadmm_frame_decode(): invalid arguments");
		return SR_ERR_ARG;
	}
	
	rdptr = &arg_data[0];
	
	u16 = read_u16le_inc(&rdptr);
	
	if (u16 != APPADMM_FRAME_START_VALUE)
		return SR_ERR_DATA;
	
	arg_out_frame->command = read_u8_inc(&rdptr);
	
	size = read_u8_inc(&rdptr);
	
	if (size + APPADMM_FRAME_HEADER_SIZE + APPADMM_FRAME_CHECKSUM_SIZE
		> arg_size)
		return SR_ERR_DATA;
	
	switch (arg_out_frame->command) {
		
	case APPADMM_COMMAND_READ_DISPLAY:
		if (size != APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_DISPLAY)
			return SR_ERR_DATA;
		
		u8 = read_u8_inc(&rdptr);
		arg_out_frame->response.read_display.function_code = u8 & 0x7f;
		arg_out_frame->response.read_display.auto_test = u8 >> 7;
		
		u8 = read_u8_inc(&rdptr);
		arg_out_frame->response.read_display.range_code = u8 & 0x7f;
		arg_out_frame->response.read_display.auto_range = u8 >> 7;
		
		arg_out_frame->response.read_display.main_display_data.reading = read_i24le_inc(&rdptr);
		
		u8 = read_u8_inc(&rdptr);
		arg_out_frame->response.read_display.main_display_data.dot = u8 & 0x7;
		arg_out_frame->response.read_display.main_display_data.unit = u8 >> 3;
		
		u8 = read_u8_inc(&rdptr);
		arg_out_frame->response.read_display.main_display_data.data_content = u8 & 0x7f;
		arg_out_frame->response.read_display.main_display_data.overload = u8 >> 7;
		
		arg_out_frame->response.read_display.sub_display_data.reading = read_i24le_inc(&rdptr);
		
		u8 = read_u8_inc(&rdptr);
		arg_out_frame->response.read_display.sub_display_data.dot = u8 & 0x7;
		arg_out_frame->response.read_display.sub_display_data.unit = u8 >> 3;
		
		u8 = read_u8_inc(&rdptr);
		arg_out_frame->response.read_display.sub_display_data.data_content = u8 & 0x7f;
		arg_out_frame->response.read_display.sub_display_data.overload = u8 >> 7;
		
		break;
		
	default:
		return SR_ERR_DATA;
		
	}
	
	u8 = read_u8_inc(&rdptr);
	if (u8 != appadmm_checksum(arg_data[0], size + APPADMM_FRAME_HEADER_SIZE))
		return SR_ERR_DATA;
	
	/** @TODO dump btle extra bytes? */
	
	return SR_OK;
}

/* *************************************** */
/* ****** Internal Resolving tables ****** */
/* *************************************** */

static const char *appadmm_model_id_name(const enum appadmm_model_id_e arg_model_id)
{todo
	switch (arg_model_id) {
	case APPADMM_MODEL_ID_INVALID:
		return APPADMM_STRING_NA;
	case APPADMM_MODEL_ID_150:
		return "APPA 150";
	case APPADMM_MODEL_ID_150B:
		return "APPA 150B";
	case APPADMM_MODEL_ID_208:
		return "APPA 208";
	case APPADMM_MODEL_ID_208B:
		return "APPA 208B";
	case APPADMM_MODEL_ID_506:
		return "APPA 506";
	case APPADMM_MODEL_ID_506B:
		return "APPA 506B";
	case APPADMM_MODEL_ID_506B_2:
		return "APPA 506B";
	case APPADMM_MODEL_ID_501:
		return "APPA 501";
	case APPADMM_MODEL_ID_502:
		return "APPA 502";
	case APPADMM_MODEL_ID_S1:
		return "APPA S1";
	case APPADMM_MODEL_ID_S2:
		return "APPA S2";
	case APPADMM_MODEL_ID_S3:
		return "APPA S3";
	case APPADMM_MODEL_ID_172:
		return "APPA 172";
	case APPADMM_MODEL_ID_173:
		return "APPA 173";
	case APPADMM_MODEL_ID_175:
		return "APPA 175";
	case APPADMM_MODEL_ID_177:
		return "APPA 177";
	case APPADMM_MODEL_ID_SFLEX_10A:
		return "APPA sFlex-10A";
	case APPADMM_MODEL_ID_SFLEX_18A:
		return "APPA sFlex-18A";
	case APPADMM_MODEL_ID_A17N:
		return "APPA A17N";
	case APPADMM_MODEL_ID_S0:
		return "APPA S0";
	case APPADMM_MODEL_ID_179:
		return "APPA 179";
	case APPADMM_MODEL_ID_503:
		return "APPA 503";
	case APPADMM_MODEL_ID_505:
		return "APPA 505";
	}

	return APPADMM_STRING_NA;
}

static const char *appadmm_wordcode_name(const enum appadmm_wordcode_e arg_wordcode)
{
	switch (arg_wordcode) {
	case APPADMM_WORDCODE_SPACE:
		return "";
	case APPADMM_WORDCODE_FULL:
		return "Full";
	case APPADMM_WORDCODE_BEEP:
		return "Beep";
	case APPADMM_WORDCODE_APO:
		return "Auto Power-Off";
	case APPADMM_WORDCODE_B_LIT:
		return "Backlight";
	case APPADMM_WORDCODE_HAZ:
		return "Hazard";
	case APPADMM_WORDCODE_ON:
		return "On";
	case APPADMM_WORDCODE_OFF:
		return "Off";
	case APPADMM_WORDCODE_RESET:
		return "Reset";
	case APPADMM_WORDCODE_START:
		return "Start";
	case APPADMM_WORDCODE_VIEW:
		return "View";
	case APPADMM_WORDCODE_PAUSE:
		return "Pause";
	case APPADMM_WORDCODE_FUSE:
		return "Fuse";
	case APPADMM_WORDCODE_PROBE:
		return "Probe";
	case APPADMM_WORDCODE_DEF:
		return "Definition";
	case APPADMM_WORDCODE_CLR:
		return "Clr";
	case APPADMM_WORDCODE_ER:
		return "Er";
	case APPADMM_WORDCODE_ER1:
		return "Er1";
	case APPADMM_WORDCODE_ER2:
		return "Er2";
	case APPADMM_WORDCODE_ER3:
		return "Er3";
	case APPADMM_WORDCODE_DASH:
		return "-----";
	case APPADMM_WORDCODE_DASH1:
		return "-";
	case APPADMM_WORDCODE_TEST:
		return "Test";
	case APPADMM_WORDCODE_DASH2:
		return "--";
	case APPADMM_WORDCODE_BATT:
		return "Battery";
	case APPADMM_WORDCODE_DISLT:
		return "diSLt";
	case APPADMM_WORDCODE_NOISE:
		return "Noise";
	case APPADMM_WORDCODE_FILTR:
		return "Filter";
	case APPADMM_WORDCODE_PASS:
		return "PASS";
	case APPADMM_WORDCODE_NULL:
		return "null";
	case APPADMM_WORDCODE_0_20:
		return "0 - 20";
	case APPADMM_WORDCODE_4_20:
		return "4 - 20";
	case APPADMM_WORDCODE_RATE:
		return "Rate";
	case APPADMM_WORDCODE_SAVE:
		return "Save";
	case APPADMM_WORDCODE_LOAD:
		return "Load";
	case APPADMM_WORDCODE_YES:
		return "Yes";
	case APPADMM_WORDCODE_SEND:
		return "Send";
	case APPADMM_WORDCODE_AHOLD:
		return "Auto Hold";
	case APPADMM_WORDCODE_AUTO:
		return "Auto";
	case APPADMM_WORDCODE_CNTIN:
		return "Continuity";
	case APPADMM_WORDCODE_CAL:
		return "CAL";
	case APPADMM_WORDCODE_VERSION:
		return "Version";
	case APPADMM_WORDCODE_OL:
		return "OL";
	case APPADMM_WORDCODE_BAT_FULL:
		return "FULL";
	case APPADMM_WORDCODE_BAT_HALF:
		return "HALF";
	case APPADMM_WORDCODE_LO:
		return "Lo";
	case APPADMM_WORDCODE_HI:
		return "Hi";
	case APPADMM_WORDCODE_DIGIT:
		return "Digits";
	case APPADMM_WORDCODE_RDY:
		return "Ready";
	case APPADMM_WORDCODE_DISC:
		return "dISC";
	case APPADMM_WORDCODE_OUTF:
		return "outF";
	case APPADMM_WORDCODE_OLA:
		return "OLA";
	case APPADMM_WORDCODE_OLV:
		return "OLV";
	case APPADMM_WORDCODE_OLVA:
		return "OLVA";
	case APPADMM_WORDCODE_BAD:
		return "BAD";
	case APPADMM_WORDCODE_TEMP:
		return "TEMP";
	}

	return APPADMM_STRING_NA;
}

/* ********************************************** */
/* ****** Internal response frame decoding ****** */
/* ********************************************** */

static int appadmm_send_frame_display_request(u_int8_t *arg_buf, int arg_len)
{
	u_int8_t write_pos;

	if (arg_buf == NULL) {
		sr_err("appadmm_write_frame_display_request(): buffer error");
		return SR_ERR_ARG;
	}

	if (arg_len < 5) {
		sr_err("appadmm_write_frame_display_request(): argument error");
		return SR_ERR_ARG;
	}

	write_pos = 0;

	arg_buf[write_pos++] = APPADMM_FRAME_START_VALUE_BYTE;
	arg_buf[write_pos++] = APPADMM_FRAME_START_VALUE_BYTE;
	arg_buf[write_pos++] = APPADMM_COMMAND_READ_DISPLAY;
	arg_buf[write_pos++] = 0;
	arg_buf[write_pos++] = appadmm_checksum(arg_buf, APPADMM_FRAME_HEADER_SIZE);

	return SR_OK;
}

static int appadmm_recv_frame_display_response(const u_int8_t *arg_buf, struct appadmm_response_data_read_display_s* arg_display_response_data)
{
	u_int8_t read_pos;
	u_int8_t reading[3];

	if (arg_buf == NULL
		|| arg_display_response_data == NULL) {
		sr_err("appadmm_read_frame_display_response(): invalid arguments for function");
		return SR_ERR;
	}

	read_pos = 0;

	if (arg_buf[read_pos++] != APPADMM_FRAME_START_VALUE_BYTE
		|| arg_buf[read_pos++] != APPADMM_FRAME_START_VALUE_BYTE)
		return SR_ERR_IO;

	if (arg_buf[read_pos++] != APPADMM_COMMAND_READ_DISPLAY)
		return SR_ERR_IO;

	if (arg_buf[read_pos++] != APPADMM_DATA_LENGTH_RESPONSE_READ_DISPLAY)
		return SR_ERR_IO;

	arg_display_response_data->function_code = arg_buf[read_pos] & 0x7f;
	arg_display_response_data->auto_test = arg_buf[read_pos++] >> 7;

	arg_display_response_data->range_code = arg_buf[read_pos] & 0x7f;
	arg_display_response_data->auto_range = arg_buf[read_pos++] >> 7;

	reading[0] = arg_buf[read_pos++];
	reading[1] = arg_buf[read_pos++];
	reading[2] = arg_buf[read_pos++];

	arg_display_response_data->main_display_data.reading =
		reading[0]
		| reading[1] << 8
		| reading[2] << 16
		| ((reading[2] >> 7 == 1) ? 0xff : 0) << 24;

	arg_display_response_data->main_display_data.dot = arg_buf[read_pos] & 0x7;
	arg_display_response_data->main_display_data.unit = arg_buf[read_pos++] >> 3;

	arg_display_response_data->main_display_data.data_content = arg_buf[read_pos] & 0x7f;
	arg_display_response_data->main_display_data.overload = arg_buf[read_pos++] >> 7;

	reading[0] = arg_buf[read_pos++];
	reading[1] = arg_buf[read_pos++];
	reading[2] = arg_buf[read_pos++];

	arg_display_response_data->sub_display_data.reading =
		reading[0]
		| reading[1] << 8
		| reading[2] << 16
		| ((reading[2] >> 7 == 1) ? 0xff : 0) << 24;

	arg_display_response_data->sub_display_data.dot = arg_buf[read_pos] & 0x7;
	arg_display_response_data->sub_display_data.unit = arg_buf[read_pos++] >> 3;

	arg_display_response_data->sub_display_data.data_content = arg_buf[read_pos] & 0x7f;
	arg_display_response_data->sub_display_data.overload = arg_buf[read_pos++] >> 7;

	return SR_OK;
}

/* *************************************************** */
/* ****** Static tables used from within sigrok ****** */
/* *************************************************** */

SR_PRIV const char *sr_appadmm_channel_formats[APPADMM_DISPLAY_COUNT] = {
	APPADMM_CHANNEL_NAME_DISPLAY_MAIN,
	APPADMM_CHANNEL_NAME_DISPLAY_SUB,
};

/* ************************************************* */
/* ****** Functions called from within sigrok ****** */
/* ************************************************* */

#ifdef HAVE_SERIAL_COMM

/**
 * Request frame from device
 *
 * Response will contain both display readings
 *
 * @param serial Serial data
 * @return @sr_error_code Status code
 */
SR_PRIV int sr_appadmm_packet_request(struct sr_serial_dev_inst *serial)
{
	u_int8_t buf[5];

	if (serial == NULL) {
		sr_err("sr_appadmm_serial_packet_request(): serial error");
		return SR_ERR_ARG;
	}

#ifdef APPADMM_ENABLE_FLUSH
	if (serial_flush(serial) != SR_OK) {
		sr_err("sr_appadmm_serial_packet_request(): flush error");
		return SR_ERR_IO;
	}
#endif/*APPADMM_ENABLE_FLUSH*/

	if (appadmm_send_frame_display_request(buf, sizeof(buf)) != SR_OK) {
		sr_err("sr_appadmm_serial_packet_request(): display_request generation error");
		return SR_ERR;
	}

#ifdef APPADMM_ENABLE_NON_BLOCKING
	if (serial_write_nonblocking(serial, &buf, sizeof(buf)) != sizeof(buf)) {
		sr_err("sr_appadmm_serial_packet_request(): display_request write error");
		return SR_ERR_IO;
	}
#else/*APPADMM_ENABLE_NON_BLOCKING*/
	if (serial_write_blocking(serial, &buf, sizeof(buf), APPADMM_WRITE_BLOCKING_TIMEOUT) != sizeof(buf)) {
		sr_err("sr_appadmm_serial_packet_request(): display_request write error");
		return SR_ERR_IO;
	}
#endif/*APPADMM_ENABLE_NON_BLOCKING*/

	return SR_OK;
}

#endif/*HAVE_SERIAL_COMM*/

/**
 * Validate APPA-Frame
 *
 * @param state session state
 * @param data data recieved
 * @param dlen reported length
 * @param pkt_len return length
 * @return TRUE if checksum is fine
 */
SR_PRIV gboolean sr_appadmm_packet_valid(const uint8_t *data)
{
	int frame_length;
	u_int8_t checksum;

	if (data == NULL) {
		sr_err("sr_appadmm_packet_valid(): data error");
		return FALSE;
	}

	if (data[0] != APPADMM_FRAME_START_VALUE_BYTE
		|| data[1] != APPADMM_FRAME_START_VALUE_BYTE)
		return FALSE;
	
	switch (data[]) {
	
	/* Currently implemented (valid) commands */
	case APPADMM_COMMAND_READ_DISPLAY:
		break;

	/* Currently unimplemented (invalid) commands */
	default:
	case APPADMM_COMMAND_READ_INFORMATION:
	case APPADMM_COMMAND_READ_PROTOCOL_VERSION:
	case APPADMM_COMMAND_READ_BATTERY_LIFE:
	case APPADMM_COMMAND_WRITE_UART_CONFIGURATION:
	case APPADMM_COMMAND_CAL_READING:
	case APPADMM_COMMAND_READ_MEMORY:
	case APPADMM_COMMAND_READ_HARMONICS_DATA:
	case APPADMM_COMMAND_FAILURE:
	case APPADMM_COMMAND_SUCCESS:
	case APPADMM_COMMAND_CAL_ENTER:
	case APPADMM_COMMAND_CAL_WRITE_FUNCTION_CODE:
	case APPADMM_COMMAND_CAL_WRITE_RANGE_CODE:
	case APPADMM_COMMAND_CAL_WRITE_MEMORY:
	case APPADMM_COMMAND_CAL_EXIT:
	case APPADMM_COMMAND_OTA_ENTER:
	case APPADMM_COMMAND_OTA_SEND_INFORMATION:
	case APPADMM_COMMAND_OTA_SEND_FIRMWARE_PACKAGE:
	case APPADMM_COMMAND_OTA_START_UPGRADE_PROCEDURE:
		return FALSE;
	}

	frame_length = APPADMM_PAYLOAD_LENGTH(data[3]);
	checksum = appadmm_checksum(data, frame_length);
	
	if (checksum != data[frame_length])
		return FALSE;
	
	return TRUE;
}

/**
 * Parse APPA-Frame and assign values to virtual channels
 *
 * @TODO include display reading as debug output?
 *
 * @param buf Buffer from Serial or BTLE
 * @param floatval Return display reading
 * @param analog Metadata of the reading
 * @param info Channel information and other things
 * @return @sr_error_code Status
 */
SR_PRIV int sr_appadmm_parse(const uint8_t *data, float *val,
			    struct sr_datafeed_analog *analog, void *info)
{
	struct appadmm_info *info_local;
	struct appadmm_response_data_read_display_s display_response_data;
	struct appadmm_display_data_s *display_data;

	gboolean is_sub;
	gboolean is_dash;

	double unit_factor;
	double display_reading_value;
	int8_t digits;

	if (data == NULL
		|| val == NULL
		|| analog == NULL
		|| info == NULL) {
		sr_err("sr_appadmm_parse(): missing arguments");
		return SR_ERR_ARG;
	}

	info_local = info;

	is_sub = (info_local->ch_idx == 1);

	if (appadmm_recv_frame_display_response(data, &display_response_data) != SR_OK) {
		sr_err("sr_appadmm_parse(): frame decode error");
		return SR_ERR_DATA;
	}

	if (!is_sub)
		display_data = &display_response_data.main_display_data;
	else
		display_data = &display_response_data.sub_display_data;

	unit_factor = 1;
	digits = 0;

	display_reading_value = (double) display_data->reading;

	is_dash = appadmm_is_wordcode_dash(display_data->reading);

	if (!appadmm_is_wordcode(display_data->reading)
		|| is_dash) {

		switch (display_data->dot) {

		default:
		case APPADMM_DOT_NONE:
			digits = 0;
			unit_factor /= 1;
			break;

		case APPADMM_DOT_9999_9:
			digits = 1;
			unit_factor /= 10;
			break;

		case APPADMM_DOT_999_99:
			digits = 2;
			unit_factor /= 100;
			break;

		case APPADMM_DOT_99_999:
			digits = 3;
			unit_factor /= 1000;
			break;

		case APPADMM_DOT_9_9999:
			digits = 4;
			unit_factor /= 10000;
			break;

		}

		switch (display_data->data_content) {

		case APPADMM_DATA_CONTENT_MAXIMUM:
			analog->meaning->mqflags |= SR_MQFLAG_MAX;
			break;

		case APPADMM_DATA_CONTENT_MINIMUM:
			analog->meaning->mqflags |= SR_MQFLAG_MIN;
			break;

		case APPADMM_DATA_CONTENT_AVERAGE:
			analog->meaning->mqflags |= SR_MQFLAG_AVG;
			break;

		case APPADMM_DATA_CONTENT_PEAK_HOLD_MAX:
			analog->meaning->mqflags |= SR_MQFLAG_MAX;
			if (is_sub)
				analog->meaning->mqflags |= SR_MQFLAG_HOLD;
			break;

		case APPADMM_DATA_CONTENT_PEAK_HOLD_MIN:
			analog->meaning->mqflags |= SR_MQFLAG_MIN;
			if (is_sub)
				analog->meaning->mqflags |= SR_MQFLAG_HOLD;
			break;

		case APPADMM_DATA_CONTENT_AUTO_HOLD:
			if (is_sub)
				analog->meaning->mqflags |= SR_MQFLAG_HOLD;
			break;

		case APPADMM_DATA_CONTENT_HOLD:
			if (is_sub)
				analog->meaning->mqflags |= SR_MQFLAG_HOLD;
			break;

		case APPADMM_DATA_CONTENT_REL_DELTA:
		case APPADMM_DATA_CONTENT_REL_PERCENT:
			if (!is_sub)
				analog->meaning->mqflags |= SR_MQFLAG_RELATIVE;
			else
				analog->meaning->mqflags |= SR_MQFLAG_REFERENCE;
			break;

		default:
		case APPADMM_DATA_CONTENT_FREQUENCY:
		case APPADMM_DATA_CONTENT_CYCLE:
		case APPADMM_DATA_CONTENT_DUTY:
		case APPADMM_DATA_CONTENT_MEMORY_STAMP:
		case APPADMM_DATA_CONTENT_MEMORY_SAVE:
		case APPADMM_DATA_CONTENT_MEMORY_LOAD:
		case APPADMM_DATA_CONTENT_LOG_SAVE:
		case APPADMM_DATA_CONTENT_LOG_LOAD:
		case APPADMM_DATA_CONTENT_LOG_RATE:
		case APPADMM_DATA_CONTENT_REL_REFERENCE:
		case APPADMM_DATA_CONTENT_DBM:
		case APPADMM_DATA_CONTENT_DB:
		case APPADMM_DATA_CONTENT_SETUP:
		case APPADMM_DATA_CONTENT_LOG_STAMP:
		case APPADMM_DATA_CONTENT_LOG_MAX:
		case APPADMM_DATA_CONTENT_LOG_MIN:
		case APPADMM_DATA_CONTENT_LOG_TP:
		case APPADMM_DATA_CONTENT_CURRENT_OUTPUT:
		case APPADMM_DATA_CONTENT_CUR_OUT_0_20MA_PERCENT:
		case APPADMM_DATA_CONTENT_CUR_OUT_4_20MA_PERCENT:
			break;

		}

		if (display_response_data.auto_range == APPADMM_AUTO_RANGE) {

			analog->meaning->mqflags |= SR_MQFLAG_AUTORANGE;

		}

		switch (display_data->unit) {

		default: case APPADMM_UNIT_NONE:
			analog->meaning->unit = SR_UNIT_UNITLESS;
			break;

		case APPADMM_UNIT_MV:
			analog->meaning->unit = SR_UNIT_VOLT;
			analog->meaning->mq = SR_MQ_VOLTAGE;
			unit_factor /= 1000;
			digits += 3;
			break;

		case APPADMM_UNIT_V:
			analog->meaning->unit = SR_UNIT_VOLT;
			analog->meaning->mq = SR_MQ_VOLTAGE;
			break;

		case APPADMM_UNIT_UA:
			analog->meaning->unit = SR_UNIT_AMPERE;
			analog->meaning->mq = SR_MQ_CURRENT;
			unit_factor /= 1000000;
			digits += 6;
			break;
		case APPADMM_UNIT_MA:
			analog->meaning->unit = SR_UNIT_AMPERE;
			analog->meaning->mq = SR_MQ_CURRENT;
			unit_factor /= 1000;
			digits += 3;
			break;

		case APPADMM_UNIT_A:
			analog->meaning->unit = SR_UNIT_AMPERE;
			analog->meaning->mq = SR_MQ_CURRENT;
			break;

		case APPADMM_UNIT_DB:
			analog->meaning->unit = SR_UNIT_DECIBEL_VOLT;
			analog->meaning->mq = SR_MQ_POWER;
			break;

		case APPADMM_UNIT_DBM:
			analog->meaning->unit = SR_UNIT_DECIBEL_MW;
			analog->meaning->mq = SR_MQ_POWER;
			break;

		case APPADMM_UNIT_NF:
			analog->meaning->unit = SR_UNIT_FARAD;
			analog->meaning->mq = SR_MQ_CAPACITANCE;
			unit_factor /= 1000000000;
			digits += 9;
			break;

		case APPADMM_UNIT_UF:
			analog->meaning->unit = SR_UNIT_FARAD;
			analog->meaning->mq = SR_MQ_CAPACITANCE;
			unit_factor /= 1000000;
			digits += 6;
			break;

		case APPADMM_UNIT_MF:
			analog->meaning->unit = SR_UNIT_FARAD;
			analog->meaning->mq = SR_MQ_CAPACITANCE;
			unit_factor /= 1000;
			digits += 3;
			break;

		case APPADMM_UNIT_GOHM:
			analog->meaning->unit = SR_UNIT_OHM;
			analog->meaning->mq = SR_MQ_RESISTANCE;
			unit_factor *= 1000000000;
			digits -= 9;
			break;

		case APPADMM_UNIT_MOHM:
			analog->meaning->unit = SR_UNIT_OHM;
			analog->meaning->mq = SR_MQ_RESISTANCE;
			unit_factor *= 1000000;
			digits -= 6;
			break;

		case APPADMM_UNIT_KOHM:
			analog->meaning->unit = SR_UNIT_OHM;
			analog->meaning->mq = SR_MQ_RESISTANCE;
			unit_factor *= 1000;
			digits -= 3;
			break;

		case APPADMM_UNIT_OHM:
			analog->meaning->unit = SR_UNIT_OHM;
			analog->meaning->mq = SR_MQ_RESISTANCE;
			break;

		case APPADMM_UNIT_PERCENT:
			analog->meaning->unit = SR_UNIT_PERCENTAGE;
			analog->meaning->mq = SR_MQ_DIFFERENCE;
			break;

		case APPADMM_UNIT_MHZ:
			analog->meaning->unit = SR_UNIT_HERTZ;
			analog->meaning->mq = SR_MQ_FREQUENCY;
			unit_factor *= 1000000;
			digits -= 6;
			break;

		case APPADMM_UNIT_KHZ:
			analog->meaning->unit = SR_UNIT_HERTZ;
			analog->meaning->mq = SR_MQ_FREQUENCY;
			unit_factor *= 1000;
			digits -= 3;
			break;

		case APPADMM_UNIT_HZ:
			analog->meaning->unit = SR_UNIT_HERTZ;
			analog->meaning->mq = SR_MQ_FREQUENCY;
			break;

		case APPADMM_UNIT_DEGC:
			analog->meaning->unit = SR_UNIT_CELSIUS;
			analog->meaning->mq = SR_MQ_TEMPERATURE;
			break;

		case APPADMM_UNIT_DEGF:
			analog->meaning->unit = SR_UNIT_FAHRENHEIT;
			analog->meaning->mq = SR_MQ_TEMPERATURE;
			break;

		case APPADMM_UNIT_NS:
			analog->meaning->unit = SR_UNIT_SECOND;
			analog->meaning->mq = SR_MQ_TIME;
			unit_factor /= 1000000000;
			digits += 9;
			break;

		case APPADMM_UNIT_US:
			analog->meaning->unit = SR_UNIT_SECOND;
			analog->meaning->mq = SR_MQ_TIME;
			unit_factor /= 1000000;
			digits += 6;
			break;

		case APPADMM_UNIT_MS:
			analog->meaning->unit = SR_UNIT_SECOND;
			analog->meaning->mq = SR_MQ_TIME;
			unit_factor /= 1000;
			digits += 3;
			break;

		case APPADMM_UNIT_SEC:
			analog->meaning->unit = SR_UNIT_SECOND;
			analog->meaning->mq = SR_MQ_TIME;
			break;

		case APPADMM_UNIT_MIN:
			analog->meaning->unit = SR_UNIT_SECOND;
			analog->meaning->mq = SR_MQ_TIME;
			unit_factor *= 60;
			break;

		case APPADMM_UNIT_KW:
			analog->meaning->unit = SR_UNIT_WATT;
			analog->meaning->mq = SR_MQ_POWER;
			unit_factor *= 1000;
			digits -= 3;
			break;

		case APPADMM_UNIT_PF:
			analog->meaning->unit = SR_UNIT_UNITLESS;
			analog->meaning->mq = SR_MQ_POWER_FACTOR;
			break;

		}

		switch (display_response_data.function_code) {

		case APPADMM_FUNCTIONCODE_PEAK_HOLD_UA:
		case APPADMM_FUNCTIONCODE_AC_UA:
		case APPADMM_FUNCTIONCODE_AC_MV:
		case APPADMM_FUNCTIONCODE_AC_MA:
		case APPADMM_FUNCTIONCODE_LPF_MV:
		case APPADMM_FUNCTIONCODE_LPF_MA:
		case APPADMM_FUNCTIONCODE_AC_V:
		case APPADMM_FUNCTIONCODE_AC_A:
		case APPADMM_FUNCTIONCODE_LPF_V:
		case APPADMM_FUNCTIONCODE_LPF_A:
		case APPADMM_FUNCTIONCODE_LOZ_AC_V:
		case APPADMM_FUNCTIONCODE_AC_W:
		case APPADMM_FUNCTIONCODE_LOZ_LPF_V:
		case APPADMM_FUNCTIONCODE_V_HARM:
		case APPADMM_FUNCTIONCODE_INRUSH:
		case APPADMM_FUNCTIONCODE_A_HARM:
		case APPADMM_FUNCTIONCODE_FLEX_INRUSH:
		case APPADMM_FUNCTIONCODE_FLEX_A_HARM:
		case APPADMM_FUNCTIONCODE_AC_UA_HFR:
		case APPADMM_FUNCTIONCODE_AC_A_HFR:
		case APPADMM_FUNCTIONCODE_AC_MA_HFR:
		case APPADMM_FUNCTIONCODE_AC_UA_HFR2:
		case APPADMM_FUNCTIONCODE_AC_V_HFR:
		case APPADMM_FUNCTIONCODE_AC_MV_HFR:
		case APPADMM_FUNCTIONCODE_AC_V_PV:
		case APPADMM_FUNCTIONCODE_AC_V_PV_HFR:
			if (analog->meaning->unit == SR_UNIT_AMPERE
				|| analog->meaning->unit == SR_UNIT_VOLT
				|| analog->meaning->unit == SR_UNIT_WATT) {
				analog->meaning->mqflags |= SR_MQFLAG_AC;
				analog->meaning->mqflags |= SR_MQFLAG_RMS;
			}
			break;

		case APPADMM_FUNCTIONCODE_DC_UA:
		case APPADMM_FUNCTIONCODE_DC_MV:
		case APPADMM_FUNCTIONCODE_DC_MA:
		case APPADMM_FUNCTIONCODE_DC_V:
		case APPADMM_FUNCTIONCODE_DC_A:
		case APPADMM_FUNCTIONCODE_DC_A_OUT:
		case APPADMM_FUNCTIONCODE_DC_A_OUT_SLOW_LINEAR:
		case APPADMM_FUNCTIONCODE_DC_A_OUT_FAST_LINEAR:
		case APPADMM_FUNCTIONCODE_DC_A_OUT_SLOW_STEP:
		case APPADMM_FUNCTIONCODE_DC_A_OUT_FAST_STEP:
		case APPADMM_FUNCTIONCODE_LOOP_POWER:
		case APPADMM_FUNCTIONCODE_LOZ_DC_V:
		case APPADMM_FUNCTIONCODE_DC_W:
		case APPADMM_FUNCTIONCODE_FLEX_AC_A:
		case APPADMM_FUNCTIONCODE_FLEX_LPF_A:
		case APPADMM_FUNCTIONCODE_FLEX_PEAK_HOLD_A:
		case APPADMM_FUNCTIONCODE_DC_V_PV:
			analog->meaning->mqflags |= SR_MQFLAG_DC;
			break;

		case APPADMM_FUNCTIONCODE_CONTINUITY:
			analog->meaning->mq = SR_MQ_CONTINUITY;
			break;

		case APPADMM_FUNCTIONCODE_DIODE:
			analog->meaning->mqflags |= SR_MQFLAG_DIODE;
			analog->meaning->mqflags |= SR_MQFLAG_DC;
			break;

		case APPADMM_FUNCTIONCODE_AC_DC_MV:
		case APPADMM_FUNCTIONCODE_AC_DC_MA:
		case APPADMM_FUNCTIONCODE_AC_DC_V:
		case APPADMM_FUNCTIONCODE_AC_DC_A:
		case APPADMM_FUNCTIONCODE_VOLT_SENSE:
		case APPADMM_FUNCTIONCODE_LOZ_AC_DC_V:
		case APPADMM_FUNCTIONCODE_AC_DC_V_PV:
			if (analog->meaning->unit == SR_UNIT_AMPERE
				|| analog->meaning->unit == SR_UNIT_VOLT
				|| analog->meaning->unit == SR_UNIT_WATT) {
				analog->meaning->mqflags |= SR_MQFLAG_AC;
				analog->meaning->mqflags |= SR_MQFLAG_DC;
				analog->meaning->mqflags |= SR_MQFLAG_RMS;
			}
			break;

		/* Currently unused (usually implicitly handled with the unit */
		default:
		case APPADMM_FUNCTIONCODE_NONE:
		case APPADMM_FUNCTIONCODE_OHM:
		case APPADMM_FUNCTIONCODE_CAP:
		case APPADMM_FUNCTIONCODE_DEGC:
		case APPADMM_FUNCTIONCODE_DEGF:
		case APPADMM_FUNCTIONCODE_FREQUENCY:
		case APPADMM_FUNCTIONCODE_DUTY:
		case APPADMM_FUNCTIONCODE_HZ_V:
		case APPADMM_FUNCTIONCODE_HZ_MV:
		case APPADMM_FUNCTIONCODE_HZ_A:
		case APPADMM_FUNCTIONCODE_HZ_MA:
		case APPADMM_FUNCTIONCODE_250OHM_HART:
		case APPADMM_FUNCTIONCODE_LOZ_HZ_V:
		case APPADMM_FUNCTIONCODE_BATTERY:
		case APPADMM_FUNCTIONCODE_PF:
		case APPADMM_FUNCTIONCODE_FLEX_HZ_A:
		case APPADMM_FUNCTIONCODE_PEAK_HOLD_V:
		case APPADMM_FUNCTIONCODE_PEAK_HOLD_MV:
		case APPADMM_FUNCTIONCODE_PEAK_HOLD_A:
		case APPADMM_FUNCTIONCODE_PEAK_HOLD_MA:
		case APPADMM_FUNCTIONCODE_LOZ_PEAK_HOLD_V:
			break;
		}

		analog->spec->spec_digits = digits;
		analog->encoding->digits = digits;

		display_reading_value *= unit_factor;

		if (display_data->overload == APPADMM_OVERLOAD
			|| is_dash)
			*val = INFINITY;
		else
			*val = display_reading_value;


	} else {

		*val = INFINITY;

		switch (display_data->reading) {

		case APPADMM_WORDCODE_BATT:
		case APPADMM_WORDCODE_HAZ:
		case APPADMM_WORDCODE_FUSE:
		case APPADMM_WORDCODE_PROBE:
		case APPADMM_WORDCODE_ER:
		case APPADMM_WORDCODE_ER1:
		case APPADMM_WORDCODE_ER2:
		case APPADMM_WORDCODE_ER3:
			sr_err("ERROR [%s]: %s",
				sr_appadmm_channel_formats[info_local->ch_idx],
				appadmm_wordcode_name(display_data->reading));
			break;

		case APPADMM_WORDCODE_SPACE:
		case APPADMM_WORDCODE_DASH:
		case APPADMM_WORDCODE_DASH1:
		case APPADMM_WORDCODE_DASH2:
			/* No need for a message upon dash, space & co. */
			break;

		default:
			sr_warn("MESSAGE [%s]: %s",
				sr_appadmm_channel_formats[info_local->ch_idx],
				appadmm_wordcode_name(display_data->reading));
			break;

		case APPADMM_WORDCODE_DEF:
			/* Not beautiful but functional */
			if (display_data->unit == APPADMM_UNIT_DEGC)
				sr_warn("MESSAGE [%s]: %s °C",
					sr_appadmm_channel_formats[info_local->ch_idx],
					appadmm_wordcode_name(display_data->reading));

			else if (display_data->unit == APPADMM_UNIT_DEGF)
				sr_warn("MESSAGE [%s]: %s °F",
					sr_appadmm_channel_formats[info_local->ch_idx],
					appadmm_wordcode_name(display_data->reading));

			else
				sr_warn("MESSAGE [%s]: %s",
					sr_appadmm_channel_formats[info_local->ch_idx],
					appadmm_wordcode_name(display_data->reading));
			break;

		}
	}

	info_local->ch_idx++;

	return SR_OK;
}

/* ****************************************** */
/* ************ SIGROK-Interface ************ */
/* ****************************************** */

SR_PRIV int appa_dmm_receive_data(int fd, int revents, void *cb_data)
{
	const struct sr_dev_inst *sdi;
	struct dev_context *devc;

	(void)fd;

	if (!(sdi = cb_data))
		return TRUE;

	if (!(devc = sdi->priv))
		return TRUE;

	if (revents == G_IO_IN) {
		/* TODO */
	}

	return TRUE;
}
