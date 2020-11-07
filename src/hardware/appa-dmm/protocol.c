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

static uint8_t appadmm_checksum(const uint8_t *arg_data, int arg_size)
{
	uint8_t checksum;

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

static int appadmm_frame_response_size(enum appadmm_command_e arg_command)
{
	switch(arg_command) {
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
		
	/* these respond with APPADMM_FRAME_DATA_SIZE_RESPONSE_SUCCESS or APPADMM_FRAME_DATA_SIZE_RESPONSE_FAILURE */
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

static int appadmm_is_request_frame_data_size_valid(enum appadmm_command_e arg_command, int arg_size)
{
	int size;
	
	if ((arg_command == APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_WRITE_MEMORY
		|| arg_command == APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_SEND_FIRMWARE_PACKAGE)
		&& arg_size > APPADMM_FRAME_MAX_DATA_SIZE)
		return SR_ERR_DATA;
	
	size = appadmm_frame_request_size(arg_command);
	
	if (size != arg_size
		|| size < 0)
		return SR_ERR_DATA;
	
	return SR_OK;
}

static int appadmm_is_response_frame_data_size_valid(enum appadmm_command_e arg_command, int arg_size)
{
	int size;
	
	if (arg_command == APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_MEMORY
		&& arg_size > APPADMM_FRAME_MAX_DATA_SIZE)
		return SR_ERR_DATA;
	
	size = appadmm_frame_response_size(arg_command);
	
	if (size != arg_size
		|| size < 0)
		return SR_ERR_DATA;
	
	return SR_OK;
}

SR_PRIV int appadmm_buffer_reset(struct dev_context *devc)
{
	devc->recv_buffer_len = 0;
	devc->recv_buffer[0] = 0;
	devc->recv_buffer[1] = 0;
	devc->recv_buffer[2] = 0;
	devc->recv_buffer[3] = 0;
}

SR_PRIV int appadmm_send(const struct sr_dev_inst *sdi, const struct appadmm_frame_s *arg_frame)
{
	struct dev_context *devc;
	struct sr_serial_dev_inst *serial;
	int retr;
	
	uint8_t buf[APPADMM_FRAME_MAX_SIZE];
	int len;
	
	devc = sdi->priv;
	serial = sdi->conn;
	
	retr = appadmm_frame_encode(arg_frame, buf, sizeof(buf), &len);
	if (retr < 0)
		return retr;
	
	retr = serial_write_blocking(serial, buf, len, APPADMM_WRITE_BLOCKING_TIMEOUT);
	
	return retr;
}

SR_PRIV int appadmm_process(const struct sr_dev_inst *sdi, const struct appadmm_frame_s *arg_frame) {
	switch (arg_frame->command) {
		
	case APPADMM_COMMAND_READ_DISPLAY:
		return appadmm_response_read_display(sdi, &arg_frame->response.read_display);
		
	case APPADMM_COMMAND_READ_INFORMATION:
	case APPADMM_COMMAND_READ_PROTOCOL_VERSION:
	case APPADMM_COMMAND_READ_BATTERY_LIFE:
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
		return SR_ERR_DATA;
	case APPADMM_COMMAND_WRITE_UART_CONFIGURATION:
	default:
		return SR_ERR_DATA;
		
	}
	return SR_ERR_BUG;
}

SR_PRIV int appadmm_receive(int fd, int revents, void *cb_data)
{
	const struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_serial_dev_inst *serial;
	
	uint8_t buf[APPADMM_FRAME_MAX_SIZE * 5];
	struct appadmm_frame_s frame;
	int len;
	int retr;
	int xloop;

	(void)fd;
	
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;

	if (!(sdi = cb_data))
		return TRUE;

	if (!(devc = sdi->priv))
		return TRUE;

	serial = sdi->conn;
	
	if (revents == G_IO_IN) {
		len = serial_read_nonblocking(serial, buf, sizeof(buf));
		if (len < 0)
			return len;
		for (xloop = 0; xloop < len; xloop++) {
			/* validate header */
			if (devc->recv_buffer_len < 1) {
				if (buf[xloop] != APPADMM_FRAME_START_VALUE_BYTE) {
					continue;
				}
			} else if (devc->recv_buffer_len < 2) {
				if (buf[xloop] != APPADMM_FRAME_START_VALUE_BYTE) {
					appadmm_buffer_reset(devc);
					continue;
				}
			} else if (devc->recv_buffer_len < 3) {
				if (appadmm_frame_response_size(buf[xloop]) > -1) {
					appadmm_buffer_reset(devc);
					continue;
				}
			} else if (devc->recv_buffer_len < 4) {
				if (appadmm_is_response_frame_data_size_valid(buf[xloop - 1], buf[xloop]) != SR_OK) {
					appadmm_buffer_reset(devc);
					continue;
				}
			}
			devc->recv_buffer[devc->recv_buffer_len++] = buf[xloop];
			if (devc->recv_buffer_len > 4) {
				if (devc->recv_buffer[3] == devc->recv_buffer_len) {
					retr = appadmm_frame_decode(devc->recv_buffer, devc->recv_buffer_len, &frame);
					
					if(retr == SR_OK) {
						retr = appadmm_process(sdi, &frame);
					}

					appadmm_buffer_reset(devc);
				}
			}
			if (devc->recv_buffer_len > APPADMM_FRAME_MAX_SIZE) {
				/* impossible! get out, garbage! */
				appadmm_buffer_reset(devc);
			}
		}
	}
	
	return SR_OK;
}

static int appadmm_frame_encode(const struct appadmm_frame_s *arg_frame, uint8_t *arg_out_data, int arg_size, int *arg_out_size)
{
	uint8_t *wrptr;
	
	int size;
	
	if (arg_frame == NULL
		|| arg_out_data == NULL
		|| arg_out_size == NULL) {
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
	
	*arg_out_size = size + APPADMM_FRAME_HEADER_SIZE + APPADMM_FRAME_CHECKSUM_SIZE;
	
	return SR_OK;
}

static int appadmm_frame_decode(const uint8_t *arg_data, int arg_size, struct appadmm_frame_s *arg_out_frame)
{
	const uint8_t *rdptr;
	uint8_t u8;
	uint16_t u16;
	
	int size;
	
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
		 /* allow overflow by 1 to kill bad frames */
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
	if (u8 != appadmm_checksum(arg_data, size + APPADMM_FRAME_HEADER_SIZE))
		return SR_ERR_DATA;
	
	/** @TODO dump btle extra bytes? */
	
	return SR_OK;
}


/* *************************************** */
/* ****** Internal Resolving tables ****** */
/* *************************************** */

static const char *appadmm_model_id_name(const enum appadmm_model_id_e arg_model_id)
{
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

static int appadmm_response_read_display(const struct sr_dev_inst *sdi, const struct appadmm_response_data_read_display_s *arg_data)
{
	struct sr_datafeed_packet packet;
	
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	double val;
	
	int retr;
	
	(void)val;
	
	sr_err("Received: %i", arg_data->main_display_data.reading);
	
	retr = SR_OK;
	
	retr = sr_analog_init(&analog, &encoding, &meaning, &spec, 0);
	
	if(retr != SR_OK)
		return retr;
	
	if (analog.meaning->mq != 0) {
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;
		sr_session_send(sdi, &packet);
	}
	
	
	return retr;
}

#if 0

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

#endif//1|0
