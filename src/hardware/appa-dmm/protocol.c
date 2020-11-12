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

#include <config.h>
#include "protocol.h"

#include <math.h>

/* ************************** */
/* ****** Declarations ****** */
/* ************************** */

/* ****** Packet handling ****** */
static int appadmm_enc_read_information(const struct appadmm_request_data_read_information_s *arg_data,
	struct sr_tp_appa_packet *arg_packet);
static int appadmm_dec_read_information(const struct sr_tp_appa_packet *arg_packet,
	struct appadmm_response_data_read_information_s *arg_data);
static int appadmm_rere_read_information(struct sr_tp_appa_inst* arg_tpai,
	const struct appadmm_request_data_read_information_s *arg_request,
	struct appadmm_response_data_read_information_s *arg_response);

static int appadmm_enc_read_display(const struct appadmm_request_data_read_display_s *arg_data,
	struct sr_tp_appa_packet *arg_packet);
static int appadmm_dec_read_display(const struct sr_tp_appa_packet *arg_packet,
	struct appadmm_response_data_read_display_s *arg_data);
static int appadmm_request_read_display(struct sr_tp_appa_inst* arg_tpai,
	const struct appadmm_request_data_read_display_s *arg_request);
static int appadmm_response_read_display(struct sr_tp_appa_inst* arg_tpai,
	struct appadmm_response_data_read_display_s *arg_response);

static int appadmm_enc_read_memory(const struct appadmm_request_data_read_memory_s *arg_data,
	struct sr_tp_appa_packet *arg_packet);
static int appadmm_dec_read_memory(const struct sr_tp_appa_packet *arg_packet,
	struct appadmm_response_data_read_memory_s *arg_data);
static int appadmm_rere_read_memory(struct sr_tp_appa_inst* arg_tpai,
	const struct appadmm_request_data_read_memory_s *arg_request,
	struct appadmm_response_data_read_memory_s *arg_response);

/* ****** UTIL: Transmission/Reception, Encoding/Decoding ****** */
static int appadmm_get_request_size(enum appadmm_command_e arg_command);
static int appadmm_get_response_size(enum appadmm_command_e arg_command);
static int appadmm_is_response_size_valid(enum appadmm_command_e arg_command,
	int arg_size);

/* ****** UTIL: Struct handling ****** */
/* appadmm_clear_context() */

/* ****** UTIL: Model capability handling ****** */
/* appadmm_cap_channel() */

/* ****** Resolvers / Tables ****** */
static int appadmm_is_wordcode(const int arg_wordcode);
static int appadmm_is_wordcode_dash(const int arg_wordcode);
/* appadmm_channel_name() */
/* appadmm_model_id_name() */
static const char *appadmm_wordcode_name(const enum appadmm_wordcode_e arg_wordcode);

/* ********************** */
/* ****** Commands ****** */
/* ********************** */

SR_PRIV int appadmm_read_information(const struct sr_dev_inst *arg_sdi)
{
	char *delim;
	struct sr_dev_inst *sdi_w;
	struct appadmm_context *devc;
	
	int retr;
	
	struct appadmm_request_data_read_information_s request;
	struct appadmm_response_data_read_information_s response;

	retr = SR_OK;
	
	if (arg_sdi == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	sdi_w = (struct sr_dev_inst*) arg_sdi;
	
	retr = appadmm_rere_read_information(&devc->appa_inst,
		&request, &response);
	
	if (retr < SR_OK)
		return retr;

	delim = NULL;

	/* Parse received model string and turn it into vendor/model combination */
	if (response.model_name[0] != 0)
		delim = g_strrstr(response.model_name, " ");

	if (delim == NULL) {
		sdi_w->vendor = g_strdup("APPA");
		sdi_w->model = g_strdup(response.model_name);
	} else {
		sdi_w->model = g_strdup(delim + 1);
		sdi_w->vendor = g_strndup(response.model_name,
			strlen(response.model_name) - strlen(arg_sdi->model) - 1);
	}

	/* make fancy version */
	sdi_w->version = g_strdup_printf("%01d.%02d",
		response.firmware_version / 100,
		response.firmware_version % 100);

	devc->model_id = response.model_id;

	sdi_w->serial_num = g_strdup(response.serial_number);

	return retr;
}

/**
 * Transform the data received in COMMAND_READ_DISPLAY
 * from each display into proper analog values, send it out
 * on the correct channel to the session.
 *
 * The parser tries to universally assign values from the different
 * APPA models to the analog meaning. Things that are not supported
 * by sigrok are silently ignored for now.
 *
 * Wordcodes will be printed out as sr_warn/sr_err messages. This way,
 * even the display text is visible when using sigrok-cli with --continuous
 * argument.
 *
 * @param arg_sdi Device Instance
 * @param arg_channel Current channel (main, sub, ...)
 * @param arg_data Data received with frame
 * @return SR_OK if successfull, otherwise SR_ERR_...
 */
static int appadmm_transform_display_data(const struct sr_dev_inst *arg_sdi,
	enum appadmm_channel_e arg_channel,
	const struct appadmm_response_data_read_display_s *arg_data)
{
	struct appadmm_context *devc;
	struct sr_datafeed_packet packet;

	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_channel *channel;
	float val;

	int retr;

	gboolean is_dash;
	double unit_factor;
	double display_reading_value;
	int8_t digits;
	const struct appadmm_display_data_s *display_data;

	if (arg_sdi == NULL
		|| arg_data == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	retr = sr_analog_init(&analog, &encoding, &meaning, &spec, 0);
	val = 0;

	if (retr < SR_OK)
		return retr;

	switch (arg_channel) {

	case APPADMM_CHANNEL_INVALID:
		sr_err("Invalid channel selected when transforming readings");
		return SR_ERR_BUG;
		
	case APPADMM_CHANNEL_SAMPLE_ID:
		val = (float) devc->sample_id;
		analog.encoding->digits = 0;
		analog.meaning->mq = SR_MQ_COUNT;
		analog.meaning->unit = SR_UNIT_UNITLESS;
		break;

	case APPADMM_CHANNEL_MAIN:
	case APPADMM_CHANNEL_SUB:

		if (arg_channel == APPADMM_CHANNEL_MAIN)
			display_data = &arg_data->main_display_data;
		else
			display_data = &arg_data->sub_display_data;

		unit_factor = 1;
		digits = 0;

		display_reading_value = (float) display_data->reading;

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
				analog.meaning->mqflags |= SR_MQFLAG_MAX;
				break;

			case APPADMM_DATA_CONTENT_MINIMUM:
				analog.meaning->mqflags |= SR_MQFLAG_MIN;
				break;

			case APPADMM_DATA_CONTENT_AVERAGE:
				analog.meaning->mqflags |= SR_MQFLAG_AVG;
				break;

			case APPADMM_DATA_CONTENT_PEAK_HOLD_MAX:
				analog.meaning->mqflags |= SR_MQFLAG_MAX;
				if (arg_channel == APPADMM_CHANNEL_SUB)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_PEAK_HOLD_MIN:
				analog.meaning->mqflags |= SR_MQFLAG_MIN;
				if (arg_channel == APPADMM_CHANNEL_SUB)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_AUTO_HOLD:
				if (arg_channel == APPADMM_CHANNEL_SUB)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_HOLD:
				if (arg_channel == APPADMM_CHANNEL_SUB)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_REL_DELTA:
			case APPADMM_DATA_CONTENT_REL_PERCENT:
				if (arg_channel != APPADMM_CHANNEL_SUB)
					analog.meaning->mqflags |= SR_MQFLAG_RELATIVE;
				else
					analog.meaning->mqflags |= SR_MQFLAG_REFERENCE;
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
				/* Currently unused,
				 * unit data provides enough information */
				break;

			}

			if (arg_data->auto_range == APPADMM_AUTO_RANGE)
				analog.meaning->mqflags |= SR_MQFLAG_AUTORANGE;

			switch (display_data->unit) {

			default: case APPADMM_UNIT_NONE:
				analog.meaning->unit = SR_UNIT_UNITLESS;
				break;

			case APPADMM_UNIT_MV:
				analog.meaning->unit = SR_UNIT_VOLT;
				analog.meaning->mq = SR_MQ_VOLTAGE;
				unit_factor /= 1000;
				digits += 3;
				break;

			case APPADMM_UNIT_V:
				analog.meaning->unit = SR_UNIT_VOLT;
				analog.meaning->mq = SR_MQ_VOLTAGE;
				break;

			case APPADMM_UNIT_UA:
				analog.meaning->unit = SR_UNIT_AMPERE;
				analog.meaning->mq = SR_MQ_CURRENT;
				unit_factor /= 1000000;
				digits += 6;
				break;
			case APPADMM_UNIT_MA:
				analog.meaning->unit = SR_UNIT_AMPERE;
				analog.meaning->mq = SR_MQ_CURRENT;
				unit_factor /= 1000;
				digits += 3;
				break;

			case APPADMM_UNIT_A:
				analog.meaning->unit = SR_UNIT_AMPERE;
				analog.meaning->mq = SR_MQ_CURRENT;
				break;

			case APPADMM_UNIT_DB:
				analog.meaning->unit = SR_UNIT_DECIBEL_VOLT;
				analog.meaning->mq = SR_MQ_POWER;
				break;

			case APPADMM_UNIT_DBM:
				analog.meaning->unit = SR_UNIT_DECIBEL_MW;
				analog.meaning->mq = SR_MQ_POWER;
				break;

			case APPADMM_UNIT_NF:
				analog.meaning->unit = SR_UNIT_FARAD;
				analog.meaning->mq = SR_MQ_CAPACITANCE;
				unit_factor /= 1000000000;
				digits += 9;
				break;

			case APPADMM_UNIT_UF:
				analog.meaning->unit = SR_UNIT_FARAD;
				analog.meaning->mq = SR_MQ_CAPACITANCE;
				unit_factor /= 1000000;
				digits += 6;
				break;

			case APPADMM_UNIT_MF:
				analog.meaning->unit = SR_UNIT_FARAD;
				analog.meaning->mq = SR_MQ_CAPACITANCE;
				unit_factor /= 1000;
				digits += 3;
				break;

			case APPADMM_UNIT_GOHM:
				analog.meaning->unit = SR_UNIT_OHM;
				analog.meaning->mq = SR_MQ_RESISTANCE;
				unit_factor *= 1000000000;
				digits -= 9;
				break;

			case APPADMM_UNIT_MOHM:
				analog.meaning->unit = SR_UNIT_OHM;
				analog.meaning->mq = SR_MQ_RESISTANCE;
				unit_factor *= 1000000;
				digits -= 6;
				break;

			case APPADMM_UNIT_KOHM:
				analog.meaning->unit = SR_UNIT_OHM;
				analog.meaning->mq = SR_MQ_RESISTANCE;
				unit_factor *= 1000;
				digits -= 3;
				break;

			case APPADMM_UNIT_OHM:
				analog.meaning->unit = SR_UNIT_OHM;
				analog.meaning->mq = SR_MQ_RESISTANCE;
				break;

			case APPADMM_UNIT_PERCENT:
				analog.meaning->unit = SR_UNIT_PERCENTAGE;
				analog.meaning->mq = SR_MQ_DIFFERENCE;
				break;

			case APPADMM_UNIT_MHZ:
				analog.meaning->unit = SR_UNIT_HERTZ;
				analog.meaning->mq = SR_MQ_FREQUENCY;
				unit_factor *= 1000000;
				digits -= 6;
				break;

			case APPADMM_UNIT_KHZ:
				analog.meaning->unit = SR_UNIT_HERTZ;
				analog.meaning->mq = SR_MQ_FREQUENCY;
				unit_factor *= 1000;
				digits -= 3;
				break;

			case APPADMM_UNIT_HZ:
				analog.meaning->unit = SR_UNIT_HERTZ;
				analog.meaning->mq = SR_MQ_FREQUENCY;
				break;

			case APPADMM_UNIT_DEGC:
				analog.meaning->unit = SR_UNIT_CELSIUS;
				analog.meaning->mq = SR_MQ_TEMPERATURE;
				break;

			case APPADMM_UNIT_DEGF:
				analog.meaning->unit = SR_UNIT_FAHRENHEIT;
				analog.meaning->mq = SR_MQ_TEMPERATURE;
				break;

			case APPADMM_UNIT_NS:
				analog.meaning->unit = SR_UNIT_SECOND;
				analog.meaning->mq = SR_MQ_TIME;
				unit_factor /= 1000000000;
				digits += 9;
				break;

			case APPADMM_UNIT_US:
				analog.meaning->unit = SR_UNIT_SECOND;
				analog.meaning->mq = SR_MQ_TIME;
				unit_factor /= 1000000;
				digits += 6;
				break;

			case APPADMM_UNIT_MS:
				analog.meaning->unit = SR_UNIT_SECOND;
				analog.meaning->mq = SR_MQ_TIME;
				unit_factor /= 1000;
				digits += 3;
				break;

			case APPADMM_UNIT_SEC:
				analog.meaning->unit = SR_UNIT_SECOND;
				analog.meaning->mq = SR_MQ_TIME;
				break;

			case APPADMM_UNIT_MIN:
				analog.meaning->unit = SR_UNIT_SECOND;
				analog.meaning->mq = SR_MQ_TIME;
				unit_factor *= 60;
				break;

			case APPADMM_UNIT_KW:
				analog.meaning->unit = SR_UNIT_WATT;
				analog.meaning->mq = SR_MQ_POWER;
				unit_factor *= 1000;
				digits -= 3;
				break;

			case APPADMM_UNIT_PF:
				analog.meaning->unit = SR_UNIT_UNITLESS;
				analog.meaning->mq = SR_MQ_POWER_FACTOR;
				break;

			}

			switch (arg_data->function_code) {

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
				if (analog.meaning->unit == SR_UNIT_AMPERE
					|| analog.meaning->unit == SR_UNIT_VOLT
					|| analog.meaning->unit == SR_UNIT_WATT) {
					analog.meaning->mqflags |= SR_MQFLAG_AC;
					analog.meaning->mqflags |= SR_MQFLAG_RMS;
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
				analog.meaning->mqflags |= SR_MQFLAG_DC;
				break;

			case APPADMM_FUNCTIONCODE_CONTINUITY:
				analog.meaning->mq = SR_MQ_CONTINUITY;
				break;

			case APPADMM_FUNCTIONCODE_DIODE:
				analog.meaning->mqflags |= SR_MQFLAG_DIODE;
				analog.meaning->mqflags |= SR_MQFLAG_DC;
				break;

			case APPADMM_FUNCTIONCODE_AC_DC_MV:
			case APPADMM_FUNCTIONCODE_AC_DC_MA:
			case APPADMM_FUNCTIONCODE_AC_DC_V:
			case APPADMM_FUNCTIONCODE_AC_DC_A:
			case APPADMM_FUNCTIONCODE_VOLT_SENSE:
			case APPADMM_FUNCTIONCODE_LOZ_AC_DC_V:
			case APPADMM_FUNCTIONCODE_AC_DC_V_PV:
				if (analog.meaning->unit == SR_UNIT_AMPERE
					|| analog.meaning->unit == SR_UNIT_VOLT
					|| analog.meaning->unit == SR_UNIT_WATT) {
					analog.meaning->mqflags |= SR_MQFLAG_AC;
					analog.meaning->mqflags |= SR_MQFLAG_DC;
					analog.meaning->mqflags |= SR_MQFLAG_RMS;
				}
				break;

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
				/* Currently unused,
				 * unit data provides enough information */
				break;
			}

			analog.spec->spec_digits = digits;
			analog.encoding->digits = digits;

			display_reading_value *= unit_factor;

			if (display_data->overload == APPADMM_OVERLOAD
				|| is_dash)
				val = INFINITY;
			else
				val = display_reading_value;


		} else {

			val = INFINITY;

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
					appadmm_channel_name(arg_channel),
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
					appadmm_channel_name(arg_channel),
					appadmm_wordcode_name(display_data->reading));
				break;

			case APPADMM_WORDCODE_DEF:
				/* Not beautiful, but functional */
				if (display_data->unit == APPADMM_UNIT_DEGC)
					sr_warn("MESSAGE [%s]: %s °C",
					appadmm_channel_name(arg_channel),
					appadmm_wordcode_name(display_data->reading));

				else if (display_data->unit == APPADMM_UNIT_DEGF)
					sr_warn("MESSAGE [%s]: %s °F",
					appadmm_channel_name(arg_channel),
					appadmm_wordcode_name(display_data->reading));

				else
					sr_warn("MESSAGE [%s]: %s",
					appadmm_channel_name(arg_channel),
					appadmm_wordcode_name(display_data->reading));
				break;

			}
		}


		break;
	}

	if (analog.meaning->mq != 0) {
		channel = g_slist_nth_data(arg_sdi->channels, arg_channel);
		analog.meaning->channels = g_slist_append(NULL, channel);
		analog.num_samples = 1;
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;
		analog.data = &val;
		analog.encoding->unitsize = sizeof(val);
		retr = sr_session_send(arg_sdi, &packet);
	}

	return retr;

}

/**
 * Process response to COMMAND_READ_DISPLAY
 * Contains the display readings, units, etc.
 * Data will be transformed into a analog values,
 * assigned to channels and transmitted in the session
 * by the invoked helper function appadmm_transform_display_data()
 *
 * @param arg_sdi Device Instance
 * @param arg_data Data received with frame
 * @return SR_OK if successfull, otherwise SR_ERR_...
 */
static int appadmm_process_read_display(const struct sr_dev_inst *arg_sdi,
	const struct appadmm_response_data_read_display_s *arg_data)
{
	struct appadmm_context *devc;
	struct sr_channel *channel;

	int retr;

	if (arg_sdi == NULL
		|| arg_data == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	retr = SR_OK;

	/* Sample ID */
	channel = g_slist_nth_data(arg_sdi->channels, APPADMM_CHANNEL_SAMPLE_ID);
	if (channel != NULL
		&& channel->enabled) {
		if (appadmm_cap_channel(devc->model_id, APPADMM_CHANNEL_SAMPLE_ID))
			retr = appadmm_transform_display_data(arg_sdi,
			APPADMM_CHANNEL_SAMPLE_ID, arg_data);
		if (retr < SR_OK)
			return retr;
	}

	/* Main reading */
	channel = g_slist_nth_data(arg_sdi->channels, APPADMM_CHANNEL_MAIN);
	if (channel != NULL 
		&& channel->enabled) {
		if (appadmm_cap_channel(devc->model_id, APPADMM_CHANNEL_MAIN))
			retr = appadmm_transform_display_data(arg_sdi,
			APPADMM_CHANNEL_MAIN, arg_data);
		if (retr < SR_OK)
			return retr;
	}

	/* Sub reading */
	channel = g_slist_nth_data(arg_sdi->channels, APPADMM_CHANNEL_SUB);
	if (channel != NULL
		&& channel->enabled) {
		if (appadmm_cap_channel(devc->model_id, APPADMM_CHANNEL_SUB))
			retr = appadmm_transform_display_data(arg_sdi,
			APPADMM_CHANNEL_SUB, arg_data);
		if (retr < SR_OK)
			return retr;
	}

	return retr;
}
#if 0
/**
 * Process response to COMMAND_READ_MEMORY
 * Used to read from EEPROM memory of the device
 *
 * @param arg_sdi Device Instance
 * @param arg_data Data received with frame
 * @return SR_OK if successfull, otherwise SR_ERR_...
 */
static int appadmm_process_read_memory(const struct sr_dev_inst *arg_sdi,
	const struct appadmm_response_data_read_memory_s *arg_data)
{
	struct appadmm_context *devc;

	int retr;

	if (arg_sdi == NULL
		|| arg_data == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;

	(void) arg_data->data;
	(void) arg_data->data_length;
	(void) devc;
	
	sr_err("%d %d %d %d",
		arg_data->data[0],
		arg_data->data[1],
		arg_data->data[2],
		arg_data->data[3]);

	retr = SR_OK;

	return retr;
}
#endif//1|0

SR_PRIV int appadmm_serial_receive(int arg_fd, int arg_revents,
	void *arg_cb_data)
{
	struct appadmm_context *devc;
	struct sr_dev_inst *sdi;
	struct appadmm_request_data_read_display_s request;
	struct appadmm_response_data_read_display_s response;
	gboolean abort;
	int retr;

	(void) arg_fd;

	abort = FALSE;
	
	if (!(sdi = arg_cb_data))
		return FALSE;
	if (!(devc = sdi->priv))
		return FALSE;

	/* Try to receive and process incoming data */
	if (arg_revents == G_IO_IN) {
		if ((retr = appadmm_response_read_display(&devc->appa_inst,
			&response)) < SR_OK) {
			sr_warn("Aborted in appadmm_receive, result %d", retr);
			abort = TRUE;
		} else if(retr > FALSE) {
			if (appadmm_process_read_display(sdi, &response)
				< SR_OK) {
				abort = TRUE;
			}
			devc->request_pending = FALSE;
		}
	}

	/* if no request is pending, send out a new one */
	if (!devc->request_pending) {
		if (appadmm_request_read_display(&devc->appa_inst, &request)
			< TRUE) {
			sr_warn("Aborted in appadmm_send");
			abort = TRUE;
		} else {
			devc->request_pending = TRUE;
		}
	}

	/* check for limits or stop request */
	if (sr_sw_limits_check(&devc->limits)
		|| abort == TRUE) {
		sr_info("Stopping acquisition");
		sr_dev_acquisition_stop(sdi);
		return FALSE;
	}

	return TRUE;
}

/* ********************************* */
/* ****** Encoding / decoding ****** */
/* ********************************* */
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

/* *********************************** */
/* ****** UTIL: Struct handling ****** */
/* *********************************** */

/**
 * Initialize Device context
 * the structure contains the state machine and non-standard ID data
 * for the device.
 *
 * @param arg_devc Context
 * @return SR_OK on success, SR_ERR_... on failure
 */
SR_PRIV int appadmm_clear_context(struct appadmm_context *arg_devc)
{
	if (arg_devc == NULL)
		return SR_ERR_BUG;

	arg_devc->model_id = APPADMM_MODEL_ID_INVALID;

	arg_devc->connection_type = APPADMM_CONNECTION_TYPE_INVALID;
	arg_devc->data_source = APPADMM_DATA_SOURCE_LIVE;
	arg_devc->command_received = APPADMM_COMMAND_INVALID;

	arg_devc->protocol_id = APPADMM_PROTOCOL_ID_INVALID;
	arg_devc->major_protocol_version = 0;
	arg_devc->minor_protocol_version = 0;

	sr_sw_limits_init(&arg_devc->limits);

	arg_devc->request_pending = FALSE;

	arg_devc->sample_id = 0;
	
	return SR_OK;
};

/* ********************************************* */
/* ****** UTIL: Model capability handling ****** */
/* ********************************************* */

/**
 * Test channel capability of model
 *
 * @param arg_model_id APPA Model-ID
 * @param arg_channel Channel to check
 * @return TRUE/FALE: channel supported, SR_ERR_... on error
 */
SR_PRIV int appadmm_cap_channel(const enum appadmm_model_id_e arg_model_id,
	const enum appadmm_channel_e arg_channel)
{
	switch (arg_channel) {
	case APPADMM_CHANNEL_INVALID:
	default:
		return FALSE;

	case APPADMM_CHANNEL_SAMPLE_ID:
		return TRUE;

	case APPADMM_CHANNEL_MAIN:
		return TRUE;

	case APPADMM_CHANNEL_SUB:
		switch (arg_model_id) {
		default:
		case APPADMM_MODEL_ID_INVALID:
			return SR_ERR_NA;

		case APPADMM_MODEL_ID_208:
		case APPADMM_MODEL_ID_208B:
		case APPADMM_MODEL_ID_501:
		case APPADMM_MODEL_ID_502:
		case APPADMM_MODEL_ID_503:
		case APPADMM_MODEL_ID_505:
		case APPADMM_MODEL_ID_506:
		case APPADMM_MODEL_ID_506B:
		case APPADMM_MODEL_ID_506B_2:
			return TRUE;

		case APPADMM_MODEL_ID_150:
		case APPADMM_MODEL_ID_150B:
		case APPADMM_MODEL_ID_172:
		case APPADMM_MODEL_ID_173:
		case APPADMM_MODEL_ID_175:
		case APPADMM_MODEL_ID_177:
		case APPADMM_MODEL_ID_179:
		case APPADMM_MODEL_ID_SFLEX_10A:
		case APPADMM_MODEL_ID_SFLEX_18A:
		case APPADMM_MODEL_ID_A17N:
		case APPADMM_MODEL_ID_S0:
		case APPADMM_MODEL_ID_S1:
		case APPADMM_MODEL_ID_S2:
		case APPADMM_MODEL_ID_S3:
			return FALSE;
		}
		break;

	}
	return SR_ERR_BUG;
}

/* ******************************** */
/* ****** Resolvers / Tables ****** */
/* ******************************** */

/**
 * Test, if a received display reading is a wordcode
 * (e.g. text on display)
 *
 * @param arg_wordcode Raw reading
 * @return TRUE if it is a wordcode
 */
static int appadmm_is_wordcode(const int arg_wordcode)
{
	return arg_wordcode >= APPADMM_WORDCODE_TABLE_MIN;
}

/**
 * Test, if a received display reading is dash wordcode
 * (e.v. active channel with currently no available reading)
 *
 * @param arg_wordcode Raw reading
 * @return TRUE if it is a dash-reading
 */
static int appadmm_is_wordcode_dash(const int arg_wordcode)
{
	return
	arg_wordcode == APPADMM_WORDCODE_DASH
		|| arg_wordcode == APPADMM_WORDCODE_DASH1
		|| arg_wordcode == APPADMM_WORDCODE_DASH2;
}

/**
 * Get name as string of channel
 *
 * @param arg_channel Channel
 * @return Channel-Name
 */
SR_PRIV const char *appadmm_channel_name(const enum appadmm_channel_e arg_channel)
{
	switch (arg_channel) {
	case APPADMM_CHANNEL_INVALID:
		return APPADMM_STRING_NA;
	case APPADMM_CHANNEL_SAMPLE_ID:
		return "sample_id";
	case APPADMM_CHANNEL_MAIN:
		return "main";
	case APPADMM_CHANNEL_SUB:
		return "sub";
	}

	return APPADMM_STRING_NA;
}

/**
 * Get name as string of model id
 *
 * @param arg_model_id Model-ID
 * @return Model Name
 */
SR_PRIV const char *appadmm_model_id_name(const enum appadmm_model_id_e arg_model_id)
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

/**
 * Get Text representation of wordcode
 *
 * @param arg_wordcode Raw display reading
 * @return Display text as string
 */
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
