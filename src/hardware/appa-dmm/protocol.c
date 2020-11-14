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

#include "packet.h"
#include "tables.h"

/* ********************** */
/* ****** Commands ****** */
/* ********************** */

SR_PRIV int appadmm_identify(const struct sr_dev_inst *arg_sdi)
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
 * @param arg_channel Current channel (primary, secondary, ...)
 * @param arg_data Data received with frame
 * @return SR_OK if successfull, otherwise SR_ERR_...
 */
static int appadmm_transform_display_data(const struct sr_dev_inst *arg_sdi,
	enum appadmm_channel_e arg_channel,
	const struct appadmm_display_data_s *arg_display_data,
	const struct appadmm_response_data_read_display_s *arg_read_data)
{
	struct sr_datafeed_packet packet;

	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_channel *channel;
	float val;

	int retr;

	struct appadmm_context *devc;
	gboolean is_dash;
	double unit_factor;
	double display_reading_value;
	int8_t digits;
	enum appadmm_functioncode_e function_code;

	if (arg_sdi == NULL
		|| arg_display_data == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	retr = sr_analog_init(&analog, &encoding, &meaning, &spec, 0);
	val = 0;

	if (retr < SR_OK)
		return retr;
	
	if (arg_read_data == NULL)
		function_code = arg_display_data->log_function_code;
	else
		function_code = arg_read_data->function_code;

	switch (arg_channel) {

	case APPADMM_CHANNEL_INVALID:
		sr_err("Invalid channel selected when transforming readings");
		return SR_ERR_BUG;
		
	case APPADMM_CHANNEL_DISPLAY_PRIMARY:
	case APPADMM_CHANNEL_DISPLAY_SECONDARY:

		unit_factor = 1;
		digits = 0;

		display_reading_value = (float) arg_display_data->reading;

		is_dash = appadmm_is_wordcode_dash(arg_display_data->reading);

		if (!appadmm_is_wordcode(arg_display_data->reading)
			|| is_dash) {

			switch (arg_display_data->dot) {

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

			switch (arg_display_data->data_content) {

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
				if (arg_channel == APPADMM_CHANNEL_DISPLAY_SECONDARY)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_PEAK_HOLD_MIN:
				analog.meaning->mqflags |= SR_MQFLAG_MIN;
				if (arg_channel == APPADMM_CHANNEL_DISPLAY_SECONDARY)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_AUTO_HOLD:
				if (arg_channel == APPADMM_CHANNEL_DISPLAY_SECONDARY)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_HOLD:
				if (arg_channel == APPADMM_CHANNEL_DISPLAY_SECONDARY)
					analog.meaning->mqflags |= SR_MQFLAG_HOLD;
				break;

			case APPADMM_DATA_CONTENT_REL_DELTA:
			case APPADMM_DATA_CONTENT_REL_PERCENT:
				if (arg_channel != APPADMM_CHANNEL_DISPLAY_SECONDARY)
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

			if (arg_read_data != NULL
				&& arg_read_data->auto_range == APPADMM_AUTO_RANGE)
				analog.meaning->mqflags |= SR_MQFLAG_AUTORANGE;

			switch (arg_display_data->unit) {

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

			switch (function_code) {

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

			if (arg_display_data->overload == APPADMM_OVERLOAD
				|| is_dash)
				val = INFINITY;
			else
				val = display_reading_value;


		} else {

			val = INFINITY;

			switch (arg_display_data->reading) {

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
					appadmm_wordcode_name(arg_display_data->reading));
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
					appadmm_wordcode_name(arg_display_data->reading));
				break;

			case APPADMM_WORDCODE_DEF:
				/* Not beautiful, but functional */
				if (arg_display_data->unit == APPADMM_UNIT_DEGC)
					sr_warn("MESSAGE [%s]: %s °C",
					appadmm_channel_name(arg_channel),
					appadmm_wordcode_name(arg_display_data->reading));

				else if (arg_display_data->unit == APPADMM_UNIT_DEGF)
					sr_warn("MESSAGE [%s]: %s °F",
					appadmm_channel_name(arg_channel),
					appadmm_wordcode_name(arg_display_data->reading));

				else
					sr_warn("MESSAGE [%s]: %s",
					appadmm_channel_name(arg_channel),
					appadmm_wordcode_name(arg_display_data->reading));
				break;

			}
		}


		break;
	}

	if (analog.meaning->mq == 0) {
		val = INFINITY;
		analog.meaning->unit = SR_UNIT_UNITLESS;
		analog.meaning->mq = SR_MQ_COUNT;
		analog.meaning->mqflags = 0;
		analog.encoding->digits = 0;
		analog.spec->spec_digits = 0;
	}
	channel = g_slist_nth_data(arg_sdi->channels, arg_channel);
	analog.meaning->channels = g_slist_append(NULL, channel);
	analog.num_samples = 1;
	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	analog.data = &val;
	analog.encoding->unitsize = sizeof(val);
	retr = sr_session_send(arg_sdi, &packet);
	sr_sw_limits_update_samples_read(&devc->limits, 1);
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
	struct sr_datafeed_packet packet;

	int retr;

	if (arg_sdi == NULL
		|| arg_data == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	retr = SR_OK;

	packet.type = SR_DF_FRAME_BEGIN;
	sr_session_send(arg_sdi, &packet);
	
	/* Primary reading */
	if (appadmm_cap_channel(devc->model_id, APPADMM_CHANNEL_DISPLAY_PRIMARY))
		retr = appadmm_transform_display_data(arg_sdi,
		APPADMM_CHANNEL_DISPLAY_PRIMARY, &arg_data->primary_display_data, arg_data);
	if (retr < SR_OK)
		return retr;

	/* Secondary reading */
	if (appadmm_cap_channel(devc->model_id, APPADMM_CHANNEL_DISPLAY_SECONDARY))
		retr = appadmm_transform_display_data(arg_sdi,
		APPADMM_CHANNEL_DISPLAY_SECONDARY, &arg_data->secondary_display_data, arg_data);
	if (retr < SR_OK)
		return retr;

	packet.type = SR_DF_FRAME_END;
	sr_session_send(arg_sdi, &packet);

	return retr;
}

static int appadmm_transform_sample_id(const struct sr_dev_inst *arg_sdi,
	enum appadmm_channel_e arg_channel)
{
	struct sr_datafeed_packet packet;

	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_channel *channel;
	float val;

	int retr;

	struct appadmm_context *devc;

	if (arg_sdi == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	retr = sr_analog_init(&analog, &encoding, &meaning, &spec, 0);
	val = 0;

	if (retr < SR_OK)
		return retr;

	val = (devc->limits.samples_read / 2 + 1);
	analog.encoding->digits = 0;
	analog.spec->spec_digits = 0;
	analog.meaning->mq = SR_MQ_COUNT;
	analog.meaning->unit = SR_UNIT_UNITLESS;

	if (analog.meaning->mq != 0) {
		channel = g_slist_nth_data(arg_sdi->channels, arg_channel);
		analog.meaning->channels = g_slist_append(NULL, channel);
		analog.num_samples = 1;
		packet.type = SR_DF_ANALOG;
		packet.payload = &analog;
		analog.data = &val;
		analog.encoding->unitsize = sizeof(val);
		retr = sr_session_send(arg_sdi, &packet);
		sr_sw_limits_update_samples_read(&devc->limits, 1);
	}

	return retr;

}

SR_PRIV int appadmm_storage_info(const struct sr_dev_inst *arg_sdi,
	struct appadmm_storage_info_s *arg_storage_info)
{
	struct appadmm_context *devc;
	
	int retr;
	
	struct appadmm_request_data_read_memory_s request;
	struct appadmm_response_data_read_memory_s response;

	retr = SR_OK;
	
	if (arg_sdi == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	
	request.device_number = 0;
	request.memory_address = 0xa;
	request.data_length = 6;
	
	if((retr = appadmm_rere_read_memory(&devc->appa_inst,
		&request, &response)) < SR_OK)
		return retr;
	
	if((retr = appadmm_dec_storage_info(&response, arg_storage_info)) < SR_OK)
		return retr;

	return retr;
}


static int appadmm_process_storage(const struct sr_dev_inst *arg_sdi,
	const struct appadmm_response_data_read_memory_s *arg_data)
{
	struct appadmm_context *devc;
	struct appadmm_display_data_s display_data[13]; 
	struct sr_datafeed_packet packet;
	enum appadmm_storage_e storage;

	int retr;
	int xloop;

	if (arg_sdi == NULL
		|| arg_data == NULL)
		return SR_ERR_ARG;

	devc = arg_sdi->priv;
	retr = SR_OK;
	
	switch (devc->data_source) {
	case APPADMM_DATA_SOURCE_MEM:
		storage = APPADMM_STORAGE_MEM;
		break;
	case APPADMM_DATA_SOURCE_LOG:
		storage = APPADMM_STORAGE_LOG;
		break;
	default:
		return SR_ERR_BUG;
	}
	
	if ((retr = appadmm_dec_read_storage(arg_data, &devc->storage_info[storage], display_data))
		< SR_OK)
		return retr;

	for (xloop = 0; xloop < arg_data->data_length / 5; xloop++) {
		
		packet.type = SR_DF_FRAME_BEGIN;
		sr_session_send(arg_sdi, &packet);
		
		/* Primary */
		if (appadmm_cap_channel(devc->model_id, APPADMM_CHANNEL_DISPLAY_PRIMARY))
			retr = appadmm_transform_display_data(arg_sdi,
			APPADMM_CHANNEL_DISPLAY_PRIMARY, &display_data[xloop], NULL);
		if (retr < SR_OK)
			return retr;

		/* Secondary (Reading number in Storage) */
		if (appadmm_cap_channel(devc->model_id, APPADMM_CHANNEL_DISPLAY_SECONDARY))
			retr = appadmm_transform_sample_id(arg_sdi,
			APPADMM_CHANNEL_DISPLAY_SECONDARY);
		if (retr < SR_OK)
			return retr;
		
		packet.type = SR_DF_FRAME_END;
		sr_session_send(arg_sdi, &packet);
		
		/* check for limits or stop request */
		if (sr_sw_limits_check(&devc->limits)) {
			return SR_OK;
		}

	}
	return retr;
}

SR_PRIV int appadmm_serial_receive_live(int arg_fd, int arg_revents,
	void *arg_cb_data)
{
	struct sr_dev_inst *sdi;
	struct appadmm_context *devc;
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

SR_PRIV int appadmm_serial_receive_storage(int arg_fd, int arg_revents,
	void *arg_cb_data)
{
	struct sr_dev_inst *sdi;
	struct appadmm_context *devc;
	struct appadmm_request_data_read_memory_s request;
	struct appadmm_response_data_read_memory_s response;
	enum appadmm_storage_e storage;
	
	gboolean abort;
	int retr;

	(void) arg_fd;

	abort = FALSE;
	
	if (!(sdi = arg_cb_data))
		return FALSE;
	if (!(devc = sdi->priv))
		return FALSE;
	
	switch (devc->data_source) {
	case APPADMM_DATA_SOURCE_MEM:
		storage = APPADMM_STORAGE_MEM;
		break;
	case APPADMM_DATA_SOURCE_LOG:
		storage = APPADMM_STORAGE_LOG;
		break;
	default:
		return SR_ERR_BUG;
	}

	/* Try to receive and process incoming data */
	if (arg_revents == G_IO_IN) {
		if ((retr = appadmm_response_read_memory(&devc->appa_inst,
			&response)) < SR_OK) {
			if (devc->error_counter++ > 10) {
				sr_warn("Aborted in appadmm_response_read_memory, result %d", retr);
				abort = TRUE;
			} else {
				devc->request_pending = FALSE;
			}
		} else if(retr > FALSE) {
			if (devc->error_counter > 0)
				devc->error_counter--;
			if (appadmm_process_storage(sdi, &response)
				< SR_OK) {
				sr_warn("Aborted in appadmm_process_storage, result %d",
					retr);
				abort = TRUE;
			}
			devc->request_pending = FALSE;
		}
	}

	/* if no request is pending, send out a new one */
	if (!devc->request_pending && !abort) {
		if ((retr = appadmm_enc_read_storage(&request,
			&devc->storage_info[storage],
			devc->limits.samples_read / 2, 0xff)) < SR_OK) {
			sr_warn("Aborted in appadmm_enc_read_storage");
			abort = TRUE;
		} else if ((retr = appadmm_request_read_memory(&devc->appa_inst,
			&request)) < TRUE) {
			sr_warn("Aborted in appadmm_request_read_memory");
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

	case APPADMM_CHANNEL_DISPLAY_PRIMARY:
		return TRUE;

	case APPADMM_CHANNEL_DISPLAY_SECONDARY:
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

	arg_devc->data_source = APPADMM_DATA_SOURCE_LIVE;

	sr_sw_limits_init(&arg_devc->limits);
	appadmm_clear_storage_info(arg_devc->storage_info);

	arg_devc->request_pending = FALSE;
	
	arg_devc->error_counter = 0;
	
	return SR_OK;
};

SR_PRIV int appadmm_clear_storage_info(struct appadmm_storage_info_s *arg_storage_info)
{
	int xloop;
	
	if (arg_storage_info == NULL)
		return SR_ERR_BUG;
	
	for (xloop = 0; xloop < APPADMM_STORAGE_INFO_COUNT; xloop++) {
		arg_storage_info[xloop].amount = 0;		
		arg_storage_info[xloop].rate = 0;
		arg_storage_info[xloop].entry_size = 0;
		arg_storage_info[xloop].entry_count = 0;
		arg_storage_info[xloop].mem_count = 0;
		arg_storage_info[xloop].mem_offset = 0;
	}
	
	return SR_OK;
}
