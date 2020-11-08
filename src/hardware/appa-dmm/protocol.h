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

#ifndef LIBSIGROK_HARDWARE_APPA_DMM_PROTOCOL_H
#define LIBSIGROK_HARDWARE_APPA_DMM_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "appa-dmm"

#define APPADMM_PACKET_SIZE 17
 
#define APPPA_B_GENERAL_TIMEOUT 5000

#define APPADMM_GENERAL_DELAY 5

#define APPADMM_DISPLAY_COUNT 2

#define APPADMM_CONF_SERIAL "9600/8n1"

#define APPADMM_WRITE_BLOCKING_TIMEOUT 5

#define APPADMM_STRING_NA "N/A"

#define APPADMM_READING_TEXT_OL "OL"

#define APPADMM_CHANNEL_NAME_DISPLAY_MAIN "main"

#define APPADMM_CHANNEL_NAME_DISPLAY_SUB "sub"

#define APPADMM_FRAME_HEADER_SIZE 4
#define APPADMM_FRAME_CHECKSUM_SIZE 1
#define APPADMM_FRAME_MAX_DATA_SIZE 64
#define APPADMM_FRAME_MAX_SIZE (APPADMM_FRAME_MAX_DATA_SIZE+APPADMM_FRAME_HEADER_SIZE+APPADMM_FRAME_CHECKSUM_SIZE)

#define APPADMM_FRAME_DATA_SIZE_REQUEST_READ_INFORMATION 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_READ_DISPLAY 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_READ_PROTOCOL_VERSION 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_READ_BATTERY_LIFE 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_WRITE_UART_CONFIGURATION 1
#define APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_READING 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_READ_MEMORY 4
#define APPADMM_FRAME_DATA_SIZE_REQUEST_READ_HARMONICS_DATA 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_ENTER 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_WRITE_FUNCTION_CODE 1
#define APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_WRITE_RANGE_CODE 1
#define APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_WRITE_MEMORY 64 /* variable (max) */
#define APPADMM_FRAME_DATA_SIZE_REQUEST_CAL_EXIT 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_ENTER 0
#define APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_SEND_INFORMATION 13
#define APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_SEND_FIRMWARE_PACKAGE 64 /* variable (max) */
#define APPADMM_FRAME_DATA_SIZE_REQUEST_OTA_START_UPGRADE_PROCEDURE 1

#define APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_INFORMATION 52
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_DISPLAY 12
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_PROTOCOL_VERSION 4
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_BATTERY_LIFE 4
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_CAL_READING 23
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_MEMORY 64 /* variable (max) */
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_READ_HARMONICS_DATA 50
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_FAILURE 1
#define APPADMM_FRAME_DATA_SIZE_RESPONSE_SUCCESS 0

/**
 * Begin of word codes (minimum value)
 * All readings on a display higher than that are some sort of wordcode, resolvable or not
 */
#define APPADMM_WORDCODE_TABLE_MIN 0x700000

/**
 * Start code of valid frame
 */
#define APPADMM_FRAME_START_VALUE 0x5555

/**
 * Start code of valid frame byte part
 */
#define APPADMM_FRAME_START_VALUE_BYTE 0x55

/* ************************** */
/* ****** Enumerations ****** */
/* ************************** */

enum appadmm_connection_type_e {
	APPADMM_CONNECTION_TYPE_INVALID = 0x00,
	APPADMM_CONNECTION_TYPE_SERIAL = 0x01,
	APPADMM_CONNECTION_TYPE_BLE = 0x02,
};

enum appadmm_frame_type_e {
	APPADMM_FRAME_TYPE_INVALID = 0x00,
	APPADMM_FRAME_TYPE_REQUEST = 0x01,
	APPADMM_FRAME_TYPE_RESPONSE = 0x02,
};

/**
 * Possible commands.
 * Calibration and configuration commands not included yet.
 */
enum appadmm_command_e {
	APPADMM_COMMAND_READ_INFORMATION = 0x00, /**< Get information about Model and Brand */
	APPADMM_COMMAND_READ_DISPLAY = 0x01, /**< Get all display readings */
	APPADMM_COMMAND_READ_PROTOCOL_VERSION = 0x03, /**< Read protocol version */
	APPADMM_COMMAND_READ_BATTERY_LIFE = 0x04, /**< Read battery life */
	APPADMM_COMMAND_WRITE_UART_CONFIGURATION = 0x05, /**< Configure UART Interface */
	APPADMM_COMMAND_CAL_READING = 0x10, /**< Read calibration-related reading data */
	APPADMM_COMMAND_READ_MEMORY = 0x1a, /**< Read memory (MEM, LOG, etc.) */
	APPADMM_COMMAND_READ_HARMONICS_DATA = 0x1b, /**< Read harmonics data of clamps */
	APPADMM_COMMAND_FAILURE = 0x70, /**< Slave did not accept last command */
	APPADMM_COMMAND_SUCCESS = 0x7f, /**< Slave accepted last command */
	APPADMM_COMMAND_CAL_ENTER = 0x80, /**< Enter calibration mode */
	APPADMM_COMMAND_CAL_WRITE_FUNCTION_CODE = 0x85, /**< Write calibration function code */
	APPADMM_COMMAND_CAL_WRITE_RANGE_CODE = 0x87, /**< Write calibration range code */
	APPADMM_COMMAND_CAL_WRITE_MEMORY = 0x8a, /**< Write memory */
	APPADMM_COMMAND_CAL_EXIT = 0x8f, /**< Exit calibration mode */
	APPADMM_COMMAND_OTA_ENTER = 0xa0, /**< Enter OTA mode */
	APPADMM_COMMAND_OTA_SEND_INFORMATION = 0xa1, /**< Send OTA information */
	APPADMM_COMMAND_OTA_SEND_FIRMWARE_PACKAGE = 0xa2, /**< Send OTA Firmware package */
	APPADMM_COMMAND_OTA_START_UPGRADE_PROCEDURE = 0xa3, /**< Start Upgrade-Procedure */
};

/**
 * Currently supported models
 */
enum appadmm_model_id_e {

	/**
	 * Invalid
	 */
	APPADMM_MODEL_ID_INVALID = 0x00,

	/**
	 * APPA 150 Series
	 */
	APPADMM_MODEL_ID_150 = 0x01,

	/**
	 * APPA 150 Series (BLE)
	 * APPA 155B, 156B, 157B, 158B
	 * BENNING CM 12
	 */
	APPADMM_MODEL_ID_150B = 0x02,

	/**
	 * APPA 200 Series (Optical RS232/USB)
	 * APPA 208
	 */
	APPADMM_MODEL_ID_208 = 0x03,

	/**
	 * APPA 200 Series (Optical RS232/USB, BLE)
	 * APPA 208B
	 */
	APPADMM_MODEL_ID_208B = 0x04,

	/**
	 * APPA 500 Series (Optical RS232/USB)
	 * APPA 506
	 * Sefram 7351
	 */
	APPADMM_MODEL_ID_506 = 0x05,

	/**
	 * APPA 500 Series (Optical RS232/USB, BLE)
	 * APPA 506B
	 * BENNING MM 12
	 * Sefram 7352B
	 */
	APPADMM_MODEL_ID_506B = 0x06,

	/**
	 * Same as APPADMM_MODEL_ID_506B
	 */
	APPADMM_MODEL_ID_506B_2 = 0x600,

	/**
	 * APPA 500 Series (Optical RS232/USB)
	 * APPA 501
	 */
	APPADMM_MODEL_ID_501 = 0x07,

	/**
	 * APPA 500 Series (Optical RS232/USB)
	 * APPA 502
	 */
	APPADMM_MODEL_ID_502 = 0x08,

	/**
	 * APPA S Series (BLE)
	 * APPA S1
	 * RS PRO S1
	 */
	APPADMM_MODEL_ID_S1 = 0x09,

	/**
	 * APPA S Series (BLE)
	 * APPA S2
	 * BENNING MM 10-1
	 * RS PRO S2
	 */
	APPADMM_MODEL_ID_S2 = 0x0a,

	/**
	 * APPA S Series (BLE)
	 * APPA S3
	 * BENNING MM 10-PV
	 * RS PRO S3
	 */
	APPADMM_MODEL_ID_S3 = 0x0b,

	/**
	 * APPA 170 Series (BLE)
	 * APPA 172B
	 * BENNING CM 9-2
	 */
	APPADMM_MODEL_ID_172 = 0x0c,

	/**
	 * APPA 170 Series (BLE)
	 * APPA 173B
	 * BENNING CM 10-1
	 */
	APPADMM_MODEL_ID_173 = 0x0d,

	/**
	 * APPA 170 Series (BLE)
	 * APPA 175B
	 */
	APPADMM_MODEL_ID_175 = 0x0e,

	/**
	 * APPA 170 Series (BLE)
	 * APPA 177B
	 * BENNING CM 10-PV
	 */
	APPADMM_MODEL_ID_177 = 0x0f,

	/**
	 * APPA sFlex Series (BLE)
	 * APPA sFlex-10A
	 */
	APPADMM_MODEL_ID_SFLEX_10A = 0x10,

	/**
	 * APPA sFlex Series (BLE)
	 * APPA sFlex-18A
	 */
	APPADMM_MODEL_ID_SFLEX_18A = 0x11,

	/**
	 * APPA A Series (BLE)
	 * APPA A17N
	 */
	APPADMM_MODEL_ID_A17N = 0x12,

	/**
	 * APPA S Series (BLE)
	 * APPA S0
	 */
	APPADMM_MODEL_ID_S0 = 0x13,

	/**
	 * APPA 170 Series (BLE)
	 * APPA 179B
	 */
	APPADMM_MODEL_ID_179 = 0x14,

	/**
	 * APPA 500 Series (Optical RS232/USB)
	 * APPA 503
	 * CMT 3503
	 * Voltcraft VC-930
	 * ISO-TECH IDM503
	 */
	APPADMM_MODEL_ID_503 = 0x15,

	/**
	 * APPA 500 Series (Optical RS232/USB)
	 * APPA 505
	 * RS PRO IDM505
	 * Sefram 7355
	 * Voltcraft VC-950
	 */
	APPADMM_MODEL_ID_505 = 0x16,

	/**
	 * Unlisted / Unknown:
	 *
	 * APPA 500 Series (Optical RS232/USB) - EXPERIMENTAL
	 * APPA 507
	 * CMT 3507
	 * HT Instruments HT8100
	 */
};


/**
 * Manual / Auto range field values
 */
enum appadmm_autorange_e {
	APPADMM_MANUAL_RANGE = 0x00, /**< Manual ranging */
	APPADMM_AUTO_RANGE = 0x01, /**< Auto range active */
};

/**
 * Manual / Auto test field values
 */
enum appadmm_autotest_e {
	APPADMM_MANUAL_TEST = 0x00, /**< Manual Test */
	APPADMM_AUTO_TEST = 0x01, /**< Auto Test */
};

/**
 * Wordcodes
 *
 * Multimeter will send these codes to indicate a string visible on the
 * display. Works for main and sub.
 */
enum appadmm_wordcode_e {
	APPADMM_WORDCODE_SPACE = 0x700000, /**< Space */
	APPADMM_WORDCODE_FULL = 0x700001, /**< Full */
	APPADMM_WORDCODE_BEEP = 0x700002, /**< Beep */
	APPADMM_WORDCODE_APO = 0x700003, /**< Auto Power-Off */
	APPADMM_WORDCODE_B_LIT = 0x700004, /**< Backlight */
	APPADMM_WORDCODE_HAZ = 0x700005, /**< Hazard */
	APPADMM_WORDCODE_ON = 0x700006, /**< On */
	APPADMM_WORDCODE_OFF = 0x700007, /**< Off */
	APPADMM_WORDCODE_RESET = 0x700008, /**< Reset */
	APPADMM_WORDCODE_START = 0x700009, /**< Start */
	APPADMM_WORDCODE_VIEW = 0x70000a, /**< View */
	APPADMM_WORDCODE_PAUSE = 0x70000b, /**< Pause */
	APPADMM_WORDCODE_FUSE = 0x70000c, /**< Fuse */
	APPADMM_WORDCODE_PROBE = 0x70000d, /**< Probe */
	APPADMM_WORDCODE_DEF = 0x70000e, /**< Definition */
	APPADMM_WORDCODE_CLR = 0x70000f, /**< Clr */
	APPADMM_WORDCODE_ER = 0x700010, /**< Er */
	APPADMM_WORDCODE_ER1 = 0x700011, /**< Er1 */
	APPADMM_WORDCODE_ER2 = 0x700012, /**< Er2 */
	APPADMM_WORDCODE_ER3 = 0x700013, /**< Er3 */
	APPADMM_WORDCODE_DASH = 0x700014, /**< Dash (-----) */
	APPADMM_WORDCODE_DASH1 = 0x700015, /**< Dash1  (-) */
	APPADMM_WORDCODE_TEST = 0x700016, /**< Test */
	APPADMM_WORDCODE_DASH2 = 0x700017, /**< Dash2 (--) */
	APPADMM_WORDCODE_BATT = 0x700018, /**< Battery */
	APPADMM_WORDCODE_DISLT = 0x700019, /**< diSLt */
	APPADMM_WORDCODE_NOISE = 0x70001a, /**< Noise */
	APPADMM_WORDCODE_FILTR = 0x70001b, /**< Filter */
	APPADMM_WORDCODE_PASS = 0x70001c, /**< PASS */
	APPADMM_WORDCODE_NULL = 0x70001d, /**< null */
	APPADMM_WORDCODE_0_20 = 0x70001e, /**< 0 - 20 mA */
	APPADMM_WORDCODE_4_20 = 0x70001f, /**< 4 - 20 mA */
	APPADMM_WORDCODE_RATE = 0x700020, /**< Rate */
	APPADMM_WORDCODE_SAVE = 0x700021, /**< Save */
	APPADMM_WORDCODE_LOAD = 0x700022, /**< Load */
	APPADMM_WORDCODE_YES = 0x700023, /**< Yes */
	APPADMM_WORDCODE_SEND = 0x700024, /**< Send */
	APPADMM_WORDCODE_AHOLD = 0x700025, /**< AUTO HOLD */
	APPADMM_WORDCODE_AUTO = 0x700026, /**< AUTO */
	APPADMM_WORDCODE_CNTIN = 0x700027, /**< Continuity */
	APPADMM_WORDCODE_CAL = 0x700028, /**< CAL */
	APPADMM_WORDCODE_VERSION = 0x700029, /**< Version */
	APPADMM_WORDCODE_OL = 0x70002a, /**< OL (unused) */
	APPADMM_WORDCODE_BAT_FULL = 0x70002b, /**< Battery Full */
	APPADMM_WORDCODE_BAT_HALF = 0x70002c, /**< Battery Half */
	APPADMM_WORDCODE_LO = 0x70002d, /**< Lo */
	APPADMM_WORDCODE_HI = 0x70002e, /**< Hi */
	APPADMM_WORDCODE_DIGIT = 0x70002f, /**< Digits */
	APPADMM_WORDCODE_RDY = 0x700030, /**< Ready */
	APPADMM_WORDCODE_DISC = 0x700031, /**< dISC */
	APPADMM_WORDCODE_OUTF = 0x700032, /**< outF */
	APPADMM_WORDCODE_OLA = 0x700033, /**< OLA */
	APPADMM_WORDCODE_OLV = 0x700034, /**< OLV */
	APPADMM_WORDCODE_OLVA = 0x700035, /**< OLVA */
	APPADMM_WORDCODE_BAD = 0x700036, /**< BAD */
	APPADMM_WORDCODE_TEMP = 0x700037, /**< TEMP */
};

/**
 * Data units
 */
enum appadmm_unit_e {
	APPADMM_UNIT_NONE = 0x00, /**< None */
	APPADMM_UNIT_V = 0x01, /**< V */
	APPADMM_UNIT_MV = 0x02, /**< mV */
	APPADMM_UNIT_A = 0x03, /**< A */
	APPADMM_UNIT_MA = 0x04, /**< mA */
	APPADMM_UNIT_DB = 0x05, /**< dB */
	APPADMM_UNIT_DBM = 0x06, /**< dBm */
	APPADMM_UNIT_MF = 0x07, /**< mF */
	APPADMM_UNIT_UF = 0x08, /**< µF */
	APPADMM_UNIT_NF = 0x09, /**< nF */
	APPADMM_UNIT_GOHM = 0x0a, /**< GΩ */
	APPADMM_UNIT_MOHM = 0x0b, /**< MΩ */
	APPADMM_UNIT_KOHM = 0x0c, /**< kΩ */
	APPADMM_UNIT_OHM = 0x0d, /**< Ω */
	APPADMM_UNIT_PERCENT = 0x0e, /**< Relative percentage value */
	APPADMM_UNIT_MHZ = 0x0f, /**< MHz */
	APPADMM_UNIT_KHZ = 0x10, /**< kHz */
	APPADMM_UNIT_HZ = 0x11, /**< Hz */
	APPADMM_UNIT_DEGC = 0x12, /**< °C */
	APPADMM_UNIT_DEGF = 0x13, /**< °F */
	APPADMM_UNIT_SEC = 0x14, /**< seconds */
	APPADMM_UNIT_MS = 0x15, /**< ms */
	APPADMM_UNIT_US = 0x16, /**< µs */
	APPADMM_UNIT_NS = 0x17, /**< ns */
	APPADMM_UNIT_UA = 0x18, /**< µA */
	APPADMM_UNIT_MIN = 0x19, /**< minutes */
	APPADMM_UNIT_KW = 0x1a, /**< kW */
	APPADMM_UNIT_PF = 0x1b, /**< Power Factor (@TODO maybe pico-farat?) */
};

/**
 * Display range / dot positions
 */
enum appadmm_dot_e {
	APPADMM_DOT_NONE = 0x00,
	APPADMM_DOT_9999_9 = 0x01,
	APPADMM_DOT_999_99 = 0x02,
	APPADMM_DOT_99_999 = 0x03,
	APPADMM_DOT_9_9999 = 0x04,

};

/**
 * OL-Indication values
 */
enum appadmm_overload_e {
	APPADMM_NOT_OVERLOAD = 0x00, /**< non-OL value */
	APPADMM_OVERLOAD = 0x01, /**< OL */
};

/**
 * Data content - Menu, Min/Max/Avg, etc. selection
 */
enum appadmm_data_content_e {
	APPADMM_DATA_CONTENT_MEASURING_DATA = 0x00,
	APPADMM_DATA_CONTENT_FREQUENCY = 0x01,
	APPADMM_DATA_CONTENT_CYCLE = 0x02,
	APPADMM_DATA_CONTENT_DUTY = 0x03,
	APPADMM_DATA_CONTENT_MEMORY_STAMP = 0x04,
	APPADMM_DATA_CONTENT_MEMORY_SAVE = 0x05,
	APPADMM_DATA_CONTENT_MEMORY_LOAD = 0x06,
	APPADMM_DATA_CONTENT_LOG_SAVE = 0x07,
	APPADMM_DATA_CONTENT_LOG_LOAD = 0x08,
	APPADMM_DATA_CONTENT_LOG_RATE = 0x09,
	APPADMM_DATA_CONTENT_REL_DELTA = 0x0a,
	APPADMM_DATA_CONTENT_REL_PERCENT = 0x0b,
	APPADMM_DATA_CONTENT_REL_REFERENCE = 0x0c,
	APPADMM_DATA_CONTENT_MAXIMUM = 0x0d,
	APPADMM_DATA_CONTENT_MINIMUM = 0x0e,
	APPADMM_DATA_CONTENT_AVERAGE = 0x0f,
	APPADMM_DATA_CONTENT_PEAK_HOLD_MAX = 0x10,
	APPADMM_DATA_CONTENT_PEAK_HOLD_MIN = 0x11,
	APPADMM_DATA_CONTENT_DBM = 0x12,
	APPADMM_DATA_CONTENT_DB = 0x13,
	APPADMM_DATA_CONTENT_AUTO_HOLD = 0x14,
	APPADMM_DATA_CONTENT_SETUP = 0x15,
	APPADMM_DATA_CONTENT_LOG_STAMP = 0x16,
	APPADMM_DATA_CONTENT_LOG_MAX = 0x17,
	APPADMM_DATA_CONTENT_LOG_MIN = 0x18,
	APPADMM_DATA_CONTENT_LOG_TP = 0x19,
	APPADMM_DATA_CONTENT_HOLD = 0x1a,
	APPADMM_DATA_CONTENT_CURRENT_OUTPUT = 0x1b,
	APPADMM_DATA_CONTENT_CUR_OUT_0_20MA_PERCENT = 0x1c,
	APPADMM_DATA_CONTENT_CUR_OUT_4_20MA_PERCENT = 0x1d,
};

/**
 * Function codes
 *
 * Basically indicate the rotary position and the secondary function selected
 */
enum appadmm_functioncode_e {
	APPADMM_FUNCTIONCODE_NONE = 0x00,
	APPADMM_FUNCTIONCODE_AC_V = 0x01,
	APPADMM_FUNCTIONCODE_DC_V = 0x02,
	APPADMM_FUNCTIONCODE_AC_MV = 0x03,
	APPADMM_FUNCTIONCODE_DC_MV = 0x04,
	APPADMM_FUNCTIONCODE_OHM = 0x05,
	APPADMM_FUNCTIONCODE_CONTINUITY = 0x06,
	APPADMM_FUNCTIONCODE_DIODE = 0x07,
	APPADMM_FUNCTIONCODE_CAP = 0x08,
	APPADMM_FUNCTIONCODE_AC_A = 0x09,
	APPADMM_FUNCTIONCODE_DC_A = 0x0a,
	APPADMM_FUNCTIONCODE_AC_MA = 0x0b,
	APPADMM_FUNCTIONCODE_DC_MA = 0x0c,
	APPADMM_FUNCTIONCODE_DEGC = 0x0d,
	APPADMM_FUNCTIONCODE_DEGF = 0x0e,
	APPADMM_FUNCTIONCODE_FREQUENCY = 0x0f,
	APPADMM_FUNCTIONCODE_DUTY = 0x10,
	APPADMM_FUNCTIONCODE_HZ_V = 0x11,
	APPADMM_FUNCTIONCODE_HZ_MV = 0x12,
	APPADMM_FUNCTIONCODE_HZ_A = 0x13,
	APPADMM_FUNCTIONCODE_HZ_MA = 0x14,
	APPADMM_FUNCTIONCODE_AC_DC_V = 0x15,
	APPADMM_FUNCTIONCODE_AC_DC_MV = 0x16,
	APPADMM_FUNCTIONCODE_AC_DC_A = 0x17,
	APPADMM_FUNCTIONCODE_AC_DC_MA = 0x18,
	APPADMM_FUNCTIONCODE_LPF_V = 0x19,
	APPADMM_FUNCTIONCODE_LPF_MV = 0x1a,
	APPADMM_FUNCTIONCODE_LPF_A = 0x1b,
	APPADMM_FUNCTIONCODE_LPF_MA = 0x1c,
	APPADMM_FUNCTIONCODE_AC_UA = 0x1d,
	APPADMM_FUNCTIONCODE_DC_UA = 0x1e,
	APPADMM_FUNCTIONCODE_DC_A_OUT = 0x1f,
	APPADMM_FUNCTIONCODE_DC_A_OUT_SLOW_LINEAR = 0x20,
	APPADMM_FUNCTIONCODE_DC_A_OUT_FAST_LINEAR = 0x21,
	APPADMM_FUNCTIONCODE_DC_A_OUT_SLOW_STEP = 0x22,
	APPADMM_FUNCTIONCODE_DC_A_OUT_FAST_STEP = 0x23,
	APPADMM_FUNCTIONCODE_LOOP_POWER = 0x24,
	APPADMM_FUNCTIONCODE_250OHM_HART = 0x25,
	APPADMM_FUNCTIONCODE_VOLT_SENSE = 0x26,
	APPADMM_FUNCTIONCODE_PEAK_HOLD_V = 0x27,
	APPADMM_FUNCTIONCODE_PEAK_HOLD_MV = 0x28,
	APPADMM_FUNCTIONCODE_PEAK_HOLD_A = 0x29,
	APPADMM_FUNCTIONCODE_PEAK_HOLD_MA = 0x2a,
	APPADMM_FUNCTIONCODE_LOZ_AC_V = 0x2b,
	APPADMM_FUNCTIONCODE_LOZ_DC_V = 0x2c,
	APPADMM_FUNCTIONCODE_LOZ_AC_DC_V = 0x2d,
	APPADMM_FUNCTIONCODE_LOZ_LPF_V = 0x2e,
	APPADMM_FUNCTIONCODE_LOZ_HZ_V = 0x2f,
	APPADMM_FUNCTIONCODE_LOZ_PEAK_HOLD_V = 0x30,
	APPADMM_FUNCTIONCODE_BATTERY = 0x31,
	APPADMM_FUNCTIONCODE_AC_W = 0x32,
	APPADMM_FUNCTIONCODE_DC_W = 0x33,
	APPADMM_FUNCTIONCODE_PF = 0x34,
	APPADMM_FUNCTIONCODE_FLEX_AC_A = 0x35,
	APPADMM_FUNCTIONCODE_FLEX_LPF_A = 0x36,
	APPADMM_FUNCTIONCODE_FLEX_PEAK_HOLD_A = 0x37,
	APPADMM_FUNCTIONCODE_FLEX_HZ_A = 0x38,
	APPADMM_FUNCTIONCODE_V_HARM = 0x39,
	APPADMM_FUNCTIONCODE_INRUSH = 0x3a,
	APPADMM_FUNCTIONCODE_A_HARM = 0x3b,
	APPADMM_FUNCTIONCODE_FLEX_INRUSH = 0x3c,
	APPADMM_FUNCTIONCODE_FLEX_A_HARM = 0x3d,
	APPADMM_FUNCTIONCODE_PEAK_HOLD_UA = 0x3e,
	APPADMM_FUNCTIONCODE_AC_UA_HFR = 0x3F,
	APPADMM_FUNCTIONCODE_AC_V_HFR = 0x40,
	APPADMM_FUNCTIONCODE_AC_MV_HFR = 0x41,
	APPADMM_FUNCTIONCODE_AC_A_HFR = 0x42,
	APPADMM_FUNCTIONCODE_AC_MA_HFR = 0x43,
	APPADMM_FUNCTIONCODE_AC_UA_HFR2 = 0x44,
	APPADMM_FUNCTIONCODE_DC_V_PV = 0x45,
	APPADMM_FUNCTIONCODE_AC_V_PV = 0x46,
	APPADMM_FUNCTIONCODE_AC_V_PV_HFR = 0x47,
	APPADMM_FUNCTIONCODE_AC_DC_V_PV = 0x48,
};

/* ***************************** */
/* ****** Data Structures ****** */
/* ***************************** */

/**
 * Display Data
 */
struct appadmm_display_data_s {
	int32_t reading; /**< Measured value or wordcode in raw */

	enum appadmm_dot_e dot; /**< Dot position */
	enum appadmm_unit_e unit; /**< Unit of reading */

	enum appadmm_data_content_e data_content; /**< Specification of data content */
	enum appadmm_overload_e overload; /**< O.L or not */
};

/**
 * Frame Header
 */
struct appadmm_frame_header_s {
	u_int16_t start; /**< Start code (0x5555) */
	enum appadmm_command_e command; /**< Command */
	uint8_t dataLength; /**< Length of Data */
};

struct appadmm_request_data_read_information_s {
};

/**
 * Response Data for APPADMM_COMMAND_READ_INFORMATION
 */
struct appadmm_response_data_read_information_s {
	char model_name[32]; /**< String 0x20 filled model name of device (branded) */
	char serial_number[16]; /**< String 0x20 filled serial number of device */
	enum appadmm_model_id_e model_id; /*< Model ID Number @appadmm_model_id_e */
	u_int16_t firmware_version; /*< Firmware version */
};

struct appadmm_request_data_read_display_s {
};

/**
 * Response Data for APPADMM_COMMAND_READ_DISPLAY
 */
struct appadmm_response_data_read_display_s {
	enum appadmm_functioncode_e function_code; /**< Function Code */
	enum appadmm_autotest_e auto_test; /**< Auto or manual Test */
	uint8_t range_code; /**< @TODO Implement Table 7.1 of protocol spec, only required for calibration */
	enum appadmm_autorange_e auto_range; /**< Automatic or manual range */

	struct appadmm_display_data_s main_display_data; /**< Reading of main (lower) display value */

	struct appadmm_display_data_s sub_display_data; /**< Reading of sub (upper) display value */
};

struct appadmm_frame_s {
	
	enum appadmm_command_e command;
	
	union {
		
		union {
			
			struct appadmm_request_data_read_information_s read_information;
			
			struct appadmm_request_data_read_display_s read_display;
			
		} request;
		
		union {
			
			struct appadmm_response_data_read_information_s read_information;
			
			struct appadmm_response_data_read_display_s read_display;
			
		} response;
		
	};
	
};

struct dev_context {
	enum appadmm_connection_type_e connection_type;
	gboolean blocking;
	enum appadmm_model_id_e model_id;
	uint8_t recv_buffer[APPADMM_FRAME_MAX_SIZE];
	uint8_t recv_buffer_len;
};

/* *********************** */
/* ****** Functions ****** */
/* *********************** */

static gboolean appadmm_is_wordcode(const int arg_wordcode);

static gboolean appadmm_is_wordcode_dash(const int arg_wordcode);

static const char *appadmm_model_id_name(const enum appadmm_model_id_e arg_model_id);

static const char *appadmm_wordcode_name(const enum appadmm_wordcode_e arg_wordcode);

static uint8_t appadmm_checksum(const uint8_t *arg_data, int arg_size);

static int appadmm_frame_request_size(enum appadmm_command_e arg_command);

static int appadmm_frame_response_size(enum appadmm_command_e arg_command);

static int appadmm_is_response_frame_data_size_valid(enum appadmm_command_e arg_command, int arg_size);

static int appadmm_is_request_frame_data_size_valid(enum appadmm_command_e arg_command, int arg_size);

SR_PRIV int appadmm_buffer_reset(struct dev_context *devc);

SR_PRIV int appadmm_send(const struct sr_dev_inst *sdi, const struct appadmm_frame_s *arg_frame);

SR_PRIV int appadmm_process(const struct sr_dev_inst *sdi, const struct appadmm_frame_s *arg_frame);

SR_PRIV int appadmm_receive(int fd, int revents, void *cb_data);

static int appadmm_frame_encode(const struct appadmm_frame_s *arg_frame, uint8_t *arg_out_data, int arg_size, int *arg_out_size);

static int appadmm_frame_decode_read_information(const uint8_t **rdptr, struct appadmm_response_data_read_information_s* arg_data);
static int appadmm_frame_decode_read_display(const uint8_t **rdptr, struct appadmm_response_data_read_display_s* arg_data);

static int appadmm_frame_decode(const uint8_t *arg_data, int arg_size, struct appadmm_frame_s *arg_out_frame);

SR_PRIV int appadmm_receive(int fd, int revents, void *cb_data);

static int appadmm_response_read_information(const struct sr_dev_inst *sdi, const struct appadmm_response_data_read_information_s *arg_data);

static int appadmm_response_read_display(const struct sr_dev_inst *sdi, const struct appadmm_response_data_read_display_s *arg_data);

#endif
