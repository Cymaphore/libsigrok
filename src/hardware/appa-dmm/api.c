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
#include "scpi/vxi.h"

static const uint32_t appadmm_scanopts[] = {
	SR_CONF_CONN,
	SR_CONF_SERIALCOMM,
};

static const uint32_t appadmm_drvopts[] = {
	SR_CONF_MULTIMETER,
};

static const uint32_t appadmm_devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_DATA_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static const char *appadmm_data_sources[] = {
	"Live", /**< APPADMM_DATA_SOURCE_LIVE */
	"MEM", /**< APPADMM_DATA_SOURCE_MEM */
	"LOG", /**< APPADMM_DATA_SOURCE_LOG */
};

static GSList *appadmm_scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct appadmm_context *devc;
	GSList *devices;
	const char *conn;
	const char *serialcomm;
	struct sr_dev_inst *sdi;
	struct sr_serial_dev_inst *serial;
	struct sr_channel_group *group;
	struct sr_channel *channel_primary;
	struct sr_channel *channel_secondary;
	
	int retr;

	GSList *it;
	struct sr_config *src;

	devices = NULL;
	drvc = di->context;
	drvc->instances = NULL;

	/* Device context is used instead of another ..._info struct here */
	devc = g_malloc0(sizeof(struct appadmm_context));
	appadmm_clear_context(devc);

	serialcomm = APPADMM_CONF_SERIAL;
	conn = NULL;
	for (it = options; it; it = it->next) {
		src = it->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		case SR_CONF_SERIALCOMM:
			serialcomm = g_variant_get_string(src->data, NULL);
			break;
		}
	}

	if (!conn)
		return NULL;

	if (!serialcomm)
		serialcomm = APPADMM_CONF_SERIAL;

	devc->connection_type = APPADMM_CONNECTION_TYPE_SERIAL;

	if (conn != NULL)
		if (strncmp(conn, "bt/", 3) == 0)
			devc->connection_type = APPADMM_CONNECTION_TYPE_BLE;

	serial = sr_serial_dev_inst_new(conn, serialcomm);

	if (serial_open(serial, SERIAL_RDWR) != SR_OK)
		return NULL;

	sdi = g_malloc0(sizeof(*sdi));

	sdi->conn = serial;
	sdi->inst_type = SR_INST_SERIAL;
	sdi->status = SR_ST_INACTIVE;
	sdi->driver = di;
	sdi->priv = devc;
	
	sr_tp_appa_init(&devc->appa_inst, serial);

	/* Scan for devices by sendind READ_INFORMATION */
	appadmm_identify(sdi);

	/* If received model is invalid or nothing received, abort */
	if (devc->model_id == APPADMM_MODEL_ID_INVALID) {
		sr_err("APPA-Device NOT FOUND; No valid response to read_information request.");
		sr_serial_dev_inst_free(serial);
		serial_close(serial);
		return NULL;
	}

	sr_info("APPA-Device DETECTED; Vendor: %s, Model: %s, OEM-Model: %s, Version: %s, Serial number: %s, Model ID: %i",
		sdi->vendor,
		sdi->model,
		appadmm_model_id_name(devc->model_id),
		sdi->version,
		sdi->serial_num,
		devc->model_id);
	
	channel_primary = sr_channel_new(sdi,
		APPADMM_CHANNEL_DISPLAY_PRIMARY,
		SR_CHANNEL_ANALOG,
		TRUE,
		appadmm_channel_name(APPADMM_CHANNEL_DISPLAY_PRIMARY));
	
	channel_secondary = sr_channel_new(sdi,
		APPADMM_CHANNEL_DISPLAY_SECONDARY,
		SR_CHANNEL_ANALOG,
		TRUE,
		appadmm_channel_name(APPADMM_CHANNEL_DISPLAY_SECONDARY));
	
	group = g_malloc0(sizeof(*group));
	group->name = g_strdup("Display");
	sdi->channel_groups = g_slist_append(sdi->channel_groups, group);
	
	group->channels = g_slist_append(group->channels, channel_primary);
	group->channels = g_slist_append(group->channels, channel_secondary);
	
	devices = g_slist_append(devices, sdi);

	retr = serial_close(serial);
	if (retr < SR_OK) {
		sr_err("Unable to close device after scan");
		return NULL;
	}

	return std_scan_complete(di, devices);
}

static int appadmm_config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct appadmm_context *devc;

	int retr;

	(void) cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	retr = SR_OK;
	switch (key) {
	case SR_CONF_LIMIT_SAMPLES:
	case SR_CONF_LIMIT_FRAMES:
	case SR_CONF_LIMIT_MSEC:
		return sr_sw_limits_config_get(&devc->limits, key, data);
	case SR_CONF_DATA_SOURCE:
		*data = g_variant_new_string(appadmm_data_sources[devc->data_source]);
		break;
	default:
		return SR_ERR_NA;
	}

	return retr;
}

static int appadmm_config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct appadmm_context *devc;

	int idx;
	int retr;

	(void) cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	retr = SR_OK;
	switch (key) {
	case SR_CONF_LIMIT_SAMPLES:
	case SR_CONF_LIMIT_FRAMES:
	case SR_CONF_LIMIT_MSEC:
		return sr_sw_limits_config_set(&devc->limits, key, data);
	case SR_CONF_DATA_SOURCE:
		if ((idx = std_str_idx(data, ARRAY_AND_SIZE(appadmm_data_sources))) < 0)
			return SR_ERR_ARG;
		devc->data_source = idx;
		break;
	default:
		retr = SR_ERR_NA;
	}

	return retr;
}

static int appadmm_config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	int retr;

	retr = SR_OK;

	if (!sdi)
		return STD_CONFIG_LIST(key, data, sdi, cg, appadmm_scanopts, appadmm_drvopts, appadmm_devopts);

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, appadmm_scanopts, appadmm_drvopts, appadmm_devopts);
	case SR_CONF_DATA_SOURCE:
		*data = g_variant_new_strv(ARRAY_AND_SIZE(appadmm_data_sources));
		break;
	default:
		return SR_ERR_NA;
	}

	return retr;
}

static int appadmm_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct appadmm_context *devc;
	struct sr_serial_dev_inst *serial;

	int retr;

	devc = sdi->priv;
	serial = sdi->conn;
	
	
	
	sr_sw_limits_acquisition_start(&devc->limits);
	retr = std_session_send_df_header(sdi);
	if (retr < SR_OK)
		return retr;

	retr = serial_source_add(sdi->session, serial, G_IO_IN, 10,
		appadmm_serial_receive, (void *) sdi);

	return retr;
}

#define APPADMM_DRIVER_ENTRY(ARG_NAME, ARG_LONGNAME) \
&((struct sr_dev_driver){ \
	.name = ARG_NAME, \
	.longname = ARG_LONGNAME, \
	.api_version = 1, \
	.init = std_init, \
	.cleanup = std_cleanup, \
	.scan = appadmm_scan, \
	.dev_list = std_dev_list, \
	.dev_clear = std_dev_clear, \
	.config_get = appadmm_config_get, \
	.config_set = appadmm_config_set, \
	.config_list = appadmm_config_list, \
	.dev_open = std_serial_dev_open, \
	.dev_close = std_serial_dev_close, \
	.dev_acquisition_start = appadmm_acquisition_start, \
	.dev_acquisition_stop = std_serial_dev_acquisition_stop, \
	.context = NULL, \
})

SR_REGISTER_DEV_DRIVER_LIST(appadmm_drivers,
	APPADMM_DRIVER_ENTRY("appa-dmm", "APPA 150, 170, 200, 500, A, S and sFlex-Series"),
	APPADMM_DRIVER_ENTRY("benning-dmm", "BENNING MM 10-1, MM 12, CM 9-2, CM 10-1, CM 12, -PV"),
	APPADMM_DRIVER_ENTRY("cmt-35xx", "CMT 35xx Series"),
	APPADMM_DRIVER_ENTRY("ht-8100", "HT Instruments HT8100"),
	APPADMM_DRIVER_ENTRY("iso-tech-idm50x", "ISO-TECH IDM50x Series"),
	APPADMM_DRIVER_ENTRY("rspro-dmm", "RS PRO IDM50x and S Series"),
	APPADMM_DRIVER_ENTRY("sefram-7xxx", "Sefram 7xxx Series"),
	APPADMM_DRIVER_ENTRY("voltcraft-vc930", "Voltcraft VC-930"),
	APPADMM_DRIVER_ENTRY("voltcraft-vc950", "Voltcraft VC-950"),
	);
