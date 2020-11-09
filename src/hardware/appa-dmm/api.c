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

#include <config.h>
#include "protocol.h"
#include "scpi/vxi.h"

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
	SR_CONF_SERIALCOMM,
};

static const uint32_t drvopts[] = {
	SR_CONF_MULTIMETER,
};

static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
};

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct appadmm_context *devc;
	GSList *devices;
	const char *conn;
	const char *serialcomm;
	struct sr_dev_inst *sdi;
	struct sr_serial_dev_inst *serial;
	
	GSList *it;
	struct sr_config *src;
	struct appadmm_frame_s frame;
	int handshake_cycles;

	devices = NULL;
	drvc = di->context;
	drvc->instances = NULL;

	devc = g_malloc0(sizeof(struct appadmm_context));
	/// @TODO why does it segf?
	//appadmm_clear_context(&devc);
	
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
	
	frame.command = APPADMM_COMMAND_READ_INFORMATION;
	appadmm_send(sdi, &frame);
	
	handshake_cycles = APPADMM_HANDSHAKE_TIMEOUT / APPADMM_READ_BLOCKING_TIMEOUT;
	while (handshake_cycles-- > 0) {
		appadmm_receive(sdi, TRUE);
		if (devc->model_id != APPADMM_MODEL_ID_INVALID) {
			break;
		}
	};
	
	if (devc->model_id == APPADMM_MODEL_ID_INVALID) {
		sr_err("APPA-Device NOT FOUND; No valid response to read_information request.");
		sr_serial_dev_inst_free(serial);
		serial_close(serial);
		return NULL;
	}
	
	sr_warn("APPA-Device DETECTED; Vendor: %s, Model: %s, APPA-Model: %s, Version: %s, Serial number: %s, Model ID: %i",
		sdi->vendor,
		sdi->model,
		appadmm_model_id_name(devc->model_id),
		sdi->version,
		sdi->serial_num,
		devc->model_id);
	
	//sr_channel_new(sdi, APPADMM_CHANNEL_TIMESTAMP, SR_CHANNEL_ANALOG, TRUE, appadmm_channel_name(APPADMM_CHANNEL_TIMESTAMP));
	sr_channel_new(sdi, APPADMM_CHANNEL_MAIN, SR_CHANNEL_ANALOG, TRUE, appadmm_channel_name(APPADMM_CHANNEL_MAIN));
	sr_channel_new(sdi, APPADMM_CHANNEL_SUB, SR_CHANNEL_ANALOG, TRUE, appadmm_channel_name(APPADMM_CHANNEL_SUB));
	
	devices = g_slist_append(devices, sdi);
	
	serial_close(serial);
	
	return std_scan_complete(di, devices);
}

#if 0

static int dev_open(struct sr_dev_inst *sdi)
{
	(void)sdi;

	/* TODO: get handle from sdi->conn and open it. */

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	(void)sdi;

	/* TODO: get handle from sdi->conn and close it. */

	return SR_OK;
}

#endif//1|0

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct appadmm_context *devc;
	
	int ret;

	if (!sdi)
		return SR_ERR_ARG;
	
	devc = sdi->priv;
	
	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_LIMIT_SAMPLES:
	case SR_CONF_LIMIT_FRAMES:
	case SR_CONF_LIMIT_MSEC:
		return sr_sw_limits_config_get(&devc->limits, key, data);
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct appadmm_context *devc;
	
	int ret;

	if (!sdi)
		return SR_ERR_ARG;
	
	devc = sdi->priv;
	
	(void)data;
	(void)cg;

	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	case SR_CONF_LIMIT_SAMPLES:
	case SR_CONF_LIMIT_FRAMES:
	case SR_CONF_LIMIT_MSEC:
		return sr_sw_limits_config_set(&devc->limits, key, data);
	default:
		ret = SR_ERR_NA;
	}

	return ret;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	int ret;

	ret = SR_OK;

	if (!sdi)
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct appadmm_context *devc;
	struct sr_serial_dev_inst *serial;
	
	int retr;
	
	devc = sdi->priv;
	serial = sdi->conn;
	
	sr_sw_limits_acquisition_start(&devc->limits);
	retr = std_session_send_df_header(sdi);
	if(retr != SR_OK)
		return retr;
	
	retr = serial_source_add(sdi->session, serial, G_IO_IN, 10,
			appadmm_receive_serial, (void *)sdi);

	return retr;
}

#if 0

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	(void)sdi;

	return SR_OK;
}

#endif//1|0

#define APPADMM_DRIVER_ENTRY(ARG_NAME, ARG_LONGNAME) \
&((struct sr_dev_driver){ \
	.name = ARG_NAME, \
	.longname = ARG_LONGNAME, \
	.api_version = 1, \
	.init = std_init, \
	.cleanup = std_cleanup, \
	.scan = scan, \
	.dev_list = std_dev_list, \
	.dev_clear = std_dev_clear, \
	.config_get = config_get, \
	.config_set = config_set, \
	.config_list = config_list, \
	.dev_open = std_serial_dev_open, \
	.dev_close = std_serial_dev_close, \
	.dev_acquisition_start = dev_acquisition_start, \
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
