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
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_DATA_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static struct sr_dev_driver appa_dmm_driver_info;

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc;
	struct dev_context *devc;
	GSList *devices;
	const char *conn;
	const char *serialcomm;
	struct sr_dev_inst *sdi;
	struct sr_serial_dev_inst *serial;
	
	GSList *it;
	struct sr_config *src;
	struct appadmm_frame_s frame;

	sr_info("Scanning...");

	devices = NULL;
	drvc = di->context;
	drvc->instances = NULL;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->blocking = TRUE;

	serialcomm = APPADMM_CONF_SERIAL;
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
	sr_info("Testing port %s...", conn);

	/*
	sr_err("Sending DISPLAY REQUEST...");
	frame.command = APPADMM_COMMAND_READ_DISPLAY;
	appadmm_send(sdi, &frame);
	
	appadmm_receive(0, G_IO_IN, sdi);
	appadmm_receive(0, G_IO_IN, sdi);
	appadmm_receive(0, G_IO_IN, sdi);
	*/
	
	sr_err("Sending INFORMATION REQUEST...");
	frame.command = APPADMM_COMMAND_READ_INFORMATION;
	appadmm_send(sdi, &frame);
	
	sr_err("R1");
	appadmm_receive(0, G_IO_IN, sdi);
	sr_err("R2");
	appadmm_receive(0, G_IO_IN, sdi);
	sr_err("R3");
	appadmm_receive(0, G_IO_IN, sdi);
	sr_err("RD");
	
	sr_err("Vendor: %s, Model: %s, Version: %s, Model ID: %i",
		sdi->vendor,
		sdi->model,
		sdi->version,
		devc->model_id);

	/*
	sr_err("Sending DISPLAY REQUEST...");
	frame.command = APPADMM_COMMAND_READ_DISPLAY;
	appadmm_send(sdi, &frame);
	
	appadmm_receive(0, G_IO_IN, sdi);
	appadmm_receive(0, G_IO_IN, sdi);
	appadmm_receive(0, G_IO_IN, sdi);
	*/
	
	devc->blocking = FALSE;
	
	sr_err("All over.");
	
	devices = NULL;
	
	return devices;
}

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

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	int ret;

	(void)sdi;
	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	/* TODO */
	default:
		return SR_ERR_NA;
	}

	return ret;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	int ret;

	(void)sdi;
	(void)data;
	(void)cg;

	ret = SR_OK;
	switch (key) {
	/* TODO */
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
	/* TODO: configure hardware, reset acquisition state, set up
	 * callbacks and send header packet. */

	(void)sdi;

	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	/* TODO: stop acquisition. */

	(void)sdi;

	return SR_OK;
}

static struct sr_dev_driver appa_dmm_driver_info = {
	.name = "appa-dmm",
	.longname = "APPA DMM",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(appa_dmm_driver_info);
