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
 * APPA Transport Protocol
 *
 *
 */

#ifndef LIBSIGROK_TP_APPA_H
#define LIBSIGROK_TP_APPA_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define SR_TP_APPA_MAX_DATA_SIZE 64
#define SR_TP_APPA_HEADER_SIZE 4
#define SR_TP_APPA_MAX_PAYLOAD_SIZE 68
#define SR_TP_APPA_MAX_FRAME_SIZE 69
#define SR_TP_APPA_RECEIVE_TIMEOUT 500

struct sr_tp_appa_inst {
	struct sr_serial_dev_inst *serial;
	uint8_t buffer[SR_TP_APPA_MAX_FRAME_SIZE];
	uint8_t buffer_size;
};

struct sr_tp_appa_packet {
	uint8_t command;
	uint8_t length;
	uint8_t data[SR_TP_APPA_MAX_DATA_SIZE];
};

SR_PRIV int sr_tp_appa_init(struct sr_tp_appa_inst* arg_tpai,
	struct sr_serial_dev_inst *arg_serial);
SR_PRIV int sr_tp_appa_term(struct sr_tp_appa_inst* arg_tpai);

SR_PRIV int sr_tp_appa_send(struct sr_tp_appa_inst* arg_tpai,
	const struct sr_tp_appa_packet* arg_s_packet);
SR_PRIV int sr_tp_appa_receive(struct sr_tp_appa_inst* arg_tpai,
	struct sr_tp_appa_packet* arg_r_packet);
SR_PRIV int sr_tp_appa_send_receive(struct sr_tp_appa_inst* arg_tpai,
	const struct sr_tp_appa_packet* arg_s_packet,
	struct sr_tp_appa_packet* arg_r_packet);

#endif/*LIBSIGROK_TP_APPA_H*/
