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

#include <config.h>
#include "tp/appa.h"

#define LOG_PREFIX "tp-appa"

#define SR_TP_APPA_START_WORD 0x5555
#define SR_TP_APPA_START_BYTE 0x55

static uint8_t sr_tp_appa_checksum(const uint8_t *arg_data, uint8_t arg_size);
static int sr_tp_appa_buffer_reset(struct sr_tp_appa_inst* arg_tpai);

SR_PRIV int sr_tp_appa_init(struct sr_tp_appa_inst* arg_tpai,
	struct sr_serial_dev_inst *arg_serial)
 {
	if (arg_tpai == NULL
		|| arg_serial == NULL)
		return SR_ERR_BUG;
	
	if (sr_tp_appa_buffer_reset(arg_tpai) < SR_OK)
		return SR_ERR_BUG;
	
	arg_tpai->serial = arg_serial;
	
	return SR_ERR_BUG;
 }
 
SR_PRIV int sr_tp_appa_term(struct sr_tp_appa_inst* arg_tpai)
{
	if (arg_tpai == NULL)
		return SR_ERR_BUG;
	return SR_ERR_BUG;
}

SR_PRIV int sr_tp_appa_send(struct sr_tp_appa_inst* arg_tpai,
	const struct sr_tp_appa_packet* arg_s_packet)
{
	int retr;
	uint8_t header[SR_TP_APPA_HEADER_SIZE];
	uint8_t checksum;
	
	if (arg_tpai == NULL
		|| arg_s_packet == NULL)
		return SR_ERR_BUG;
	if (arg_s_packet->length > SR_TP_APPA_MAX_PAYLOAD_SIZE)
		return SR_ERR_DATA;
	
	/* encode frame header */
	
	header[0] = SR_TP_APPA_START_BYTE;
	header[1] = SR_TP_APPA_START_BYTE;
	header[2] = arg_s_packet->command;
	header[4] = arg_s_packet->length;

	/* calculate checksum */
	
	checksum = sr_tp_appa_checksum((uint8_t*)&header, sizeof(header))
		+ sr_tp_appa_checksum(arg_s_packet->data, arg_s_packet->length);
	
	/* transmit packet */
	
	if((retr = serial_write_nonblocking(arg_tpai->serial, &header,
		sizeof(header))) !=  sizeof(header))
		return(SR_ERR_IO);
		
	if((retr = serial_write_nonblocking(arg_tpai->serial, arg_s_packet->data,
		arg_s_packet->length)) != arg_s_packet->length)
		return(SR_ERR_IO);
		
	if((retr = serial_write_nonblocking(arg_tpai->serial, &checksum,
		sizeof(checksum))) != sizeof(checksum))
		return(SR_ERR_IO);
	
	return SR_OK;
}

SR_PRIV int sr_tp_appa_receive(struct sr_tp_appa_inst* arg_tpai,
	struct sr_tp_appa_packet* arg_r_packet)
{
	int len;
	int xloop;
	int retr;
	uint8_t buf[SR_TP_APPA_MAX_FRAME_SIZE * 3];
	
	if (arg_tpai == NULL
		|| arg_r_packet == NULL)
		return SR_ERR_BUG;
	
	retr = FALSE;
	
	len = serial_read_nonblocking(arg_tpai->serial,
		buf, sizeof(buf));
	
	if (len < SR_OK)
		return len;
	
	for (xloop = 0; xloop < len; xloop++) {
		/* validate header */
		if (arg_tpai->buffer_size < 1) {
			if (buf[xloop] != SR_TP_APPA_START_BYTE) {
				continue;
			}
		} else if (arg_tpai->buffer_size < 2) {
			if (buf[xloop] != SR_TP_APPA_START_BYTE) {
				sr_tp_appa_buffer_reset(arg_tpai);
				continue;
			}
		} else if (arg_tpai->buffer_size < 4) {
			if (buf[xloop] > SR_TP_APPA_MAX_DATA_SIZE) {
				sr_tp_appa_buffer_reset(arg_tpai);
				continue;
			}
		}
		
		/* add data to buffer */
		arg_tpai->buffer[arg_tpai->buffer_size++] = buf[xloop];
		
		/* process data */
		if (arg_tpai->buffer_size > 4) {
			
			/* packet complete */
			if (arg_tpai->buffer[3]
				+ SR_TP_APPA_HEADER_SIZE
				+ 1
				== arg_tpai->buffer_size) {

				/* validate checksum */
				if (sr_tp_appa_checksum(arg_tpai->buffer,
					arg_tpai->buffer[3])
					== arg_tpai->buffer[arg_tpai->buffer_size - 1]) {
					arg_r_packet->command = arg_tpai->buffer[2];
					arg_r_packet->length = arg_tpai->buffer[3];
					if (memcpy(arg_r_packet->data,
						&arg_tpai->buffer[4],
						arg_tpai->buffer[3])
						!= arg_r_packet->data)
						retr = SR_ERR_BUG;
					else
						retr = TRUE;
					break;
				} else {
					retr = SR_ERR_IO;
					break;
				}
			}
		}
		
		/* catch impossible situations, abort */
		if (arg_tpai->buffer_size > SR_TP_APPA_MAX_FRAME_SIZE) {
			sr_tp_appa_buffer_reset(arg_tpai);
			return SR_ERR_BUG;
		}
	}
	
	/* for now, just silently forget about the rest of the data. */
	
	sr_tp_appa_buffer_reset(arg_tpai);
	
	return retr;
}

SR_PRIV int sr_tp_appa_send_receive(struct sr_tp_appa_inst* arg_tpai,
	const struct sr_tp_appa_packet* arg_s_packet,
	struct sr_tp_appa_packet* arg_r_packet)
{
	int sr_cycles;
	int retr;
	
	if (arg_tpai == NULL
		|| arg_s_packet == NULL
		|| arg_r_packet == NULL)
		return SR_ERR_BUG;
	
	if ((retr = sr_tp_appa_send(arg_tpai, arg_s_packet)) < SR_OK)
		return retr;
	
	sr_cycles = SR_TP_APPA_RECEIVE_TIMEOUT / 50;
	while (sr_cycles-- > 0) {
		retr = sr_tp_appa_receive(arg_tpai, arg_r_packet);
		
		if(retr < SR_OK
			|| retr == TRUE)
			break;
	};
	
	
	return retr;
}

static uint8_t sr_tp_appa_checksum(const uint8_t *arg_data, uint8_t arg_size)
{
	uint8_t checksum;

	if (arg_data == NULL)
		return 0;

	checksum = 0;
	while (arg_size-- > 0)
		checksum += arg_data[arg_size];

	return checksum;
}

static int sr_tp_appa_buffer_reset(struct sr_tp_appa_inst* arg_tpai)
{
	if (arg_tpai == NULL)
		return SR_ERR_BUG;
	arg_tpai->buffer_size = 0;
	arg_tpai->buffer[0] = 0;
	arg_tpai->buffer[1] = 0;
	arg_tpai->buffer[2] = 0;
	arg_tpai->buffer[3] = 0;
	return SR_OK;
}
