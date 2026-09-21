/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr I2C target backend for one OCCP link.
 *
 * Receive: the STM32 driver calls write_requested once at the address match
 * of a controller write, write_received once per byte, and stop at the STOP
 * condition. Bytes go into the link's RX mailbox; the stop marks the message
 * complete and wakes the ROM loop.
 *
 * Send: an I2C target has nothing to arm. The controller decides how many
 * bytes it reads, and the driver asks for them one at a time: read_requested
 * for the first byte of a read transaction, read_processed for each byte
 * after it. So the reply lives in the link's TX queue and the callbacks hand
 * it out from tx_pos.
 *
 * The OCCP host reads a reply in two transactions on I2C: the 8-byte header,
 * a STOP, then the rest. tx_pos survives the STOP, so the second read carries
 * on where the first one ended. A reply is released once the last byte has
 * been handed out and the read that took it has stopped.
 *
 * The STM32 I2C block prefetches: while byte N is on the wire it raises TXIS
 * for byte N+1, so the driver fetches one byte past the last one the
 * controller ACKs, and flushes it at the STOP. Measured on the bench
 * (2026-09-18): the body read after an 8-byte header read came back one byte
 * late. So stop() gives that byte back: it rewinds tx_pos by one when the
 * last fetch was reply data, or undoes the underrun count when the fetch ran
 * past the reply. The real SMC's DesignWare block asks the ROM for a byte
 * only when the controller is clocking one out and needs no such fix.
 */

#include <zephyr/drivers/i2c.h>

#include "smc_occp_port.h"

LOG_MODULE_DECLARE(occp_tgt, CONFIG_DM_TEST_APP_OCCP_TARGET_LOG_LEVEL);

struct occp_i2c_backend {
	const struct device *dev;
	struct i2c_target_config target_config;
	occp_link_t *link;
	/* Bytes handed to the driver in the read transaction in progress. */
	uint16_t tx_fetched;
	bool reading;
	/* Whether the last byte handed out was reply data (else underrun filler). */
	bool last_was_data;
};

/*
 * One backend per I2C controller the target answers on. The Nucleo has the
 * Arduino I2C pins on i2c1 (rev D) or i2c2 (rev E), so the bench registers
 * both and the wires land on whichever the board has.
 */
#define OCCP_I2C_BACKENDS 2
static struct occp_i2c_backend g_backends[OCCP_I2C_BACKENDS];

static inline struct occp_i2c_backend *backend_of(struct i2c_target_config *config)
{
	return CONTAINER_OF(config, struct occp_i2c_backend, target_config);
}

static int i2c_write_requested_cb(struct i2c_target_config *config)
{
	occp_link_t *link = backend_of(config)->link;

	if (link->rx_complete) {
		/*
		 * A completed transaction is still in the mailbox. If the ROM
		 * loop has not consumed it, the host sent a new command before
		 * reading the reply: a host-side fault, and the new command
		 * wins. A fully consumed one is just not released yet.
		 */
		if (link->rx_pos < link->rx_len) {
			link->rx_overrun++;
		}
		link->rx_complete = false;
	}
	if (link->rx_active) {
		/* The previous write never got a stop. Drop it. */
		link->rx_aborted++;
	}
	link->rx_len = 0;
	link->rx_pos = 0;
	link->rx_active = true;
	return 0;
}

static int i2c_write_received_cb(struct i2c_target_config *config, uint8_t val)
{
	occp_link_t *link = backend_of(config)->link;

	if (link->rx_len < sizeof(link->rx_buf)) {
		link->rx_buf[link->rx_len++] = val;
	} else {
		link->rx_dropped++;
	}
	return 0;
}

static void i2c_next_tx_byte(struct occp_i2c_backend *be, uint8_t *val)
{
	occp_link_t *link = be->link;

	if (link->tx_pos < link->tx_len) {
		*val = link->tx_buf[link->tx_pos++];
		be->last_was_data = true;
	} else {
		*val = 0xFF;
		link->tx_underrun++;
		be->last_was_data = false;
	}
	be->tx_fetched++;
}

static int i2c_read_requested_cb(struct i2c_target_config *config, uint8_t *val)
{
	struct occp_i2c_backend *be = backend_of(config);

	be->reading = true;
	be->tx_fetched = 0;
	i2c_next_tx_byte(be, val);
	return 0;
}

static int i2c_read_processed_cb(struct i2c_target_config *config, uint8_t *val)
{
	i2c_next_tx_byte(backend_of(config), val);
	return 0;
}

static int i2c_stop_cb(struct i2c_target_config *config)
{
	struct occp_i2c_backend *be = backend_of(config);
	occp_link_t *link = be->link;

	if (link->rx_active) {
		link->rx_active = false;
		link->rx_pos = 0;
		link->rx_complete = true;
		link->rx_transactions++;
		occp_port_signal_data();
	}

	if (be->reading) {
		be->reading = false;
		/*
		 * Give back the prefetched byte the driver flushed at this STOP
		 * (see the file comment). It was never seen by the controller.
		 */
		if (be->tx_fetched > 0) {
			if (be->last_was_data) {
				link->tx_pos--;
			} else if (link->tx_underrun > 0) {
				link->tx_underrun--;
			}
		}
		LOG_DBG("%s read stop: fetched %u, tx %u/%u", link->name, be->tx_fetched, link->tx_pos,
			link->tx_len);
		if (link->tx_len != 0 && link->tx_pos >= link->tx_len) {
			/* Whole reply taken. Release it. */
			link->tx_len = 0;
			link->tx_pos = 0;
		}
	}
	return 0;
}

static const struct i2c_target_callbacks i2c_callbacks = {
	.write_requested = i2c_write_requested_cb,
	.write_received = i2c_write_received_cb,
	.read_requested = i2c_read_requested_cb,
	.read_processed = i2c_read_processed_cb,
	.stop = i2c_stop_cb,
};

static int i2c_arm_tx(occp_link_t *link)
{
	/*
	 * Nothing to arm: the bytes are in tx_buf and the callbacks serve them
	 * when the controller reads. A reply the host never finished reading is
	 * simply overwritten by occp_link_send; count it.
	 */
	ARG_UNUSED(link);
	return 0;
}

int occp_link_i2c_init(occp_link_t *link, const struct device *dev, uint16_t address)
{
	struct occp_i2c_backend *be = NULL;
	int ret;

	for (size_t i = 0; i < OCCP_I2C_BACKENDS; i++) {
		if (g_backends[i].link == NULL) {
			be = &g_backends[i];
			break;
		}
	}
	if (be == NULL) {
		return -ENOMEM;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	memset(link, 0, sizeof(*link));
	link->type = OCCP_LINK_I2C;
	link->name = dev->name;
	link->arm_tx = i2c_arm_tx;
	link->backend = be;

	be->dev = dev;
	be->target_config.address = address;
	be->target_config.flags = 0;
	be->target_config.callbacks = &i2c_callbacks;
	be->link = link;

	ret = i2c_target_register(dev, &be->target_config);
	if (ret < 0) {
		be->link = NULL;
		return ret;
	}
	return 0;
}
