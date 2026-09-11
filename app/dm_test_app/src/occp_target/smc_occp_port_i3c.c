/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr I3C target backend for one OCCP link.
 *
 * Receive: the STM32 driver calls write_received_cb once per byte and
 * stop_cb at frame complete. Bytes go into the link's RX mailbox; the stop
 * marks the message complete and wakes the ROM loop.
 *
 * Send: i3c_target_tx_write() on the STM32 driver ignores its buffer
 * argument. It only arms a TX preload of N bytes; the driver then calls
 * read_processed_cb from the TX-FIFO-not-full interrupt to fetch each byte.
 * So the reply lives in the link's TX queue and the callback hands it out.
 */

#include <zephyr/drivers/i3c.h>
#include <zephyr/drivers/i3c/target_device.h>
#include <zephyr/irq.h>

#include <stm32_ll_i3c.h>

#include "smc_occp_port.h"

LOG_MODULE_DECLARE(occp_tgt, CONFIG_DM_TEST_APP_OCCP_TARGET_LOG_LEVEL);

struct occp_i3c_backend {
	const struct device *dev;
	I3C_TypeDef *regs;
	struct i3c_target_config target_config;
};

/* One I3C target instance per board is all the STM32 driver supports today. */
static struct occp_i3c_backend g_backend;
static occp_link_t *g_link;

static int i3c_write_requested_cb(struct i3c_target_config *config)
{
	ARG_UNUSED(config);
	/*
	 * Not a message boundary: the driver calls this on every RX interrupt
	 * batch, so several times inside one transaction. Boundaries come
	 * from stop_cb.
	 */
	return 0;
}

/*
 * Bytes inside one I3C write arrive about 9 us apart at 1 MHz. A gap far
 * longer than that means a new transaction began, even if the previous one
 * never produced a stop (aborted frame, bus error). Cycles, not ms, so it
 * works from the interrupt.
 */
#define OCCP_RX_NEW_FRAME_GAP_CYCLES (k_us_to_cyc_ceil32(500))

static uint32_t g_last_rx_cycle;

static int i3c_write_received_cb(struct i3c_target_config *config, uint8_t val)
{
	ARG_UNUSED(config);
	occp_link_t *link = g_link;
	uint32_t now = k_cycle_get_32();

	if (link->rx_active && (now - g_last_rx_cycle) > OCCP_RX_NEW_FRAME_GAP_CYCLES) {
		/* The previous frame ended without a stop callback. Drop it. */
		link->rx_aborted++;
		link->rx_len = 0;
		link->rx_pos = 0;
		link->rx_active = false;
	}
	g_last_rx_cycle = now;

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
		link->rx_len = 0;
		link->rx_pos = 0;
		link->rx_complete = false;
	}
	if (link->rx_len < sizeof(link->rx_buf)) {
		link->rx_buf[link->rx_len++] = val;
	} else {
		link->rx_dropped++;
	}
	link->rx_active = true;
	return 0;
}

static int i3c_read_requested_cb(struct i3c_target_config *config, uint8_t *val)
{
	ARG_UNUSED(config);
	ARG_UNUSED(val);
	/*
	 * The STM32 driver calls this when a TX preload has finished loading.
	 * The stock app re-armed a counter here. The emulator arms only when
	 * the ROM loop has a reply, so nothing to do.
	 */
	return 0;
}

static int i3c_read_processed_cb(struct i3c_target_config *config, uint8_t *val)
{
	ARG_UNUSED(config);
	occp_link_t *link = g_link;

	if (link->tx_pos < link->tx_len) {
		*val = link->tx_buf[link->tx_pos++];
	} else {
		*val = 0xFF;
		link->tx_underrun++;
	}
	return 0;
}

static int i3c_stop_cb(struct i3c_target_config *config)
{
	ARG_UNUSED(config);
	occp_link_t *link = g_link;

	/* Frame complete fires after reads too; only a write makes a message. */
	if (link->rx_active) {
		link->rx_active = false;
		link->rx_pos = 0;
		link->rx_complete = true;
		link->rx_transactions++;
		occp_port_signal_data();
	}
	return 0;
}

static const struct i3c_target_callbacks i3c_callbacks = {
	.write_requested_cb = i3c_write_requested_cb,
	.write_received_cb = i3c_write_received_cb,
	.read_requested_cb = i3c_read_requested_cb,
	.read_processed_cb = i3c_read_processed_cb,
	.stop_cb = i3c_stop_cb,
};

static int i3c_arm_tx(occp_link_t *link)
{
	struct occp_i3c_backend *be = link->backend;

	if (LL_I3C_IsActiveTxPreload(be->regs)) {
		/*
		 * A previous reply is still loading: the host never read it all.
		 * The driver would refuse a new preload with -EBUSY, so drop the
		 * stale bytes at the register level and start clean.
		 */
		link->tx_stale++;
		MODIFY_REG(be->regs->TGTTDR, I3C_TGTTDR_PRELOAD | I3C_TGTTDR_TGTTDCNT, 0);
		LL_I3C_RequestTxFIFOFlush(be->regs);
	}

	/* hdr_mode is ignored by the STM32 driver; 0 is SDR. */
	return i3c_target_tx_write(be->dev, NULL, link->tx_len, 0);
}

int occp_link_i3c_init(occp_link_t *link, const struct device *dev, uintptr_t base_addr)
{
	int ret;

	if (g_link != NULL) {
		return -EALREADY;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	memset(link, 0, sizeof(*link));
	link->type = OCCP_LINK_I3C;
	link->name = dev->name;
	link->arm_tx = i3c_arm_tx;
	link->backend = &g_backend;

	g_backend.dev = dev;
	g_backend.regs = (I3C_TypeDef *)base_addr;
	g_backend.target_config.callbacks = &i3c_callbacks;
	g_link = link;

	ret = i3c_target_register(dev, &g_backend.target_config);
	if (ret < 0) {
		g_link = NULL;
		return ret;
	}
	return 0;
}
