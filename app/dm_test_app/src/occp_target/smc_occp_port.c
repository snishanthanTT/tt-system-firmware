/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Bus-independent half of the OCCP target port: the link mailbox operations
 * the ROM loop calls, the emulated memory behind ReadData and WriteData, the
 * SMC-only service stubs, and a shell command to inspect it all.
 */

#include <stdlib.h>

#include <zephyr/shell/shell.h>
#include <zephyr/sys/byteorder.h>

#include "smc_occp_port.h"

LOG_MODULE_REGISTER(occp_tgt, CONFIG_DM_TEST_APP_OCCP_TARGET_LOG_LEVEL);

/*********************************************************************
 * Link mailbox
 ********************************************************************/

K_SEM_DEFINE(g_rx_sem, 0, 1);

void occp_port_signal_data(void)
{
	k_sem_give(&g_rx_sem);
}

void occp_port_wait_for_data(k_timeout_t timeout)
{
	(void)k_sem_take(&g_rx_sem, timeout);
}

bool occp_link_has_data(occp_link_t *link)
{
	return link != NULL && link->rx_complete && link->rx_pos < link->rx_len;
}

size_t occp_link_available(occp_link_t *link)
{
	if (link == NULL || !link->rx_complete) {
		return 0;
	}
	return link->rx_len - link->rx_pos;
}

occp_error_code_t occp_link_read(occp_link_t *link, uint8_t *buf, size_t len, bool expect_excess)
{
	size_t avail = occp_link_available(link);

	if (avail < len) {
		/* Take what there is so the caller sees the partial header. */
		memcpy(buf, &link->rx_buf[link->rx_pos], avail);
		link->rx_pos += avail;
		return OCCP_ERROR_TRANSPORT_INCOMPLETE;
	}

	memcpy(buf, &link->rx_buf[link->rx_pos], len);
	link->rx_pos += len;

	if (!expect_excess && link->rx_pos < link->rx_len) {
		/* Leave the excess for the flush that the error path does. */
		return OCCP_ERROR_TRANSPORT_OVERFLOW;
	}
	if (link->rx_pos == link->rx_len) {
		/*
		 * Whole transaction consumed. Release the mailbox now: the ROM
		 * loop flushes only on error paths, and a successful ReadData
		 * would otherwise leave it marked complete until the next write.
		 */
		occp_link_flush(link);
	}
	return OCCP_ERROR_NONE;
}

size_t occp_link_flush(occp_link_t *link)
{
	size_t dropped = occp_link_available(link);
	unsigned int key = irq_lock();

	/* Only release the mailbox if no new transaction has started meanwhile. */
	if (link->rx_complete) {
		link->rx_complete = false;
		link->rx_len = 0;
		link->rx_pos = 0;
	}
	irq_unlock(key);
	return dropped;
}

occp_error_code_t occp_link_send(occp_link_t *link, const uint8_t *data, size_t len)
{
	int ret;

	if (len == 0 || len > sizeof(link->tx_buf)) {
		return OCCP_ERROR_BUFFER_OVERFLOW;
	}

	unsigned int key = irq_lock();

	memcpy(link->tx_buf, data, len);
	link->tx_len = len;
	link->tx_pos = 0;
	irq_unlock(key);

	ret = link->arm_tx(link);
	if (ret < 0) {
		LOG_ERR("%s: arm tx of %u bytes failed: %d", link->name, (unsigned int)len, ret);
		return OCCP_ERROR_INTERFACE_ERROR;
	}
	link->tx_replies++;
	return OCCP_ERROR_NONE;
}

/*********************************************************************
 * Emulated memory
 ********************************************************************/

struct occp_mem_window {
	const char *name;
	uint64_t base;
	size_t size;
	uint8_t *mem;
	bool writable;
};

static uint8_t g_emu_sram[CONFIG_DM_TEST_APP_OCCP_TARGET_MEM_SIZE];

/*
 * The address map the emulator answers for. Real chiplet addresses; the
 * bytes live wherever the STM32 has room: SRAM in RAM, the ROM in flash.
 */
static struct occp_mem_window g_windows[] = {
	{
		.name = "sram",
		.base = CONFIG_DM_TEST_APP_OCCP_TARGET_MEM_BASE,
		.size = sizeof(g_emu_sram),
		.mem = g_emu_sram,
		.writable = true,
	},
#ifdef CONFIG_DM_TEST_APP_OCCP_TARGET_ROM_IMAGE
	{
		.name = "rom",
		.base = CONFIG_DM_TEST_APP_OCCP_TARGET_ROM_BASE,
		.size = 0, /* set from occp_rom_image_len at init */
		.mem = (uint8_t *)occp_rom_image,
		.writable = false,
	},
#endif
};

/* Byte-order-revealing pattern: 0x0123456789ABCDEF stored little-endian. */
#define OCCP_MEM_DEFAULT_PATTERN 0x0123456789ABCDEFULL

static struct occp_mem_window *occp_mem_find(uint64_t addr, size_t len)
{
	for (size_t i = 0; i < ARRAY_SIZE(g_windows); i++) {
		struct occp_mem_window *w = &g_windows[i];

		if (addr >= w->base && len <= w->size && addr - w->base <= w->size - len) {
			return w;
		}
	}
	return NULL;
}

void occp_mem_fill(uint64_t pattern)
{
	uint8_t bytes[8];

	sys_put_le64(pattern, bytes);
	for (size_t i = 0; i < sizeof(g_emu_sram); i++) {
		g_emu_sram[i] = bytes[i % 8];
	}
}

void occp_mem_init(void)
{
	occp_mem_fill(OCCP_MEM_DEFAULT_PATTERN);
	LOG_INF("emulated sram: 0x%016llx .. +0x%x, pattern 0x%016llx",
		(unsigned long long)g_windows[0].base, (unsigned int)g_windows[0].size,
		(unsigned long long)OCCP_MEM_DEFAULT_PATTERN);
#ifdef CONFIG_DM_TEST_APP_OCCP_TARGET_ROM_IMAGE
	g_windows[1].size = occp_rom_image_len;
	LOG_INF("rom image:     0x%016llx .. +0x%x, read-only, first bytes %02x %02x %02x %02x",
		(unsigned long long)g_windows[1].base, (unsigned int)g_windows[1].size,
		occp_rom_image[0], occp_rom_image[1], occp_rom_image[2], occp_rom_image[3]);
#endif
}

occp_error_code_t occp_mem_check(uint64_t addr, size_t len, bool is_write)
{
	struct occp_mem_window *w;

	if (len == 0) {
		return OCCP_ERROR_ACCESS_VIOLATION;
	}
	w = occp_mem_find(addr, len);
	if (w == NULL || (is_write && !w->writable)) {
		return OCCP_ERROR_ACCESS_VIOLATION;
	}
	return OCCP_ERROR_NONE;
}

void occp_mem_read(uint64_t addr, uint8_t *dst, size_t len)
{
	struct occp_mem_window *w = occp_mem_find(addr, len);

	if (w == NULL) {
		memset(dst, 0, len);
		return;
	}
	memcpy(dst, &w->mem[addr - w->base], len);
}

void occp_mem_write(uint64_t addr, const uint8_t *src, size_t len)
{
	struct occp_mem_window *w = occp_mem_find(addr, len);

	if (w == NULL || !w->writable) {
		return;
	}
	memcpy(&w->mem[addr - w->base], src, len);
}

/*********************************************************************
 * SMC service stubs
 ********************************************************************/

static uint32_t g_post_code;
static uint32_t g_status_last;
static uint32_t g_status_count;

static void post_code_set_field(uint32_t mask, uint32_t shift, uint32_t value)
{
	g_post_code = (g_post_code & ~mask) | ((value << shift) & mask);
}

void smc_post_code_set_boot_phase(uint8_t phase)
{
	post_code_set_field(0xF0000000, 28, phase);
}

void smc_post_code_set_occp_state(uint8_t state)
{
	post_code_set_field(0x0F000000, 24, state);
}

void smc_post_code_set_interface(uint8_t interface)
{
	post_code_set_field(0x00F00000, 20, interface);
}

void smc_post_code_set_error(uint8_t error)
{
	post_code_set_field(0x000F0000, 16, error);
}

uint32_t smc_post_code_get_current(void)
{
	return g_post_code;
}

void smc_status_report(uint32_t msg_type, uint32_t msg_value)
{
	g_status_last = (SMC_STATUS_FW_ID_SMC_BL0 << 24) | ((msg_type & 0xFF) << 16) |
			(msg_value & 0xFFFF);
	g_status_count++;
	if (msg_type == SMC_STATUS_TYPE_ERROR) {
		LOG_WRN("status: error 0x%03x", msg_value);
	} else {
		LOG_DBG("status: type 0x%x value 0x%03x", msg_type, msg_value);
	}
}

bool smc_status_read(uint32_t *message)
{
	*message = g_status_last;
	return g_status_count > 0;
}

bool sep_status_read(uint32_t *message)
{
	/* No SEP on this board. */
	*message = 0;
	return false;
}

uint32_t smc_status_last(void)
{
	return g_status_last;
}

uint64_t occp_port_i3c_pid(void)
{
	return CONFIG_DM_TEST_APP_OCCP_TARGET_I3C_PID;
}

/*********************************************************************
 * Shell: occp_tgt status | fill <hex64> | dump <hex addr> <len>
 ********************************************************************/

extern occp_link_t *occp_target_link(void);

static int cmd_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	occp_link_t *link = occp_target_link();

	shell_print(sh, "post code   0x%08x", g_post_code);
	shell_print(sh, "occp status 0x%08x (cmd count %u, last error 0x%02x)", occp_status_get(),
		    occp_status_get_command_count(), occp_status_get_error_code());
	shell_print(sh, "smc status  0x%08x (%u reports)", g_status_last, g_status_count);
	if (link != NULL) {
		shell_print(sh, "%s: rx %u transactions, %u overrun, %u aborted, %u dropped; pending %u/%u%s",
			    link->name, link->rx_transactions, link->rx_overrun, link->rx_aborted,
			    link->rx_dropped, link->rx_pos, link->rx_len,
			    link->rx_active ? " (frame open)" : "");
		shell_print(sh, "%s: tx %u replies, %u underrun, %u stale", link->name,
			    link->tx_replies, link->tx_underrun, link->tx_stale);
	}
	for (size_t i = 0; i < ARRAY_SIZE(g_windows); i++) {
		shell_print(sh, "window %s: 0x%016llx +0x%x %s", g_windows[i].name,
			    (unsigned long long)g_windows[i].base, (unsigned int)g_windows[i].size,
			    g_windows[i].writable ? "rw" : "ro");
	}
	return 0;
}

static int cmd_fill(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	uint64_t pattern = strtoull(argv[1], NULL, 16);

	occp_mem_fill(pattern);
	shell_print(sh, "sram window filled with 0x%016llx", (unsigned long long)pattern);
	return 0;
}

static int cmd_dump(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	uint64_t addr = strtoull(argv[1], NULL, 16);
	size_t len = strtoul(argv[2], NULL, 0);
	uint8_t row[16];

	if (len == 0 || len > 256) {
		shell_error(sh, "len must be 1..256");
		return -EINVAL;
	}
	if (occp_mem_check(addr, len, false) != OCCP_ERROR_NONE) {
		shell_error(sh, "0x%016llx +%u is outside every window", (unsigned long long)addr,
			    (unsigned int)len);
		return -EINVAL;
	}
	for (size_t off = 0; off < len; off += sizeof(row)) {
		size_t n = MIN(sizeof(row), len - off);

		occp_mem_read(addr + off, row, n);
		shell_fprintf(sh, SHELL_NORMAL, "%016llx:", (unsigned long long)(addr + off));
		for (size_t i = 0; i < n; i++) {
			shell_fprintf(sh, SHELL_NORMAL, " %02x", row[i]);
		}
		shell_fprintf(sh, SHELL_NORMAL, "\n");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	occp_tgt_cmds,
	SHELL_CMD_ARG(status, NULL, "post code, status register, link counters", cmd_status, 1, 0),
	SHELL_CMD_ARG(fill, NULL, "fill <hex64>: refill the sram window", cmd_fill, 2, 0),
	SHELL_CMD_ARG(dump, NULL, "dump <hex addr> <len>: show emulated memory", cmd_dump, 3, 0),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(occp_tgt, &occp_tgt_cmds, "OCCP target emulator", NULL);
