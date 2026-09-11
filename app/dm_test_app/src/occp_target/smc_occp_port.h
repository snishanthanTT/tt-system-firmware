/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Port layer between the SMC ROM OCCP target code (smc_occp.c) and Zephyr.
 *
 * The ROM code polls its bus drivers: "do you have bytes?", "give me N
 * bytes", "send this reply". Zephyr's I3C target API is the other way round:
 * the driver calls the app once per received byte and once per stop. This
 * header defines the mailbox that turns the second into the first, plus the
 * small SMC-only services the ROM code calls (console, POST codes, status
 * ring buffer, security mode) as constants, logs or stubs.
 */

#ifndef SMC_OCCP_PORT_H
#define SMC_OCCP_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "smc_occp.h"
#include "smc_occp_error_codes.h"
#include "smc_occp_status.h"

/*********************************************************************
 * Constants carried over from ROM headers that are not ported whole
 ********************************************************************/

/* smc_status.h: status message fields */
#define SMC_STATUS_FW_ID_SMC_BL0 0x3
#define SMC_STATUS_TYPE_STATUS   0x1
#define SMC_STATUS_TYPE_WARNING  0x8
#define SMC_STATUS_TYPE_ERROR    0xF

/* smc_post_code.h: OCCP state (bits 27:24) */
#define POST_CODE_OCCP_STATE_IDLE         0x0
#define POST_CODE_OCCP_STATE_CMD_RECEIVED 0x1
#define POST_CODE_OCCP_STATE_PROCESSING   0x2
#define POST_CODE_OCCP_STATE_RESP_READY   0x3
#define POST_CODE_OCCP_STATE_COMPLETE     0x4
#define POST_CODE_OCCP_STATE_ERROR        0x5

/* smc_post_code.h: interface (bits 23:20) */
#define POST_CODE_IFACE_NONE 0x0
#define POST_CODE_IFACE_I3C0 0x1
#define POST_CODE_IFACE_I3C1 0x2
#define POST_CODE_IFACE_I3C3 0x4
#define POST_CODE_IFACE_I2C0 0x5
#define POST_CODE_IFACE_I2C1 0x6

/* smc_post_code.h: error (bits 19:16) */
#define POST_CODE_ERROR_NONE             0x0
#define POST_CODE_ERROR_SRAM             0x1
#define POST_CODE_ERROR_INTERFACE        0x2
#define POST_CODE_ERROR_COMMAND          0x3
#define POST_CODE_ERROR_ACCESS           0x4
#define POST_CODE_ERROR_INVALID_SEC_MODE 0x5

/* smc_post_code.h: boot phase (bits 31:28) */
#define POST_CODE_BOOT_PHASE_OCCP_READY    0x4
#define POST_CODE_BOOT_PHASE_BOOT_COMPLETE 0x6

/*********************************************************************
 * Console: the ROM's simulation console becomes Zephyr debug logging
 ********************************************************************/

#define simputs(str)          LOG_DBG("%s", (str))
#define simputshex16(msg, v)  LOG_DBG("%s0x%04x", (msg), (unsigned int)(v))
#define simputshex32(msg, v)  LOG_DBG("%s0x%08x", (msg), (unsigned int)(v))
#define simputshex64(msg, v)  LOG_DBG("%s0x%016llx", (msg), (unsigned long long)(v))

/*********************************************************************
 * Bus link: one OCCP interface (I3C now, I2C later)
 ********************************************************************/

typedef enum {
	OCCP_LINK_I3C,
	OCCP_LINK_I2C,
} occp_link_type_t;

/* Largest command: 8-byte packet header + OCCP_MAX_MSG_SIZE + 4-byte CRC32. */
#define OCCP_LINK_RX_BUF_SIZE (8 + OCCP_MAX_MSG_SIZE + 4 + 5)
/* Largest reply: 8-byte packet header + OCCP_MAX_RD_SIZE + 4-byte CRC32. */
#define OCCP_LINK_TX_BUF_SIZE (8 + OCCP_MAX_RD_SIZE + 4 + 5)

struct occp_link;

typedef struct occp_link {
	occp_link_type_t type;
	const char *name;

	/*
	 * RX mailbox. Holds one bus transaction (one OCCP command). The
	 * driver callbacks fill it from interrupt context; the ROM loop
	 * drains it from thread context. rx_complete is the hand-off flag.
	 */
	uint8_t rx_buf[OCCP_LINK_RX_BUF_SIZE];
	volatile uint16_t rx_len;      /* bytes received in this transaction */
	volatile uint16_t rx_pos;      /* bytes the ROM loop has consumed */
	volatile bool rx_active;       /* a write transaction is in progress */
	volatile bool rx_complete;     /* stop seen; message ready for the loop */
	uint32_t rx_overrun;           /* new write began before the last was consumed */
	uint32_t rx_aborted;           /* frame ended without a stop; dropped on the next byte */
	uint32_t rx_dropped;           /* bytes beyond the buffer */
	uint32_t rx_transactions;

	/*
	 * TX queue. The ROM loop writes one whole reply here, then the
	 * backend arms the hardware. The driver pulls bytes from it in the
	 * read callback, one per byte clocked out by the controller.
	 */
	uint8_t tx_buf[OCCP_LINK_TX_BUF_SIZE];
	volatile uint16_t tx_len;
	volatile uint16_t tx_pos;
	uint32_t tx_underrun;          /* controller read past the reply */
	uint32_t tx_stale;             /* a reply was armed while one was pending */
	uint32_t tx_replies;

	/* Backend: arm the hardware to send tx_buf[0..tx_len). */
	int (*arm_tx)(struct occp_link *link);
	void *backend;
} occp_link_t;

/* Zephyr I3C target backend. base_addr is the I3C peripheral register base. */
int occp_link_i3c_init(occp_link_t *link, const struct device *dev, uintptr_t base_addr);

/* True when a complete transaction with unconsumed bytes is waiting. */
bool occp_link_has_data(occp_link_t *link);

/* Unconsumed bytes of the waiting transaction, 0 if none. */
size_t occp_link_available(occp_link_t *link);

/*
 * Copy len bytes out of the waiting transaction. Mirrors the ROM driver's
 * receive_payload_stream: returns OCCP_ERROR_TRANSPORT_INCOMPLETE when the
 * transaction holds fewer bytes, and OCCP_ERROR_TRANSPORT_OVERFLOW when
 * expect_excess is false and bytes remain after the copy.
 */
occp_error_code_t occp_link_read(occp_link_t *link, uint8_t *buf, size_t len, bool expect_excess);

/* Discard what is left of the waiting transaction. Returns the byte count. */
size_t occp_link_flush(occp_link_t *link);

/* Queue one reply and arm the hardware. */
occp_error_code_t occp_link_send(occp_link_t *link, const uint8_t *data, size_t len);

/* Block until any link signals a complete transaction, or the timeout. */
void occp_port_wait_for_data(k_timeout_t timeout);

/* Called from the backends when a transaction completes. ISR safe. */
void occp_port_signal_data(void);

/*********************************************************************
 * Emulated chiplet memory behind ReadData and WriteData
 ********************************************************************/

void occp_mem_init(void);

/*
 * OCCP_ERROR_NONE if [addr, addr+len) lies inside one window, and that
 * window is writable when is_write is set.
 */
occp_error_code_t occp_mem_check(uint64_t addr, size_t len, bool is_write);

/* The SMC production ROM image served by the read-only ROM window. */
extern const uint8_t occp_rom_image[];
extern const size_t occp_rom_image_len;
void occp_mem_read(uint64_t addr, uint8_t *dst, size_t len);
void occp_mem_write(uint64_t addr, const uint8_t *src, size_t len);

/* Fill the SRAM window with a repeating 64-bit pattern (little-endian). */
void occp_mem_fill(uint64_t pattern);

/*********************************************************************
 * SMC-only services the ROM code calls, as stubs
 ********************************************************************/

/* smc_post_code.h: kept in a variable, shown by the shell. */
void smc_post_code_set_boot_phase(uint8_t phase);
void smc_post_code_set_occp_state(uint8_t state);
void smc_post_code_set_interface(uint8_t interface);
void smc_post_code_set_error(uint8_t error);
uint32_t smc_post_code_get_current(void);

/* smc_status.h: the SRAM ring buffers become a log line and a last-value. */
void smc_status_report(uint32_t msg_type, uint32_t msg_value);
bool smc_status_read(uint32_t *message);
bool sep_status_read(uint32_t *message);
uint32_t smc_status_last(void);

/* smc_security.h / smc_strap.h: the bench is never in secure mode. */
static inline uint8_t smc_security_is_secure_mode(void)
{
	return 0;
}

static inline uint8_t smc_strap_is_status_rpt_disable(void)
{
	return 0;
}

/* smc_efuse.h / smc_strap.h: the PID is fixed by Kconfig on this board. */
uint64_t occp_port_i3c_pid(void);

#endif /* SMC_OCCP_PORT_H */
