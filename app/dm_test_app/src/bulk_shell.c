/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * `bulk` -- one bus transfer per shell command, payload as a single hex token.
 *
 * Upstream's i2c/i3c/spi shells spend one argv token per payload byte. That
 * makes CONFIG_SHELL_ARGC_MAX, rather than the bus or the driver buffer, the
 * thing that caps a transfer: 15 bytes for `i2c write` and 17 for
 * `spi transceive` at the stock ARGC_MAX of 20. Raising ARGC_MAX (prj.conf
 * does) moves that wall to 253 -- the `optional` count in struct
 * shell_static_args is a uint8_t whose 0xFE and 0xFF are already spoken for --
 * but does not remove it, and each token still costs 3-5 characters on the
 * wire where a byte needs two.
 *
 * These commands take the payload as ONE contiguous-hex token:
 *
 *   bulk i2c write     <bus> <addr> <hex>
 *   bulk i2c read      <bus> <addr> <count>
 *   bulk i2c writeread <bus> <addr> <hex> <count>
 *   bulk i3c write     <bus> <target> <hex>
 *   bulk i3c read      <bus> <target> <count>
 *   bulk i3c writeread <bus> <target> <hex> <count>
 *   bulk spi conf      <frequency> [settings]
 *   bulk spi cs        <port> <pin> [al|ah] | none
 *   bulk spi txrx      <bus> <hex>
 *
 * argc is then constant per command, ARGC_MAX leaves the picture entirely, and
 * the only limit left is BULK_MAX_BYTES against CONFIG_SHELL_CMD_BUFF_SIZE.
 *
 * Three things here are deliberate departures from the upstream commands, each
 * fixing something boardy's host side had to work around:
 *
 *  - Over-long payloads are REFUSED, never truncated. `i2c write` clamps to its
 *    buffer and reports it through shell_info, so the transfer still ACKs with
 *    part of the payload; boardy has to carry "too many bytes" in
 *    _SHELL_ERROR_MARKERS to notice.
 *  - A read prints the received bytes ONLY. `spi transceive` hexdumps TX and
 *    then RX, which is how a host-side parser came to concatenate the two and
 *    read a frame at twice its length (see DMCShellSPI._parse_transceive).
 *  - `bulk spi txrx` pulses chip select around its own transfer. That collapses
 *    three round trips into one and closes the hazard documented in
 *    DMCShellSPI.transceive: the shell does not reliably separate commands
 *    arriving in one burst, and a `gpio set` merged into a transfer becomes an
 *    extra payload byte -- unrecoverable on a NOR flash PAGE PROGRAM. A single
 *    command cannot be merged with itself.
 *
 * Failure wording is upstream's, verbatim where it exists ("Failed to write to
 * device: 0x50"), because boardy classifies a NACK by matching that text in the
 * reply -- see _NACK_MARKERS. Every other fault says "error", which
 * _SHELL_ERROR_MARKERS already looks for.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#ifdef CONFIG_I3C_CONTROLLER
#include <zephyr/drivers/i3c.h>
#endif

#ifdef CONFIG_SPI
#include <zephyr/drivers/spi.h>
#endif

/*
 * Payload ceiling, in bytes. Static, not on the stack: CONFIG_SHELL_STACK_SIZE
 * is 3072 and there are two shell threads, so a 1 KB frame buffer per direction
 * does not belong there.
 *
 * CONFIG_SHELL_CMD_BUFF_SIZE has to carry 2 characters per byte plus the
 * command prefix, which prj.conf sizes at 2304 for exactly this value. The
 * command line is the tighter of the two for `writeread`, whose prefix is
 * longest; hex_decode() reports whichever bites.
 */
#define BULK_MAX_BYTES 1024

/* Bytes of payload formatted per shell_fprintf() call in bulk_print_rx(). */
#define BULK_HEX_CHUNK_BYTES 32

static const char bulk_hex_digits[] = "0123456789abcdef";

/*
 * One buffer pair for every bus, guarded by one mutex. The app runs two shell
 * instances (uart4 and the CDC ACM port, see usb_shell.c) on two threads, and
 * both can reach these commands, so the buffers cannot be bare statics. The
 * transfer itself is held inside the lock: the bus drivers have their own
 * locking, but the decode-transfer-print sequence has to be atomic with respect
 * to the buffers it uses.
 */
static uint8_t bulk_tx[BULK_MAX_BYTES];
static uint8_t bulk_rx[BULK_MAX_BYTES];
K_MUTEX_DEFINE(bulk_xfer_lock);

static int bulk_hex_nibble(char c)
{
	if (c >= '0' && c <= '9') {
		return c - '0';
	}
	if (c >= 'a' && c <= 'f') {
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F') {
		return c - 'A' + 10;
	}

	return -1;
}

/*
 * Decode one contiguous-hex token into `out`, returning the byte count or a
 * negative errno (having already said why on `sh`).
 *
 * An odd digit count is rejected rather than padded. Padding would shift the
 * whole payload by a nibble and still transfer it, which is the same class of
 * silent-corruption bug as truncating.
 */
static int bulk_hex_decode(const struct shell *sh, const char *s, uint8_t *out, size_t out_len)
{
	size_t len = strlen(s);

	/* A leading 0x is tolerated so a single byte can be pasted by hand. */
	if (len >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
		s += 2;
		len -= 2;
	}

	if (len == 0) {
		shell_error(sh, "bulk: error: empty payload");
		return -EINVAL;
	}
	if ((len % 2) != 0) {
		shell_error(sh, "bulk: error: %zu hex digits is odd, one byte is two digits", len);
		return -EINVAL;
	}
	if ((len / 2) > out_len) {
		shell_error(sh, "bulk: error: %zu bytes exceeds the %zu-byte limit", len / 2,
			    out_len);
		return -EINVAL;
	}

	for (size_t i = 0; i < len; i += 2) {
		int hi = bulk_hex_nibble(s[i]);
		int lo = bulk_hex_nibble(s[i + 1]);

		if (hi < 0 || lo < 0) {
			shell_error(sh, "bulk: error: non-hex digit at offset %zu", i);
			return -EINVAL;
		}
		out[i / 2] = (uint8_t)((hi << 4) | lo);
	}

	return (int)(len / 2);
}

/*
 * Reply with `RX <n> <hex>`: one line, contiguous lowercase hex, no separators.
 *
 * Not shell_hexdump(): its offset column and ASCII gutter cost roughly four
 * characters per byte where this costs two, and at BULK_MAX_BYTES that is the
 * difference between a 2 KB reply and an 8 KB one. `n` is printed so the host
 * can check the length it got against the length it asked for without counting
 * characters.
 */
static void bulk_print_rx(const struct shell *sh, const uint8_t *buf, size_t len)
{
	char chunk[BULK_HEX_CHUNK_BYTES * 2 + 1];

	shell_fprintf(sh, SHELL_NORMAL, "RX %zu ", len);

	/* In blocks, not byte at a time: one shell_fprintf() per byte is a
	 * format-string parse and a transport flush each, 1024 times over.
	 */
	for (size_t i = 0; i < len; i += BULK_HEX_CHUNK_BYTES) {
		size_t n = MIN((size_t)BULK_HEX_CHUNK_BYTES, len - i);

		for (size_t j = 0; j < n; j++) {
			chunk[2 * j] = bulk_hex_digits[buf[i + j] >> 4];
			chunk[2 * j + 1] = bulk_hex_digits[buf[i + j] & 0x0f];
		}
		chunk[2 * n] = '\0';
		shell_fprintf(sh, SHELL_NORMAL, "%s", chunk);
	}

	shell_fprintf(sh, SHELL_NORMAL, "\n");
}

/* strtoul with the whole token required to parse, and a 1..BULK_MAX_BYTES range. */
static int bulk_parse_count(const struct shell *sh, const char *s, size_t *count)
{
	char *end;
	unsigned long v;

	v = strtoul(s, &end, 0);
	if (*s == '\0' || *end != '\0') {
		shell_error(sh, "bulk: error: '%s' is not a byte count", s);
		return -EINVAL;
	}
	if (v == 0 || v > BULK_MAX_BYTES) {
		shell_error(sh, "bulk: error: count %lu outside 1..%d", v, BULK_MAX_BYTES);
		return -EINVAL;
	}

	*count = (size_t)v;

	return 0;
}

static int bulk_parse_addr(const struct shell *sh, const char *s, uint16_t *addr)
{
	char *end;
	unsigned long v;

	v = strtoul(s, &end, 0);
	if (*s == '\0' || *end != '\0') {
		shell_error(sh, "bulk: error: '%s' is not an address", s);
		return -EINVAL;
	}
	if (v > 0x7f) {
		shell_error(sh, "bulk: error: address 0x%lx is not a 7-bit address", v);
		return -EINVAL;
	}

	*addr = (uint16_t)v;

	return 0;
}

static const struct device *bulk_device(const struct shell *sh, const char *name)
{
	const struct device *dev = shell_device_get_binding(name);

	if (dev == NULL) {
		/* "not found" is one of boardy's _SHELL_ERROR_MARKERS, and it has
		 * to be: a mistyped bus name otherwise reports success on every
		 * write and failure on every read.
		 */
		shell_error(sh, "bulk: error: device %s not found", name);
	}

	return dev;
}

static int cmd_bulk_i2c_write(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint16_t addr;
	int len;
	int ret;

	ARG_UNUSED(argc);

	dev = bulk_device(sh, argv[1]);
	if (dev == NULL) {
		return -ENODEV;
	}
	if (bulk_parse_addr(sh, argv[2], &addr) < 0) {
		return -EINVAL;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);

	len = bulk_hex_decode(sh, argv[3], bulk_tx, sizeof(bulk_tx));
	if (len < 0) {
		k_mutex_unlock(&bulk_xfer_lock);
		return len;
	}

	ret = i2c_write(dev, bulk_tx, (uint32_t)len, addr);

	k_mutex_unlock(&bulk_xfer_lock);

	if (ret < 0) {
		shell_error(sh, "Failed to write to device: 0x%02x", addr);
		return ret;
	}

	shell_print(sh, "OK %d", len);

	return 0;
}

/*
 * `bulk i2c quick <bus> <addr> [w|r]` -- address the device and stop, no data.
 *
 * The SMBus Quick Command, and the canonical non-destructive presence probe:
 * `i2cdetect -q` is this transfer. boardy's DMCShellI2C.check_presence has to
 * fall back to a 1-byte read today, and says why -- "DMC must use 1-byte read as
 * it does not support 0-byte writing (Must write a register address)". That is
 * true of the `i2c write` *command*, whose register argument is mandatory. It is
 * not true of the hardware: upstream's `i2c scan` probes every address with
 * exactly this transfer (i2c_shell.c, `msgs[0].len = 0U` with
 * `I2C_MSG_WRITE | I2C_MSG_STOP`) and that command works on this board. The only
 * thing missing was a way to aim it at one address.
 *
 * CONFIG_SMBUS + CONFIG_SMBUS_SHELL would also provide it, as `smbus quick`.
 * Worth knowing that it would not provide anything more: drivers/smbus/
 * smbus_stm32.c implements smbus_quick() as `i2c_write(i2c_dev, NULL, 0, addr)`
 * -- the identical call, reached through a second driver and a devicetree node
 * that borrows i2c1's pins. This is that call without either.
 *
 * The read direction is offered because some parts ACK only a read, but it is
 * NOT the proven one. `i2c scan` establishes the write direction on this
 * controller; nothing establishes that it can issue a zero-length read. A
 * controller that cannot will report it here rather than quietly succeeding.
 */
static int cmd_bulk_i2c_quick(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	struct i2c_msg msg = {.buf = NULL, .len = 0U, .flags = I2C_MSG_WRITE | I2C_MSG_STOP};
	uint16_t addr;
	bool write = true;
	int ret;

	dev = bulk_device(sh, argv[1]);
	if (dev == NULL) {
		return -ENODEV;
	}
	if (bulk_parse_addr(sh, argv[2], &addr) < 0) {
		return -EINVAL;
	}

	if (argc > 3) {
		if (strcmp(argv[3], "w") == 0) {
			write = true;
		} else if (strcmp(argv[3], "r") == 0) {
			write = false;
		} else {
			shell_error(sh, "bulk: error: want 'w' or 'r', got '%s'", argv[3]);
			return -EINVAL;
		}
	}

	if (!write) {
		msg.flags = I2C_MSG_READ | I2C_MSG_STOP;
	}

	/* No payload, so no bulk_xfer_lock: this touches nothing shared. */
	ret = i2c_transfer(dev, &msg, 1, addr);
	if (ret < 0) {
		/* Upstream's two phrasings exactly, so that boardy's _nacked()
		 * reads an absent part as absent rather than as a bus fault.
		 */
		if (write) {
			shell_error(sh, "Failed to write to device: 0x%02x", addr);
		} else {
			shell_error(sh, "Failed to read from device: 0x%02x", addr);
		}
		return ret;
	}

	/* Zero bytes, but still the OK <n> grammar the other writes reply in. */
	shell_print(sh, "OK 0");

	return 0;
}

static int cmd_bulk_i2c_read(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint16_t addr;
	size_t count;
	int ret;

	ARG_UNUSED(argc);

	dev = bulk_device(sh, argv[1]);
	if (dev == NULL) {
		return -ENODEV;
	}
	if (bulk_parse_addr(sh, argv[2], &addr) < 0) {
		return -EINVAL;
	}
	if (bulk_parse_count(sh, argv[3], &count) < 0) {
		return -EINVAL;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);

	ret = i2c_read(dev, bulk_rx, (uint32_t)count, addr);
	if (ret == 0) {
		bulk_print_rx(sh, bulk_rx, count);
	}

	k_mutex_unlock(&bulk_xfer_lock);

	if (ret < 0) {
		shell_error(sh, "Failed to read from device: 0x%02x", addr);
		return ret;
	}

	return 0;
}

static int cmd_bulk_i2c_writeread(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;
	uint16_t addr;
	size_t count;
	int len;
	int ret;

	ARG_UNUSED(argc);

	dev = bulk_device(sh, argv[1]);
	if (dev == NULL) {
		return -ENODEV;
	}
	if (bulk_parse_addr(sh, argv[2], &addr) < 0) {
		return -EINVAL;
	}
	if (bulk_parse_count(sh, argv[4], &count) < 0) {
		return -EINVAL;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);

	len = bulk_hex_decode(sh, argv[3], bulk_tx, sizeof(bulk_tx));
	if (len < 0) {
		k_mutex_unlock(&bulk_xfer_lock);
		return len;
	}

	/* One transaction: write, repeated start, read. Splitting it into a
	 * `write` then a `read` releases the bus in between, which for a
	 * register-pointer device is a different thing entirely.
	 */
	ret = i2c_write_read(dev, addr, bulk_tx, (size_t)len, bulk_rx, count);
	if (ret == 0) {
		bulk_print_rx(sh, bulk_rx, count);
	}

	k_mutex_unlock(&bulk_xfer_lock);

	if (ret < 0) {
		shell_error(sh, "Failed to read from device: 0x%02x", addr);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_I3C_CONTROLLER

static struct i3c_device_desc *bulk_i3c_desc(const struct shell *sh, const char *bus_name,
					     const char *tdev_name)
{
	const struct device *bus;
	const struct device *tdev;
	struct i3c_device_desc *desc;

	bus = bulk_device(sh, bus_name);
	if (bus == NULL) {
		return NULL;
	}
	tdev = bulk_device(sh, tdev_name);
	if (tdev == NULL) {
		return NULL;
	}

	I3C_BUS_FOR_EACH_I3CDEV(bus, desc) {
		if (strcmp(desc->dev->name, tdev->name) == 0) {
			return desc;
		}
	}

	shell_error(sh, "bulk: error: %s not attached to %s", tdev->name, bus->name);

	return NULL;
}

/*
 * Refuse a transfer longer than the target's negotiated Max Read/Write Length.
 *
 * This is not belt-and-braces. MRL/MWL are per-target and come from the
 * device-info read at attach; when that read fails they stay 0, and
 * app/dm_test_app/mk_i3c_controller.overlay records the resulting bug -- a
 * GETBCR that returned -5 left mrl/mwl at 0x00 and every transfer was truncated
 * to a single byte, silently. Raising BULK_MAX_BYTES does nothing for I3C on its
 * own, so say which limit is actually in the way rather than letting the
 * controller quietly shorten the transfer.
 *
 * A zero limit means "never successfully read", which is not the same as "no
 * limit", so it is reported rather than treated as unlimited.
 */
static int bulk_i3c_check_len(const struct shell *sh, const struct i3c_device_desc *desc,
			      size_t len, bool write)
{
	uint16_t limit = write ? desc->data_length.mwl : desc->data_length.mrl;
	const char *what = write ? "MWL" : "MRL";

	if (limit == 0) {
		shell_error(sh,
			    "bulk: error: %s %s is 0 -- device info was never read, so the "
			    "controller would truncate this transfer. Re-attach the target or "
			    "SETMRL/SETMWL it first.",
			    desc->dev->name, what);
		return -EINVAL;
	}
	if (len > limit) {
		shell_error(sh, "bulk: error: %zu bytes exceeds %s %s of %u", len, desc->dev->name,
			    what, limit);
		return -EINVAL;
	}

	return 0;
}

static int cmd_bulk_i3c_write(const struct shell *sh, size_t argc, char **argv)
{
	struct i3c_device_desc *desc;
	int len;
	int ret;

	ARG_UNUSED(argc);

	desc = bulk_i3c_desc(sh, argv[1], argv[2]);
	if (desc == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);

	len = bulk_hex_decode(sh, argv[3], bulk_tx, sizeof(bulk_tx));
	if (len < 0) {
		k_mutex_unlock(&bulk_xfer_lock);
		return len;
	}
	if (bulk_i3c_check_len(sh, desc, (size_t)len, true) < 0) {
		k_mutex_unlock(&bulk_xfer_lock);
		return -EINVAL;
	}

	ret = i3c_write(desc, bulk_tx, (uint32_t)len);

	k_mutex_unlock(&bulk_xfer_lock);

	if (ret < 0) {
		shell_error(sh, "Failed to write to device: %s", desc->dev->name);
		return ret;
	}

	shell_print(sh, "OK %d", len);

	return 0;
}

static int cmd_bulk_i3c_read(const struct shell *sh, size_t argc, char **argv)
{
	struct i3c_device_desc *desc;
	size_t count;
	int ret;

	ARG_UNUSED(argc);

	desc = bulk_i3c_desc(sh, argv[1], argv[2]);
	if (desc == NULL) {
		return -ENODEV;
	}
	if (bulk_parse_count(sh, argv[3], &count) < 0) {
		return -EINVAL;
	}
	if (bulk_i3c_check_len(sh, desc, count, false) < 0) {
		return -EINVAL;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);

	ret = i3c_read(desc, bulk_rx, (uint32_t)count);
	if (ret == 0) {
		bulk_print_rx(sh, bulk_rx, count);
	}

	k_mutex_unlock(&bulk_xfer_lock);

	if (ret < 0) {
		shell_error(sh, "Failed to read from device: %s", desc->dev->name);
		return ret;
	}

	return 0;
}

static int cmd_bulk_i3c_writeread(const struct shell *sh, size_t argc, char **argv)
{
	struct i3c_device_desc *desc;
	size_t count;
	int len;
	int ret;

	ARG_UNUSED(argc);

	desc = bulk_i3c_desc(sh, argv[1], argv[2]);
	if (desc == NULL) {
		return -ENODEV;
	}
	if (bulk_parse_count(sh, argv[4], &count) < 0) {
		return -EINVAL;
	}
	if (bulk_i3c_check_len(sh, desc, count, false) < 0) {
		return -EINVAL;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);

	len = bulk_hex_decode(sh, argv[3], bulk_tx, sizeof(bulk_tx));
	if (len < 0) {
		k_mutex_unlock(&bulk_xfer_lock);
		return len;
	}
	if (bulk_i3c_check_len(sh, desc, (size_t)len, true) < 0) {
		k_mutex_unlock(&bulk_xfer_lock);
		return -EINVAL;
	}

	ret = i3c_write_read(desc, bulk_tx, (size_t)len, bulk_rx, count);
	if (ret == 0) {
		bulk_print_rx(sh, bulk_rx, count);
	}

	k_mutex_unlock(&bulk_xfer_lock);

	if (ret < 0) {
		shell_error(sh, "Failed to read from device: %s", desc->dev->name);
		return ret;
	}

	return 0;
}

#endif /* CONFIG_I3C_CONTROLLER */

#ifdef CONFIG_SPI

/*
 * `bulk spi` carries its own bus configuration.
 *
 * It cannot share upstream's: drivers/spi/spi_shell.c keeps its specs in a
 * file-static table built from the devicetree, mutated by `spi conf` and
 * `spi cs`, with no way in from out of tree. Two independent configurations
 * would be worse than one of our own, so `bulk spi conf` and `bulk spi cs` are
 * what configure `bulk spi txrx` -- the upstream commands do not affect it.
 *
 * Chip select is a plain GPIO driven from here, not spi_config.cs. That is the
 * arrangement boardy found actually works on this board: `spi cs` left PB9
 * asserted and never pulsing, and the flash returned only zeros, while driving
 * the pin by hand around each frame read its JEDEC ID every time (see
 * DMCShellSPI.use_gpio_cs). The difference now is that the pulse happens on this
 * side of the serial link.
 */
static struct {
	uint32_t frequency;
	spi_operation_t operation;
	const struct device *cs_port;
	gpio_pin_t cs_pin;
	bool cs_active_low;
} bulk_spi = {
	.frequency = 1000000,
	.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
	.cs_active_low = true,
};

static void bulk_spi_cs_drive(bool assert)
{
	if (bulk_spi.cs_port == NULL) {
		return;
	}

	/* Raw, not logical: the pin is configured as a plain GPIO_OUTPUT rather
	 * than with GPIO_ACTIVE_LOW, so that a level spelled here and a level
	 * spelled by boardy's `gpio set` mean the same thing.
	 */
	(void)gpio_pin_set_raw(bulk_spi.cs_port, bulk_spi.cs_pin,
			       (assert == bulk_spi.cs_active_low) ? 0 : 1);
}

static int cmd_bulk_spi_conf(const struct shell *sh, size_t argc, char **argv)
{
	spi_operation_t op = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	char *end;
	unsigned long freq;

	freq = strtoul(argv[1], &end, 0);
	if (*argv[1] == '\0' || *end != '\0' || freq == 0) {
		shell_error(sh, "bulk: error: '%s' is not a frequency", argv[1]);
		return -EINVAL;
	}

	/* Same letters upstream's `spi conf` takes, so one habit covers both. */
	if (argc > 2) {
		for (const char *s = argv[2]; *s != '\0'; s++) {
			switch (*s) {
			case 'o':
				op |= SPI_MODE_CPOL;
				break;
			case 'h':
				op |= SPI_MODE_CPHA;
				break;
			case 'l':
				op |= SPI_TRANSFER_LSB;
				break;
			case 'T':
				op |= SPI_FRAME_FORMAT_TI;
				break;
			default:
				shell_error(sh, "bulk: error: unknown setting '%c', want o h l T",
					    *s);
				return -EINVAL;
			}
		}
	}

	/* Same lock as the transfers, so a `conf` from the USB shell cannot land
	 * half-applied in a `txrx` already running on the UART one.
	 */
	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);
	bulk_spi.frequency = (uint32_t)freq;
	bulk_spi.operation = op;
	k_mutex_unlock(&bulk_xfer_lock);

	shell_print(sh, "OK %u Hz", bulk_spi.frequency);

	return 0;
}

static int cmd_bulk_spi_cs(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *port;
	char *end;
	unsigned long pin;
	bool active_low = true;
	int ret;

	if (strcmp(argv[1], "none") == 0) {
		k_mutex_lock(&bulk_xfer_lock, K_FOREVER);
		bulk_spi.cs_port = NULL;
		k_mutex_unlock(&bulk_xfer_lock);
		shell_print(sh, "OK none");
		return 0;
	}

	if (argc < 3) {
		shell_error(sh, "bulk: error: want <port> <pin> [al|ah], or 'none'");
		return -EINVAL;
	}

	port = bulk_device(sh, argv[1]);
	if (port == NULL) {
		return -ENODEV;
	}

	pin = strtoul(argv[2], &end, 0);
	if (*argv[2] == '\0' || *end != '\0' || pin > 15) {
		shell_error(sh, "bulk: error: '%s' is not a pin", argv[2]);
		return -EINVAL;
	}

	if (argc > 3) {
		if (strcmp(argv[3], "al") == 0) {
			active_low = true;
		} else if (strcmp(argv[3], "ah") == 0) {
			active_low = false;
		} else {
			shell_error(sh, "bulk: error: want 'al' or 'ah', got '%s'", argv[3]);
			return -EINVAL;
		}
	}

	ret = gpio_pin_configure(port, (gpio_pin_t)pin, GPIO_OUTPUT);
	if (ret < 0) {
		shell_error(sh, "bulk: error: cannot drive %s pin %lu (%d)", port->name, pin, ret);
		return ret;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);
	bulk_spi.cs_port = port;
	bulk_spi.cs_pin = (gpio_pin_t)pin;
	bulk_spi.cs_active_low = active_low;

	/* Park it idle before the first transfer. A chip select left asserted
	 * makes the next transfer a continuation of the last one.
	 */
	bulk_spi_cs_drive(false);
	k_mutex_unlock(&bulk_xfer_lock);

	shell_print(sh, "OK %s %lu %s", port->name, pin, active_low ? "al" : "ah");

	return 0;
}

static int cmd_bulk_spi_txrx(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *bus;
	struct spi_config cfg = {
		.frequency = bulk_spi.frequency,
		.operation = bulk_spi.operation,
	};
	struct spi_buf tx_buf;
	struct spi_buf rx_buf;
	struct spi_buf_set tx_set;
	struct spi_buf_set rx_set;
	int len;
	int ret;

	ARG_UNUSED(argc);

	bus = bulk_device(sh, argv[1]);
	if (bus == NULL) {
		return -ENODEV;
	}

	k_mutex_lock(&bulk_xfer_lock, K_FOREVER);

	len = bulk_hex_decode(sh, argv[2], bulk_tx, sizeof(bulk_tx));
	if (len < 0) {
		k_mutex_unlock(&bulk_xfer_lock);
		return len;
	}

	tx_buf.buf = bulk_tx;
	tx_buf.len = (size_t)len;
	rx_buf.buf = bulk_rx;
	rx_buf.len = (size_t)len;
	tx_set.buffers = &tx_buf;
	tx_set.count = 1;
	rx_set.buffers = &rx_buf;
	rx_set.count = 1;

	bulk_spi_cs_drive(true);
	ret = spi_transceive(bus, &cfg, &tx_set, &rx_set);
	/* Released even on failure: see the comment on bulk_spi_cs_drive's
	 * parking call. One bad frame must not poison every later one.
	 */
	bulk_spi_cs_drive(false);

	if (ret == 0) {
		bulk_print_rx(sh, bulk_rx, (size_t)len);
	}

	k_mutex_unlock(&bulk_xfer_lock);

	if (ret < 0) {
		shell_error(sh, "bulk: error: spi_transceive failed (%d)", ret);
		return ret;
	}

	return 0;
}

#endif /* CONFIG_SPI */

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_bulk_i2c,
	SHELL_CMD_ARG(write, NULL,
		      SHELL_HELP("Write one hex payload to an I2C device",
				 "<bus> <addr> <hex>\n"
				 "Example: bulk i2c write i2c1 0x50 0000deadbeef"),
		      cmd_bulk_i2c_write, 4, 0),
	SHELL_CMD_ARG(read, NULL,
		      SHELL_HELP("Read bytes from an I2C device, no register write",
				 "<bus> <addr> <count>\n"
				 "Example: bulk i2c read i2c1 0x50 256"),
		      cmd_bulk_i2c_read, 4, 0),
	SHELL_CMD_ARG(quick, NULL,
		      SHELL_HELP("Address a device and stop, no data -- a presence probe",
				 "<bus> <addr> [w|r]\n"
				 "w (default) is the direction `i2c scan` uses and the one\n"
				 "proven on this controller; r is offered for parts that ACK\n"
				 "only a read and is unverified here.\n"
				 "Example: bulk i2c quick i2c1 0x50"),
		      cmd_bulk_i2c_quick, 3, 1),
	SHELL_CMD_ARG(writeread, NULL,
		      SHELL_HELP("Write then read in one transaction, repeated start between",
				 "<bus> <addr> <hex> <count>\n"
				 "Example: bulk i2c writeread i2c1 0x50 0000 256"),
		      cmd_bulk_i2c_writeread, 5, 0),
	SHELL_SUBCMD_SET_END);

#ifdef CONFIG_I3C_CONTROLLER
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_bulk_i3c,
	SHELL_CMD_ARG(write, NULL,
		      SHELL_HELP("Write one hex payload to an I3C target",
				 "<bus> <target> <hex>\n"
				 "Example: bulk i3c write i3c1 binho_target 0000deadbeef"),
		      cmd_bulk_i3c_write, 4, 0),
	SHELL_CMD_ARG(read, NULL,
		      SHELL_HELP("Read bytes from an I3C target",
				 "<bus> <target> <count>\n"
				 "Example: bulk i3c read i3c1 binho_target 64"),
		      cmd_bulk_i3c_read, 4, 0),
	SHELL_CMD_ARG(writeread, NULL,
		      SHELL_HELP("Write then read in one transaction, repeated start between",
				 "<bus> <target> <hex> <count>\n"
				 "Example: bulk i3c writeread i3c1 binho_target 00 64"),
		      cmd_bulk_i3c_writeread, 5, 0),
	SHELL_SUBCMD_SET_END);
#endif

#ifdef CONFIG_SPI
SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_bulk_spi,
	SHELL_CMD_ARG(conf, NULL,
		      SHELL_HELP("Set the clock and mode `bulk spi txrx` uses",
				 "<frequency> [<settings>]\n"
				 "<settings> any sequence of: o CPOL, h CPHA, l LSB first,\n"
				 "T TI frame format\n"
				 "Example: bulk spi conf 8000000 ol"),
		      cmd_bulk_spi_conf, 2, 1),
	SHELL_CMD_ARG(cs, NULL,
		      SHELL_HELP("Set the GPIO chip select `bulk spi txrx` pulses",
				 "<port> <pin> [al|ah] | none\n"
				 "al (default) active low, ah active high\n"
				 "Example: bulk spi cs gpiob 9 al"),
		      cmd_bulk_spi_cs, 2, 2),
	SHELL_CMD_ARG(txrx, NULL,
		      SHELL_HELP("Full-duplex transfer of one hex payload, CS pulsed here",
				 "<bus> <hex>\n"
				 "Prints the received bytes only. RX length equals TX length.\n"
				 "Example: bulk spi txrx spi2 9f000000"),
		      cmd_bulk_spi_txrx, 3, 0),
	SHELL_SUBCMD_SET_END);
#endif

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_bulk, SHELL_CMD(i2c, &sub_bulk_i2c, "I2C transfers, payload as one hex token", NULL),
#ifdef CONFIG_I3C_CONTROLLER
	SHELL_CMD(i3c, &sub_bulk_i3c, "I3C transfers, payload as one hex token", NULL),
#endif
#ifdef CONFIG_SPI
	SHELL_CMD(spi, &sub_bulk_spi, "SPI transfers, payload as one hex token", NULL),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(bulk, &sub_bulk,
		   "Long bus transfers: one command, payload as a single hex token.\n"
		   "Replies 'OK <n>' for a write, 'RX <n> <hex>' for a read.\n"
		   "An over-long payload is refused and names the limit, never truncated.",
		   NULL);
