# SMC production ROM image for the OCCP target emulator

`prod_rom-1.1.1.bin` is the SMC boot ROM, release 1.1.1, as a flat 128 KB
image. The Nucleo OCCP target emulator serves it read-only at the ROM's own
address, `0xC0040000`, so a host reading the ROM over OCCP gets the bytes a
Mimir chiplet would return.

Source: `tt_smc/firmware/prod_rom-1.1.1-20260117-794e39bc/build/release/bin/prod_rom.bin`
(tt_smc commit 4ab71e643, 2026-01-21). md5 `56c32354f0a4718508e80297745d847e`.

The ROM's linker script places it at `ORIGIN = 0xc0040000, LENGTH = 128K`
(`prod_rom/linker/quasar/smc_rom.ld`). The image is padded to that length.
