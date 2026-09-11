/*
 * Copyright (c) 2026 Tenstorrent AI ULC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The SMC production ROM image as a const byte array in flash. CMake turns
 * rom/prod_rom-1.1.1.bin into the .inc at build time (see rom/README.md).
 */

#include <stddef.h>
#include <stdint.h>

#include "smc_occp_port.h"

#ifdef CONFIG_DM_TEST_APP_OCCP_TARGET_ROM_IMAGE

const uint8_t occp_rom_image[] = {
#include <occp_rom_image.inc>
};

const size_t occp_rom_image_len = sizeof(occp_rom_image);

#else

const uint8_t occp_rom_image[1] = {0};
const size_t occp_rom_image_len = 0;

#endif
