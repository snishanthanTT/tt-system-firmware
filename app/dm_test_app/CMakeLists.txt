# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dm_test_app)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# OCCP target emulator: the SMC ROM's OCCP loop ported onto the Zephyr I3C
# target API. Built only when the board config asks for it (Nucleo bench).
FILE(GLOB occp_target_sources src/occp_target/*.c)
target_sources_ifdef(CONFIG_DM_TEST_APP_OCCP_TARGET app PRIVATE ${occp_target_sources})
target_include_directories(app PRIVATE src/occp_target)

# The SMC production ROM image, turned into a C byte list at build time and
# served read-only at the ROM's address. See rom/README.md for provenance.
if(CONFIG_DM_TEST_APP_OCCP_TARGET_ROM_IMAGE)
  generate_inc_file_for_target(app rom/prod_rom-1.1.1.bin
    ${ZEPHYR_BINARY_DIR}/include/generated/occp_rom_image.inc)
endif()
