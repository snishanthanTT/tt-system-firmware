#!/usr/bin/env bash

# Copyright (c) 2026 Tenstorrent AI ULC
# SPDX-License-Identifier: Apache-2.0

# Build app/dm_test_app for the MK DMC in the firmware CI image.
# Builds the checkout this script lives in; no fetch of a remote branch.
set -euo pipefail

IMAGE="${IMAGE:-ghcr.io/tenstorrent/tt-system-firmware/ci-image:v19.11.0}"
BOARD="${BOARD:-tt_grendel_mk_bu_dmc}"
OUT="${OUT:-${HOME}/fw-out/dm_test_app}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$(dirname "${SCRIPT_DIR}")"

mkdir -p "${OUT}"
DOCKER=(docker)
docker info >/dev/null 2>&1 || DOCKER=(sudo docker)

"${DOCKER[@]}" run --rm --entrypoint bash \
  -v "${SRC}:/tt-zephyr/tt-system-firmware" \
  -v "${OUT}:/out" \
  "${IMAGE}" \
  -lc "
set -euo pipefail
git config --global --add safe.directory '*'
cd /tt-zephyr
west packages pip --install
west update
west build -p -b ${BOARD} tt-system-firmware/app/dm_test_app
cp build/zephyr/zephyr.bin build/zephyr/zephyr.hex build/zephyr/zephyr.elf /out/
chown -R $(id -u):$(id -g) /out
"

echo "Built ${OUT}/zephyr.hex"
ls -l "${OUT}/zephyr".*
