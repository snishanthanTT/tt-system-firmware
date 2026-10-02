#!/usr/bin/env bash

# Copyright (c) 2026 Tenstorrent AI ULC
# SPDX-License-Identifier: Apache-2.0

# One step for app/dm_test_app on a NUCLEO-U385RG-Q: build it in the CI image
# (via build-dm-test-app.sh), flash it over the on-board ST-LINK-V3 with pyocd,
# then open the rtt:~$ shell (via nucleo-rtt-shell.sh).
#
#   scripts/nucleo-dm-test-app.sh              # build, flash, shell
#   scripts/nucleo-dm-test-app.sh --no-build   # flash the last build, shell
#   scripts/nucleo-dm-test-app.sh --shell      # shell only
#
# pyocd needs the STM32U3 pack once: pyocd pack install stm32u385rg
# Exit the shell with Ctrl-C.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$(dirname "${SCRIPT_DIR}")"

BOARD="nucleo_u385rg_q"
TARGET="${TARGET:-stm32u385rgtxq}"
OUT="${OUT:-${SRC}/build-output/${BOARD}}"
# Set PROBE to an ST-LINK unique ID (see `pyocd list`) when more than one is attached.
PROBE="${PROBE:-}"
export TARGET OUT PROBE

build=1
flash=1
case "${1:-}" in
"") ;;
--no-build) build=0 ;;
--shell) build=0 flash=0 ;;
*)
  echo "usage: $0 [--no-build | --shell]" >&2
  exit 2
  ;;
esac

if command -v pyocd >/dev/null 2>&1; then
  PYOCD=(pyocd)
else
  PYOCD=(uvx pyocd)
fi
PROBE_ARGS=(-t "${TARGET}")
[[ -n "${PROBE}" ]] && PROBE_ARGS+=(-u "${PROBE}")

if ((build)); then
  BOARD="${BOARD}" "${SCRIPT_DIR}/build-dm-test-app.sh"
fi

if ((flash)); then
  "${PYOCD[@]}" flash "${PROBE_ARGS[@]}" "${OUT}/zephyr.hex"
fi
