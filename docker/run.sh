#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Run a single command inside the AES67 FPGA toolchain container.
#
# Usage: ./docker/run.sh <command...>
#   ./docker/run.sh make synth
#   ./docker/run.sh make sim
#   ./docker/run.sh make litex
#   ./docker/run.sh yosys -p "synth_ecp5 -top aes67_top" rtl/aes67_top.v

set -euo pipefail

cd "$(dirname "$0")/.."

if [[ $# -eq 0 ]]; then
    echo "Usage: $0 <command...>"
    exit 1
fi

DOCKER_ARGS=(
    --rm
    -v "$PWD:/work"
    -w /work
    -e TERM="${TERM:-xterm}"
)

# Allocate a TTY only if stdin is a terminal
if [[ -t 0 ]]; then
    DOCKER_ARGS+=(-it)
fi

if [[ "$(uname)" != "Darwin" ]]; then
    DOCKER_ARGS+=(--user "$(id -u):$(id -g)")
fi

if [[ "$(uname)" == "Linux" ]] && [[ -d /dev/bus/usb ]]; then
    DOCKER_ARGS+=(--device /dev/bus/usb)
fi

exec docker run "${DOCKER_ARGS[@]}" aes67-fpga:latest "$@"
