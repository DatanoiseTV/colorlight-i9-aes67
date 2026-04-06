#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Open an interactive shell inside the AES67 FPGA toolchain container.
#
# Mounts the project directory at /work. On Linux, also passes through USB
# devices for board programming.

set -euo pipefail

cd "$(dirname "$0")/.."

DOCKER_ARGS=(
    --rm -it
    -v "$PWD:/work"
    -w /work
    -e TERM="$TERM"
)

# Match host UID/GID so files created in /work are owned by you (Linux/macOS).
if [[ "$(uname)" != "Darwin" ]]; then
    DOCKER_ARGS+=(--user "$(id -u):$(id -g)")
fi

# USB passthrough for board programming on Linux
if [[ "$(uname)" == "Linux" ]] && [[ -d /dev/bus/usb ]]; then
    DOCKER_ARGS+=(--device /dev/bus/usb)
fi

exec docker run "${DOCKER_ARGS[@]}" aes67-fpga:latest /bin/bash
