#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Build the AES67 FPGA toolchain Docker image.
#
# Usage: ./docker/build.sh

set -euo pipefail

cd "$(dirname "$0")/.."

echo ">>> Building AES67 FPGA toolchain image..."
docker build \
    -f docker/Dockerfile \
    -t aes67-fpga:latest \
    --build-arg OSS_CAD_SUITE_DATE=2024-09-01 \
    .

echo ""
echo ">>> Image built. Quick start:"
echo "    ./docker/shell.sh             # interactive shell"
echo "    ./docker/run.sh make synth    # one-shot build"
echo "    ./docker/run.sh make sim      # run testbench"
