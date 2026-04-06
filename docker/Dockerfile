# SPDX-License-Identifier: MIT
# AES67 FPGA Toolchain
#
# Provides a fully reproducible build environment for the AES67 FPGA project.
# Includes:
#   - Yosys (synthesis)
#   - nextpnr-ecp5 (place & route)
#   - Project Trellis (ECP5 bitstream)
#   - Icarus Verilog (simulation)
#   - Verilator (lint + simulation)
#   - GTKWave (waveform viewer)
#   - openFPGAloader (board programming)
#   - LiteX + migen (Python SoC builder)
#   - RISC-V GCC toolchain (for VexRiscv firmware)
#
# Build:    docker build -t aes67-fpga -f docker/Dockerfile .
# Run:      docker run --rm -it -v $PWD:/work aes67-fpga

FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# ---- Base packages ------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        curl \
        git \
        xz-utils \
        unzip \
        sudo \
        python3 \
        python3-pip \
        python3-venv \
        python3-setuptools \
        python3-wheel \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        libftdi1-dev \
        libusb-1.0-0-dev \
        libhidapi-dev \
        libudev-dev \
        zlib1g-dev \
        gcc-riscv64-unknown-elf \
        device-tree-compiler \
        srecord \
        less \
        vim-tiny \
        nano \
    && rm -rf /var/lib/apt/lists/*

# ---- OSS CAD Suite (Yosys, nextpnr, Trellis, Icarus, Verilator, openFPGAloader) ----
# Installed as a single tarball release maintained by YosysHQ.
# Supports both x86_64 and aarch64 hosts (Linux and Apple Silicon via Rosetta or native).
ARG OSS_CAD_SUITE_DATE=2024-09-01

RUN set -eux; \
    case "${TARGETARCH:-amd64}" in \
        amd64)  ARCH=x64 ;; \
        arm64)  ARCH=arm64 ;; \
        *)      echo "Unsupported arch: $TARGETARCH"; exit 1 ;; \
    esac; \
    DATE_NODASH="$(echo $OSS_CAD_SUITE_DATE | tr -d -)"; \
    URL="https://github.com/YosysHQ/oss-cad-suite-build/releases/download/${OSS_CAD_SUITE_DATE}/oss-cad-suite-linux-${ARCH}-${DATE_NODASH}.tgz"; \
    cd /opt && \
    wget -q --show-progress "$URL" -O oss-cad-suite.tgz && \
    tar -xzf oss-cad-suite.tgz && \
    rm oss-cad-suite.tgz

ENV PATH="/opt/oss-cad-suite/bin:/opt/oss-cad-suite/py3bin:${PATH}"
ENV PYTHONPATH="/opt/oss-cad-suite/lib/python3/dist-packages:${PYTHONPATH}"

# ---- LiteX ecosystem ----------------------------------------------------
# Install migen, litex, litedram, liteeth, litex-boards, pythondata-cpu-vexriscv
RUN mkdir -p /opt/litex && cd /opt/litex && \
    wget -q https://raw.githubusercontent.com/enjoy-digital/litex/master/litex_setup.py && \
    python3 litex_setup.py --init --install --tag=2024.04 || \
    python3 litex_setup.py --init --install

# meson + ninja are required by the LiteX BIOS build
RUN pip3 install --no-cache-dir meson

# ---- lwIP (TCP/IP stack for the soft CPU firmware) ----------------------
# Pinned to STABLE-2_2_0_RELEASE for reproducibility.
ARG LWIP_VERSION=STABLE-2_2_0_RELEASE
RUN cd /opt && \
    git clone --depth 1 --branch ${LWIP_VERSION} \
        https://git.savannah.nongnu.org/git/lwip.git && \
    rm -rf lwip/.git
ENV LWIP_DIR=/opt/lwip

# ---- Verify tool availability -------------------------------------------
RUN yosys -V && \
    nextpnr-ecp5 --version && \
    iverilog -V | head -1 && \
    verilator --version | head -1 && \
    riscv64-unknown-elf-gcc --version | head -1 && \
    python3 -c "import litex; print('litex', litex.__version__ if hasattr(litex, '__version__') else 'ok')" && \
    python3 -c "import migen; print('migen ok')"

# ---- USB device permissions (so non-root can program) ------------------
# When running with --device or --privileged, the udev rules below allow
# openFPGALoader to access the DAPLink debugger on the i9 ext-board.
RUN mkdir -p /etc/udev/rules.d && \
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0d28", ATTRS{idProduct}=="0204", MODE="0666"' \
        > /etc/udev/rules.d/99-daplink.rules

# ---- Non-root user (UID set at runtime via --user $(id -u):$(id -g)) ----
RUN useradd -m -s /bin/bash dev && \
    echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev

WORKDIR /work
USER dev

CMD ["/bin/bash"]
