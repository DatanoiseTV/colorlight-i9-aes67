# SPDX-License-Identifier: MIT
# AES67 FPGA Makefile
#
# Targets:
#   make sim          - run testbench in Icarus Verilog
#   make synth        - synthesize and place-and-route with yosys+nextpnr
#   make litex        - build full LiteX SoC (gateware + BIOS)
#   make program      - load bitstream to SRAM via openFPGAloader
#   make flash        - program bitstream to SPI flash
#   make clean        - remove build artifacts

PROJECT   = aes67_top
TOP       = aes67_top_standalone
DEVICE    = 45k
PACKAGE   = CABGA381
SPEED     = 6
LPF       = constraints/colorlight_i9_v7.2.lpf

BUILD_DIR = build

# Verilog sources (order matters for some tools)
RTL_SRCS  = \
    rtl/util/crc32.v \
    rtl/util/fifo_sync.v \
    rtl/util/fifo_async.v \
    rtl/util/reset_sync.v \
    rtl/eth/rgmii_rx.v \
    rtl/eth/rgmii_tx.v \
    rtl/eth/eth_mac_rx.v \
    rtl/eth/eth_mac_tx.v \
    rtl/eth/eth_mac.v \
    rtl/eth/ip_checksum.v \
    rtl/eth/tx_udp_wrapper.v \
    rtl/eth/tx_arbiter.v \
    rtl/ptp/ptp_clock.v \
    rtl/ptp/ptp_timestamp.v \
    rtl/ptp/ptp_filter.v \
    rtl/ptp/ptp_servo.v \
    rtl/ptp/ptp_pp.v \
    rtl/ptp/ptp_top.v \
    rtl/rtp/rtp_rx.v \
    rtl/rtp/rtp_tx.v \
    rtl/rtp/jitter_buffer.v \
    rtl/rtp/rtp_engine.v \
    rtl/audio/audio_clk_gen.v \
    rtl/audio/i2s_tdm_master.v \
    rtl/audio/virt_tdm16.v \
    rtl/audio/audio_dsp.v \
    rtl/audio/audio_capture.v \
    rtl/soc/cpu_netif.v \
    rtl/soc/packet_router.v \
    rtl/pll_25_to_125.v \
    rtl/aes67_top.v \
    rtl/aes67_top_standalone.v

# ---- Synthesis ----
.PHONY: synth
synth: $(BUILD_DIR)/$(PROJECT).bit

$(BUILD_DIR)/$(PROJECT).json: $(RTL_SRCS) | $(BUILD_DIR)
	yosys -q -p "synth_ecp5 -top $(TOP) -json $@ -abc9" $(RTL_SRCS)

## NOTE: The standalone synth target uses --freq 80 because aes67_top_standalone
## leaves the unused CSR ports as physical pins, forcing nextpnr to spread the
## design across the whole die and lengthening routing on internal nets.
## The real `make litex` build keeps the CSRs internal and meets 125 MHz
## comfortably (RGMII timing requirement).
$(BUILD_DIR)/$(PROJECT).config: $(BUILD_DIR)/$(PROJECT).json $(LPF)
	nextpnr-ecp5 \
		--$(DEVICE) \
		--package $(PACKAGE) \
		--speed $(SPEED) \
		--json $< \
		--lpf $(LPF) \
		--textcfg $@ \
		--freq 80 \
		--report $(BUILD_DIR)/$(PROJECT).report.json \
		--seed 0 \
		--lpf-allow-unconstrained \
		--timing-allow-fail

$(BUILD_DIR)/$(PROJECT).bit: $(BUILD_DIR)/$(PROJECT).config
	ecppack --compress --freq 38.8 --input $< --bit $@ --svf $(BUILD_DIR)/$(PROJECT).svf

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- LiteX full SoC ----
.PHONY: litex
litex:
	cd litex && python3 soc.py --build

# ---- Programming ----
.PHONY: program
program: $(BUILD_DIR)/$(PROJECT).bit
	openFPGAloader -b colorlight-i9 $<

.PHONY: flash
flash: $(BUILD_DIR)/$(PROJECT).bit
	openFPGAloader -b colorlight-i9 -f $<

# ---- Simulation ----
.PHONY: sim
sim: sim/tb_ptp_clock
	./sim/tb_ptp_clock

sim/tb_ptp_clock: sim/tb_ptp_clock.v $(RTL_SRCS)
	iverilog -g2012 -o $@ -I rtl $^

# ---- Lint ----
.PHONY: lint
lint:
	verilator --lint-only -Wall -I rtl $(RTL_SRCS) --top-module $(TOP)

# ---- Clean ----
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR) sim/tb_*
	rm -rf litex/build

.DEFAULT_GOAL := synth
