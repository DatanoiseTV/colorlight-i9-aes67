# SPDX-License-Identifier: MIT
# Firmware Makefile (lwIP-based) for the AES67 SoC management plane.
#
# Built by `make litex` in the parent directory after the LiteX SoC has been
# generated. Includes:
#   - Our application (main.c, litex_netif.c, sys_now.c, aes67_status.c)
#   - lwIP core + apps (httpd, mdns) from $(LWIP_DIR) (provided by Docker image)

BUILD_DIR ?= ../litex/build/colorlight_i9_v7_2
include $(BUILD_DIR)/software/include/generated/variables.mak
include $(SOC_DIRECTORY)/software/common.mak

LWIP_DIR ?= /opt/lwip

# ---- Application sources ---------------------------------------------------
APP_SRCS = \
    main.c \
    litex_netif.c \
    sys_now.c \
    aes67_status.c \
    sap.c

# ---- lwIP core sources -----------------------------------------------------
LWIP_CORE_SRCS = \
    $(LWIP_DIR)/src/core/init.c \
    $(LWIP_DIR)/src/core/def.c \
    $(LWIP_DIR)/src/core/dns.c \
    $(LWIP_DIR)/src/core/inet_chksum.c \
    $(LWIP_DIR)/src/core/ip.c \
    $(LWIP_DIR)/src/core/mem.c \
    $(LWIP_DIR)/src/core/memp.c \
    $(LWIP_DIR)/src/core/netif.c \
    $(LWIP_DIR)/src/core/pbuf.c \
    $(LWIP_DIR)/src/core/raw.c \
    $(LWIP_DIR)/src/core/stats.c \
    $(LWIP_DIR)/src/core/sys.c \
    $(LWIP_DIR)/src/core/tcp.c \
    $(LWIP_DIR)/src/core/tcp_in.c \
    $(LWIP_DIR)/src/core/tcp_out.c \
    $(LWIP_DIR)/src/core/timeouts.c \
    $(LWIP_DIR)/src/core/udp.c \
    $(LWIP_DIR)/src/core/altcp.c \
    $(LWIP_DIR)/src/core/altcp_alloc.c \
    $(LWIP_DIR)/src/core/altcp_tcp.c

LWIP_IPV4_SRCS = \
    $(LWIP_DIR)/src/core/ipv4/autoip.c \
    $(LWIP_DIR)/src/core/ipv4/dhcp.c \
    $(LWIP_DIR)/src/core/ipv4/etharp.c \
    $(LWIP_DIR)/src/core/ipv4/icmp.c \
    $(LWIP_DIR)/src/core/ipv4/igmp.c \
    $(LWIP_DIR)/src/core/ipv4/ip4_frag.c \
    $(LWIP_DIR)/src/core/ipv4/ip4.c \
    $(LWIP_DIR)/src/core/ipv4/ip4_addr.c

LWIP_NETIF_SRCS = \
    $(LWIP_DIR)/src/netif/ethernet.c

LWIP_APPS_SRCS = \
    $(LWIP_DIR)/src/apps/http/httpd.c \
    $(LWIP_DIR)/src/apps/http/fs.c \
    $(LWIP_DIR)/src/apps/mdns/mdns.c \
    $(LWIP_DIR)/src/apps/mdns/mdns_domain.c \
    $(LWIP_DIR)/src/apps/mdns/mdns_out.c

LWIP_SRCS = $(LWIP_CORE_SRCS) $(LWIP_IPV4_SRCS) $(LWIP_NETIF_SRCS) $(LWIP_APPS_SRCS)

# ---- Object files ----------------------------------------------------------
APP_OBJS  = $(APP_SRCS:.c=.o)
LWIP_OBJS = $(LWIP_SRCS:.c=.o)
OBJECTS   = $(APP_OBJS) $(LWIP_OBJS)

# ---- Compile flags ---------------------------------------------------------
CFLAGS += \
    -I. \
    -I./arch \
    -I$(LWIP_DIR)/src/include \
    -I$(LWIP_DIR)/src/include/ipv4 \
    -DLWIP_NO_INTTYPES_H=0

# ---- Targets ---------------------------------------------------------------
all: firmware.bin

firmware.elf: $(OBJECTS)
	$(CC) $(LDFLAGS) -T linker.ld -N -o $@ \
		../libbase/crt0.o \
		$(OBJECTS) \
		-L../libbase -L../libcompiler_rt \
		-Wl,--whole-archive -Wl,--gc-sections -Wl,-Map,$@.map \
		-lbase -lcompiler_rt
	chmod -x $@

firmware.bin: firmware.elf
	$(OBJCOPY) -O binary $< $@
	chmod -x $@

%.o: %.c
	$(compile)

clean:
	$(RM) $(APP_OBJS) firmware.elf firmware.bin firmware.elf.map .*~ *~
	$(RM) $(LWIP_OBJS)

.PHONY: all clean
