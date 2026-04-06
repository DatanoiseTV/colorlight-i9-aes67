// SPDX-License-Identifier: MIT
// lwIP architecture header — RV32 / GCC / no OS

#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// Byte order: RISC-V is little endian
#define BYTE_ORDER       LITTLE_ENDIAN

// Platform diagnostic / abort hooks
#define LWIP_PLATFORM_DIAG(x)   do { printf x; } while (0)
#define LWIP_PLATFORM_ASSERT(x) do { printf("Assert: %s\n", x); while (1); } while (0)

// Structure packing
#define PACK_STRUCT_FIELD(x)     x
#define PACK_STRUCT_STRUCT       __attribute__((packed))
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_END

// We use stdint.h types — explicit typedefs not needed (lwIP picks them up)
#define LWIP_NO_INTTYPES_H 0

// Random number for DHCP xid etc. — backed by hardware PTP nanosecond counter
unsigned int lwip_rand(void);
#define LWIP_RAND() (lwip_rand())

#endif // LWIP_ARCH_CC_H
