#ifndef _LINUX_TYPES_H
#define _LINUX_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint8_t     u8;
typedef uint16_t    u16;
typedef uint32_t    u32;
typedef uint64_t    u64;
typedef int8_t      s8;
typedef int16_t     s16;
typedef int32_t     s32;
typedef int64_t     s64;

typedef unsigned long   ulong;
typedef unsigned int    uint;

#define BIT(n) (1UL << (n))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#endif
