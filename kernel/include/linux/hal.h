#ifndef _LINUX_HAL_H
#define _LINUX_HAL_H

#include "types.h"

#define JETSON_AGX_ORIN     0x2301
#define JETSON_AGX_XAVIER   0x1940

struct jetson_info {
    uint32_t soc_id;
    uint32_t revision;
    uint64_t memory_size;
    uint32_t num_cores;
    char soc_name[32];
};

extern int hal_init(void);
extern struct jetson_info *hal_get_info(void);
extern uint64_t get_time_us(void);
extern void udelay(uint32_t us);
extern void mdelay(uint32_t ms);
extern void cache_invalidate(void *addr, size_t size);
extern void cache_flush(void *addr, size_t size);

#define writel(v, addr) (*(volatile uint32_t *)(addr) = (v))
#define readl(addr) (*(volatile uint32_t *)(addr))
#define writeq(v, addr) (*(volatile uint64_t *)(addr) = (v))
#define readq(addr) (*(volatile uint64_t *)(addr))

#define ioremap(paddr, size) ((void *)(paddr))
#define iounmap(vaddr)

#endif