#include "linux/hal.h"
#include <time.h>
#include <unistd.h>

static struct jetson_info jetson_hw_info;

int hal_init(void)
{
    jetson_hw_info.soc_id = JETSON_AGX_ORIN;
    jetson_hw_info.revision = 1;
    jetson_hw_info.memory_size = 32ULL * 1024 * 1024 * 1024;
    jetson_hw_info.num_cores = 12;
    strcpy(jetson_hw_info.soc_name, "Tegra Orin");
    
    return 0;
}

struct jetson_info *hal_get_info(void)
{
    return &jetson_hw_info;
}

uint64_t get_time_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}

void udelay(uint32_t us)
{
    struct timespec req = { 0, us * 1000 };
    nanosleep(&req, NULL);
}

void mdelay(uint32_t ms)
{
    usleep(ms * 1000);
}

void cache_invalidate(void *addr, size_t size)
{
    __asm__ volatile("dc ivac, %0" :: "r"(addr) : "memory");
}

void cache_flush(void *addr, size_t size)
{
    uint64_t end = (uint64_t)addr + size;
    uint64_t ptr = (uint64_t)addr;
    
    while (ptr < end) {
        __asm__ volatile("dc civac, %0" :: "r"(ptr) : "memory");
        ptr += 64;
    }
    __asm__ volatile("dsb sy" ::: "memory");
}
