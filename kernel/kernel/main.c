/*
 * Cule OS Kernel Main Entry Point
 */

#include <linux/types.h>
#include <linux/hal.h>
#include <linux/sched.h>
#include "mm/mm.h"
#include "ipc/ipc.h"
#include "drivers/camera/csi_camera.h"
#include "drivers/camera/usb_camera.h"
#include "drivers/gps/gps_uart.h"
#include "drivers/satcom/iridium_satcom.h"
#include "drivers/telemetry/mavlink_telemetry.h"

#define CULE_VERSION_MAJOR  1
#define CULE_VERSION_MINOR  0
#define CULE_VERSION_PATCH  0

/* Forward declarations */
extern void handle_sync_exception(uint64_t esr, uint64_t elr, uint64_t spsr);
extern void handle_irq(void);
extern void secondary_start_kernel(void);

/* System info structure */
struct cule_system_info {
    uint32_t version;
    uint64_t boot_time;
    struct jetson_info *hw_info;
    uint32_t num_cpus;
    uint64_t total_memory;
    uint64_t free_memory;
};

static struct cule_system_info sys_info;

/*
 * Kernel banner
 */
static void print_banner(void)
{
    const char *banner = 
        "\n"
        "  _____      _ _         ____   _____\n"
        " / ____|    | | |       / __ \\ / ____|\n"
        "| |     ___ | | |_ ___ | |  | | (___  \n"
        "| |    / _ \\| | __/ _ \\| |  | |\\___ \\ \n"
        "| |___| (_) | | || (_) | |__| |____) |\n"
        " \\_____\\___/|_|\\__\\___/ \\____/|_____/ \n"
        "\n"
        "  Cule OS v%d.%d.%d - Embedded Real-Time Operating System\n"
        "  Copyright (c) 2024 Cule Aerospace\n"
        "  Built for NVIDIA Jetson Platforms\n"
        "\n";
    
    printk(banner, CULE_VERSION_MAJOR, CULE_VERSION_MINOR, CULE_VERSION_PATCH);
}

/*
 * Print system information
 */
static void print_system_info(void)
{
    struct jetson_info *hw = sys_info.hw_info;
    
    printk("System Information:\n");
    printk("  SoC:        %s (ID: 0x%04X)\n", hw->soc_name, hw->soc_id);
    printk("  Revision:   %d\n", hw->revision);
    printk("  CPU Cores:  %d\n", hw->num_cores);
    printk("  Memory:     %lu MB (%lu MB free)\n", 
           (unsigned long)(sys_info.total_memory / (1024*1024)),
           (unsigned long)(sys_info.free_memory / (1024*1024)));
    printk("\n");
}

/*
 * Kernel initialization
 */
static int kernel_init(void *unused)
{
    int ret;
    
    printk("Initializing Cule OS subsystems...\n\n");
    
    /* Initialize hardware abstraction layer */
    printk("[HAL] Initializing hardware abstraction layer...\n");
    ret = hal_init();
    if (ret < 0) {
        printk("[HAL] Failed to initialize: %d\n", ret);
        return ret;
    }
    sys_info.hw_info = hal_get_info();
    printk("[HAL] Initialized for %s\n", sys_info.hw_info->soc_name);
    
    /* Initialize memory management */
    printk("[MM] Initializing memory management...\n");
    ret = mm_init(0x80000000, 0x80000000);
    if (ret < 0) {
        printk("[MM] Failed to initialize: %d\n", ret);
        return ret;
    }
    sys_info.total_memory = sys_info.hw_info->memory_size;
    sys_info.free_memory = get_free_pages() * PAGE_SIZE;
    printk("[MM] Memory initialized: %lu pages available\n", (unsigned long)get_free_pages());
    
    /* Initialize paging */
    printk("[MM] Setting up virtual memory...\n");
    ret = paging_init();
    if (ret < 0) {
        printk("[MM] Failed to setup paging: %d\n", ret);
        return ret;
    }
    printk("[MM] Virtual memory enabled\n");
    
    /* Initialize slab allocator */
    printk("[MM] Initializing slab allocator...\n");
    ret = slab_init();
    if (ret < 0) {
        printk("[MM] Failed to initialize slab: %d\n", ret);
        return ret;
    }
    printk("[MM] Slab allocator ready\n");
    
    /* Initialize IPC subsystem */
    printk("[IPC] Initializing inter-process communication...\n");
    ret = ipc_init();
    if (ret < 0) {
        printk("[IPC] Failed to initialize: %d\n", ret);
        return ret;
    }
    printk("[IPC] IPC subsystem ready\n");
    
    /* Initialize scheduler */
    printk("[SCHED] Initializing real-time scheduler...\n");
    ret = sched_init();
    if (ret < 0) {
        printk("[SCHED] Failed to initialize: %d\n", ret);
        return ret;
    }
    printk("[SCHED] TT/NIS scheduler ready\n");
    
    /* Initialize camera drivers */
    printk("[CAMERA] Initializing camera subsystem...\n");
    ret = csi_camera_init();
    if (ret < 0) {
        printk("[CAMERA] CSI camera init warning: %d\n", ret);
    }
    ret = usb_camera_init();
    if (ret < 0) {
        printk("[CAMERA] USB camera init warning: %d\n", ret);
    }
    printk("[CAMERA] Camera subsystem ready\n");
    
    /* Initialize GPS driver */
    printk("[GPS] Initializing GPS receiver...\n");
    ret = gps_init(4, 115200);
    if (ret < 0) {
        printk("[GPS] GPS init warning: %d\n", ret);
    }
    printk("[GPS] GPS ready on UART4\n");
    
    /* Initialize Iridium SATCOM */
    printk("[SATCOM] Initializing Iridium modem...\n");
    ret = iridium_init();
    if (ret < 0) {
        printk("[SATCOM] Iridium init warning: %d\n", ret);
    }
    printk("[SATCOM] Iridium modem ready\n");
    
    /* Initialize MAVLink telemetry */
    printk("[TELEMETRY] Initializing MAVLink...\n");
    ret = mavlink_init(1, 1);  /* System ID 1, Component ID 1 */
    if (ret < 0) {
        printk("[TELEMETRY] MAVLink init warning: %d\n", ret);
    }
    printk("[TELEMETRY] MAVLink ready on UART2\n");
    
    printk("\n");
    print_system_info();
    
    printk("Cule OS initialization complete.\n");
    printk("System ready for flight operations.\n\n");
    
    return 0;
}

/*
 * Idle task
 */
static void idle_task(void *unused)
{
    while (1) {
        /* Enter low power state */
        __asm__ volatile("wfi" ::: "memory");
    }
}

/*
 * Sensor fusion task - Time-triggered
 */
static void sensor_fusion_task(void *unused)
{
    struct gps_position gps_pos;
    struct mavlink_gps_raw gps_raw;
    
    printk("[TASK] Sensor fusion started\n");
    
    while (1) {
        /* Read GPS position */
        if (gps_read_position(&gps_pos) == 0 && gps_has_fix()) {
            /* Convert to MAVLink format */
            gps_raw.time_usec = gps_pos.timestamp_us;
            gps_raw.lat = (int32_t)(gps_pos.latitude * 1e7);
            gps_raw.lon = (int32_t)(gps_pos.longitude * 1e7);
            gps_raw.alt = (int32_t)(gps_pos.altitude * 1000);
            gps_raw.fix_type = gps_pos.fix_type;
            gps_raw.satellites_visible = gps_pos.num_satellites;
            
            /* Send via telemetry */
            mavlink_send_gps_raw(&gps_raw);
        }
        
        /* Task runs at 10Hz */
        task_sleep(100000);
    }
}

/*
 * Telemetry task - Time-triggered
 */
static void telemetry_task(void *unused)
{
    struct mavlink_heartbeat heartbeat = {
        .custom_mode = 0,
        .type = 2,      /* Quadrotor */
        .autopilot = 12, /* PX4 */
        .base_mode = 81, /* Armed, manual */
        .system_status = 4, /* Active */
        .mavlink_version = 3
    };
    
    printk("[TASK] Telemetry task started\n");
    
    while (1) {
        /* Send heartbeat at 1Hz */
        mavlink_send_heartbeat(&heartbeat);
        
        task_sleep(1000000);
    }
}

/*
 * SATCOM task - Event-driven
 */
static void satcom_task(void *unused)
{
    uint8_t msg[256];
    uint32_t msg_len;
    
    printk("[TASK] SATCOM task started\n");
    
    while (1) {
        /* Check for incoming messages */
        int queued = iridium_poll_mt_message();
        if (queued > 0) {
            msg_len = sizeof(msg);
            if (iridium_receive_sbd(msg, &msg_len) == 0 && msg_len > 0) {
                printk("[SATCOM] Received %d byte message\n", msg_len);
            }
        }
        
        /* Check signal strength periodically */
        uint32_t signal;
        if (iridium_get_signal_strength(&signal) == 0) {
            /* Store for health monitoring */
        }
        
        task_sleep(5000000);  /* 5 second poll interval */
    }
}

/*
 * Main kernel entry point
 */
void kernel_main(uint64_t dtb_addr)
{
    int ret;
    
    /* Record boot time */
    sys_info.boot_time = get_time_us();
    sys_info.version = (CULE_VERSION_MAJOR << 16) | 
                       (CULE_VERSION_MINOR << 8) | 
                        CULE_VERSION_PATCH;
    
    /* Early console output */
    print_banner();
    
    printk("Booting from DTB at 0x%016lx\n", dtb_addr);
    
    /* Initialize kernel */
    ret = kernel_init(NULL);
    if (ret < 0) {
        printk("Kernel initialization failed: %d\n", ret);
        panic("Cannot continue");
    }
    
    /* Create system tasks */
    printk("Creating system tasks...\n");
    
    /* Idle task - NIS policy */
    struct task_struct *idle = task_create("idle", idle_task, NULL, SCHED_NIS);
    if (!idle)
        panic("Failed to create idle task");
    
    /* Sensor fusion - TT policy, 100ms period */
    struct task_struct *sensor = task_create("sensor", sensor_fusion_task, NULL, SCHED_TT);
    if (sensor) {
        extern int tt_task_register(struct task_struct *, uint64_t, uint64_t, uint64_t);
        tt_task_register(sensor, 100000, 50000, 10000);
    }
    
    /* Telemetry - TT policy, 1000ms period */
    struct task_struct *telem = task_create("telemetry", telemetry_task, NULL, SCHED_TT);
    if (telem) {
        extern int tt_task_register(struct task_struct *, uint64_t, uint64_t, uint64_t);
        tt_task_register(telem, 1000000, 500000, 10000);
    }
    
    /* SATCOM - NIS policy */
    struct task_struct *satcom = task_create("satcom", satcom_task, NULL, SCHED_NIS);
    if (!satcom)
        printk("Warning: Failed to create SATCOM task\n");
    
    printk("System tasks created. Starting scheduler...\n\n");
    
    /* Enable interrupts and start scheduling */
    __asm__ volatile("msr daifclr, #0xf" ::: "memory");
    
    /* Start scheduling */
    current = idle;
    idle->state = TASK_RUNNING;
    idle_task(NULL);
    
    /* Should never reach here */
    panic("Scheduler returned");
}

/*
 * Panic handler
 */
void panic(const char *msg)
{
    printk("\n*** KERNEL PANIC ***\n");
    printk("%s\n", msg);
    printk("System halted.\n");
    
    /* Disable interrupts and halt */
    __asm__ volatile("msr daifset, #0xf" ::: "memory");
    
    while (1) {
        __asm__ volatile("wfi" ::: "memory");
    }
}

/*
 * Exception handlers
 */
void handle_sync_exception(uint64_t esr, uint64_t elr, uint64_t spsr)
{
    uint32_t ec = (esr >> 26) & 0x3f;
    uint32_t iss = esr & 0x1ffffff;
    
    printk("Synchronous exception:\n");
    printk("  ESR: 0x%016lx\n", esr);
    printk("  ELR: 0x%016lx\n", elr);
    printk("  SPSR: 0x%016lx\n", spsr);
    printk("  EC: 0x%x, ISS: 0x%x\n", ec, iss);
    
    panic("Sync exception");
}

void handle_irq(void)
{
    /* TODO: Implement interrupt controller handling */
    sched_tick();
}

void secondary_start_kernel(void)
{
    /* Secondary CPU entry point */
    while (1) {
        __asm__ volatile("wfi" ::: "memory");
    }
}

/*
 * Kernel printk - simplified console output
 */
void printk(const char *fmt, ...)
{
    /* Simple implementation - just output to UART */
    extern void early_putc(char);
    
    va_list args;
    va_start(args, fmt);
    
    while (*fmt) {
        if (*fmt == '%' && *(fmt + 1)) {
            fmt++;
            switch (*fmt) {
                case 'd':
                case 'i': {
                    int val = va_arg(args, int);
                    char buf[16];
                    int i = 0;
                    if (val < 0) {
                        early_putc('-');
                        val = -val;
                    }
                    do {
                        buf[i++] = '0' + (val % 10);
                        val /= 10;
                    } while (val > 0);
                    while (i > 0)
                        early_putc(buf[--i]);
                    break;
                }
                case 'u': {
                    unsigned int val = va_arg(args, unsigned int);
                    char buf[16];
                    int i = 0;
                    do {
                        buf[i++] = '0' + (val % 10);
                        val /= 10;
                    } while (val > 0);
                    while (i > 0)
                        early_putc(buf[--i]);
                    break;
                }
                case 'x':
                case 'X': {
                    unsigned long val = va_arg(args, unsigned long);
                    char buf[16];
                    int i = 0;
                    early_putc('0');
                    early_putc('x');
                    do {
                        int digit = val & 0xf;
                        buf[i++] = digit < 10 ? '0' + digit : 'a' + digit - 10;
                        val >>= 4;
                    } while (val > 0);
                    while (i < 16)
                        buf[i++] = '0';
                    while (i > 0)
                        early_putc(buf[--i]);
                    break;
                }
                case 'l': {
                    fmt++;
                    if (*fmt == 'u' || *fmt == 'd') {
                        unsigned long val = va_arg(args, unsigned long);
                        char buf[32];
                        int i = 0;
                        do {
                            buf[i++] = '0' + (val % 10);
                            val /= 10;
                        } while (val > 0);
                        while (i > 0)
                            early_putc(buf[--i]);
                    } else if (*fmt == 'x') {
                        unsigned long val = va_arg(args, unsigned long);
                        char buf[16];
                        int i = 0;
                        early_putc('0');
                        early_putc('x');
                        do {
                            int digit = val & 0xf;
                            buf[i++] = digit < 10 ? '0' + digit : 'a' + digit - 10;
                            val >>= 4;
                        } while (val > 0);
                        while (i < 16)
                            buf[i++] = '0';
                        while (i > 0)
                            early_putc(buf[--i]);
                    }
                    break;
                }
                case 's': {
                    const char *str = va_arg(args, const char *);
                    while (str && *str)
                        early_putc(*str++);
                    break;
                }
                case 'c': {
                    char c = (char)va_arg(args, int);
                    early_putc(c);
                    break;
                }
                case '%':
                    early_putc('%');
                    break;
                default:
                    early_putc('%');
                    early_putc(*fmt);
                    break;
            }
        } else {
            early_putc(*fmt);
        }
        fmt++;
    }
    
    va_end(args);
}

#include <stdarg.h>
