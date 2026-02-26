/*
 * Cule OS Kernel Entry Point
 * ARM64 assembly boot code
 */

#include <asm/asm-offsets.h>

.section ".boot.text", "ax"

.global _start
.global _stext
.global __boot_cpu_mode
.global secondary_startup

/*
 * Kernel entry point
 * Expected to be loaded at 0x80000 (512KB)
 * x0 = dtb physical address
 */
_start:
    /* Save dtb pointer */
    mov x20, x0
    
    /* Check processor ID */
    mrs x0, mpidr_el1
    and x0, x0, #0xFF
    cbz x0, primary_cpu
    
    /* Secondary CPUs - park for now */
secondary_park:
    wfe
    b secondary_park

primary_cpu:
    /* Disable interrupts */
    msr daifset, #0xf
    
    /* Ensure we're in EL2 or higher */
    mrs x0, currentel
    cmp x0, #0x8
    b.lt 1f
    
    /* Running in EL2 - setup and drop to EL1 */
    msr sp_el1, xzr
    
    /* Setup Hypervisor Configuration */
    mov x0, #0x33ff
    msr cptr_el2, x0
    msr hstr_el2, xzr
    mov x0, #3 << 20
    msr cpacr_el1, x0
    
    /* Setup SCTLR for EL1 */
    ldr x0, =0x30d00800
    msr sctlr_el1, x0
    
    /* Setup exception vectors */
    ldr x0, =vectors
    msr vbar_el1, x0
    
    /* Setup EL1 entry */
    mov x0, #0x3c5
    msr spsr_el2, x0
    adr x0, el1_entry
    msr elr_el2, x0
    eret

1:  /* Running in EL1 */
    adr x0, el1_entry
    br x0

el1_entry:
    /* Setup stack */
    ldr x0, =_end
    mov sp, x0
    
    /* Clear BSS */
    ldr x0, =_bss
    ldr x1, =_ebss
    sub x1, x1, x0
1:  cbz x1, 2f
    str xzr, [x0], #8
    sub x1, x1, #8
    b 1b
2:
    /* Jump to C code */
    mov x0, x20
    bl kernel_main
    
    /* Should never return */
    b .

/*
 * Secondary CPU startup
 */
secondary_startup:
    /* Similar setup for secondary CPUs */
    mrs x0, mpidr_el1
    and x0, x0, #0xFF
    
    /* Setup stack for this CPU */
    ldr x1, =_end
    sub x1, x1, x0, lsl #14
    mov sp, x1
    
    b secondary_start_kernel

/*
 * Early UART output for debugging
 * x0 = character to print
 */
.global early_putc
early_putc:
    ldr x1, =0x3100000
1:  ldr w2, [x1, #0x18]
    and w2, w2, #0x20
    cbz w2, 1b
    str w0, [x1]
    ret

/*
 * Exception vectors
 */
.section ".text.vectors", "ax"
.balign 2048
.global vectors
vectors:
    /* Current EL with SP0 */
    b .   /* Synchronous */
    b .   /* IRQ */
    b .   /* FIQ */
    b .   /* SError */
    
    /* Current EL with SPx */
    b el1_sync
    b el1_irq
    b el1_fiq
    b el1_error
    
    /* Lower EL using AArch64 */
    b .   /* Synchronous */
    b .   /* IRQ */
    b .   /* FIQ */
    b .   /* SError */
    
    /* Lower EL using AArch32 */
    b .   /* Synchronous */
    b .   /* IRQ */
    b .   /* FIQ */
    b .   /* SError */

/*
 * EL1 synchronous exception handler
 */
el1_sync:
    /* Save context */
    sub sp, sp, #256
    stp x0, x1, [sp, #16 * 0]
    stp x2, x3, [sp, #16 * 1]
    stp x4, x5, [sp, #16 * 2]
    stp x6, x7, [sp, #16 * 3]
    stp x8, x9, [sp, #16 * 4]
    stp x10, x11, [sp, #16 * 5]
    stp x12, x13, [sp, #16 * 6]
    stp x14, x15, [sp, #16 * 7]
    stp x16, x17, [sp, #16 * 8]
    stp x18, x19, [sp, #16 * 9]
    stp x20, x21, [sp, #16 * 10]
    stp x22, x23, [sp, #16 * 11]
    stp x24, x25, [sp, #16 * 12]
    stp x26, x27, [sp, #16 * 13]
    stp x28, x29, [sp, #16 * 14]
    str x30, [sp, #16 * 15]
    
    mrs x0, esr_el1
    mrs x1, elr_el1
    mrs x2, spsr_el1
    
    bl handle_sync_exception
    
    /* Restore context */
    ldp x0, x1, [sp, #16 * 0]
    ldp x2, x3, [sp, #16 * 1]
    ldp x4, x5, [sp, #16 * 2]
    ldp x6, x7, [sp, #16 * 3]
    ldp x8, x9, [sp, #16 * 4]
    ldp x10, x11, [sp, #16 * 5]
    ldp x12, x13, [sp, #16 * 6]
    ldp x14, x15, [sp, #16 * 7]
    ldp x16, x17, [sp, #16 * 8]
    ldp x18, x19, [sp, #16 * 9]
    ldp x20, x21, [sp, #16 * 10]
    ldp x22, x23, [sp, #16 * 11]
    ldp x24, x25, [sp, #16 * 12]
    ldp x26, x27, [sp, #16 * 13]
    ldp x28, x29, [sp, #16 * 14]
    ldr x30, [sp, #16 * 15]
    add sp, sp, #256
    eret

/*
 * EL1 IRQ handler
 */
el1_irq:
    sub sp, sp, #256
    stp x0, x1, [sp, #16 * 0]
    stp x2, x3, [sp, #16 * 1]
    stp x4, x5, [sp, #16 * 2]
    stp x6, x7, [sp, #16 * 3]
    stp x8, x9, [sp, #16 * 4]
    stp x10, x11, [sp, #16 * 5]
    stp x12, x13, [sp, #16 * 6]
    stp x14, x15, [sp, #16 * 7]
    stp x16, x17, [sp, #16 * 8]
    stp x18, x19, [sp, #16 * 9]
    stp x20, x21, [sp, #16 * 10]
    stp x22, x23, [sp, #16 * 11]
    stp x24, x25, [sp, #16 * 12]
    stp x26, x27, [sp, #16 * 13]
    stp x28, x29, [sp, #16 * 14]
    str x30, [sp, #16 * 15]
    
    bl handle_irq
    
    ldp x0, x1, [sp, #16 * 0]
    ldp x2, x3, [sp, #16 * 1]
    ldp x4, x5, [sp, #16 * 2]
    ldp x6, x7, [sp, #16 * 3]
    ldp x8, x9, [sp, #16 * 4]
    ldp x10, x11, [sp, #16 * 5]
    ldp x12, x13, [sp, #16 * 6]
    ldp x14, x15, [sp, #16 * 7]
    ldp x16, x17, [sp, #16 * 8]
    ldp x18, x19, [sp, #16 * 9]
    ldp x20, x21, [sp, #16 * 10]
    ldp x22, x23, [sp, #16 * 11]
    ldp x24, x25, [sp, #16 * 12]
    ldp x26, x27, [sp, #16 * 13]
    ldp x28, x29, [sp, #16 * 14]
    ldr x30, [sp, #16 * 15]
    add sp, sp, #256
    eret

el1_fiq:
    b el1_irq

el1_error:
    b .

.section ".data"
__boot_cpu_mode:
    .quad 0
