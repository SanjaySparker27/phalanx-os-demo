/*
 * Cule OS Kernel Linker Script
 * ARM64 architecture layout
 */

OUTPUT_FORMAT("elf64-littleaarch64")
OUTPUT_ARCH(aarch64)

ENTRY(_start)

KERNEL_OFFSET = 0xFFFFFF8000000000;

SECTIONS
{
    /* Boot code runs at physical address */
    . = 0x80000;
    
    .boot : {
        *(.boot.text)
        *(.boot.data)
        . = ALIGN(4096);
        __boot_end = .;
    }
    
    /* Kernel code and data at higher virtual address */
    . = KERNEL_OFFSET + 0x100000;
    
    .text : AT(ADDR(.text) - KERNEL_OFFSET) {
        _text = .;
        *(.text.startup)
        *(.text)
        *(.text.*)
        *(.fixup)
        *(.gnu.warning)
        . = ALIGN(8);
        _etext = .;
    }
    
    .rodata : AT(ADDR(.rodata) - KERNEL_OFFSET) {
        _rodata = .;
        *(.rodata)
        *(.rodata.*)
        . = ALIGN(8);
        __initconst_begin = .;
        *(.init.rodata)
        __initconst_end = .;
        . = ALIGN(4096);
        _erodata = .;
    }
    
    .data : AT(ADDR(.data) - KERNEL_OFFSET) {
        _data = .;
        *(.data)
        *(.data.*)
        . = ALIGN(8);
        _edata = .;
    }
    
    .init : AT(ADDR(.init) - KERNEL_OFFSET) {
        __init_begin = .;
        *(.init.text)
        *(.init.data)
        . = ALIGN(4096);
        __init_end = .;
    }
    
    .bss : AT(ADDR(.bss) - KERNEL_OFFSET) {
        _bss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = .;
    }
    
    /* Page tables and early boot allocations */
    . = ALIGN(4096);
    __init_pg_dir = .;
    . += 4096 * 6;
    __init_pg_dir_end = .;
    
    /* End of kernel image */
    . = ALIGN(4096);
    _end = .;
    
    /* Discard unused sections */
    /DISCARD/ : {
        *(.comment)
        *(.note)
        *(.note.gnu.build-id)
    }
    
    /* Symbol definitions */
    __kernel_virt_offset = KERNEL_OFFSET;
    __kernel_phys_start = 0x100000;
    __kernel_virt_start = KERNEL_OFFSET + 0x100000;
}
