#ifndef _MM_H
#define _MM_H

#include "linux/types.h"

#define PAGE_SHIFT      12
#define PAGE_SIZE       (1UL << PAGE_SHIFT)
#define PAGE_MASK       (~(PAGE_SIZE - 1))

#define PAGE_PRESENT    0x001
#define PAGE_RW         0x002
#define PAGE_USER       0x004
#define PAGE_ACCESSED   0x020
#define PAGE_DIRTY      0x040
#define PAGE_PTE_ATTR   (PAGE_PRESENT | PAGE_RW | PAGE_ACCESSED | PAGE_DIRTY)

#define PHYS_OFFSET     0x80000000
#define KERNEL_OFFSET   0xFFFFFF8000000000UL

#define VA_TO_PA(va)    ((uint64_t)(va) - KERNEL_OFFSET)
#define PA_TO_VA(pa)    ((void *)((uint64_t)(pa) + KERNEL_OFFSET))

#define PFN_UP(x)       (((x) + PAGE_SIZE - 1) >> PAGE_SHIFT)
#define PFN_DOWN(x)     ((x) >> PAGE_SHIFT)
#define PFN_PHYS(x)     ((uint64_t)(x) << PAGE_SHIFT)

#define PGD_SHIFT       39
#define PUD_SHIFT       30
#define PMD_SHIFT       21
#define PTE_SHIFT       12

#define PTRS_PER_PGD    512
#define PTRS_PER_PUD    512
#define PTRS_PER_PMD    512
#define PTRS_PER_PTE    512

#define PGD_INDEX(va)   (((va) >> PGD_SHIFT) & (PTRS_PER_PGD - 1))
#define PUD_INDEX(va)   (((va) >> PUD_SHIFT) & (PTRS_PER_PUD - 1))
#define PMD_INDEX(va)   (((va) >> PMD_SHIFT) & (PTRS_PER_PMD - 1))
#define PTE_INDEX(va)   (((va) >> PTE_SHIFT) & (PTRS_PER_PTE - 1))

typedef uint64_t pgd_t;
typedef uint64_t pud_t;
typedef uint64_t pmd_t;
typedef uint64_t pte_t;

struct page {
    uint32_t flags;
    uint32_t refcount;
    struct list_head lru;
    void *virtual;
};

#define PG_reserved     0
#define PG_private      1
#define PG_locked       2
#define PG_dirty        3

struct mem_section {
    uint64_t start_pfn;
    uint64_t end_pfn;
    struct page *page_array;
    uint64_t free_pages;
    spinlock_t lock;
};

int mm_init(uint64_t mem_start, uint64_t mem_size);
int paging_init(void);
void *alloc_page(void);
void free_page(void *page);
void *alloc_pages(uint32_t order);
void free_pages(void *pages, uint32_t order);
void *kmalloc(size_t size);
void kfree(void *ptr);
int map_page(uint64_t vaddr, uint64_t paddr, uint64_t flags);
void unmap_page(uint64_t vaddr);
uint64_t virt_to_phys(void *vaddr);
void *phys_to_virt(uint64_t paddr);

#endif