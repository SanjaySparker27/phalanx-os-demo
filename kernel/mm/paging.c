#include "mm.h"
#include <string.h>

static pgd_t *init_pgd;

static inline uint64_t mk_entry(uint64_t paddr, uint64_t flags)
{
    return (paddr & PAGE_MASK) | flags;
}

static inline uint64_t entry_to_phys(uint64_t entry)
{
    return entry & PAGE_MASK;
}

static inline bool entry_present(uint64_t entry)
{
    return entry & PAGE_PRESENT;
}

static pud_t *get_pud_alloc(pgd_t *pgd, uint64_t vaddr)
{
    pgd_t *pgd_entry = &pgd[PGD_INDEX(vaddr)];
    
    if (!entry_present(*pgd_entry)) {
        void *new_pud = alloc_page();
        if (!new_pud)
            return NULL;
        memset(new_pud, 0, PAGE_SIZE);
        *pgd_entry = mk_entry(VA_TO_PA(new_pud), PAGE_PTE_ATTR);
    }
    
    return PA_TO_VA(entry_to_phys(*pgd_entry));
}

static pmd_t *get_pmd_alloc(pud_t *pud, uint64_t vaddr)
{
    pud_t *pud_entry = &pud[PUD_INDEX(vaddr)];
    
    if (!entry_present(*pud_entry)) {
        void *new_pmd = alloc_page();
        if (!new_pmd)
            return NULL;
        memset(new_pmd, 0, PAGE_SIZE);
        *pud_entry = mk_entry(VA_TO_PA(new_pmd), PAGE_PTE_ATTR);
    }
    
    return PA_TO_VA(entry_to_phys(*pud_entry));
}

static pte_t *get_pte_alloc(pmd_t *pmd, uint64_t vaddr)
{
    pmd_t *pmd_entry = &pmd[PMD_INDEX(vaddr)];
    
    if (!entry_present(*pmd_entry)) {
        void *new_pte = alloc_page();
        if (!new_pte)
            return NULL;
        memset(new_pte, 0, PAGE_SIZE);
        *pmd_entry = mk_entry(VA_TO_PA(new_pte), PAGE_PTE_ATTR);
    }
    
    return PA_TO_VA(entry_to_phys(*pmd_entry));
}

int map_page(uint64_t vaddr, uint64_t paddr, uint64_t flags)
{
    pgd_t *pgd;
    pud_t *pud;
    pmd_t *pmd;
    pte_t *pte;
    
    pgd = (pgd_t *)PA_TO_VA(read_sysreg(ttbr0_el1));
    
    pud = get_pud_alloc(pgd, vaddr);
    if (!pud)
        return -ENOMEM;
    
    pmd = get_pmd_alloc(pud, vaddr);
    if (!pmd)
        return -ENOMEM;
    
    pte = get_pte_alloc(pmd, vaddr);
    if (!pte)
        return -ENOMEM;
    
    pte[PTE_INDEX(vaddr)] = mk_entry(paddr, flags);
    
    __asm__ volatile("dsb ishst" ::: "memory");
    __asm__ volatile("tlbi vmalle1is" ::: "memory");
    __asm__ volatile("dsb ish" ::: "memory");
    __asm__ volatile("isb" ::: "memory");
    
    return 0;
}

void unmap_page(uint64_t vaddr)
{
    pgd_t *pgd;
    pud_t *pud;
    pmd_t *pmd;
    pte_t *pte;
    
    pgd = (pgd_t *)PA_TO_VA(read_sysreg(ttbr0_el1));
    
    pud = PA_TO_VA(entry_to_phys(pgd[PGD_INDEX(vaddr)]));
    if (!entry_present(pgd[PGD_INDEX(vaddr)]))
        return;
    
    pmd = PA_TO_VA(entry_to_phys(pud[PUD_INDEX(vaddr)]));
    if (!entry_present(pud[PUD_INDEX(vaddr)]))
        return;
    
    pte = PA_TO_VA(entry_to_phys(pmd[PMD_INDEX(vaddr)]));
    if (!entry_present(pmd[PMD_INDEX(vaddr)]))
        return;
    
    pte[PTE_INDEX(vaddr)] = 0;
    
    __asm__ volatile("dsb ishst" ::: "memory");
    __asm__ volatile("tlbi vae1is, %0" :: "r"(vaddr >> 12) : "memory");
    __asm__ volatile("dsb ish" ::: "memory");
    __asm__ volatile("isb" ::: "memory");
}

uint64_t virt_to_phys(void *vaddr)
{
    uint64_t va = (uint64_t)vaddr;
    pgd_t *pgd;
    pud_t *pud;
    pmd_t *pmd;
    pte_t *pte;
    
    if (va >= KERNEL_OFFSET)
        return VA_TO_PA(va);
    
    pgd = (pgd_t *)PA_TO_VA(read_sysreg(ttbr0_el1));
    
    if (!entry_present(pgd[PGD_INDEX(va)]))
        return 0;
    
    pud = PA_TO_VA(entry_to_phys(pgd[PGD_INDEX(va)]));
    if (!entry_present(pud[PUD_INDEX(va)]))
        return 0;
    
    pmd = PA_TO_VA(entry_to_phys(pud[PUD_INDEX(va)]));
    if (!entry_present(pmd[PMD_INDEX(va)]))
        return 0;
    
    pte = PA_TO_VA(entry_to_phys(pmd[PMD_INDEX(va)]));
    if (!entry_present(pte[PTE_INDEX(va)]))
        return 0;
    
    return entry_to_phys(pte[PTE_INDEX(va)]) | (va & ~PAGE_MASK);
}

void *phys_to_virt(uint64_t paddr)
{
    return PA_TO_VA(paddr);
}

int paging_init(void)
{
    init_pgd = alloc_page();
    if (!init_pgd)
        return -ENOMEM;
    
    memset(init_pgd, 0, PAGE_SIZE);
    
    for (uint64_t va = KERNEL_OFFSET; va < KERNEL_OFFSET + 0x80000000; va += 0x200000) {
        uint64_t pa = va - KERNEL_OFFSET;
        
        pgd_t *pgd = &init_pgd[PGD_INDEX(va)];
        
        if (!entry_present(*pgd)) {
            pud_t *pud = alloc_page();
            if (!pud)
                return -ENOMEM;
            memset(pud, 0, PAGE_SIZE);
            *pgd = mk_entry(VA_TO_PA(pud), PAGE_PTE_ATTR);
        }
        
        pud_t *pud = PA_TO_VA(entry_to_phys(*pgd));
        pud_t *pud_entry = &pud[PUD_INDEX(va)];
        
        if (!entry_present(*pud_entry)) {
            pmd_t *pmd = alloc_page();
            if (!pmd)
                return -ENOMEM;
            memset(pmd, 0, PAGE_SIZE);
            *pud_entry = mk_entry(VA_TO_PA(pmd), PAGE_PTE_ATTR);
        }
        
        pmd_t *pmd = PA_TO_VA(entry_to_phys(*pud_entry));
        pmd[PMD_INDEX(va)] = mk_entry(pa, PAGE_PTE_ATTR | (1 << 10));
    }
    
    write_sysreg(ttbr0_el1, VA_TO_PA(init_pgd));
    
    uint64_t mair = 0xFF << 0;
    write_sysreg(mair_el1, mair);
    
    uint64_t tcr = (16ULL << 0) | (16ULL << 16) | (2ULL << 8) | (2ULL << 24) | (1ULL << 23);
    write_sysreg(tcr_el1, tcr);
    
    uint64_t sctlr = read_sysreg(sctlr_el1);
    sctlr |= (1 << 0) | (1 << 2) | (1 << 12);
    write_sysreg(sctlr_el1, sctlr);
    
    __asm__ volatile("isb" ::: "memory");
    
    return 0;
}

#define read_sysreg(reg) ({ \
    uint64_t val; \
    __asm__ volatile("mrs %0, " #reg : "=r"(val)); \
    val; \
})

#define write_sysreg(reg, val) \
    __asm__ volatile("msr " #reg ", %0" :: "r"(val) : "memory")