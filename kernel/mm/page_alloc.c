#include "mm.h"
#include "linux/list.h"
#include "linux/spinlock.h"
#include <string.h>

#define MAX_ORDER       11
#define MIN_BLOCK_SIZE  PAGE_SIZE

struct free_area {
    struct list_head free_list;
    uint32_t nr_free;
};

static struct free_area free_area[MAX_ORDER];
static struct mem_section mem_section;
static struct page *mem_map;
static uint64_t total_pages;

static inline uint32_t page_to_pfn(struct page *page)
{
    return page - mem_map;
}

static inline struct page *pfn_to_page(uint32_t pfn)
{
    return &mem_map[pfn];
}

static inline int page_is_buddy(struct page *page, int order)
{
    return page->flags == (1 << order);
}

static inline void set_page_order(struct page *page, int order)
{
    page->flags = (1 << order);
}

int mm_init(uint64_t mem_start, uint64_t mem_size)
{
    uint64_t num_pages;
    uint32_t pfn;
    
    for (int i = 0; i < MAX_ORDER; i++) {
        INIT_LIST_HEAD(&free_area[i].free_list);
        free_area[i].nr_free = 0;
    }
    
    mem_section.start_pfn = PFN_UP(mem_start);
    mem_section.end_pfn = PFN_DOWN(mem_start + mem_size);
    spin_lock_init(&mem_section.lock);
    
    num_pages = mem_section.end_pfn - mem_section.start_pfn;
    total_pages = num_pages;
    
    mem_map = (struct page *)PA_TO_VA(mem_start);
    memset(mem_map, 0, sizeof(struct page) * num_pages);
    
    for (pfn = 0; pfn < num_pages; pfn++) {
        mem_map[pfn].refcount = 0;
        mem_map[pfn].flags = 0;
        INIT_LIST_HEAD(&mem_map[pfn].lru);
    }
    
    pfn = PFN_UP(sizeof(struct page) * num_pages);
    
    while (pfn < num_pages) {
        int order = MAX_ORDER - 1;
        uint32_t block_size;
        
        while (order >= 0) {
            block_size = 1U << order;
            if (pfn + block_size <= num_pages)
                break;
            order--;
        }
        
        if (order < 0)
            break;
        
        set_page_order(pfn_to_page(pfn), order);
        list_add(&pfn_to_page(pfn)->lru, &free_area[order].free_list);
        free_area[order].nr_free++;
        
        pfn += block_size;
    }
    
    mem_section.free_pages = num_pages - PFN_UP(sizeof(struct page) * num_pages);
    
    return 0;
}

static void expand(struct page *page, int low, int high)
{
    uint32_t size = 1 << high;
    
    while (high > low) {
        high--;
        size >>= 1;
        
        set_page_order(pfn_to_page(page_to_pfn(page) + size), high);
        list_add(&pfn_to_page(page_to_pfn(page) + size)->lru, 
                 &free_area[high].free_list);
        free_area[high].nr_free++;
    }
}

void *alloc_pages(uint32_t order)
{
    struct page *page;
    int current_order;
    
    if (order >= MAX_ORDER)
        return NULL;
    
    spin_lock(&mem_section.lock);
    
    for (current_order = order; current_order < MAX_ORDER; current_order++) {
        if (!list_empty(&free_area[current_order].free_list))
            break;
    }
    
    if (current_order >= MAX_ORDER) {
        spin_unlock(&mem_section.lock);
        return NULL;
    }
    
    page = list_entry(free_area[current_order].free_list.next, 
                      struct page, lru);
    list_del(&page->lru);
    free_area[current_order].nr_free--;
    
    expand(page, order, current_order);
    
    set_page_order(page, 0);
    page->refcount = 1;
    mem_section.free_pages -= (1 << order);
    
    spin_unlock(&mem_section.lock);
    
    return PA_TO_VA(PFN_PHYS(page_to_pfn(page)));
}

void free_pages(void *pages, uint32_t order)
{
    struct page *page;
    uint32_t pfn, buddy_pfn;
    struct page *buddy;
    
    if (!pages)
        return;
    
    pfn = PFN_DOWN(VA_TO_PA(pages));
    page = pfn_to_page(pfn);
    
    spin_lock(&mem_section.lock);
    
    page->refcount = 0;
    
    while (order < MAX_ORDER - 1) {
        buddy_pfn = pfn ^ (1 << order);
        buddy = pfn_to_page(buddy_pfn);
        
        if (!page_is_buddy(buddy, order))
            break;
        
        list_del(&buddy->lru);
        free_area[order].nr_free--;
        
        pfn &= buddy_pfn;
        order++;
    }
    
    set_page_order(pfn_to_page(pfn), order);
    list_add(&pfn_to_page(pfn)->lru, &free_area[order].free_list);
    free_area[order].nr_free++;
    mem_section.free_pages += (1 << order);
    
    spin_unlock(&mem_section.lock);
}

void *alloc_page(void)
{
    return alloc_pages(0);
}

void free_page(void *page)
{
    free_pages(page, 0);
}

uint64_t get_free_pages(void)
{
    return mem_section.free_pages;
}

uint64_t get_total_pages(void)
{
    return total_pages;
}