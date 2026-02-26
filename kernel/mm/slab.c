#include "mm.h"
#include "linux/list.h"
#include "linux/spinlock.h"
#include <string.h>

#define SLAB_MIN_SIZE       16
#define SLAB_MAX_SIZE       2048
#define SLAB_NUM_SIZES      8

static const size_t slab_sizes[SLAB_NUM_SIZES] = {
    16, 32, 64, 128, 256, 512, 1024, 2048
};

struct slab_obj {
    struct list_head list;
};

struct kmem_cache {
    size_t size;
    size_t objsize;
    struct list_head slabs_partial;
    struct list_head slabs_full;
    struct list_head slabs_free;
    spinlock_t lock;
};

static struct kmem_cache kmalloc_caches[SLAB_NUM_SIZES];

struct slab {
    struct list_head list;
    struct kmem_cache *cache;
    void *s_mem;
    uint32_t inuse;
    uint32_t free;
    struct slab_obj free_list;
};

static inline size_t slab_index(size_t size)
{
    for (int i = 0; i < SLAB_NUM_SIZES; i++) {
        if (slab_sizes[i] >= size)
            return i;
    }
    return SLAB_NUM_SIZES - 1;
}

static struct slab *alloc_slab(struct kmem_cache *cache)
{
    struct slab *slab;
    struct page *page;
    void *obj;
    uint32_t num_objs;
    uint32_t i;
    
    page = alloc_page();
    if (!page)
        return NULL;
    
    slab = (struct slab *)page;
    slab->cache = cache;
    slab->s_mem = (void *)page + sizeof(struct slab);
    slab->inuse = 0;
    
    num_objs = (PAGE_SIZE - sizeof(struct slab)) / cache->objsize;
    slab->free = num_objs;
    
    INIT_LIST_HEAD(&slab->list);
    INIT_LIST_HEAD(&slab->free_list.list);
    
    for (i = 0; i < num_objs; i++) {
        obj = slab->s_mem + i * cache->objsize;
        ((struct slab_obj *)obj)->list.next = slab->free_list.list.next;
        slab->free_list.list.next = &((struct slab_obj *)obj)->list;
    }
    
    return slab;
}

static void free_slab(struct slab *slab)
{
    free_page(slab);
}

int slab_init(void)
{
    for (int i = 0; i < SLAB_NUM_SIZES; i++) {
        kmalloc_caches[i].size = slab_sizes[i];
        kmalloc_caches[i].objsize = ALIGN(slab_sizes[i], sizeof(void *));
        INIT_LIST_HEAD(&kmalloc_caches[i].slabs_partial);
        INIT_LIST_HEAD(&kmalloc_caches[i].slabs_full);
        INIT_LIST_HEAD(&kmalloc_caches[i].slabs_free);
        spin_lock_init(&kmalloc_caches[i].lock);
    }
    return 0;
}

void *kmalloc(size_t size)
{
    struct kmem_cache *cache;
    struct slab *slab;
    struct slab_obj *obj;
    size_t idx;
    
    if (size == 0)
        return NULL;
    
    if (size > SLAB_MAX_SIZE) {
        uint32_t order = 0;
        size_t alloc_size = PAGE_SIZE;
        
        while (alloc_size < size) {
            alloc_size <<= 1;
            order++;
        }
        
        return alloc_pages(order);
    }
    
    idx = slab_index(size);
    cache = &kmalloc_caches[idx];
    
    spin_lock(&cache->lock);
    
    if (!list_empty(&cache->slabs_partial)) {
        slab = list_entry(cache->slabs_partial.next, struct slab, list);
    } else if (!list_empty(&cache->slabs_free)) {
        slab = list_entry(cache->slabs_free.next, struct slab, list);
        list_del(&slab->list);
        list_add(&slab->list, &cache->slabs_partial);
    } else {
        slab = alloc_slab(cache);
        if (!slab) {
            spin_unlock(&cache->lock);
            return NULL;
        }
        list_add(&slab->list, &cache->slabs_partial);
    }
    
    obj = list_entry(slab->free_list.list.next, struct slab_obj, list);
    list_del(&obj->list);
    
    slab->inuse++;
    slab->free--;
    
    if (slab->free == 0) {
        list_del(&slab->list);
        list_add(&slab->list, &cache->slabs_full);
    }
    
    spin_unlock(&cache->lock);
    
    return obj;
}

void kfree(void *ptr)
{
    struct page *page;
    struct slab *slab;
    struct kmem_cache *cache;
    struct slab_obj *obj;
    
    if (!ptr)
        return;
    
    page = pfn_to_page(PFN_DOWN(VA_TO_PA(ptr)));
    
    if (page->flags == 0) {
        free_page(ptr);
        return;
    }
    
    slab = (struct slab *)page;
    cache = slab->cache;
    obj = (struct slab_obj *)ptr;
    
    spin_lock(&cache->lock);
    
    obj->list.next = slab->free_list.list.next;
    slab->free_list.list.next = &obj->list;
    
    slab->inuse--;
    slab->free++;
    
    if (slab->inuse == 0) {
        list_del(&slab->list);
        list_add(&slab->list, &cache->slabs_free);
    } else if (slab->free == 1) {
        list_del(&slab->list);
        list_add(&slab->list, &cache->slabs_partial);
    }
    
    spin_unlock(&cache->lock);
}