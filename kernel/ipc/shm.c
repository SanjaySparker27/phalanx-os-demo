#include "ipc.h"
#include "mm/mm.h"
#include <string.h>

static struct list_head shm_regions;
static uint32_t next_shm_id = 1;
static spinlock_t shm_lock;

int shm_init(void)
{
    INIT_LIST_HEAD(&shm_regions);
    spin_lock_init(&shm_lock);
    return 0;
}

void *shmget(uint32_t key, size_t size, int flags)
{
    struct shm_region *shm;
    void *addr;
    
    spin_lock(&shm_lock);
    
    list_for_each_entry(shm, &shm_regions, list) {
        if (shm->id == key) {
            spin_unlock(&shm_lock);
            return shm->addr;
        }
    }
    
    if (!(flags & 0x200)) {
        spin_unlock(&shm_lock);
        return NULL;
    }
    
    shm = kmalloc(sizeof(*shm));
    if (!shm) {
        spin_unlock(&shm_lock);
        return NULL;
    }
    
    size = ALIGN(size, PAGE_SIZE);
    addr = alloc_pages(0);
    if (!addr) {
        kfree(shm);
        spin_unlock(&shm_lock);
        return NULL;
    }
    
    for (uint32_t i = 1; i < (size / PAGE_SIZE); i++) {
        void *p = alloc_page();
        if (!p) {
            for (uint32_t j = 0; j < i; j++)
                free_page(addr + j * PAGE_SIZE);
            kfree(shm);
            spin_unlock(&shm_lock);
            return NULL;
        }
    }
    
    memset(addr, 0, size);
    
    shm->id = next_shm_id++;
    shm->addr = addr;
    shm->size = size;
    shm->ref_count = 0;
    shm->permissions = flags & 0x1FF;
    shm->attached = false;
    spin_lock_init(&shm->lock);
    list_add_tail(&shm->list, &shm_regions);
    
    spin_unlock(&shm_lock);
    
    return addr;
}

int shmat(uint32_t shmid, void *addr, int flags)
{
    struct shm_region *shm;
    
    spin_lock(&shm_lock);
    
    list_for_each_entry(shm, &shm_regions, list) {
        if (shm->id == shmid) {
            spin_lock(&shm->lock);
            shm->ref_count++;
            shm->attached = true;
            spin_unlock(&shm->lock);
            spin_unlock(&shm_lock);
            return 0;
        }
    }
    
    spin_unlock(&shm_lock);
    return -EINVAL;
}

int shmdt(uint32_t shmid)
{
    struct shm_region *shm;
    
    spin_lock(&shm_lock);
    
    list_for_each_entry(shm, &shm_regions, list) {
        if (shm->id == shmid) {
            spin_lock(&shm->lock);
            
            if (shm->ref_count > 0)
                shm->ref_count--;
            
            if (shm->ref_count == 0)
                shm->attached = false;
            
            spin_unlock(&shm->lock);
            spin_unlock(&shm_lock);
            return 0;
        }
    }
    
    spin_unlock(&shm_lock);
    return -EINVAL;
}

int shmctl(uint32_t shmid, int cmd, void *buf)
{
    struct shm_region *shm, *tmp;
    
    if (cmd == 0) {
        spin_lock(&shm_lock);
        
        list_for_each_entry_safe(shm, tmp, &shm_regions, list) {
            if (shm->id == shmid) {
                spin_lock(&shm->lock);
                
                if (shm->ref_count == 0) {
                    free_pages(shm->addr, 0);
                    list_del(&shm->list);
                    spin_unlock(&shm->lock);
                    kfree(shm);
                    spin_unlock(&shm_lock);
                    return 0;
                }
                
                spin_unlock(&shm->lock);
            }
        }
        
        spin_unlock(&shm_lock);
    }
    
    return -EINVAL;
}