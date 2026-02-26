#include "ipc.h"
#include "linux/sched.h"
#include <string.h>

int sem_init(struct semaphore *sem, int32_t value)
{
    if (!sem)
        return -EINVAL;
    
    sem->count = value;
    spin_lock_init(&sem->lock);
    INIT_LIST_HEAD(&sem->wait_list);
    
    return 0;
}

void sem_wait(struct semaphore *sem)
{
    struct sem_waiter waiter;
    
    if (!sem)
        return;
    
    spin_lock(&sem->lock);
    
    sem->count--;
    
    if (sem->count < 0) {
        waiter.task = current;
        waiter.up = false;
        INIT_LIST_HEAD(&waiter.list);
        list_add_tail(&waiter.list, &sem->wait_list);
        
        spin_unlock(&sem->lock);
        
        task_sleep(0);
        return;
    }
    
    spin_unlock(&sem->lock);
}

void sem_post(struct semaphore *sem)
{
    struct sem_waiter *waiter;
    
    if (!sem)
        return;
    
    spin_lock(&sem->lock);
    
    sem->count++;
    
    if (sem->count <= 0) {
        if (!list_empty(&sem->wait_list)) {
            waiter = list_entry(sem->wait_list.next, struct sem_waiter, list);
            list_del(&waiter->list);
            waiter->up = true;
            task_wake(waiter->task);
        }
    }
    
    spin_unlock(&sem->lock);
}

bool sem_trywait(struct semaphore *sem)
{
    if (!sem)
        return false;
    
    spin_lock(&sem->lock);
    
    if (sem->count > 0) {
        sem->count--;
        spin_unlock(&sem->lock);
        return true;
    }
    
    spin_unlock(&sem->lock);
    return false;
}

int sem_getvalue(struct semaphore *sem, int32_t *value)
{
    if (!sem || !value)
        return -EINVAL;
    
    spin_lock(&sem->lock);
    *value = sem->count;
    spin_unlock(&sem->lock);
    
    return 0;
}