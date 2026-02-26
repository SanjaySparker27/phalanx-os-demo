#ifndef _LINUX_SPINLOCK_H
#define _LINUX_SPINLOCK_H

#include <stdatomic.h>

typedef struct {
    atomic_flag flag;
} spinlock_t;

#define SPINLOCK_INIT { ATOMIC_FLAG_INIT }

static inline void spin_lock_init(spinlock_t *lock)
{
    atomic_flag_clear(&lock->flag);
}

static inline void spin_lock(spinlock_t *lock)
{
    while (atomic_flag_test_and_set(&lock->flag))
        ;
}

static inline void spin_unlock(spinlock_t *lock)
{
    atomic_flag_clear(&lock->flag);
}

static inline bool spin_trylock(spinlock_t *lock)
{
    return !atomic_flag_test_and_set(&lock->lock);
}

#define spin_lock_irqsave(lock, flags) \
    do { flags = 0; spin_lock(lock); } while (0)
#define spin_unlock_irqrestore(lock, flags) \
    do { spin_unlock(lock); } while (0)

#endif
