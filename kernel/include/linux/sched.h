#ifndef _LINUX_SCHED_H
#define _LINUX_SCHED_H

#include "types.h"
#include "list.h"
#include "spinlock.h"

#define TASK_NAME_LEN       32
#define MAX_TASKS           64
#define STACK_SIZE          8192

#define TASK_RUNNING        0
#define TASK_READY          1
#define TASK_BLOCKED        2
#define TASK_SLEEPING       3
#define TASK_ZOMBIE         4

#define SCHED_TT            0
#define SCHED_NIS           1

struct regs {
    u64 x0, x1, x2, x3, x4, x5, x6, x7;
    u64 x8, x9, x10, x11, x12, x13, x14, x15;
    u64 x16, x17, x18, x19, x20, x21, x22, x23;
    u64 x24, x25, x26, x27, x28;
    u64 fp, lr, sp, pc;
    u64 pstate;
};

struct task_struct {
    uint32_t pid;
    char name[TASK_NAME_LEN];
    volatile int state;
    uint32_t sched_policy;
    
    uint64_t period_us;
    uint64_t deadline_us;
    uint64_t wcet_us;
    uint64_t next_release;
    uint64_t deadline_abs;
    
    int priority;
    uint64_t vruntime;
    
    struct regs *regs;
    uint8_t *stack;
    
    struct list_head run_list;
    struct list_head all_tasks;
    
    void *private_data;
};

extern struct task_struct *current;
extern struct list_head runqueue;
extern spinlock_t runqueue_lock;

extern int sched_init(void);
extern void schedule(void);
extern void sched_tick(void);
extern struct task_struct *task_create(const char *name, void (*fn)(void *), 
                                        void *arg, uint32_t policy);
extern void task_sleep(uint64_t us);
extern void task_wake(struct task_struct *task);

static inline void yield(void)
{
    schedule();
}

#endif
