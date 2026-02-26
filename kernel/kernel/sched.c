#include "linux/sched.h"
#include "linux/list.h"
#include "linux/spinlock.h"
#include <string.h>

struct task_struct *current = NULL;
struct list_head runqueue;
spinlock_t runqueue_lock = SPINLOCK_INIT;

static struct task_struct task_table[MAX_TASKS];
static uint32_t next_pid = 1;
static uint32_t num_tasks = 0;

extern int tt_sched_init(void);
extern int nis_sched_init(void);
extern void tt_sched_tick(void);
extern void nis_sched_tick(void);
extern struct task_struct *tt_pick_next(void);
extern struct task_struct *nis_pick_next(void);
extern void nis_task_enqueue(struct task_struct *task);
extern void nis_task_dequeue(struct task_struct *task);

static void *kzalloc(size_t size)
{
    void *ptr = malloc(size);
    if (ptr)
        memset(ptr, 0, size);
    return ptr;
}

int sched_init(void)
{
    INIT_LIST_HEAD(&runqueue);
    memset(task_table, 0, sizeof(task_table));
    
    tt_sched_init();
    nis_sched_init();
    
    return 0;
}

struct task_struct *task_create(const char *name, void (*fn)(void *),
                                 void *arg, uint32_t policy)
{
    struct task_struct *task;
    
    if (num_tasks >= MAX_TASKS)
        return NULL;
    
    task = &task_table[num_tasks++];
    
    strncpy(task->name, name, TASK_NAME_LEN - 1);
    task->name[TASK_NAME_LEN - 1] = '\0';
    task->pid = next_pid++;
    task->state = TASK_READY;
    task->sched_policy = policy;
    
    task->stack = kzalloc(STACK_SIZE);
    if (!task->stack) {
        num_tasks--;
        return NULL;
    }
    
    INIT_LIST_HEAD(&task->run_list);
    INIT_LIST_HEAD(&task->all_tasks);
    
    list_add_tail(&task->all_tasks, &runqueue);
    
    if (policy == SCHED_NIS) {
        task->priority = 20;
        nis_task_enqueue(task);
    }
    
    return task;
}

struct task_struct *pick_next_task(void)
{
    struct task_struct *tt_task, *nis_task;
    
    tt_task = tt_pick_next();
    nis_task = nis_pick_next();
    
    if (tt_task && nis_task) {
        return tt_task;
    } else if (tt_task) {
        return tt_task;
    } else {
        return nis_task;
    }
}

void schedule(void)
{
    struct task_struct *prev, *next;
    
    prev = current;
    
    spin_lock(&runqueue_lock);
    next = pick_next_task();
    spin_unlock(&runqueue_lock);
    
    if (!next || next == prev)
        return;
    
    if (prev && prev->state == TASK_RUNNING)
        prev->state = TASK_READY;
    
    next->state = TASK_RUNNING;
    current = next;
    
    context_switch(prev, next);
}

void sched_tick(void)
{
    tt_sched_tick();
    nis_sched_tick();
}

void task_sleep(uint64_t us)
{
    struct task_struct *task = current;
    
    if (!task)
        return;
    
    task->state = TASK_SLEEPING;
    
    if (task->sched_policy == SCHED_NIS)
        nis_task_dequeue(task);
    
    schedule();
}

void task_wake(struct task_struct *task)
{
    if (!task || task->state != TASK_SLEEPING)
        return;
    
    task->state = TASK_READY;
    
    spin_lock(&runqueue_lock);
    
    if (task->sched_policy == SCHED_NIS)
        nis_task_enqueue(task);
    else
        list_add_tail(&task->run_list, &runqueue);
    
    spin_unlock(&runqueue_lock);
}

void context_switch(struct task_struct *prev, struct task_struct *next)
{
    __asm__ volatile(
        "stp x19, x20, [sp, #-16]!\n"
        "stp x21, x22, [sp, #-16]!\n"
        "stp x23, x24, [sp, #-16]!\n"
        "stp x25, x26, [sp, #-16]!\n"
        "stp x27, x28, [sp, #-16]!\n"
        "stp x29, x30, [sp, #-16]!\n"
        "mov x0, sp\n"
        "str x0, [%[prev_regs]]\n"
        "ldr x0, [%[next_regs]]\n"
        "mov sp, x0\n"
        "ldp x29, x30, [sp], #16\n"
        "ldp x27, x28, [sp], #16\n"
        "ldp x25, x26, [sp], #16\n"
        "ldp x23, x24, [sp], #16\n"
        "ldp x21, x22, [sp], #16\n"
        "ldp x19, x20, [sp], #16\n"
        :: [prev_regs] "r"(&prev->regs),
           [next_regs] "r"(&next->regs)
        : "memory"
    );
}
