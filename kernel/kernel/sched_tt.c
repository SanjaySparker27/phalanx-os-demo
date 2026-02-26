#include "linux/sched.h"
#include "linux/list.h"
#include "linux/spinlock.h"
#include <string.h>

struct tt_task {
    struct task_struct *task;
    uint64_t period_us;
    uint64_t deadline_us;
    uint64_t wcet_us;
    uint64_t next_release;
    uint64_t deadline_abs;
    struct list_head tt_list;
};

static struct list_head tt_tasks;
static spinlock_t tt_lock = SPINLOCK_INIT;

int tt_sched_init(void)
{
    INIT_LIST_HEAD(&tt_tasks);
    return 0;
}

int tt_task_register(struct task_struct *task, uint64_t period_us, 
                     uint64_t deadline_us, uint64_t wcet_us)
{
    struct tt_task *tt;
    
    tt = kzalloc(sizeof(*tt));
    if (!tt)
        return -ENOMEM;
    
    tt->task = task;
    tt->period_us = period_us;
    tt->deadline_us = deadline_us;
    tt->wcet_us = wcet_us;
    tt->next_release = get_time_us();
    tt->deadline_abs = tt->next_release + deadline_us;
    
    task->period_us = period_us;
    task->deadline_us = deadline_us;
    task->wcet_us = wcet_us;
    task->next_release = tt->next_release;
    task->deadline_abs = tt->deadline_abs;
    task->sched_policy = SCHED_TT;
    
    spin_lock(&tt_lock);
    list_add_tail(&tt->tt_list, &tt_tasks);
    spin_unlock(&tt_lock);
    
    return 0;
}

void tt_sched_tick(void)
{
    struct tt_task *tt;
    uint64_t now = get_time_us();
    
    spin_lock(&tt_lock);
    
    list_for_each_entry(tt, &tt_tasks, tt_list) {
        if (now >= tt->next_release && tt->task->state == TASK_SLEEPING) {
            tt->task->state = TASK_READY;
            tt->next_release += tt->period_us;
            tt->deadline_abs = now + tt->deadline_us;
            tt->task->next_release = tt->next_release;
            tt->task->deadline_abs = tt->deadline_abs;
            
            spin_lock(&runqueue_lock);
            list_add_tail(&tt->task->run_list, &runqueue);
            spin_unlock(&runqueue_lock);
        }
    }
    
    spin_unlock(&tt_lock);
}

struct task_struct *tt_pick_next(void)
{
    struct tt_task *tt, *earliest = NULL;
    uint64_t earliest_deadline = ~0ULL;
    
    spin_lock(&tt_lock);
    
    list_for_each_entry(tt, &tt_tasks, tt_list) {
        if (tt->task->state == TASK_READY || tt->task->state == TASK_RUNNING) {
            if (tt->deadline_abs < earliest_deadline) {
                earliest_deadline = tt->deadline_abs;
                earliest = tt;
            }
        }
    }
    
    spin_unlock(&tt_lock);
    
    return earliest ? earliest->task : NULL;
}

bool tt_schedulable(struct tt_task *new_task)
{
    struct tt_task *tt;
    double utilization = 0.0;
    
    utilization = (double)new_task->wcet_us / new_task->period_us;
    
    spin_lock(&tt_lock);
    list_for_each_entry(tt, &tt_tasks, tt_list) {
        utilization += (double)tt->wcet_us / tt->period_us;
    }
    spin_unlock(&tt_lock);
    
    return utilization <= 1.0;
}
