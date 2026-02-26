#include "linux/sched.h"
#include "linux/list.h"
#include "linux/spinlock.h"
#include <string.h>

#define NIS_MIN_GRANULARITY     1000
#define NIS_LATENCY_TARGET      10000
#define NIS_PRIO_TO_WEIGHT(p)   (1024 >> ((p) >> 3))

struct nis_rq {
    struct list_head tasks;
    uint64_t min_vruntime;
    uint32_t nr_running;
    spinlock_t lock;
};

static struct nis_rq nis_rq;

int nis_sched_init(void)
{
    INIT_LIST_HEAD(&nis_rq.tasks);
    nis_rq.min_vruntime = 0;
    nis_rq.nr_running = 0;
    spin_lock_init(&nis_rq.lock);
    return 0;
}

int nis_task_register(struct task_struct *task, int priority)
{
    task->priority = priority;
    task->vruntime = nis_rq.min_vruntime;
    task->sched_policy = SCHED_NIS;
    return 0;
}

void nis_task_enqueue(struct task_struct *task)
{
    struct task_struct *pos;
    
    spin_lock(&nis_rq.lock);
    
    if (list_empty(&nis_rq.tasks)) {
        list_add(&task->run_list, &nis_rq.tasks);
    } else {
        list_for_each_entry(pos, &nis_rq.tasks, run_list) {
            if (task->vruntime < pos->vruntime) {
                list_add_tail(&task->run_list, &pos->run_list);
                goto done;
            }
        }
        list_add_tail(&task->run_list, &nis_rq.tasks);
    }
done:
    nis_rq.nr_running++;
    spin_unlock(&nis_rq.lock);
}

void nis_task_dequeue(struct task_struct *task)
{
    spin_lock(&nis_rq.lock);
    list_del(&task->run_list);
    nis_rq.nr_running--;
    spin_unlock(&nis_rq.lock);
}

struct task_struct *nis_pick_next(void)
{
    struct task_struct *next;
    
    spin_lock(&nis_rq.lock);
    
    if (list_empty(&nis_rq.tasks)) {
        spin_unlock(&nis_rq.lock);
        return NULL;
    }
    
    next = list_entry(nis_rq.tasks.next, struct task_struct, run_list);
    
    spin_unlock(&nis_rq.lock);
    
    return next;
}

void nis_update_vruntime(struct task_struct *task, uint64_t delta_us)
{
    uint64_t weight = NIS_PRIO_TO_WEIGHT(task->priority);
    task->vruntime += (delta_us * 1024) / weight;
    
    spin_lock(&nis_rq.lock);
    if (task->vruntime > nis_rq.min_vruntime)
        nis_rq.min_vruntime = task->vruntime;
    spin_unlock(&nis_rq.lock);
}

uint64_t nis_slice(struct task_struct *task)
{
    uint64_t slice = NIS_LATENCY_TARGET / (nis_rq.nr_running ?: 1);
    if (slice < NIS_MIN_GRANULARITY)
        slice = NIS_MIN_GRANULARITY;
    return slice;
}

void nis_sched_tick(void)
{
    struct task_struct *curr = current;
    
    if (curr->sched_policy != SCHED_NIS)
        return;
    
    nis_update_vruntime(curr, 1000);
    
    if (curr->vruntime - nis_rq.min_vruntime > nis_slice(curr)) {
        curr->state = TASK_READY;
        nis_task_enqueue(curr);
        schedule();
    }
}
