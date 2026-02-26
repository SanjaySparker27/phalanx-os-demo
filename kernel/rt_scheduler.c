/**
 * @file rt_scheduler.c
 * @brief Cule OS Preemptible Real-Time Scheduler with TT/NIS
 * 
 * Implements Time-Triggered (TT) and Non-Interruptible Section (NIS)
 * scheduling for deterministic real-time behavior on ARM64.
 * 
 * Target: NVIDIA Jetson AGX Orin (ARM64, PREEMPT_RT)
 * License: GPL-2.0
 */

#define pr_fmt(fmt) "cule_sched: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/percpu.h>
#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/tracepoint.h>
#include <asm/barrier.h>

#include "../include/cule/kernel.h"

MODULE_AUTHOR("Cule OS Team");
MODULE_DESCRIPTION("Cule OS Real-Time Scheduler with TT/NIS");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");

/* Scheduling modes */
enum cule_sched_mode {
	CULE_SCHED_TT,      /* Time-Triggered */
	CULE_SCHED_EDF,     /* Earliest Deadline First */
	CULE_SCHED_FP,      /* Fixed Priority */
};

/* Task states within the scheduler */
enum cule_task_state {
	CULE_TASK_IDLE,
	CULE_TASK_READY,
	CULE_TASK_RUNNING,
	CULE_TASK_BLOCKED,
	CULE_TASK_NIS,      /* In Non-Interruptible Section */
	CULE_TASK_CHECKPOINT, /* Neural checkpoint in progress */
};

/* Task descriptor */
struct cule_task {
	struct list_head list;
	pid_t pid;
	u32 task_id;
	
	/* Timing parameters */
	u64 period_ns;          /* TT: Task period */
	u64 deadline_ns;        /* Relative deadline */
	u64 wcet_ns;            /* Worst-case execution time */
	u64 next_release_ns;    /* Next TT release time */
	u64 absolute_deadline_ns;
	
	/* Priority */
	int priority;           /* Fixed priority (1-99 for RT) */
	
	/* State */
	enum cule_task_state state;
	enum cule_sched_mode mode;
	
	/* NIS tracking */
	u64 nis_enter_time;
	u64 nis_max_duration_ns;
	struct hrtimer nis_timer;
	
	/* Statistics */
	u64 exec_count;
	u64 miss_count;
	u64 total_exec_ns;
	
	/* CPU affinity */
	cpumask_t cpu_mask;
	
	/* Neural server integration */
	bool is_neural_task;
	u32 neural_layer_id;
};

/* Scheduler per-CPU data */
struct cule_sched_cpu {
	spinlock_t lock;
	struct list_head ready_queue;
	struct list_head wait_queue;
	struct cule_task *current_task;
	struct hrtimer tick_timer;
	u64 tick_period_ns;
	bool active;
	
	/* TT schedule table */
	struct cule_task **tt_table;
	size_t tt_count;
	u64 tt_hyperperiod_ns;
	
	/* Preemption control */
	atomic_t preempt_count;
	bool in_nis;
};

static DEFINE_PER_CPU(struct cule_sched_cpu, cule_sched_percpu);
static struct cule_kernel_api *g_api;
static bool scheduler_initialized = false;

/* Default tick period: 1ms (1kHz) */
#define CULE_DEFAULT_TICK_NS    1000000ULL
#define CULE_MAX_NIS_NS         50000000ULL   /* 50ms max NIS */
#define CULE_MAX_TASKS          256
#define CULE_MAX_NEURAL_LAYERS  128

static atomic_t task_id_counter = ATOMIC_INIT(0);

/**
 * @brief Check if we're in an NIS - prevents preemption
 */
static inline bool cule_in_nis(struct cule_sched_cpu *sched)
{
	return READ_ONCE(sched->in_nis);
}

/**
 * @brief Enter Non-Interruptible Section
 * 
 * Disables preemption for critical sections. Must be short!
 */
int cule_nis_enter(u32 task_id)
{
	struct cule_sched_cpu *sched = this_cpu_ptr(&cule_sched_percpu);
	unsigned long flags;
	
	spin_lock_irqsave(&sched->lock, flags);
	
	if (sched->in_nis) {
		spin_unlock_irqrestore(&sched->lock, flags);
		pr_err("Nested NIS not allowed for task %u\n", task_id);
		return -EBUSY;
	}
	
	sched->in_nis = true;
	sched->current_task->state = CULE_TASK_NIS;
	sched->current_task->nis_enter_time = ktime_get_ns();
	
	/* Start NIS watchdog timer */
	hrtimer_start(&sched->current_task->nis_timer, 
			ns_to_ktime(CULE_MAX_NIS_NS),
			HRTIMER_MODE_REL_PINNED_HARD);
	
	spin_unlock_irqrestore(&sched->lock, flags);
	
	/* Disable preemption at kernel level */
	preempt_disable();
	raw_local_irq_disable();
	
	pr_debug("Task %u entered NIS\n", task_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_nis_enter);

/**
 * @brief Exit Non-Interruptible Section
 */
int cule_nis_exit(u32 task_id)
{
	struct cule_sched_cpu *sched = this_cpu_ptr(&cule_sched_percpu);
	struct cule_task *task = sched->current_task;
	unsigned long flags;
	u64 nis_duration;
	
	raw_local_irq_enable();
	preempt_enable();
	
	hrtimer_cancel(&task->nis_timer);
	
	spin_lock_irqsave(&sched->lock, flags);
	
	nis_duration = ktime_get_ns() - task->nis_enter_time;
	sched->in_nis = false;
	task->state = CULE_TASK_RUNNING;
	
	if (nis_duration > task->nis_max_duration_ns) {
		pr_warn("Task %u NIS duration %llu ns exceeded limit\n",
			task_id, nis_duration);
	}
	
	spin_unlock_irqrestore(&sched->lock, flags);
	
	pr_debug("Task %u exited NIS (duration: %llu ns)\n", task_id, nis_duration);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_nis_exit);

/**
 * @brief NIS watchdog timer callback
 * Fires if NIS exceeds maximum duration
 */
static enum hrtimer_restart cule_nis_timer_cb(struct hrtimer *timer)
{
	struct cule_task *task = container_of(timer, struct cule_task, nis_timer);
	
	pr_emerg("NIS VIOLATION: Task %u exceeded max NIS duration!\n",
		task->task_id);
	
	/* In production, this would trigger safety mechanism */
	if (g_api && g_api->safety_trigger) {
		g_api->safety_trigger(CULE_SAFETY_NIS_VIOLATION, task->task_id);
	}
	
	return HRTIMER_NORESTART;
}

/**
 * @brief Compare tasks for EDF scheduling
 */
static inline bool cule_edf_compare(struct cule_task *a, struct cule_task *b)
{
	return a->absolute_deadline_ns < b->absolute_deadline_ns;
}

/**
 * @brief Insert task into ready queue (sorted by deadline for EDF)
 */
static void cule_rq_insert(struct cule_sched_cpu *sched, struct cule_task *task)
{
	struct cule_task *pos;
	struct list_head *insert_point = &sched->ready_queue;
	
	if (task->mode == CULE_SCHED_EDF) {
		list_for_each_entry(pos, &sched->ready_queue, list) {
			if (cule_edf_compare(task, pos)) {
				insert_point = &pos->list;
				break;
			}
		}
	}
	
	list_add_tail(&task->list, insert_point);
	task->state = CULE_TASK_READY;
}

/**
 * @brief Select next task to run
 */
static struct cule_task *cule_select_next(struct cule_sched_cpu *sched)
{
	struct cule_task *next = NULL;
	
	/* TT tasks have highest priority */
	if (sched->tt_table && sched->tt_count > 0) {
		u64 now = ktime_get_ns();
		for (size_t i = 0; i < sched->tt_count; i++) {
			if (sched->tt_table[i]->next_release_ns <= now &&
			    sched->tt_table[i]->state == CULE_TASK_READY) {
				next = sched->tt_table[i];
				break;
			}
		}
	}
	
	/* Fall back to ready queue (EDF/FP) */
	if (!next && !list_empty(&sched->ready_queue)) {
		next = list_first_entry(&sched->ready_queue, struct cule_task, list);
	}
	
	return next;
}

/**
 * @brief Scheduler tick handler
 */
static enum hrtimer_restart cule_tick_handler(struct hrtimer *timer)
{
	struct cule_sched_cpu *sched = container_of(timer, struct cule_sched_cpu, tick_timer);
	struct cule_task *current, *next;
	unsigned long flags;
	u64 now = ktime_get_ns();
	bool should_resched = false;
	
	spin_lock_irqsave(&sched->lock, flags);
	
	/* Skip if in NIS - TT scheduling guarantees no preemption during NIS */
	if (cule_in_nis(sched)) {
		spin_unlock_irqrestore(&sched->lock, flags);
		goto restart_timer;
	}
	
	current = sched->current_task;
	
	/* Check deadline misses */
	if (current && current->state == CULE_TASK_RUNNING) {
		if (now > current->absolute_deadline_ns) {
			current->miss_count++;
			pr_warn("Deadline miss for task %u (pid %d)\n",
				current->task_id, current->pid);
		}
	}
	
	/* Release TT tasks */
	if (sched->tt_table) {
		for (size_t i = 0; i < sched->tt_count; i++) {
			struct cule_task *tt_task = sched->tt_table[i];
			if (now >= tt_task->next_release_ns && 
			    tt_task->state == CULE_TASK_IDLE) {
				tt_task->next_release_ns += tt_task->period_ns;
				tt_task->absolute_deadline_ns = now + tt_task->deadline_ns;
				tt_task->state = CULE_TASK_READY;
				should_resched = true;
			}
		}
	}
	
	/* Select next task */
	next = cule_select_next(sched);
	
	if (next && next != current) {
		/* Context switch needed */
		if (current) {
			current->state = CULE_TASK_READY;
			if (current->mode != CULE_SCHED_TT) {
				cule_rq_insert(sched, current);
			}
		}
		
		list_del(&next->list);
		next->state = CULE_TASK_RUNNING;
		sched->current_task = next;
		should_resched = true;
	}
	
	spin_unlock_irqrestore(&sched->lock, flags);
	
	if (should_resched) {
		/* Trigger Linux scheduler */
		set_tsk_need_resched(current->pid_task);
	}

restart_timer:
	hrtimer_forward_now(timer, ns_to_ktime(sched->tick_period_ns));
	return HRTIMER_RESTART;
}

/**
 * @brief Initialize TT schedule table
 */
int cule_tt_init(struct cule_task **tasks, size_t count, u64 hyperperiod_ns)
{
	struct cule_sched_cpu *sched = this_cpu_ptr(&cule_sched_percpu);
	unsigned long flags;
	
	if (count > CULE_MAX_TASKS) {
		pr_err("Too many TT tasks: %zu (max %d)\n", count, CULE_MAX_TASKS);
		return -EINVAL;
	}
	
	spin_lock_irqsave(&sched->lock, flags);
	
	sched->tt_table = kmalloc_array(count, sizeof(struct cule_task *), GFP_KERNEL);
	if (!sched->tt_table) {
		spin_unlock_irqrestore(&sched->lock, flags);
		return -ENOMEM;
	}
	
	memcpy(sched->tt_table, tasks, count * sizeof(struct cule_task *));
	sched->tt_count = count;
	sched->tt_hyperperiod_ns = hyperperiod_ns;
	
	/* Initialize release times */
	u64 now = ktime_get_ns();
	for (size_t i = 0; i < count; i++) {
		sched->tt_table[i]->next_release_ns = now;
		sched->tt_table[i]->mode = CULE_SCHED_TT;
	}
	
	spin_unlock_irqrestore(&sched->lock, flags);
	
	pr_info("TT schedule initialized with %zu tasks, hyperperiod=%llu ms\n",
		count, hyperperiod_ns / 1000000);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_tt_init);

/**
 * @brief Register a real-time task with the scheduler
 */
int cule_task_register(struct cule_rt_params *params, u32 *task_id_out)
{
	struct cule_sched_cpu *sched;
	struct cule_task *task;
	unsigned long flags;
	u32 task_id;
	
	if (!params || !task_id_out)
		return -EINVAL;
	
	if (params->priority < 1 || params->priority > 99)
		return -EINVAL;
	
	task = kzalloc(sizeof(*task), GFP_KERNEL);
	if (!task)
		return -ENOMEM;
	
	task_id = atomic_inc_return(&task_id_counter);
	task->task_id = task_id;
	task->pid = current->pid;
	task->period_ns = params->period_ns;
	task->deadline_ns = params->deadline_ns;
	task->wcet_ns = params->wcet_ns;
	task->priority = params->priority;
	task->state = CULE_TASK_IDLE;
	task->mode = params->mode;
	task->absolute_deadline_ns = ktime_get_ns() + params->deadline_ns;
	task->nis_max_duration_ns = min(params->wcet_ns / 10, CULE_MAX_NIS_NS);
	
	if (params->cpu_mask)
		cpumask_copy(&task->cpu_mask, params->cpu_mask);
	else
		cpumask_setall(&task->cpu_mask);
	
	hrtimer_init(&task->nis_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED_HARD);
	task->nis_timer.function = cule_nis_timer_cb;
	
	/* Add to appropriate CPU's scheduler */
	int cpu = cpumask_first(&task->cpu_mask);
	sched = per_cpu_ptr(&cule_sched_percpu, cpu);
	
	spin_lock_irqsave(&sched->lock, flags);
	cule_rq_insert(sched, task);
	spin_unlock_irqrestore(&sched->lock, flags);
	
	*task_id_out = task_id;
	
	pr_info("Registered task %u (pid %d) priority=%d period=%llu us\n",
		task_id, task->pid, params->priority, params->period_ns / 1000);
	
	return 0;
}
EXPORT_SYMBOL_GPL(cule_task_register);

/**
 * @brief Unregister a task
 */
int cule_task_unregister(u32 task_id)
{
	struct cule_sched_cpu *sched;
	struct cule_task *task, *tmp;
	unsigned long flags;
	bool found = false;
	int cpu;
	
	for_each_possible_cpu(cpu) {
		sched = per_cpu_ptr(&cule_sched_percpu, cpu);
		
		spin_lock_irqsave(&sched->lock, flags);
		
		/* Check current task */
		if (sched->current_task && sched->current_task->task_id == task_id) {
			task = sched->current_task;
			sched->current_task = NULL;
			found = true;
		}
		
		/* Check ready queue */
		if (!found) {
			list_for_each_entry_safe(task, tmp, &sched->ready_queue, list) {
				if (task->task_id == task_id) {
					list_del(&task->list);
					found = true;
					break;
				}
			}
		}
		
		spin_unlock_irqrestore(&sched->lock, flags);
		
		if (found)
			break;
	}
	
	if (!found)
		return -ENOENT;
	
	hrtimer_cancel(&task->nis_timer);
	kfree(task);
	
	pr_info("Unregistered task %u\n", task_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_task_unregister);

/**
 * @brief Scheduler initialization
 */
static int __init cule_scheduler_init(void)
{
	struct cule_sched_cpu *sched;
	int cpu;
	
	pr_info("Initializing Cule RT Scheduler v%s\n", MODULE_VERSION);
	
	for_each_possible_cpu(cpu) {
		sched = per_cpu_ptr(&cule_sched_percpu, cpu);
		
		spin_lock_init(&sched->lock);
		INIT_LIST_HEAD(&sched->ready_queue);
		INIT_LIST_HEAD(&sched->wait_queue);
		sched->current_task = NULL;
		sched->tick_period_ns = CULE_DEFAULT_TICK_NS;
		sched->active = false;
		sched->tt_table = NULL;
		sched->tt_count = 0;
		sched->in_nis = false;
		atomic_set(&sched->preempt_count, 0);
		
		hrtimer_init(&sched->tick_timer, CLOCK_MONOTONIC, 
			     HRTIMER_MODE_ABS_PINNED_HARD);
		sched->tick_timer.function = cule_tick_handler;
	}
	
	scheduler_initialized = true;
	pr_info("Cule RT Scheduler initialized\n");
	
	return 0;
}

/**
 * @brief Scheduler cleanup
 */
static void __exit cule_scheduler_exit(void)
{
	struct cule_sched_cpu *sched;
	struct cule_task *task, *tmp;
	int cpu;
	
	for_each_possible_cpu(cpu) {
		sched = per_cpu_ptr(&cule_sched_percpu, cpu);
		
		hrtimer_cancel(&sched->tick_timer);
		
		spin_lock(&sched->lock);
		
		list_for_each_entry_safe(task, tmp, &sched->ready_queue, list) {
			list_del(&task->list);
			hrtimer_cancel(&task->nis_timer);
			kfree(task);
		}
		
		if (sched->tt_table) {
			kfree(sched->tt_table);
			sched->tt_table = NULL;
		}
		
		spin_unlock(&sched->lock);
	}
	
	scheduler_initialized = false;
	pr_info("Cule RT Scheduler exited\n");
}

module_init(cule_scheduler_init);
module_exit(cule_scheduler_exit);

/* Kernel API registration */
static struct cule_kernel_api scheduler_api = {
	.task_register = cule_task_register,
	.task_unregister = cule_task_unregister,
	.nis_enter = cule_nis_enter,
	.nis_exit = cule_nis_exit,
	.tt_init = cule_tt_init,
};

int cule_sched_register_api(struct cule_kernel_api *api)
{
	if (!api)
		return -EINVAL;
	g_api = api;
	memcpy(api, &scheduler_api, sizeof(scheduler_api));
	return 0;
}
EXPORT_SYMBOL_GPL(cule_sched_register_api);
