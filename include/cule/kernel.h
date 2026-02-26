/**
 * @file kernel.h
 * @brief Cule OS Kernel Public API Header
 * 
 * This header defines the public API for the Cule OS real-time kernel
 * modules. Applications and kernel modules should include this header
 * to interact with the scheduler, neural server, memory protection,
 * and IPC subsystems.
 * 
 * @copyright Copyright (c) 2024 Cule OS Project
 * @license GPL-2.0
 * 
 * Target: NVIDIA Jetson AGX Orin (ARM64, PREEMPT_RT)
 */

#ifndef _CULE_KERNEL_H
#define _CULE_KERNEL_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/completion.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * Version Information
 * ========================================================================== */
#define CULE_KERNEL_VERSION_MAJOR   1
#define CULE_KERNEL_VERSION_MINOR   0
#define CULE_KERNEL_VERSION_PATCH   0
#define CULE_KERNEL_VERSION_STRING  "1.0.0"

/* ==========================================================================
 * Constants
 * ========================================================================== */
#define CULE_MAX_AGENTS             64
#define CULE_MAX_TASKS_PER_AGENT    32
#define CULE_MAX_CHANNELS           256
#define CULE_MAX_MESSAGE_SIZE       65536
#define CULE_MAX_LAYERS             512
#define CULE_MAX_MODELS             32

/* Agent IDs */
#define CULE_AGENT_KERNEL           0
#define CULE_AGENT_NAVIGATION       1
#define CULE_AGENT_PERCEPTION       2
#define CULE_AGENT_PLANNING         3
#define CULE_AGENT_COMMUNICATION    4
#define CULE_AGENT_SAFETY           5
#define CULE_AGENT_USER_START       10

/* Safety trigger types */
enum cule_safety_type {
	CULE_SAFETY_NONE = 0,
	CULE_SAFETY_NIS_VIOLATION,
	CULE_SAFETY_DEADLINE_MISS,
	CULE_SAFETY_MEMORY_FAULT,
	CULE_SAFETY_WATCHDOG,
	CULE_SAFETY_THERMAL,
	CULE_SAFETY_POWER,
};

/* ==========================================================================
 * Real-Time Scheduler API
 * ========================================================================== */

/* Scheduling modes */
enum cule_sched_mode {
	CULE_SCHED_TT = 0,      /* Time-Triggered */
	CULE_SCHED_EDF,         /* Earliest Deadline First */
	CULE_SCHED_FP,          /* Fixed Priority */
	CULE_SCHED_MIXED,       /* Mixed criticality */
};

/* Task states */
enum cule_task_state {
	CULE_TASK_IDLE = 0,
	CULE_TASK_READY,
	CULE_TASK_RUNNING,
	CULE_TASK_BLOCKED,
	CULE_TASK_PREEMPTED,
	CULE_TASK_COMPLETED,
	CULE_TASK_FAILED,
};

/* Job states for neural server */
enum cule_job_state {
	CULE_JOB_PENDING = 0,
	CULE_JOB_RUNNING,
	CULE_JOB_PREEMPTED,
	CULE_JOB_COMPLETED,
	CULE_JOB_FAILED,
};

/* Real-time task parameters */
struct cule_rt_params {
	u32 priority;           /* 1-99 for RT tasks */
	u64 period_ns;          /* Task period in nanoseconds */
	u64 deadline_ns;        /* Relative deadline */
	u64 wcet_ns;            /* Worst-case execution time */
	enum cule_sched_mode mode;
	cpumask_t *cpu_mask;    /* CPU affinity (NULL = all) */
	bool is_neural_task;    /* Uses neural server */
};

/* Task statistics */
struct cule_task_stats {
	u64 exec_count;
	u64 miss_count;
	u64 preempt_count;
	u64 total_exec_ns;
	u64 min_exec_ns;
	u64 max_exec_ns;
	u64 avg_exec_ns;
};

/* Scheduler statistics */
struct cule_sched_stats {
	u64 tick_count;
	u64 context_switches;
	u64 deadline_misses;
	u64 nis_violations;
	u32 active_tasks;
	u32 pending_tasks;
};

/* Scheduler API functions */
extern int cule_task_register(struct cule_rt_params *params, u32 *task_id_out);
extern int cule_task_unregister(u32 task_id);
extern int cule_task_start(u32 task_id);
extern int cule_task_stop(u32 task_id);
extern int cule_task_get_stats(u32 task_id, struct cule_task_stats *stats);
extern int cule_sched_get_stats(struct cule_sched_stats *stats);

/* TT Scheduling */
extern int cule_tt_init(struct cule_task **tasks, size_t count, u64 hyperperiod_ns);
extern int cule_tt_add_slot(u32 task_id, u64 offset_ns, u64 duration_ns);
extern int cule_tt_remove_slot(u32 task_id, u64 offset_ns);

/* Non-Interruptible Section (NIS) */
extern int cule_nis_enter(u32 task_id);
extern int cule_nis_exit(u32 task_id);
extern bool cule_nis_active(u32 task_id);

/* ==========================================================================
 * Neural Inference Server API
 * ========================================================================== */

/* Neural accelerator types */
enum cule_accel_type {
	CULE_ACCEL_CPU = 0,
	CULE_ACCEL_GPU_CUDA,
	CULE_ACCEL_GPU_TRT,
	CULE_ACCEL_NPU_DLA,
	CULE_ACCEL_NPU_PVA,
};

/* Layer types */
enum cule_layer_type {
	CULE_LAYER_CONV2D = 0,
	CULE_LAYER_LINEAR,
	CULE_LAYER_BATCHNORM,
	CULE_LAYER_RELU,
	CULE_LAYER_POOL,
	CULE_LAYER_DROPOUT,
	CULE_LAYER_ATTENTION,
	CULE_LAYER_CUSTOM,
};

/* Layer information for model registration */
struct cule_layer_info {
	enum cule_layer_type type;
	u32 input_dims[4];      /* NCHW format */
	u32 output_dims[4];
	size_t weight_size;
	size_t bias_size;
	u64 exec_time_ns;       /* Estimated execution time */
	bool is_preemptible;    /* Can checkpoint at this layer */
};

/* Model configuration */
struct cule_model_config {
	const char *name;
	u32 version;
	enum cule_accel_type accel;
	u32 batch_size;
	u32 optimization_level;
	bool enable_checkpointing;
	u64 max_inference_time_ns;
};

/* Inference request */
struct cule_inference_request {
	u32 model_id;
	u32 task_id;
	void *input_data;
	size_t input_size;
	void *output_buffer;
	size_t output_size;
	u64 deadline_ns;
	u32 priority;
	bool async;
};

/* Inference result */
struct cule_inference_result {
	u32 job_id;
	enum cule_job_state state;
	void *output_data;
	size_t output_size;
	u64 exec_time_ns;
	u32 layers_completed;
	u32 total_layers;
	int error_code;
};

/* Neural server statistics */
struct cule_neural_stats {
	u64 total_inferences;
	u64 preempted_inferences;
	u64 checkpoint_saves;
	u64 checkpoint_restores;
	u64 avg_inference_time_ns;
	u64 max_inference_time_ns;
	u32 model_count;
	u32 active_jobs;
};

/* Neural Server API functions */
extern int cule_model_register(const char *name, u32 num_layers,
				struct cule_layer_info *layer_info,
				enum cule_accel_type accel,
				u32 *model_id_out);
extern int cule_model_unregister(u32 model_id);
extern int cule_model_load_weights(u32 model_id, void *weights, size_t size);

extern int cule_inference_submit(u32 model_id, u32 task_id,
				 u64 deadline_ns,
				 u32 *job_id_out);
extern int cule_inference_execute(u32 job_id);
extern int cule_inference_wait(u32 job_id, u64 timeout_ns,
			       struct cule_inference_result *result);
extern int cule_inference_cancel(u32 job_id);

extern int cule_neural_preempt(u32 task_id);
extern int cule_neural_resume(u32 job_id);
extern bool cule_neural_job_active(u32 job_id);

extern int cule_neural_get_stats(struct cule_neural_stats *stats);
extern int cule_neural_get_model_stats(u32 model_id, struct cule_neural_stats *stats);

/* ==========================================================================
 * Memory Protection API
 * ========================================================================== */

/* Memory region types */
enum cule_mem_type {
	CULE_MEM_KERNEL = 0,
	CULE_MEM_AGENT_PRIVATE,
	CULE_MEM_AGENT_SHARED,
	CULE_MEM_DMA_POOL,
	CULE_MEM_DEVICE,
};

/* Memory protection attributes */
enum cule_mem_prot {
	CULE_PROT_NONE      = 0,
	CULE_PROT_READ      = (1 << 0),
	CULE_PROT_WRITE     = (1 << 1),
	CULE_PROT_EXEC      = (1 << 2),
	CULE_PROT_DMA       = (1 << 3),
	CULE_PROT_NOCACHE   = (1 << 4),
	CULE_PROT_UNCACHED  = (1 << 5),
};

/* Agent memory context (opaque) */
struct cule_agent_mem_ctx;

/* Memory region handle */
typedef u64 cule_mem_handle_t;

/* Memory allocation request */
struct cule_mem_request {
	size_t size;
	size_t alignment;
	enum cule_mem_type type;
	enum cule_mem_prot prot;
	bool contiguous;
	bool dma_accessible;
};

/* Memory region info */
struct cule_mem_info {
	cule_mem_handle_t handle;
	void *virtual_addr;
	phys_addr_t physical_addr;
	dma_addr_t dma_addr;
	size_t size;
	enum cule_mem_type type;
	enum cule_mem_prot prot;
	u32 owner_agent;
};

/* Memory statistics */
struct cule_mem_stats {
	u64 total_allocated;
	u64 current_usage;
	u64 peak_usage;
	size_t total_region_size;
	u32 region_count;
	u64 fault_count;
};

/* Memory Protection API functions */
extern int cule_agent_mem_create(u32 agent_id, struct cule_agent_mem_ctx **ctx_out);
extern int cule_agent_mem_destroy(u32 agent_id);

extern int cule_mem_alloc(u32 agent_id, size_t size, enum cule_mem_type type,
			  enum cule_mem_prot prot, u64 *region_id_out);
extern int cule_mem_free(u32 agent_id, u64 region_id);
extern int cule_mem_map(u32 agent_id, u64 region_id, void **virt_addr_out);
extern int cule_mem_unmap(u32 agent_id, void *virt_addr);

extern int cule_mem_share(u32 owner_id, u64 region_id, u32 *agent_ids, u32 count);
extern int cule_mem_revoke(u32 owner_id, u64 region_id, u32 agent_id);

extern bool cule_mem_check_access(u32 agent_id, void *addr, size_t size,
				  enum cule_mem_prot required_prot);
extern int cule_mem_protect(u64 region_id, enum cule_mem_prot new_prot);

extern int cule_mem_get_stats(u32 agent_id, struct cule_mem_stats *stats);

/* DMA memory functions */
extern int cule_dma_alloc(u32 agent_id, size_t size, dma_addr_t *dma_handle,
			  void **virt_addr);
extern int cule_dma_free(u32 agent_id, size_t size, dma_addr_t dma_handle,
			 void *virt_addr);
extern int cule_dma_sync_for_cpu(dma_addr_t dma_handle, size_t size);
extern int cule_dma_sync_for_device(dma_addr_t dma_handle, size_t size);

/* ==========================================================================
 * Inter-Process Communication API
 * ========================================================================== */

/* IPC message types */
enum cule_msg_type {
	CULE_MSG_DATA = 0,
	CULE_MSG_CONTROL,
	CULE_MSG_SIGNAL,
	CULE_MSG_RPC_REQUEST,
	CULE_MSG_RPC_RESPONSE,
	CULE_MSG_DMA_DESCRIPTOR,
};

/* Message priority */
enum cule_msg_priority {
	CULE_PRIO_LOW = 0,
	CULE_PRIO_NORMAL,
	CULE_PRIO_HIGH,
	CULE_PRIO_REALTIME,
};

/* Message flags */
#define CULE_MSG_FLAG_ASYNC     (1 << 0)
#define CULE_MSG_FLAG_URGENT    (1 << 1)
#define CULE_MSG_FLAG_NO_COPY   (1 << 2)
#define CULE_MSG_FLAG_DMA       (1 << 3)
#define CULE_MSG_FLAG_RESPONSE  (1 << 4)

/* Message header (inline in ring buffer) */
struct cule_ipc_msg_hdr {
	enum cule_msg_type type;
	u32 src_agent;
	u32 dst_agent;
	u32 length;
	u32 flags;
	enum cule_msg_priority priority;
	u64 timestamp_ns;
	dma_addr_t dma_handle;
	u32 pool_block;
	u8 data[];  /* Inline data follows */
} __attribute__((aligned(64)));

/* IPC message structure */
struct cule_ipc_msg {
	enum cule_msg_type type;
	u32 src_agent;
	u32 dst_agent;
	u32 length;
	u32 flags;
	enum cule_msg_priority priority;
	u64 timestamp_ns;
	dma_addr_t dma_handle;
	u32 pool_block;
	u8 *data;
	
	/* Internal fields */
	u64 send_latency_ns;
};

/* IPC endpoint handle */
typedef u32 cule_endpoint_t;

/* IPC channel handle */
typedef u32 cule_channel_t;

/* IPC statistics */
struct cule_ipc_stats {
	u64 messages_sent;
	u64 messages_recv;
	u64 bytes_sent;
	u64 bytes_recv;
	u64 drops;
	u64 avg_latency_ns;
	u32 active_channels;
};

/* IPC API functions */
extern int cule_ipc_endpoint_register(u32 agent_id, cule_endpoint_t *endpoint_out);
extern int cule_ipc_endpoint_unregister(cule_endpoint_t endpoint);
extern int cule_ipc_set_callback(cule_endpoint_t endpoint,
				 void (*callback)(struct cule_ipc_msg *msg, void *priv),
				 void *priv);

extern int cule_ipc_channel_create(u32 agent_a, u32 agent_b, cule_channel_t *channel_out);
extern int cule_ipc_channel_destroy(cule_channel_t channel);
extern int cule_ipc_channel_get_info(cule_channel_t channel, u32 *agent_a, u32 *agent_b);

extern int cule_ipc_send(cule_endpoint_t src, cule_endpoint_t dst,
			 struct cule_ipc_msg *msg);
extern int cule_ipc_recv(cule_endpoint_t endpoint, struct cule_ipc_msg *msg,
			 bool blocking);
extern int cule_ipc_try_recv(cule_endpoint_t endpoint, struct cule_ipc_msg *msg);
extern int cule_ipc_poll(cule_endpoint_t endpoint, u64 timeout_ns);

/* Zero-copy functions */
extern void *cule_ipc_alloc_shared(cule_channel_t channel, size_t size,
				   dma_addr_t *dma_handle);
extern int cule_ipc_free_shared(cule_channel_t channel, void *ptr, size_t size);
extern void *cule_ipc_map_shared(cule_channel_t channel, dma_addr_t dma_handle,
				 size_t size);
extern int cule_ipc_release_shared(cule_channel_t channel, u32 pool_block,
				   size_t size);

extern int cule_ipc_flush_cache(void *addr, size_t size);
extern int cule_ipc_invalidate_cache(void *addr, size_t size);

/* IPC statistics */
extern int cule_ipc_get_stats(cule_endpoint_t endpoint, struct cule_ipc_stats *stats);
extern int cule_ipc_get_channel_stats(cule_channel_t channel, struct cule_ipc_stats *stats);

/* ==========================================================================
 * Kernel Integration API
 * ========================================================================== */

/* Kernel API registration structure - used internally by modules */
struct cule_kernel_api {
	/* Scheduler */
	int (*task_register)(struct cule_rt_params *params, u32 *task_id_out);
	int (*task_unregister)(u32 task_id);
	int (*nis_enter)(u32 task_id);
	int (*nis_exit)(u32 task_id);
	int (*tt_init)(struct cule_task **tasks, size_t count, u64 hyperperiod_ns);
	
	/* Neural */
	int (*model_register)(const char *name, u32 num_layers,
			      struct cule_layer_info *layer_info,
			      enum cule_accel_type accel,
			      u32 *model_id_out);
	int (*inference_submit)(u32 model_id, u32 task_id, u64 deadline_ns,
				u32 *job_id_out);
	int (*neural_preempt)(u32 task_id);
	int (*neural_resume)(u32 job_id);
	
	/* Memory */
	int (*agent_mem_create)(u32 agent_id, struct cule_agent_mem_ctx **ctx_out);
	int (*agent_mem_destroy)(u32 agent_id);
	int (*mem_alloc)(u32 agent_id, size_t size, enum cule_mem_type type,
			 enum cule_mem_prot prot, u64 *region_id_out);
	bool (*mem_check_access)(u32 agent_id, void *addr, size_t size,
				 enum cule_mem_prot required_prot);
	
	/* IPC */
	int (*ipc_endpoint_register)(u32 agent_id, u32 *endpoint_id_out);
	int (*ipc_channel_create)(u32 agent_a, u32 agent_b, u32 *channel_id_out);
	int (*ipc_send)(u32 src_endpoint, u32 dst_endpoint, struct cule_ipc_msg *msg);
	
	/* Safety */
	void (*safety_trigger)(enum cule_safety_type type, u32 agent_id);
};

/* Registration functions for subsystems */
extern int cule_sched_register_api(struct cule_kernel_api *api);
extern int cule_neural_register_api(struct cule_kernel_api *api);
extern int cule_mem_register_api(struct cule_kernel_api *api);
extern int cule_ipc_register_api(struct cule_kernel_api *api);

/* ==========================================================================
 * Utility Functions
 * ========================================================================== */

/* Time utilities */
static inline u64 cule_time_now_ns(void)
{
	return ktime_get_ns();
}

static inline u64 cule_time_deadline_from_now_ns(u64 delta_ns)
{
	return ktime_get_ns() + delta_ns;
}

static inline s64 cule_time_remaining_ns(u64 deadline_ns)
{
	return (s64)deadline_ns - (s64)ktime_get_ns();
}

static inline bool cule_time_deadline_missed(u64 deadline_ns)
{
	return ktime_get_ns() > deadline_ns;
}

/* Priority helpers */
static inline int cule_prio_rt(int priority)
{
	return max(1, min(99, priority));
}

static inline bool cule_prio_is_rt(int priority)
{
	return priority >= 1 && priority <= 99;
}

/* Cache operations */
static inline void cule_cache_flush(void *addr, size_t size)
{
	__flush_dcache_area(addr, size);
}

static inline void cule_cache_invalidate(void *addr, size_t size)
{
	__inval_dcache_area(addr, size);
}

static inline void cule_cache_clean_invalidate(void *addr, size_t size)
{
	__clean_dcache_area_poc(addr, size);
}

/* Memory barriers */
static inline void cule_mb(void)
{
	smp_mb();
}

static inline void cule_rmb(void)
{
	smp_rmb();
}

static inline void cule_wmb(void)
{
	smp_wmb();
}

/* ==========================================================================
 * Error Codes
 * ========================================================================== */

#define CULE_OK                     0
#define CULE_ERROR_INVALID         -1
#define CULE_ERROR_NOMEM           -2
#define CULE_ERROR_NOTFOUND        -3
#define CULE_ERROR_ACCESS          -4
#define CULE_ERROR_TIMEOUT         -5
#define CULE_ERROR_DEADLINE        -6
#define CULE_ERROR_PREEMPTED       -7
#define CULE_ERROR_CHECKPOINT      -8
#define CULE_ERROR_SAFETY          -9

#ifdef __cplusplus
}
#endif

#endif /* _CULE_KERNEL_H */
