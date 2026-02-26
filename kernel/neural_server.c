/**
 * @file neural_server.c
 * @brief Cule OS Neural Inference Server with Layer-wise Checkpointing
 * 
 * Provides preemptible neural network inference with cooperative checkpointing
 * at layer boundaries for real-time systems.
 * 
 * Target: NVIDIA Jetson AGX Orin (ARM64, TensorRT, CUDA)
 * License: GPL-2.0
 */

#define pr_fmt(fmt) "cule_neural: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/ktime.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <asm/cacheflush.h>

#include "../include/cule/kernel.h"

MODULE_AUTHOR("Cule OS Team");
MODULE_DESCRIPTION("Cule OS Neural Inference Server");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");

/* Neural accelerator types */
enum cule_accel_type {
	CULE_ACCEL_CPU,
	CULE_ACCEL_GPU_CUDA,
	CULE_ACCEL_GPU_TRT,      /* TensorRT */
	CULE_ACCEL_NPU_DLA,      /* Deep Learning Accelerator */
	CULE_ACCEL_NPU_PVA,      /* Programmable Vision Accelerator */
};

/* Layer types */
enum cule_layer_type {
	CULE_LAYER_CONV2D,
	CULE_LAYER_LINEAR,
	CULE_LAYER_BATCHNORM,
	CULE_LAYER_RELU,
	CULE_LAYER_POOL,
	CULE_LAYER_DROPOUT,
	CULE_LAYER_ATTENTION,
};

/* Checkpoint states */
enum cule_checkpoint_state {
	CULE_CKPT_INVALID,
	CULE_CKPT_VALID,
	CULE_CKPT_IN_PROGRESS,
	CULE_CKPT_RESTORED,
};

/* Layer descriptor */
struct cule_layer {
	u32 layer_id;
	enum cule_layer_type type;
	
	/* Dimensions */
	u32 input_dims[4];   /* NCHW format */
	u32 output_dims[4];
	u32 weight_size;
	u32 bias_size;
	
	/* Memory */
	void *weight_data;
	void *bias_data;
	void *intermediate_output;
	dma_addr_t weight_dma;
	dma_addr_t bias_dma;
	dma_addr_t output_dma;
	
	/* Timing estimates */
	u64 exec_time_ns;
	u64 checkpoint_time_ns;
	
	/* Preemption point */
	bool is_preemptible;
	struct list_head checkpoint_list;
};

/* Layer-wise checkpoint */
struct cule_layer_checkpoint {
	u32 layer_id;
	enum cule_checkpoint_state state;
	
	/* Saved activation tensor */
	void *activation_data;
	size_t activation_size;
	dma_addr_t activation_dma;
	
	/* Metadata */
	u64 timestamp_ns;
	u32 inference_id;
	
	/* Statistics */
	u64 save_time_ns;
	u64 restore_time_ns;
	
	struct list_head list;
};

/* Inference job */
struct cule_inference_job {
	u32 job_id;
	u32 model_id;
	u32 user_task_id;
	
	/* Progress tracking */
	atomic_t current_layer;
	u32 total_layers;
	
	/* State */
	enum cule_job_state state;
	
	/* Timing */
	u64 start_time_ns;
	u64 deadline_ns;
	u64 checkpoint_start_ns;
	
	/* Preemption */
	bool preempt_requested;
	struct completion resume_complete;
	
	/* Linked list */
	struct list_head list;
};

/* Model descriptor */
struct cule_model {
	u32 model_id;
	char name[64];
	u32 version;
	
	/* Layer info */
	struct cule_layer *layers;
	u32 num_layers;
	
	/* Checkpoint pool */
	struct cule_layer_checkpoint *checkpoint_pool;
	u32 checkpoint_pool_size;
	struct list_head available_checkpoints;
	spinlock_t checkpoint_lock;
	
	/* Statistics */
	u64 total_inferences;
	u64 preempted_inferences;
	u64 checkpoint_saves;
	u64 checkpoint_restores;
	
	/* Accelerator */
	enum cule_accel_type accel;
	void *accel_context;
	
	struct list_head list;
};

/* Neural server per-CPU data */
struct cule_neural_cpu {
	spinlock_t lock;
	struct list_head active_jobs;
	struct cule_inference_job *current_job;
	
	/* Priority queue for inference jobs */
	struct list_head pending_queue;
	
	/* Preemption tracking */
	bool preempt_in_progress;
	u32 preempt_count;
};

/* Global state */
static DEFINE_PER_CPU(struct cule_neural_cpu, cule_neural_percpu);
static LIST_HEAD(cule_models);
static DEFINE_SPINLOCK(cule_models_lock);
static atomic_t job_id_counter = ATOMIC_INIT(0);
static atomic_t model_id_counter = ATOMIC_INIT(0);

/* Memory pools */
static struct kmem_cache *layer_cache;
static struct kmem_cache *checkpoint_cache;
static struct kmem_cache *job_cache;

/* Configuration */
#define CULE_NEURAL_MAX_LAYERS      512
#define CULE_NEURAL_MAX_MODELS      32
#define CULE_NEURAL_MAX_JOBS        128
#define CULE_NEURAL_CHECKPOINT_BUFS 16
#define CULE_NEURAL_DMA_ALIGN       256

/* Preemption latency target: 100 microseconds */
#define CULE_PREEMPT_TARGET_NS      100000ULL

/**
 * @brief Calculate checkpoint size for a layer
 */
static size_t cule_calc_checkpoint_size(struct cule_layer *layer)
{
	size_t size = layer->output_dims[0] * layer->output_dims[1] * 
		      layer->output_dims[2] * layer->output_dims[3];
	return ALIGN(size * sizeof(float), CULE_NEURAL_DMA_ALIGN);
}

/**
 * @brief Initialize layer checkpoint pool
 */
static int cule_init_checkpoint_pool(struct cule_model *model)
{
	size_t max_size = 0;
	int i;
	
	/* Find maximum layer output size */
	for (i = 0; i < model->num_layers; i++) {
		size_t ckpt_size = cule_calc_checkpoint_size(&model->layers[i]);
		if (ckpt_size > max_size)
			max_size = ckpt_size;
	}
	
	model->checkpoint_pool_size = CULE_NEURAL_CHECKPOINT_BUFS;
	model->checkpoint_pool = kvmalloc_array(model->checkpoint_pool_size,
						sizeof(struct cule_layer_checkpoint),
						GFP_KERNEL);
	if (!model->checkpoint_pool)
		return -ENOMEM;
	
	INIT_LIST_HEAD(&model->available_checkpoints);
	spin_lock_init(&model->checkpoint_lock);
	
	for (i = 0; i < model->checkpoint_pool_size; i++) {
		struct cule_layer_checkpoint *ckpt = &model->checkpoint_pool[i];
		
		ckpt->activation_size = max_size;
		ckpt->activation_data = dma_alloc_coherent(NULL, max_size,
							   &ckpt->activation_dma,
							   GFP_KERNEL);
		if (!ckpt->activation_data) {
			pr_err("Failed to allocate checkpoint buffer %d\n", i);
			/* Cleanup */
			while (--i >= 0) {
				dma_free_coherent(NULL, max_size,
						  model->checkpoint_pool[i].activation_data,
						  model->checkpoint_pool[i].activation_dma);
			}
			kvfree(model->checkpoint_pool);
			return -ENOMEM;
		}
		
		ckpt->state = CULE_CKPT_INVALID;
		list_add_tail(&ckpt->list, &model->available_checkpoints);
	}
	
	pr_info("Model %u: Allocated %d checkpoint buffers (%zu bytes each)\n",
		model->model_id, model->checkpoint_pool_size, max_size);
	return 0;
}

/**
 * @brief Save checkpoint at layer boundary
 * 
 * Called when preemption is requested. Saves current layer output
 * to allow fast resume later.
 */
static int cule_checkpoint_save(struct cule_inference_job *job,
				struct cule_model *model,
				u32 layer_id)
{
	struct cule_layer_checkpoint *ckpt;
	struct cule_layer *layer;
	unsigned long flags;
	u64 start_ns;
	
	if (layer_id >= model->num_layers)
		return -EINVAL;
	
	layer = &model->layers[layer_id];
	
	/* Skip if layer doesn't support checkpointing */
	if (!layer->is_preemptible)
		return 0;
	
	spin_lock_irqsave(&model->checkpoint_lock, flags);
	
	if (list_empty(&model->available_checkpoints)) {
		spin_unlock_irqrestore(&model->checkpoint_lock, flags);
		pr_warn("No available checkpoint buffers for model %u\n", 
			model->model_id);
		return -EBUSY;
	}
	
	ckpt = list_first_entry(&model->available_checkpoints, 
				struct cule_layer_checkpoint, list);
	list_del(&ckpt->list);
	
	spin_unlock_irqrestore(&model->checkpoint_lock, flags);
	
	start_ns = ktime_get_ns();
	
	/* Copy layer output to checkpoint buffer */
	/* DMA transfer for GPU/NPU, memcpy for CPU */
	switch (model->accel) {
	case CULE_ACCEL_CPU:
		memcpy(ckpt->activation_data, layer->intermediate_output,
		       cule_calc_checkpoint_size(layer));
		break;
		
	case CULE_ACCEL_GPU_CUDA:
	case CULE_ACCEL_GPU_TRT:
		/* Async DMA from GPU to CPU */
		/* cudaMemcpyAsync equivalent */
		break;
		
	case CULE_ACCEL_NPU_DLA:
	case CULE_ACCEL_NPU_PVA:
		/* NPU DMA sync */
		break;
	}
	
	/* Flush cache for CPU/GPU coherence */
	if (model->accel != CULE_ACCEL_CPU) {
		__flush_dcache_area(ckpt->activation_data, 
				    ckpt->activation_size);
	}
	
	ckpt->layer_id = layer_id;
	ckpt->state = CULE_CKPT_VALID;
	ckpt->inference_id = job->job_id;
	ckpt->timestamp_ns = start_ns;
	ckpt->save_time_ns = ktime_get_ns() - start_ns;
	
	job->checkpoint_start_ns = start_ns;
	model->checkpoint_saves++;
	
	pr_debug("Job %u: Checkpoint saved at layer %u (took %llu us)\n",
		job->job_id, layer_id, ckpt->save_time_ns / 1000);
	
	return 0;
}

/**
 * @brief Restore checkpoint to resume inference
 */
static int cule_checkpoint_restore(struct cule_inference_job *job,
				   struct cule_model *model)
{
	struct cule_layer_checkpoint *ckpt = NULL;
	struct cule_layer *layer;
	unsigned long flags;
	u64 start_ns;
	u32 layer_id;
	int i;
	
	spin_lock_irqsave(&model->checkpoint_lock, flags);
	
	/* Find checkpoint for this job */
	for (i = 0; i < model->checkpoint_pool_size; i++) {
		if (model->checkpoint_pool[i].inference_id == job->job_id &&
		    model->checkpoint_pool[i].state == CULE_CKPT_VALID) {
			ckpt = &model->checkpoint_pool[i];
			break;
		}
	}
	
	spin_unlock_irqrestore(&model->checkpoint_lock, flags);
	
	if (!ckpt) {
		pr_err("No valid checkpoint found for job %u\n", job->job_id);
		return -ENOENT;
	}
	
	layer_id = ckpt->layer_id;
	layer = &model->layers[layer_id];
	
	start_ns = ktime_get_ns();
	
	/* Restore activation */
	switch (model->accel) {
	case CULE_ACCEL_CPU:
		memcpy(layer->intermediate_output, ckpt->activation_data,
		       cule_calc_checkpoint_size(layer));
		break;
		
	case CULE_ACCEL_GPU_CUDA:
	case CULE_ACCEL_GPU_TRT:
		/* Async DMA from CPU to GPU */
		break;
		
	case CULE_ACCEL_NPU_DLA:
	case CULE_ACCEL_NPU_PVA:
		break;
	}
	
	ckpt->state = CULE_CKPT_RESTORED;
	ckpt->restore_time_ns = ktime_get_ns() - start_ns;
	
	/* Return checkpoint to pool */
	spin_lock_irqsave(&model->checkpoint_lock, flags);
	ckpt->state = CULE_CKPT_INVALID;
	ckpt->inference_id = 0;
	list_add_tail(&ckpt->list, &model->available_checkpoints);
	spin_unlock_irqrestore(&model->checkpoint_lock, flags);
	
	/* Resume from next layer */
	atomic_set(&job->current_layer, layer_id + 1);
	model->checkpoint_restores++;
	
	pr_debug("Job %u: Checkpoint restored, resuming from layer %u\n",
		job->job_id, layer_id + 1);
	
	return 0;
}

/**
 * @brief Request preemption of current neural job
 * 
 * Called by RT scheduler when higher priority task needs to run.
 */
int cule_neural_preempt(u32 task_id)
{
	struct cule_neural_cpu *ncpu = this_cpu_ptr(&cule_neural_percpu);
	struct cule_inference_job *job = ncpu->current_job;
	struct cule_model *model;
	unsigned long flags;
	int ret;
	
	if (!job)
		return -ENOENT;
	
	spin_lock_irqsave(&ncpu->lock, flags);
	
	if (job->state != CULE_JOB_RUNNING || job->preempt_requested) {
		spin_unlock_irqrestore(&ncpu->lock, flags);
		return -EBUSY;
	}
	
	job->preempt_requested = true;
	ncpu->preempt_in_progress = true;
	
	spin_unlock_irqrestore(&ncpu->lock, flags);
	
	/* Find model */
	model = NULL;
	spin_lock(&cule_models_lock);
	list_for_each_entry(model, &cule_models, list) {
		if (model->model_id == job->model_id)
			break;
	}
	spin_unlock(&cule_models_lock);
	
	if (!model)
		return -EINVAL;
	
	/* Save checkpoint at current layer boundary */
	u32 current_layer = atomic_read(&job->current_layer);
	ret = cule_checkpoint_save(job, model, current_layer);
	
	if (ret == 0) {
		job->state = CULE_JOB_PREEMPTED;
		ncpu->current_job = NULL;
		
		/* Add to pending queue for later resume */
		spin_lock_irqsave(&ncpu->lock, flags);
		list_add_tail(&job->list, &ncpu->pending_queue);
		spin_unlock_irqrestore(&ncpu->lock, flags);
		
		model->preempted_inferences++;
		pr_info("Job %u preempted at layer %u by task %u\n",
			job->job_id, current_layer, task_id);
	}
	
	ncpu->preempt_in_progress = false;
	return ret;
}
EXPORT_SYMBOL_GPL(cule_neural_preempt);

/**
 * @brief Resume a preempted inference job
 */
int cule_neural_resume(u32 job_id)
{
	struct cule_neural_cpu *ncpu = this_cpu_ptr(&cule_neural_percpu);
	struct cule_inference_job *job = NULL;
	struct cule_model *model;
	struct list_head *pos;
	unsigned long flags;
	int ret;
	
	spin_lock_irqsave(&ncpu->lock, flags);
	
	/* Find job in pending queue */
	list_for_each(pos, &ncpu->pending_queue) {
		struct cule_inference_job *j = list_entry(pos, 
							  struct cule_inference_job, list);
		if (j->job_id == job_id) {
			job = j;
			list_del(&j->list);
			break;
		}
	}
	
	spin_unlock_irqrestore(&ncpu->lock, flags);
	
	if (!job)
		return -ENOENT;
	
	/* Find model */
	spin_lock(&cule_models_lock);
	list_for_each_entry(model, &cule_models, list) {
		if (model->model_id == job->model_id)
			break;
	}
	spin_unlock(&cule_models_lock);
	
	/* Restore checkpoint */
	ret = cule_checkpoint_restore(job, model);
	if (ret) {
		pr_err("Failed to restore checkpoint for job %u\n", job_id);
		job->state = CULE_JOB_FAILED;
		return ret;
	}
	
	job->preempt_requested = false;
	job->state = CULE_JOB_RUNNING;
	
	/* Set as current job */
	spin_lock_irqsave(&ncpu->lock, flags);
	ncpu->current_job = job;
	spin_unlock_irqrestore(&ncpu->lock, flags);
	
	complete(&job->resume_complete);
	
	pr_info("Job %u resumed from checkpoint\n", job_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_neural_resume);

/**
 * @brief Register a neural model for inference
 */
int cule_model_register(const char *name, u32 num_layers,
			struct cule_layer_info *layer_info,
			enum cule_accel_type accel,
			u32 *model_id_out)
{
	struct cule_model *model;
	int i, ret;
	
	if (!name || !layer_info || !model_id_out)
		return -EINVAL;
	
	if (num_layers > CULE_NEURAL_MAX_LAYERS)
		return -E2BIG;
	
	model = kzalloc(sizeof(*model), GFP_KERNEL);
	if (!model)
		return -ENOMEM;
	
	model->model_id = atomic_inc_return(&model_id_counter);
	strscpy(model->name, name, sizeof(model->name));
	model->num_layers = num_layers;
	model->accel = accel;
	model->version = 1;
	
	model->layers = kvmalloc_array(num_layers, sizeof(struct cule_layer),
				       GFP_KERNEL);
	if (!model->layers) {
		kfree(model);
		return -ENOMEM;
	}
	
	/* Initialize layers */
	for (i = 0; i < num_layers; i++) {
		struct cule_layer *layer = &model->layers[i];
		struct cule_layer_info *info = &layer_info[i];
		
		layer->layer_id = i;
		layer->type = info->type;
		memcpy(layer->input_dims, info->input_dims, sizeof(layer->input_dims));
		memcpy(layer->output_dims, info->output_dims, sizeof(layer->output_dims));
		layer->weight_size = info->weight_size;
		layer->bias_size = info->bias_size;
		layer->exec_time_ns = info->exec_time_ns;
		
		/* Mark every N layers as preemptible */
		layer->is_preemptible = (i % 4 == 0) || (i == num_layers - 1);
		
		/* Estimate checkpoint time (typically 10-20% of exec time) */
		layer->checkpoint_time_ns = layer->exec_time_ns / 5;
		
		INIT_LIST_HEAD(&layer->checkpoint_list);
	}
	
	/* Initialize checkpoint pool */
	ret = cule_init_checkpoint_pool(model);
	if (ret) {
		kvfree(model->layers);
		kfree(model);
		return ret;
	}
	
	/* Add to global model list */
	spin_lock(&cule_models_lock);
	list_add_tail(&model->list, &cule_models);
	spin_unlock(&cule_models_lock);
	
	*model_id_out = model->model_id;
	
	pr_info("Registered model '%s' (id=%u, layers=%u, accel=%d)\n",
		name, model->model_id, num_layers, accel);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_model_register);

/**
 * @brief Submit inference job
 */
int cule_inference_submit(u32 model_id, u32 task_id,
			  u64 deadline_ns,
			  u32 *job_id_out)
{
	struct cule_neural_cpu *ncpu = this_cpu_ptr(&cule_neural_percpu);
	struct cule_inference_job *job;
	struct cule_model *model;
	
	if (!job_id_out)
		return -EINVAL;
	
	/* Verify model exists */
	model = NULL;
	spin_lock(&cule_models_lock);
	list_for_each_entry(model, &cule_models, list) {
		if (model->model_id == model_id)
			break;
	}
	spin_unlock(&cule_models_lock);
	
	if (!model)
		return -ENOENT;
	
	job = kmem_cache_alloc(job_cache, GFP_KERNEL);
	if (!job)
		return -ENOMEM;
	
	job->job_id = atomic_inc_return(&job_id_counter);
	job->model_id = model_id;
	job->user_task_id = task_id;
	job->total_layers = model->num_layers;
	job->deadline_ns = deadline_ns;
	job->state = CULE_JOB_PENDING;
	job->preempt_requested = false;
	init_completion(&job->resume_complete);
	atomic_set(&job->current_layer, 0);
	
	/* Add to queue */
	unsigned long flags;
	spin_lock_irqsave(&ncpu->lock, flags);
	list_add_tail(&job->list, &ncpu->pending_queue);
	spin_unlock_irqrestore(&ncpu->lock, flags);
	
	*job_id_out = job->job_id;
	
	pr_debug("Submitted inference job %u for model %u (task %u)\n",
		job->job_id, model_id, task_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_inference_submit);

/**
 * @brief Execute layer with preemption check
 */
static int cule_execute_layer(struct cule_inference_job *job,
			      struct cule_model *model,
			      u32 layer_id)
{
	struct cule_layer *layer = &model->layers[layer_id];
	u64 start_ns = ktime_get_ns();
	
	/* Execute layer based on accelerator type */
	switch (model->accel) {
	case CULE_ACCEL_CPU:
		/* CPU execution */
		break;
		
	case CULE_ACCEL_GPU_CUDA:
		/* CUDA kernel launch */
		break;
		
	case CULE_ACCEL_GPU_TRT:
		/* TensorRT execution */
		break;
		
	case CULE_ACCEL_NPU_DLA:
		/* DLA execution */
		break;
		
	case CULE_ACCEL_NPU_PVA:
		/* PVA execution */
		break;
	}
	
	/* Update progress */
	atomic_inc(&job->current_layer);
	
	/* Check for preemption request */
	if (job->preempt_requested && layer->is_preemptible) {
		pr_debug("Job %u: Preemption point at layer %u\n",
			job->job_id, layer_id);
		return -EINTR;  /* Interrupted for preemption */
	}
	
	layer->exec_time_ns = ktime_get_ns() - start_ns;
	
	return 0;
}

/**
 * @brief Main inference execution loop
 */
int cule_inference_execute(u32 job_id)
{
	struct cule_neural_cpu *ncpu = this_cpu_ptr(&cule_neural_percpu);
	struct cule_inference_job *job = NULL;
	struct cule_model *model;
	unsigned long flags;
	u32 current_layer;
	int ret = 0;
	
	/* Find job */
	spin_lock_irqsave(&ncpu->lock, flags);
	list_for_each_entry(job, &ncpu->pending_queue, list) {
		if (job->job_id == job_id) {
			list_del(&job->list);
			break;
		}
	}
	spin_unlock_irqrestore(&ncpu->lock, flags);
	
	if (!job || job->job_id != job_id)
		return -ENOENT;
	
	/* Find model */
	spin_lock(&cule_models_lock);
	list_for_each_entry(model, &cule_models, list) {
		if (model->model_id == job->model_id)
			break;
	}
	spin_unlock(&cule_models_lock);
	
	if (!model)
		return -EINVAL;
	
	/* Set as current job */
	ncpu->current_job = job;
	job->state = CULE_JOB_RUNNING;
	job->start_time_ns = ktime_get_ns();
	
	pr_debug("Starting inference job %u\n", job_id);
	
	/* Execute layers */
	while ((current_layer = atomic_read(&job->current_layer)) < model->num_layers) {
		ret = cule_execute_layer(job, model, current_layer);
		
		if (ret == -EINTR) {
			/* Preempted */
			ncpu->current_job = NULL;
			return 0;  /* Will resume later */
		}
		
		if (ret)
			break;
	}
	
	job->state = (ret == 0) ? CULE_JOB_COMPLETED : CULE_JOB_FAILED;
	ncpu->current_job = NULL;
	model->total_inferences++;
	
	pr_info("Job %u completed: %s (took %llu ms)\n",
		job_id, ret == 0 ? "SUCCESS" : "FAILED",
		(ktime_get_ns() - job->start_time_ns) / 1000000);
	
	kmem_cache_free(job_cache, job);
	
	return ret;
}
EXPORT_SYMBOL_GPL(cule_inference_execute);

/**
 * @brief Get neural server statistics
 */
int cule_neural_get_stats(struct cule_neural_stats *stats)
{
	struct cule_model *model;
	
	if (!stats)
		return -EINVAL;
	
	memset(stats, 0, sizeof(*stats));
	
	spin_lock(&cule_models_lock);
	list_for_each_entry(model, &cule_models, list) {
		stats->total_inferences += model->total_inferences;
		stats->preempted_inferences += model->preempted_inferences;
		stats->checkpoint_saves += model->checkpoint_saves;
		stats->checkpoint_restores += model->checkpoint_restores;
		stats->model_count++;
	}
	spin_unlock(&cule_models_lock);
	
	return 0;
}
EXPORT_SYMBOL_GPL(cule_neural_get_stats);

/**
 * @brief Module initialization
 */
static int __init cule_neural_init(void)
{
	struct cule_neural_cpu *ncpu;
	int cpu;
	
	pr_info("Initializing Cule Neural Server v%s\n", MODULE_VERSION);
	
	/* Create slab caches */
	layer_cache = KMEM_CACHE(cule_layer, SLAB_HWCACHE_ALIGN);
	checkpoint_cache = KMEM_CACHE(cule_layer_checkpoint, SLAB_HWCACHE_ALIGN);
	job_cache = KMEM_CACHE(cule_inference_job, SLAB_HWCACHE_ALIGN);
	
	if (!layer_cache || !checkpoint_cache || !job_cache)
		return -ENOMEM;
	
	/* Initialize per-CPU structures */
	for_each_possible_cpu(cpu) {
		ncpu = per_cpu_ptr(&cule_neural_percpu, cpu);
		spin_lock_init(&ncpu->lock);
		INIT_LIST_HEAD(&ncpu->active_jobs);
		INIT_LIST_HEAD(&ncpu->pending_queue);
		ncpu->current_job = NULL;
		ncpu->preempt_in_progress = false;
		ncpu->preempt_count = 0;
	}
	
	pr_info("Cule Neural Server initialized\n");
	return 0;
}

/**
 * @brief Module cleanup
 */
static void __exit cule_neural_exit(void)
{
	struct cule_model *model, *tmp;
	
	pr_info("Exiting Cule Neural Server\n");
	
	/* Cleanup models */
	spin_lock(&cule_models_lock);
	list_for_each_entry_safe(model, tmp, &cule_models, list) {
		if (model->checkpoint_pool) {
			for (int i = 0; i < model->checkpoint_pool_size; i++) {
				dma_free_coherent(NULL, model->checkpoint_pool[i].activation_size,
						  model->checkpoint_pool[i].activation_data,
						  model->checkpoint_pool[i].activation_dma);
			}
			kvfree(model->checkpoint_pool);
		}
		kvfree(model->layers);
		list_del(&model->list);
		kfree(model);
	}
	spin_unlock(&cule_models_lock);
	
	/* Destroy caches */
	kmem_cache_destroy(job_cache);
	kmem_cache_destroy(checkpoint_cache);
	kmem_cache_destroy(layer_cache);
	
	pr_info("Cule Neural Server exited\n");
}

module_init(cule_neural_init);
module_exit(cule_neural_exit);
