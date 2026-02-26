/**
 * @file ipc.c
 * @brief Cule OS Inter-Agent Communication with Zero-Copy Message Passing
 * 
 * Implements high-performance IPC using shared memory rings and
 * cache-optimized zero-copy transfer between agents.
 * 
 * Target: NVIDIA Jetson AGX Orin (ARM64, cache-coherent)
 * License: GPL-2.0
 */

#define pr_fmt(fmt) "cule_ipc: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/ktime.h>
#include <linux/cache.h>
#include <linux/skbuff.h>
#include <asm/cacheflush.h>
#include <asm/barrier.h>

#include "../include/cule/kernel.h"

MODULE_AUTHOR("Cule OS Team");
MODULE_DESCRIPTION("Cule OS Zero-Copy IPC Subsystem");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");

/* IPC message types */
enum cule_msg_type {
	CULE_MSG_DATA = 0,          /* Regular data message */
	CULE_MSG_CONTROL,           /* Control/command message */
	CULE_MSG_SIGNAL,            /* Lightweight signal */
	CULE_MSG_RPC_REQUEST,       /* RPC request */
	CULE_MSG_RPC_RESPONSE,      /* RPC response */
	CULE_MSG_DMA_DESCRIPTOR,    /* DMA descriptor for large transfers */
};

/* Message priority */
enum cule_msg_priority {
	CULE_PRIO_LOW = 0,
	CULE_PRIO_NORMAL,
	CULE_PRIO_HIGH,
	CULE_PRIO_REALTIME,
};

/* Message flags */
#define CULE_MSG_FLAG_ASYNC     (1 << 0)  /* Async send, no reply expected */
#define CULE_MSG_FLAG_URGENT    (1 << 1)  /* Skip to front of queue */
#define CULE_MSG_FLAG_NO_COPY   (1 << 2)  /* Use zero-copy path */
#define CULE_MSG_FLAG_DMA       (1 << 3)  /* Large data via DMA */

/* Ring buffer descriptor */
struct cule_ipc_ring {
	/* Producer/consumer indices - cache line aligned */
	union {
		struct {
			volatile u32 head;
			volatile u32 tail;
		};
		u8 padding[L1_CACHE_BYTES];
	};
	
	/* Ring configuration */
	u32 size;                   /* Number of slots */
	u32 msg_size;               /* Size of each message slot */
	u32 flags;
	
	/* Buffer pointers */
	void *buffer;               /* Message buffer */
	dma_addr_t buffer_dma;      /* DMA address for buffer */
	
	/* Statistics */
	u64 msg_count;
	u64 drop_count;
	u64 bytes_transferred;
	
	/* Synchronization */
	spinlock_t lock;
	wait_queue_head_t wait;
};

/* IPC channel endpoint */
struct cule_ipc_endpoint {
	u32 endpoint_id;
	u32 agent_id;
	
	/* Ring buffers */
	struct cule_ipc_ring *rx_ring;      /* Receive ring */
	struct cule_ipc_ring *tx_ring;      /* Transmit ring */
	
	/* Callbacks */
	void (*recv_callback)(struct cule_ipc_msg *msg, void *priv);
	void *callback_priv;
	
	/* State */
	atomic_t ref_count;
	bool registered;
	
	struct list_head list;
};

/* IPC channel between two agents */
struct cule_ipc_channel {
	u32 channel_id;
	u32 agent_a;
	u32 agent_b;
	
	/* Endpoints */
	struct cule_ipc_endpoint *ep_a;
	struct cule_ipc_endpoint *ep_b;
	
	/* Shared memory pool for zero-copy */
	struct cule_ipc_pool *shared_pool;
	
	/* State */
	bool active;
	u64 created_ns;
	
	struct list_head list;
};

/* Zero-copy shared memory pool */
struct cule_ipc_pool {
	void *base;
	size_t size;
	dma_addr_t dma_base;
	
	/* Free list - simple bitmap allocator */
	unsigned long *bitmap;
	u32 num_blocks;
	u32 block_size;
	spinlock_t alloc_lock;
	
	struct list_head active_chunks;
};

/* DMA descriptor for large transfers */
struct cule_dma_descriptor {
	u64 src_addr;
	u64 dst_addr;
	size_t length;
	u32 flags;
	u32 completion_cookie;
};

/* Global IPC state */
static LIST_HEAD(cule_endpoints);
static LIST_HEAD(cule_channels);
static DEFINE_SPINLOCK(cule_ipc_lock);
static atomic_t endpoint_counter = ATOMIC_INIT(0);
static atomic_t channel_counter = ATOMIC_INIT(0);

/* Configuration */
#define CULE_IPC_RING_SIZE      256     /* Messages per ring */
#define CULE_IPC_MSG_SIZE       256     /* Small message size */
#define CULE_IPC_MAX_MSG_SIZE   65536   /* Large message threshold */
#define CULE_IPC_POOL_SIZE      (4 * 1024 * 1024)  /* 4MB shared pool */
#define CULE_IPC_POOL_BLOCK     4096    /* 4KB allocation unit */

/* Cache line optimization */
#define CULE_CACHE_ALIGN        L1_CACHE_BYTES
#define CULE_RING_IDX_MASK(idx, size) ((idx) & ((size) - 1))

/**
 * @brief Memory barrier for cache coherency
 */
static inline void cule_ipc_mb(void)
{
	smp_mb();
}

static inline void cule_ipc_rmb(void)
{
	smp_rmb();
}

static inline void cule_ipc_wmb(void)
{
	smp_wmb();
}

/**
 * @brief Create IPC ring buffer
 */
static struct cule_ipc_ring *cule_ring_create(u32 size, u32 msg_size)
{
	struct cule_ipc_ring *ring;
	void *buffer;
	
	ring = kzalloc(sizeof(*ring), GFP_KERNEL);
	if (!ring)
		return NULL;
	
	ring->size = roundup_pow_of_two(size);
	ring->msg_size = ALIGN(msg_size, CULE_CACHE_ALIGN);
	
	/* Allocate cache-aligned buffer */
	buffer = kzalloc(ring->size * ring->msg_size, GFP_KERNEL | GFP_DMA);
	if (!buffer) {
		kfree(ring);
		return NULL;
	}
	
	ring->buffer = buffer;
	ring->head = 0;
	ring->tail = 0;
	ring->flags = 0;
	
	spin_lock_init(&ring->lock);
	init_waitqueue_head(&ring->wait);
	
	/* Get DMA address if coherent */
	ring->buffer_dma = dma_map_single(NULL, buffer, 
					  ring->size * ring->msg_size,
					  DMA_BIDIRECTIONAL);
	
	pr_debug("Created IPC ring: %u slots x %u bytes\n", 
		ring->size, ring->msg_size);
	
	return ring;
}

/**
 * @brief Destroy IPC ring buffer
 */
static void cule_ring_destroy(struct cule_ipc_ring *ring)
{
	if (!ring)
		return;
	
	if (ring->buffer) {
		dma_unmap_single(NULL, ring->buffer_dma,
				 ring->size * ring->msg_size,
				 DMA_BIDIRECTIONAL);
		kfree(ring->buffer);
	}
	
	kfree(ring);
}

/**
 * @brief Create shared memory pool for zero-copy
 */
static struct cule_ipc_pool *cule_pool_create(size_t size)
{
	struct cule_ipc_pool *pool;
	
	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return NULL;
	
	pool->size = size;
	pool->block_size = CULE_IPC_POOL_BLOCK;
	pool->num_blocks = size / pool->block_size;
	
	/* Allocate shared memory */
	pool->base = dma_alloc_coherent(NULL, size, &pool->dma_base, GFP_KERNEL);
	if (!pool->base) {
		kfree(pool);
		return NULL;
	}
	
	/* Allocate bitmap */
	pool->bitmap = kcalloc(BITS_TO_LONGS(pool->num_blocks), 
			       sizeof(unsigned long), GFP_KERNEL);
	if (!pool->bitmap) {
		dma_free_coherent(NULL, size, pool->base, pool->dma_base);
		kfree(pool);
		return NULL;
	}
	
	/* Mark all blocks free */
	bitmap_zero(pool->bitmap, pool->num_blocks);
	
	spin_lock_init(&pool->alloc_lock);
	INIT_LIST_HEAD(&pool->active_chunks);
	
	pr_info("Created IPC pool: %zu bytes (%u blocks)\n", size, pool->num_blocks);
	return pool;
}

/**
 * @brief Destroy shared memory pool
 */
static void cule_pool_destroy(struct cule_ipc_pool *pool)
{
	if (!pool)
		return;
	
	if (pool->base)
		dma_free_coherent(NULL, pool->size, pool->base, pool->dma_base);
	
	kfree(pool->bitmap);
	kfree(pool);
}

/**
 * @brief Allocate from shared pool
 */
static void *cule_pool_alloc(struct cule_ipc_pool *pool, size_t size,
			     dma_addr_t *dma_handle, u32 *block_offset)
{
	u32 blocks_needed = DIV_ROUND_UP(size, pool->block_size);
	u32 start_block;
	unsigned long flags;
	void *ptr;
	
	spin_lock_irqsave(&pool->alloc_lock, flags);
	
	start_block = bitmap_find_next_zero_area(pool->bitmap, pool->num_blocks,
						0, blocks_needed, 0);
	
	if (start_block >= pool->num_blocks) {
		spin_unlock_irqrestore(&pool->alloc_lock, flags);
		return NULL;
	}
	
	bitmap_set(pool->bitmap, start_block, blocks_needed);
	
	spin_unlock_irqrestore(&pool->alloc_lock, flags);
	
	ptr = pool->base + (start_block * pool->block_size);
	*dma_handle = pool->dma_base + (start_block * pool->block_size);
	*block_offset = start_block;
	
	return ptr;
}

/**
 * @brief Free to shared pool
 */
static void cule_pool_free(struct cule_ipc_pool *pool, u32 block_offset, 
			   u32 blocks)
{
	unsigned long flags;
	
	spin_lock_irqsave(&pool->alloc_lock, flags);
	bitmap_clear(pool->bitmap, block_offset, blocks);
	spin_unlock_irqrestore(&pool->alloc_lock, flags);
}

/**
 * @brief Register IPC endpoint for agent
 */
int cule_ipc_endpoint_register(u32 agent_id, u32 *endpoint_id_out)
{
	struct cule_ipc_endpoint *ep;
	u32 ep_id;
	
	if (!endpoint_id_out)
		return -EINVAL;
	
	ep = kzalloc(sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;
	
	ep_id = atomic_inc_return(&endpoint_counter);
	ep->endpoint_id = ep_id;
	ep->agent_id = agent_id;
	
	/* Create rings */
	ep->rx_ring = cule_ring_create(CULE_IPC_RING_SIZE, CULE_IPC_MSG_SIZE);
	ep->tx_ring = cule_ring_create(CULE_IPC_RING_SIZE, CULE_IPC_MSG_SIZE);
	
	if (!ep->rx_ring || !ep->tx_ring) {
		cule_ring_destroy(ep->rx_ring);
		cule_ring_destroy(ep->tx_ring);
		kfree(ep);
		return -ENOMEM;
	}
	
	atomic_set(&ep->ref_count, 1);
	ep->registered = true;
	
	spin_lock(&cule_ipc_lock);
	list_add_tail(&ep->list, &cule_endpoints);
	spin_unlock(&cule_ipc_lock);
	
	*endpoint_id_out = ep_id;
	
	pr_info("Registered IPC endpoint %u for agent %u\n", ep_id, agent_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_ipc_endpoint_register);

/**
 * @brief Unregister IPC endpoint
 */
int cule_ipc_endpoint_unregister(u32 endpoint_id)
{
	struct cule_ipc_endpoint *ep, *tmp;
	bool found = false;
	
	spin_lock(&cule_ipc_lock);
	list_for_each_entry_safe(ep, tmp, &cule_endpoints, list) {
		if (ep->endpoint_id == endpoint_id) {
			list_del(&ep->list);
			found = true;
			break;
		}
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!found)
		return -ENOENT;
	
	ep->registered = false;
	
	/* Wait for refs to drop */
	while (atomic_read(&ep->ref_count) > 0)
		schedule_timeout_uninterruptible(1);
	
	cule_ring_destroy(ep->rx_ring);
	cule_ring_destroy(ep->tx_ring);
	kfree(ep);
	
	pr_info("Unregistered IPC endpoint %u\n", endpoint_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_ipc_endpoint_unregister);

/**
 * @brief Create IPC channel between agents
 */
int cule_ipc_channel_create(u32 agent_a, u32 agent_b, u32 *channel_id_out)
{
	struct cule_ipc_channel *ch;
	struct cule_ipc_endpoint *ep_a = NULL, *ep_b = NULL;
	struct cule_ipc_endpoint *ep;
	u32 ch_id;
	
	if (agent_a == agent_b || !channel_id_out)
		return -EINVAL;
	
	/* Find or create endpoints */
	spin_lock(&cule_ipc_lock);
	list_for_each_entry(ep, &cule_endpoints, list) {
		if (ep->agent_id == agent_a && !ep_a)
			ep_a = ep;
		if (ep->agent_id == agent_b && !ep_b)
			ep_b = ep;
	}
	spin_unlock(&cule_ipc_lock);
	
	/* Create endpoints if not found */
	if (!ep_a) {
		u32 ep_id;
		int ret = cule_ipc_endpoint_register(agent_a, &ep_id);
		if (ret)
			return ret;
		
		spin_lock(&cule_ipc_lock);
		list_for_each_entry(ep, &cule_endpoints, list) {
			if (ep->endpoint_id == ep_id) {
				ep_a = ep;
				break;
			}
		}
		spin_unlock(&cule_ipc_lock);
	}
	
	if (!ep_b) {
		u32 ep_id;
		int ret = cule_ipc_endpoint_register(agent_b, &ep_id);
		if (ret)
			return ret;
		
		spin_lock(&cule_ipc_lock);
		list_for_each_entry(ep, &cule_endpoints, list) {
			if (ep->endpoint_id == ep_id) {
				ep_b = ep;
				break;
			}
		}
		spin_unlock(&cule_ipc_lock);
	}
	
	if (!ep_a || !ep_b)
		return -EFAULT;
	
	ch = kzalloc(sizeof(*ch), GFP_KERNEL);
	if (!ch)
		return -ENOMEM;
	
	ch_id = atomic_inc_return(&channel_counter);
	ch->channel_id = ch_id;
	ch->agent_a = agent_a;
	ch->agent_b = agent_b;
	ch->ep_a = ep_a;
	ch->ep_b = ep_b;
	ch->active = true;
	ch->created_ns = ktime_get_ns();
	
	/* Create shared pool for zero-copy */
	ch->shared_pool = cule_pool_create(CULE_IPC_POOL_SIZE);
	if (!ch->shared_pool) {
		kfree(ch);
		return -ENOMEM;
	}
	
	atomic_inc(&ep_a->ref_count);
	atomic_inc(&ep_b->ref_count);
	
	spin_lock(&cule_ipc_lock);
	list_add_tail(&ch->list, &cule_channels);
	spin_unlock(&cule_ipc_lock);
	
	*channel_id_out = ch_id;
	
	pr_info("Created IPC channel %u: agents %u <-> %u\n", 
		ch_id, agent_a, agent_b);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_ipc_channel_create);

/**
 * @brief Destroy IPC channel
 */
int cule_ipc_channel_destroy(u32 channel_id)
{
	struct cule_ipc_channel *ch, *tmp;
	bool found = false;
	
	spin_lock(&cule_ipc_lock);
	list_for_each_entry_safe(ch, tmp, &cule_channels, list) {
		if (ch->channel_id == channel_id) {
			list_del(&ch->list);
			found = true;
			break;
		}
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!found)
		return -ENOENT;
	
	ch->active = false;
	
	atomic_dec(&ch->ep_a->ref_count);
	atomic_dec(&ch->ep_b->ref_count);
	
	cule_pool_destroy(ch->shared_pool);
	kfree(ch);
	
	pr_info("Destroyed IPC channel %u\n", channel_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_ipc_channel_destroy);

/**
 * @brief Send message (zero-copy for large messages)
 */
int cule_ipc_send(u32 src_endpoint, u32 dst_endpoint, 
		  struct cule_ipc_msg *msg)
{
	struct cule_ipc_endpoint *src_ep = NULL, *dst_ep = NULL;
	struct cule_ipc_endpoint *ep;
	struct cule_ipc_ring *tx_ring;
	struct cule_ipc_channel *ch = NULL;
	u32 idx;
	void *slot;
	u64 start_ns;
	int ret = 0;
	
	if (!msg)
		return -EINVAL;
	
	start_ns = ktime_get_ns();
	
	/* Find endpoints */
	spin_lock(&cule_ipc_lock);
	list_for_each_entry(ep, &cule_endpoints, list) {
		if (ep->endpoint_id == src_endpoint)
			src_ep = ep;
		if (ep->endpoint_id == dst_endpoint)
			dst_ep = ep;
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!src_ep || !dst_ep)
		return -ENOENT;
	
	/* Find channel */
	spin_lock(&cule_ipc_lock);
	list_for_each_entry(ch, &cule_channels, list) {
		if ((ch->ep_a == src_ep && ch->ep_b == dst_ep) ||
		    (ch->ep_a == dst_ep && ch->ep_b == src_ep)) {
			break;
		}
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!ch)
		return -ENOTCONN;
	
	tx_ring = src_ep->tx_ring;
	
	/* Check if message needs zero-copy */
	if (msg->length > CULE_IPC_MSG_SIZE - sizeof(struct cule_ipc_msg_hdr)) {
		/* Use zero-copy path */
		dma_addr_t dma_handle;
		u32 block_offset;
		void *shared_buf;
		
		shared_buf = cule_pool_alloc(ch->shared_pool, msg->length,
					     &dma_handle, &block_offset);
		if (!shared_buf)
			return -ENOMEM;
		
		/* Copy to shared memory */
		memcpy(shared_buf, msg->data, msg->length);
		__flush_dcache_area(shared_buf, msg->length);
		
		/* Send descriptor */
		msg->flags |= CULE_MSG_FLAG_NO_COPY;
		msg->dma_handle = dma_handle;
		msg->pool_block = block_offset;
	}
	
	/* Write to ring buffer */
	spin_lock(&tx_ring->lock);
	
	idx = CULE_RING_IDX_MASK(tx_ring->head, tx_ring->size);
	slot = tx_ring->buffer + (idx * tx_ring->msg_size);
	
	/* Build message header */
	struct cule_ipc_msg_hdr *hdr = slot;
	hdr->type = msg->type;
	hdr->src_agent = src_ep->agent_id;
	hdr->dst_agent = dst_ep->agent_id;
	hdr->length = msg->length;
	hdr->flags = msg->flags;
	hdr->timestamp_ns = start_ns;
	hdr->priority = msg->priority;
	
	if (msg->flags & CULE_MSG_FLAG_NO_COPY) {
		hdr->dma_handle = msg->dma_handle;
		hdr->pool_block = msg->pool_block;
	} else {
		/* Copy inline */
		memcpy(hdr->data, msg->data, min(msg->length, 
				(u32)(tx_ring->msg_size - sizeof(*hdr))));
	}
	
	/* Memory barrier before updating head */
	cule_ipc_wmb();
	tx_ring->head++;
	tx_ring->msg_count++;
	tx_ring->bytes_transferred += msg->length;
	
	spin_unlock(&tx_ring->lock);
	
	/* Wake receiver */
	wake_up(&dst_ep->rx_ring->wait);
	
	msg->send_latency_ns = ktime_get_ns() - start_ns;
	
	pr_debug("IPC send: %u->%u, type=%u, len=%u, flags=%x\n",
		src_endpoint, dst_endpoint, msg->type, msg->length, msg->flags);
	
	return ret;
}
EXPORT_SYMBOL_GPL(cule_ipc_send);

/**
 * @brief Receive message
 */
int cule_ipc_recv(u32 endpoint_id, struct cule_ipc_msg *msg, bool blocking)
{
	struct cule_ipc_endpoint *ep = NULL;
	struct cule_ipc_endpoint *iter;
	struct cule_ipc_ring *rx_ring;
	struct cule_ipc_msg_hdr *hdr;
	u32 idx;
	void *slot;
	int ret = 0;
	
	if (!msg)
		return -EINVAL;
	
	/* Find endpoint */
	spin_lock(&cule_ipc_lock);
	list_for_each_entry(iter, &cule_endpoints, list) {
		if (iter->endpoint_id == endpoint_id) {
			ep = iter;
			break;
		}
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!ep)
		return -ENOENT;
	
	rx_ring = ep->rx_ring;
	
	/* Check if data available */
	if (CULE_RING_IDX_MASK(rx_ring->head, rx_ring->size) == 
	    CULE_RING_IDX_MASK(rx_ring->tail, rx_ring->size)) {
		if (!blocking)
			return -EAGAIN;
		
		/* Wait for data */
		ret = wait_event_interruptible(rx_ring->wait,
			rx_ring->head != rx_ring->tail);
		if (ret)
			return ret;
	}
	
	/* Read message */
	spin_lock(&rx_ring->lock);
	
	idx = CULE_RING_IDX_MASK(rx_ring->tail, rx_ring->size);
	slot = rx_ring->buffer + (idx * rx_ring->msg_size);
	hdr = slot;
	
	msg->type = hdr->type;
	msg->src_agent = hdr->src_agent;
	msg->dst_agent = hdr->dst_agent;
	msg->length = hdr->length;
	msg->flags = hdr->flags;
	msg->timestamp_ns = hdr->timestamp_ns;
	msg->priority = hdr->priority;
	
	if (hdr->flags & CULE_MSG_FLAG_NO_COPY) {
		msg->dma_handle = hdr->dma_handle;
		msg->pool_block = hdr->pool_block;
		/* Receiver will map shared memory */
	} else {
		memcpy(msg->data, hdr->data, min(hdr->length, 
				(u32)(rx_ring->msg_size - sizeof(*hdr))));
	}
	
	/* Memory barrier after read */
	cule_ipc_rmb();
	rx_ring->tail++;
	
	spin_unlock(&rx_ring->lock);
	
	pr_debug("IPC recv: ep=%u, type=%u, len=%u\n",
		endpoint_id, msg->type, msg->length);
	
	return 0;
}
EXPORT_SYMBOL_GPL(cule_ipc_recv);

/**
 * @brief Map shared memory from zero-copy message
 */
void *cule_ipc_map_shared(u32 channel_id, dma_addr_t dma_handle, 
			  size_t size)
{
	struct cule_ipc_channel *ch, *iter;
	void *ptr = NULL;
	
	spin_lock(&cule_ipc_lock);
	list_for_each_entry(iter, &cule_channels, list) {
		if (iter->channel_id == channel_id) {
			ch = iter;
			break;
		}
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!ch)
		return NULL;
	
	/* Calculate offset in pool */
	if (dma_handle >= ch->shared_pool->dma_base &&
	    dma_handle < ch->shared_pool->dma_base + ch->shared_pool->size) {
		ptr = ch->shared_pool->base + (dma_handle - ch->shared_pool->dma_base);
		__inval_dcache_area(ptr, size);
	}
	
	return ptr;
}
EXPORT_SYMBOL_GPL(cule_ipc_map_shared);

/**
 * @brief Release shared memory back to pool
 */
int cule_ipc_release_shared(u32 channel_id, u32 pool_block, size_t size)
{
	struct cule_ipc_channel *ch, *iter;
	u32 blocks;
	
	spin_lock(&cule_ipc_lock);
	list_for_each_entry(iter, &cule_channels, list) {
		if (iter->channel_id == channel_id) {
			ch = iter;
			break;
		}
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!ch)
		return -ENOENT;
	
	blocks = DIV_ROUND_UP(size, ch->shared_pool->block_size);
	cule_pool_free(ch->shared_pool, pool_block, blocks);
	
	return 0;
}
EXPORT_SYMBOL_GPL(cule_ipc_release_shared);

/**
 * @brief Get IPC statistics
 */
int cule_ipc_get_stats(u32 endpoint_id, struct cule_ipc_stats *stats)
{
	struct cule_ipc_endpoint *ep = NULL;
	struct cule_ipc_endpoint *iter;
	
	if (!stats)
		return -EINVAL;
	
	spin_lock(&cule_ipc_lock);
	list_for_each_entry(iter, &cule_endpoints, list) {
		if (iter->endpoint_id == endpoint_id) {
			ep = iter;
			break;
		}
	}
	spin_unlock(&cule_ipc_lock);
	
	if (!ep)
		return -ENOENT;
	
	memset(stats, 0, sizeof(*stats));
	
	stats->messages_sent = ep->tx_ring->msg_count;
	stats->messages_recv = ep->rx_ring->msg_count;
	stats->bytes_sent = ep->tx_ring->bytes_transferred;
	stats->bytes_recv = ep->rx_ring->bytes_transferred;
	stats->drops = ep->tx_ring->drop_count + ep->rx_ring->drop_count;
	
	return 0;
}
EXPORT_SYMBOL_GPL(cule_ipc_get_stats);

/**
 * @brief Module initialization
 */
static int __init cule_ipc_init(void)
{
	pr_info("Initializing Cule IPC Subsystem v%s\n", MODULE_VERSION);
	
	/* Verify cache line size */
	BUILD_BUG_ON(CULE_IPC_MSG_SIZE % L1_CACHE_BYTES != 0);
	
	pr_info("IPC ring size: %u messages x %u bytes\n",
		CULE_IPC_RING_SIZE, CULE_IPC_MSG_SIZE);
	pr_info("Zero-copy threshold: %u bytes\n", CULE_IPC_MAX_MSG_SIZE);
	
	return 0;
}

/**
 * @brief Module cleanup
 */
static void __exit cule_ipc_exit(void)
{
	struct cule_ipc_channel *ch, *ch_tmp;
	struct cule_ipc_endpoint *ep, *ep_tmp;
	
	pr_info("Exiting Cule IPC Subsystem\n");
	
	/* Destroy all channels */
	list_for_each_entry_safe(ch, ch_tmp, &cule_channels, list) {
		cule_ipc_channel_destroy(ch->channel_id);
	}
	
	/* Destroy all endpoints */
	list_for_each_entry_safe(ep, ep_tmp, &cule_endpoints, list) {
		cule_ipc_endpoint_unregister(ep->endpoint_id);
	}
}

module_init(cule_ipc_init);
module_exit(cule_ipc_exit);
