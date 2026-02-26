/**
 * @file memory.c
 * @brief Cule OS Memory Protection Subsystem for Inter-Agent Isolation
 * 
 * Implements hardware-backed memory protection using ARM64 MMU features
 * to isolate agent memory spaces and prevent unauthorized access.
 * 
 * Target: NVIDIA Jetson AGX Orin (ARM64, MMU, SMMU)
 * License: GPL-2.0
 */

#define pr_fmt(fmt) "cule_mem: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/mmu_context.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/genalloc.h>
#include <linux/spinlock.h>
#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>

#include "../include/cule/kernel.h"

MODULE_AUTHOR("Cule OS Team");
MODULE_DESCRIPTION("Cule OS Memory Protection Subsystem");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0");

/* Memory region types */
enum cule_mem_type {
	CULE_MEM_KERNEL,        /* Kernel-only access */
	CULE_MEM_AGENT_PRIVATE, /* Single agent private memory */
	CULE_MEM_AGENT_SHARED,  /* Shared between specific agents */
	CULE_MEM_DMA_POOL,      /* DMA coherent pool */
	CULE_MEM_DEVICE,        /* Device memory (MMIO) */
};

/* Memory protection attributes */
enum cule_mem_prot {
	CULE_PROT_NONE  = 0,
	CULE_PROT_READ  = (1 << 0),
	CULE_PROT_WRITE = (1 << 1),
	CULE_PROT_EXEC  = (1 << 2),
	CULE_PROT_DMA   = (1 << 3),   /* DMA access allowed */
	CULE_PROT_NOCACHE = (1 << 4), /* Non-cacheable */
};

/* Memory region descriptor */
struct cule_mem_region {
	struct rb_node node;
	u64 region_id;
	
	/* Address range */
	phys_addr_t phys_start;
	void *virt_start;
	size_t size;
	
	/* Ownership */
	u32 owner_agent_id;
	u32 *shared_agents;
	u32 shared_count;
	bitmap_t *access_bitmap;  /* Which agents have access */
	
	/* Protection */
	enum cule_mem_type type;
	enum cule_mem_prot prot;
	
	/* Page tables */
	pgd_t *pgd;
	struct mm_struct *mm;
	
	/* DMA */
	dma_addr_t dma_handle;
	struct device *dma_dev;
	
	/* Statistics */
	u64 access_count;
	u64 fault_count;
	
	struct list_head list;
};

/* Agent memory context */
struct cule_agent_mem_ctx {
	u32 agent_id;
	struct mm_struct *mm;
	pgd_t *pgd;
	struct rb_root regions;
	rwlock_t regions_lock;
	
	/* Allocation pools */
	struct gen_pool *small_pool;   /* < 4KB allocations */
	struct gen_pool *medium_pool;  /* 4KB - 1MB */
	struct gen_pool *large_pool;   /* > 1MB */
	
	/* DMA */
	struct device *dma_dev;
	dma_addr_t dma_mask;
	
	/* Statistics */
	u64 total_allocated;
	u64 current_usage;
	u64 peak_usage;
	u64 fault_count;
};

/* Global memory state */
static DEFINE_MUTEX(cule_mem_mutex);
static struct rb_root cule_mem_regions = RB_ROOT;
static struct cule_agent_mem_ctx *agent_contexts[CULE_MAX_AGENTS];
static DEFINE_SPINLOCK(agent_ctx_lock);

/* Hardware capabilities */
static bool has_smmu = false;
static bool has_stage2 = false;
static u64 max_phys_addr = 0;

/* Pool sizes */
#define CULE_SMALL_POOL_SIZE    (4 * 1024 * 1024ULL)    /* 4 MB */
#define CULE_MEDIUM_POOL_SIZE   (64 * 1024 * 1024ULL)   /* 64 MB */
#define CULE_LARGE_POOL_SIZE    (256 * 1024 * 1024ULL)  /* 256 MB */

#define CULE_PAGE_SIZE          PAGE_SIZE
#define CULE_HUGE_PAGE_SIZE     (2 * 1024 * 1024ULL)    /* 2MB huge pages */

/**
 * @brief Insert region into RB tree
 */
static void cule_mem_region_insert(struct rb_root *root, 
				   struct cule_mem_region *region)
{
	struct rb_node **new = &(root->rb_node), *parent = NULL;
	
	while (*new) {
		struct cule_mem_region *this = container_of(*new, 
							    struct cule_mem_region, node);
		parent = *new;
		if (region->phys_start < this->phys_start)
			new = &((*new)->rb_left);
		else if (region->phys_start > this->phys_start)
			new = &((*new)->rb_right);
		else
			BUG();  /* Duplicate region */
	}
	
	rb_link_node(&region->node, parent, new);
	rb_insert_color(&region->node, root);
}

/**
 * @brief Find region containing address
 */
static struct cule_mem_region *cule_mem_region_find(struct rb_root *root, 
						    phys_addr_t addr)
{
	struct rb_node *node = root->rb_node;
	
	while (node) {
		struct cule_mem_region *region = container_of(node, 
							    struct cule_mem_region, node);
		
		if (addr < region->phys_start)
			node = node->rb_left;
		else if (addr >= region->phys_start + region->size)
			node = node->rb_right;
		else
			return region;
	}
	
	return NULL;
}

/**
 * @brief Create memory protection context for agent
 */
int cule_agent_mem_create(u32 agent_id, struct cule_agent_mem_ctx **ctx_out)
{
	struct cule_agent_mem_ctx *ctx;
	struct mm_struct *mm;
	int ret;
	
	if (agent_id >= CULE_MAX_AGENTS)
		return -EINVAL;
	
	if (agent_contexts[agent_id])
		return -EEXIST;
	
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	
	ctx->agent_id = agent_id;
	
	/* Allocate private page tables */
	mm = mm_alloc();
	if (!mm) {
		ret = -ENOMEM;
		goto free_ctx;
	}
	
	ctx->mm = mm;
	ctx->pgd = mm->pgd;
	
	/* Initialize region tree */
	ctx->regions = RB_ROOT;
	rwlock_init(&ctx->regions_lock);
	
	/* Create allocation pools */
	ctx->small_pool = gen_pool_create(ilog2(256), NUMA_NO_NODE);  /* 256 byte min */
	if (!ctx->small_pool) {
		ret = -ENOMEM;
		goto free_mm;
	}
	
	ctx->medium_pool = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
	if (!ctx->medium_pool) {
		ret = -ENOMEM;
		goto free_small;
	}
	
	ctx->large_pool = gen_pool_create(ilog2(CULE_HUGE_PAGE_SIZE), NUMA_NO_NODE);
	if (!ctx->large_pool) {
		ret = -ENOMEM;
		goto free_medium;
	}
	
	/* Add memory to pools */
	void *small_mem = vmalloc(CULE_SMALL_POOL_SIZE);
	void *medium_mem = vmalloc(CULE_MEDIUM_POOL_SIZE);
	void *large_mem = vmalloc(CULE_LARGE_POOL_SIZE);
	
	if (!small_mem || !medium_mem || !large_mem) {
		ret = -ENOMEM;
		goto free_pools;
	}
	
	gen_pool_add(ctx->small_pool, (unsigned long)small_mem, 
		     CULE_SMALL_POOL_SIZE, NUMA_NO_NODE);
	gen_pool_add(ctx->medium_pool, (unsigned long)medium_mem,
		     CULE_MEDIUM_POOL_SIZE, NUMA_NO_NODE);
	gen_pool_add(ctx->large_pool, (unsigned long)large_mem,
		     CULE_LARGE_POOL_SIZE, NUMA_NO_NODE);
	
	spin_lock(&agent_ctx_lock);
	agent_contexts[agent_id] = ctx;
	spin_unlock(&agent_ctx_lock);
	
	*ctx_out = ctx;
	
	pr_info("Created memory context for agent %u\n", agent_id);
	return 0;

free_pools:
	if (small_mem) vfree(small_mem);
	if (medium_mem) vfree(medium_mem);
	if (large_mem) vfree(large_mem);
	gen_pool_destroy(ctx->large_pool);
free_medium:
	gen_pool_destroy(ctx->medium_pool);
free_small:
	gen_pool_destroy(ctx->small_pool);
free_mm:
	mmput(mm);
free_ctx:
	kfree(ctx);
	return ret;
}
EXPORT_SYMBOL_GPL(cule_agent_mem_create);

/**
 * @brief Destroy agent memory context
 */
int cule_agent_mem_destroy(u32 agent_id)
{
	struct cule_agent_mem_ctx *ctx;
	struct cule_mem_region *region;
	struct rb_node *node;
	
	if (agent_id >= CULE_MAX_AGENTS)
		return -EINVAL;
	
	spin_lock(&agent_ctx_lock);
	ctx = agent_contexts[agent_id];
	agent_contexts[agent_id] = NULL;
	spin_unlock(&agent_ctx_lock);
	
	if (!ctx)
		return -ENOENT;
	
	/* Free all regions */
	write_lock(&ctx->regions_lock);
	node = rb_first(&ctx->regions);
	while (node) {
		region = rb_entry(node, struct cule_mem_region, node);
		rb_erase(&region->node, &ctx->regions);
		
		/* Free page tables for this region */
		if (region->pgd) {
			/* Walk and free page tables */
			cule_mem_free_pagetables(region->pgd, region->virt_start, 
						 region->size);
		}
		
		/* Free DMA memory */
		if (region->dma_handle && region->dma_dev) {
			dma_free_coherent(region->dma_dev, region->size,
					  region->virt_start, region->dma_handle);
		}
		
		kfree(region->shared_agents);
		kvfree(region->access_bitmap);
		kfree(region);
		
		node = rb_first(&ctx->regions);
	}
	write_unlock(&ctx->regions_lock);
	
	/* Destroy pools */
	gen_pool_destroy(ctx->small_pool);
	gen_pool_destroy(ctx->medium_pool);
	gen_pool_destroy(ctx->large_pool);
	
	/* Release mm */
	mmput(ctx->mm);
	
	kfree(ctx);
	
	pr_info("Destroyed memory context for agent %u\n", agent_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_agent_mem_destroy);

/**
 * @brief Allocate protected memory region for agent
 */
int cule_mem_alloc(u32 agent_id, size_t size, enum cule_mem_type type,
		   enum cule_mem_prot prot, u64 *region_id_out)
{
	struct cule_agent_mem_ctx *ctx;
	struct cule_mem_region *region;
	struct gen_pool *pool;
	void *virt_addr;
	phys_addr_t phys_addr;
	u64 region_id;
	int ret = 0;
	
	if (agent_id >= CULE_MAX_AGENTS || !region_id_out)
		return -EINVAL;
	
	ctx = agent_contexts[agent_id];
	if (!ctx)
		return -ENOENT;
	
	/* Select appropriate pool */
	if (size < 4096)
		pool = ctx->small_pool;
	else if (size < 1024 * 1024)
		pool = ctx->medium_pool;
	else
		pool = ctx->large_pool;
	
	region = kzalloc(sizeof(*region), GFP_KERNEL);
	if (!region)
		return -ENOMEM;
	
	region_id = (u64)agent_id << 32 | atomic64_inc_return(&region_counter);
	region->region_id = region_id;
	region->owner_agent_id = agent_id;
	region->type = type;
	region->prot = prot;
	region->size = ALIGN(size, PAGE_SIZE);
	
	/* Allocate from pool */
	virt_addr = (void *)gen_pool_alloc(pool, region->size);
	if (!virt_addr) {
		ret = -ENOMEM;
		goto free_region;
	}
	
	region->virt_start = virt_addr;
	phys_addr = virt_to_phys(virt_addr);
	region->phys_start = phys_addr;
	
	/* Set up page tables with protection */
	ret = cule_mem_setup_protection(region, ctx->pgd);
	if (ret)
		goto free_pool;
	
	/* Add to agent's region tree */
	write_lock(&ctx->regions_lock);
	cule_mem_region_insert(&ctx->regions, region);
	write_unlock(&ctx->regions_lock);
	
	/* Add to global regions */
	mutex_lock(&cule_mem_mutex);
	cule_mem_region_insert(&cule_mem_regions, region);
	mutex_unlock(&cule_mem_mutex);
	
	/* Update statistics */
	ctx->current_usage += region->size;
	ctx->total_allocated += region->size;
	if (ctx->current_usage > ctx->peak_usage)
		ctx->peak_usage = ctx->current_usage;
	
	*region_id_out = region_id;
	
	pr_debug("Agent %u: Allocated region %llx (%zu bytes, type=%d, prot=%x)\n",
		agent_id, region_id, size, type, prot);
	return 0;

free_pool:
	gen_pool_free(pool, (unsigned long)virt_addr, region->size);
free_region:
	kfree(region);
	return ret;
}
EXPORT_SYMBOL_GPL(cule_mem_alloc);

/**
 * @brief Set up MMU protection for memory region
 */
static int cule_mem_setup_protection(struct cule_mem_region *region, pgd_t *pgd)
{
	pgprot_t prot;
	unsigned long addr = (unsigned long)region->virt_start;
	unsigned long end = addr + region->size;
	
	/* Build page protection flags */
	prot = PAGE_NONE;
	
	if (region->prot & CULE_PROT_READ)
		prot = PAGE_KERNEL_RO;
	
	if (region->prot & CULE_PROT_WRITE)
		prot = PAGE_KERNEL;
	
	if (region->prot & CULE_PROT_EXEC)
		prot = pgprot_val(prot) | PTE_PXN;
	else
		prot = pgprot_val(prot) & ~PTE_PXN;
	
	if (region->prot & CULE_PROT_NOCACHE)
		prot = pgprot_noncached(prot);
	
	/* Map pages with protection */
	while (addr < end) {
		pte_t *pte;
		pmd_t *pmd;
		pud_t *pud;
		pgd_t *pgd_entry;
		
		pgd_entry = pgd_offset_pgd(pgd, addr);
		
		if (pgd_none(*pgd_entry)) {
			pud_t *new = pud_alloc_one(&init_mm, addr);
			if (!new)
				return -ENOMEM;
			pgd_populate(&init_mm, pgd_entry, new);
		}
		
		pud = pud_offset(pgd_entry, addr);
		if (pud_none(*pud)) {
			pmd_t *new = pmd_alloc_one(&init_mm, addr);
			if (!new)
				return -ENOMEM;
			pud_populate(&init_mm, pud, new);
		}
		
		pmd = pmd_offset(pud, addr);
		if (pmd_none(*pmd)) {
			pte_t *new = pte_alloc_one_kernel(&init_mm);
			if (!new)
				return -ENOMEM;
			pmd_populate_kernel(&init_mm, pmd, new);
		}
		
		pte = pte_offset_kernel(pmd, addr);
		set_pte(pte, pfn_pte(__pa(addr) >> PAGE_SHIFT, prot));
		
		addr += PAGE_SIZE;
	}
	
	/* Flush TLB for this region */
	flush_tlb_all();
	
	return 0;
}

/**
 * @brief Free memory region
 */
int cule_mem_free(u32 agent_id, u64 region_id)
{
	struct cule_agent_mem_ctx *ctx;
	struct cule_mem_region *region;
	struct rb_node *node;
	bool found = false;
	
	if (agent_id >= CULE_MAX_AGENTS)
		return -EINVAL;
	
	ctx = agent_contexts[agent_id];
	if (!ctx)
		return -ENOENT;
	
	write_lock(&ctx->regions_lock);
	
	/* Find and remove from agent tree */
	for (node = rb_first(&ctx->regions); node; node = rb_next(node)) {
		region = rb_entry(node, struct cule_mem_region, node);
		if (region->region_id == region_id) {
			rb_erase(&region->node, &ctx->regions);
			found = true;
			break;
		}
	}
	
	write_unlock(&ctx->regions_lock);
	
	if (!found)
		return -ENOENT;
	
	/* Remove from global tree */
	mutex_lock(&cule_mem_mutex);
	rb_erase(&region->node, &cule_mem_regions);
	mutex_unlock(&cule_mem_mutex);
	
	/* Free page tables */
	cule_mem_free_pagetables(region->pgd, region->virt_start, region->size);
	
	/* Free memory */
	gen_pool_free(ctx->medium_pool, (unsigned long)region->virt_start, 
		      region->size);
	
	ctx->current_usage -= region->size;
	
	kfree(region->shared_agents);
	kvfree(region->access_bitmap);
	kfree(region);
	
	pr_debug("Agent %u: Freed region %llx\n", agent_id, region_id);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_mem_free);

/**
 * @brief Grant access to shared memory region
 */
int cule_mem_share(u32 owner_id, u64 region_id, u32 *agent_ids, u32 count)
{
	struct cule_agent_mem_ctx *ctx;
	struct cule_mem_region *region;
	struct rb_node *node;
	bool found = false;
	
	if (owner_id >= CULE_MAX_AGENTS || !agent_ids || count == 0)
		return -EINVAL;
	
	ctx = agent_contexts[owner_id];
	if (!ctx)
		return -ENOENT;
	
	read_lock(&ctx->regions_lock);
	
	for (node = rb_first(&ctx->regions); node; node = rb_next(node)) {
		region = rb_entry(node, struct cule_mem_region, node);
		if (region->region_id == region_id) {
			found = true;
			break;
		}
	}
	
	read_unlock(&ctx->regions_lock);
	
	if (!found)
		return -ENOENT;
	
	/* Allocate shared agent list */
	region->shared_agents = kmalloc_array(count, sizeof(u32), GFP_KERNEL);
	if (!region->shared_agents)
		return -ENOMEM;
	
	memcpy(region->shared_agents, agent_ids, count * sizeof(u32));
	region->shared_count = count;
	region->type = CULE_MEM_AGENT_SHARED;
	
	/* Update page tables for each agent */
	for (u32 i = 0; i < count; i++) {
		struct cule_agent_mem_ctx *target_ctx = agent_contexts[agent_ids[i]];
		if (target_ctx) {
			/* Map region into target's address space */
			cule_mem_map_to_agent(region, target_ctx);
		}
	}
	
	pr_info("Agent %u shared region %llx with %u agents\n", 
		owner_id, region_id, count);
	return 0;
}
EXPORT_SYMBOL_GPL(cule_mem_share);

/**
 * @brief Verify if agent can access memory address
 */
bool cule_mem_check_access(u32 agent_id, void *addr, size_t size, 
			   enum cule_mem_prot required_prot)
{
	struct cule_agent_mem_ctx *ctx;
	struct cule_mem_region *region;
	phys_addr_t phys = virt_to_phys(addr);
	
	if (agent_id >= CULE_MAX_AGENTS)
		return false;
	
	ctx = agent_contexts[agent_id];
	if (!ctx)
		return false;
	
	/* Check kernel access first */
	if (agent_id == CULE_AGENT_KERNEL)
		return true;
	
	/* Find region containing address */
	read_lock(&ctx->regions_lock);
	region = cule_mem_region_find(&ctx->regions, phys);
	read_unlock(&ctx->regions_lock);
	
	if (!region)
		return false;
	
	/* Check protection */
	if ((region->prot & required_prot) != required_prot)
		return false;
	
	/* Check if within bounds */
	if (phys + size > region->phys_start + region->size)
		return false;
	
	region->access_count++;
	return true;
}
EXPORT_SYMBOL_GPL(cule_mem_check_access);

/**
 * @brief Handle memory access fault
 */
int cule_mem_fault_handler(u32 agent_id, unsigned long addr, 
			   unsigned int esr)
{
	struct cule_agent_mem_ctx *ctx;
	struct cule_mem_region *region;
	
	ctx = agent_contexts[agent_id];
	if (!ctx)
		return -ENOENT;
	
	ctx->fault_count++;
	
	/* Try to find region */
	region = cule_mem_region_find(&cule_mem_regions, virt_to_phys((void *)addr));
	if (!region) {
		pr_err("Agent %u: Access violation at %lx - no region found\n",
			agent_id, addr);
		return -EFAULT;
	}
	
	pr_warn("Agent %u: Memory fault at %lx (region %llx, owner %u)\n",
		agent_id, addr, region->region_id, region->owner_agent_id);
	
	region->fault_count++;
	
	/* If this is the owner, might be a permission issue */
	if (region->owner_agent_id == agent_id) {
		/* Grant access if legitimate */
		return 0;
	}
	
	return -EACCES;
}
EXPORT_SYMBOL_GPL(cule_mem_fault_handler);

/**
 * @brief Get memory statistics for agent
 */
int cule_mem_get_stats(u32 agent_id, struct cule_mem_stats *stats)
{
	struct cule_agent_mem_ctx *ctx;
	struct cule_mem_region *region;
	struct rb_node *node;
	
	if (agent_id >= CULE_MAX_AGENTS || !stats)
		return -EINVAL;
	
	ctx = agent_contexts[agent_id];
	if (!ctx)
		return -ENOENT;
	
	memset(stats, 0, sizeof(*stats));
	
	stats->total_allocated = ctx->total_allocated;
	stats->current_usage = ctx->current_usage;
	stats->peak_usage = ctx->peak_usage;
	stats->fault_count = ctx->fault_count;
	
	/* Count regions */
	read_lock(&ctx->regions_lock);
	for (node = rb_first(&ctx->regions); node; node = rb_next(node)) {
		region = rb_entry(node, struct cule_mem_region, node);
		stats->region_count++;
		stats->total_region_size += region->size;
	}
	read_unlock(&ctx->regions_lock);
	
	return 0;
}
EXPORT_SYMBOL_GPL(cule_mem_get_stats);

/**
 * @brief Module initialization
 */
static int __init cule_mem_init(void)
{
	pr_info("Initializing Cule Memory Protection v%s\n", MODULE_VERSION);
	
	/* Check hardware capabilities */
	max_phys_addr = (1ULL << PHYS_MASK_SHIFT);
	
	/* Detect SMMU */
	struct device_node *smmu_node = of_find_compatible_node(NULL, NULL, 
							"arm,smmu-v3");
	has_smmu = (smmu_node != NULL);
	of_node_put(smmu_node);
	
	/* Detect stage 2 translation support */
	has_stage2 = system_supports_stage2();
	
	pr_info("Memory protection: SMMU=%s, Stage2=%s, max_phys=%llx\n",
		has_smmu ? "yes" : "no",
		has_stage2 ? "yes" : "no",
		max_phys_addr);
	
	/* Initialize agent contexts */
	memset(agent_contexts, 0, sizeof(agent_contexts));
	
	pr_info("Cule Memory Protection initialized\n");
	return 0;
}

/**
 * @brief Module cleanup
 */
static void __exit cule_mem_exit(void)
{
	pr_info("Exiting Cule Memory Protection\n");
	
	/* Cleanup all agent contexts */
	for (int i = 0; i < CULE_MAX_AGENTS; i++) {
		if (agent_contexts[i])
			cule_agent_mem_destroy(i);
	}
	
	pr_info("Cule Memory Protection exited\n");
}

module_init(cule_mem_init);
module_exit(cule_mem_exit);
