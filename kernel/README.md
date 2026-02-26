# Cule OS Kernel Module Documentation

## Overview

Cule OS is a real-time kernel extension for Linux (PREEMPT_RT) designed for
autonomous systems running on ARM64 platforms like the NVIDIA Jetson AGX Orin.

## Components

### 1. Real-Time Scheduler (`cule_rt_scheduler`)

Implements a preemptible real-time scheduler with support for:

- **Time-Triggered (TT) Scheduling**: Deterministic task release based on schedule tables
- **EDF Scheduling**: Dynamic priority based on deadlines
- **Fixed Priority Scheduling**: Static priorities for traditional RT tasks
- **Non-Interruptible Sections (NIS)**: Critical sections with bounded duration

Key features:
- 1kHz tick frequency for sub-millisecond resolution
- NIS watchdog with 50ms timeout
- Per-CPU scheduling queues
- Integration with Linux CFS for non-RT tasks

### 2. Neural Inference Server (`cule_neural_server`)

Provides preemptible neural network inference:

- **Layer-wise Checkpointing**: Save/restore state at layer boundaries
- **Multi-Accelerator Support**: CUDA, TensorRT, DLA, PVA
- **Deterministic Timing**: WCET estimates per layer
- **Preemption Latency**: <100us target

Checkpointing strategy:
- Every 4th layer is marked preemptible
- DMA transfers for GPU/NPU state
- CPU cache coherence management
- Fast restore for resumed jobs

### 3. Memory Protection (`cule_memory`)

Hardware-backed memory isolation between agents:

- **Per-Agent Address Spaces**: Separate page tables per agent
- **SMMU Integration**: DMA isolation for accelerators
- **Shared Memory**: Controlled sharing between agents
- **Access Control**: Read/write/execute/DMA permissions

Memory pools:
- Small pool: < 4KB allocations
- Medium pool: 4KB - 1MB allocations  
- Large pool: > 1MB allocations

### 4. Zero-Copy IPC (`cule_ipc`)

High-performance inter-agent communication:

- **Ring Buffers**: Lock-free SPSC queues per endpoint
- **Zero-Copy**: Messages > 1KB use shared memory pool
- **Cache Optimization**: Cache-line aligned structures
- **Priority Support**: Real-time message priority

Message types:
- DATA: Regular payload
- CONTROL: Commands and configuration
- SIGNAL: Lightweight notification
- RPC: Request/response pattern
- DMA_DESCRIPTOR: Large transfer descriptors

## Building

### Prerequisites

- Linux kernel source with PREEMPT_RT patches
- ARM64 cross-compiler (aarch64-linux-gnu-gcc)
- Device tree compiler (dtc)

### Build Commands

```bash
# Build all modules
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

# Build and install device tree overlay
make dtbo
sudo make install_dtbo

# Install modules
sudo make modules_install
sudo depmod -a
```

### Kernel Configuration Requirements

Required kernel config options:
```
CONFIG_PREEMPT_RT=y
CONFIG_HIGH_RES_TIMERS=y
CONFIG_CGROUP_SCHED=y
CONFIG_RT_GROUP_SCHED=y
CONFIG_ARM_SMMU=y
CONFIG_DMA_SHARED_BUFFER=y
```

## Installation

### 1. Install Device Tree Overlay

```bash
sudo cp cule-jetson-agx-orin.dtbo /boot/kernel-overlays/
sudo reboot
```

### 2. Load Kernel Modules

```bash
# In order of dependencies
sudo insmod cule_memory.ko
sudo insmod cule_ipc.ko
sudo insmod cule_neural_server.ko
sudo insmod cule_rt_scheduler.ko
```

Or use the Makefile:
```bash
sudo make load
```

### 3. Verify Installation

```bash
# Check modules loaded
lsmod | grep cule

# Check kernel messages
dmesg | grep cule

# Verify scheduler is running
cat /proc/cule/sched_stats
```

## API Usage

### Real-Time Task Registration

```c
#include <cule/kernel.h>

struct cule_rt_params params = {
    .priority = 80,
    .period_ns = 10000000,      // 10ms
    .deadline_ns = 10000000,    // 10ms
    .wcet_ns = 5000000,         // 5ms
    .mode = CULE_SCHED_TT,
    .cpu_mask = NULL,           // Any CPU
};

u32 task_id;
int ret = cule_task_register(&params, &task_id);
```

### Neural Model Registration

```c
struct cule_layer_info layers[] = {
    {
        .type = CULE_LAYER_CONV2D,
        .input_dims = {1, 3, 224, 224},
        .output_dims = {1, 64, 112, 112},
        .exec_time_ns = 500000,  // 500us
    },
    // ... more layers
};

u32 model_id;
cule_model_register("yolov5", ARRAY_SIZE(layers), layers,
                    CULE_ACCEL_GPU_TRT, &model_id);
```

### Memory Allocation

```c
u64 region_id;
cule_mem_alloc(agent_id, 1024*1024, CULE_MEM_AGENT_PRIVATE,
               CULE_PROT_READ | CULE_PROT_WRITE, &region_id);
```

### IPC Communication

```c
// Create endpoints
cule_endpoint_t ep_a, ep_b;
cule_ipc_endpoint_register(agent_a, &ep_a);
cule_ipc_endpoint_register(agent_b, &ep_b);

// Create channel
cule_channel_t channel;
cule_ipc_channel_create(agent_a, agent_b, &channel);

// Send message
struct cule_ipc_msg msg = {
    .type = CULE_MSG_DATA,
    .length = data_len,
    .data = data,
    .priority = CULE_PRIO_HIGH,
};
cule_ipc_send(ep_a, ep_b, &msg);
```

## Hardware Resources

### Memory Reservations (Device Tree)

- **RT Heap**: 512MB @ 0xB8000000
- **Neural Buffers**: 1GB @ 0xD8000000
- **IPC Memory**: 256MB @ 0x180000000
- **Agent Memory**: 512MB @ 0x190000000

### CPU Isolation

- CPUs 0-3: General purpose / Linux tasks
- CPUs 4-7: Isolated for real-time tasks

### Accelerators

- **DLA0/DLA1**: Deep Learning Accelerators
- **PVA0**: Programmable Vision Accelerator
- **GPU**: CUDA/TensorRT

## Safety Features

### Watchdog

- System watchdog: 5 second timeout
- Per-agent watchdogs: 100ms timeout for safety-critical agents
- Scheduler watchdog: 50ms timeout

### Thermal Management

- Warning at 85C: Throttle non-critical tasks
- Critical at 95C: Trigger safety shutdown

### GPIO Safety Outputs

- Safety kill switch output
- Hardware interlock input

## Debugging

### Kernel Parameters

```bash
# Enable debug output
cule.debug=1

# Set scheduler tick frequency
cule.tick_hz=2000

# Disable NIS enforcement (testing only)
cule.nis_strict=0
```

### Debug FS Entries

```bash
# Scheduler statistics
cat /sys/kernel/debug/cule/sched_stats

# Neural server stats
cat /sys/kernel/debug/cule/neural_stats

# Memory regions
cat /sys/kernel/debug/cule/memory_regions

# IPC channels
cat /sys/kernel/debug/cule/ipc_channels
```

### Tracing

```bash
# Enable scheduler trace events
echo 1 > /sys/kernel/tracing/events/cule/enable

# View trace
cat /sys/kernel/tracing/trace
```

## Performance Tuning

### Scheduler

- Increase tick_hz for better resolution (more overhead)
- Use TT scheduling for periodic tasks
- Use EDF for aperiodic tasks with deadlines
- Keep NIS sections < 50us

### Neural Server

- Batch size affects throughput vs latency tradeoff
- More checkpoint buffers = more preemptions possible
- Use DLA for energy efficiency
- Use GPU for flexible models

### IPC

- Messages < 1KB use inline ring buffer
- Messages > 1KB use zero-copy shared memory
- Tune ring size based on message rate

## Troubleshooting

### Module Load Failures

```bash
# Check kernel version compatibility
uname -r

# Check for missing symbols
modinfo cule_rt_scheduler.ko

# Check dmesg for errors
dmesg | tail -50
```

### Deadline Misses

- Check WCET estimates are accurate
- Verify task priorities
- Check for priority inversion
- Monitor NIS durations

### Neural Preemption Issues

- Verify checkpoint buffer allocation
- Check layer preemptible flags
- Monitor DMA completion times
- Verify cache coherence

## License

GPL-2.0

## Support

For issues and contributions, see the Cule OS project repository.
