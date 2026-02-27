# System Architecture

This document provides a comprehensive overview of Cule OS architecture, including design principles, component interactions, and internal mechanisms.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Kernel Architecture](#kernel-architecture)
3. [User Space Architecture](#user-space-architecture)
4. [Service Architecture](#service-architecture)
5. [Security Architecture](#security-architecture)
6. [Storage Architecture](#storage-architecture)
7. [Network Architecture](#network-architecture)

---

## Architecture Overview

### Design Principles

Cule OS is built on five core principles:

| Principle | Description | Implementation |
|-----------|-------------|----------------|
| **Minimalism** | Minimal kernel, maximum in user space | Microkernel design |
| **Security** | Security by default, never optional | Capability-based access control |
| **Performance** | Zero-copy operations, efficient scheduling | Adaptive scheduler, ring buffers |
| **Reliability** | Fail-fast components, graceful degradation | Service isolation, health monitoring |
| **Observability** | Everything is measurable | Built-in tracing and metrics |

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                        │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │   Apps   │  │   CLI    │  │  System  │  │  Custom  │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
├───────┼─────────────┼─────────────┼─────────────┼─────────┤
│       │         Service Layer (User Space)              │
│  ┌────┴─────┐  ┌────┴─────┐  ┌────┴─────┐  ┌────┴─────┐   │
│  │ API      │  │ Monitor  │  │ Config   │  │ Network  │   │
│  │ Gateway  │  │ Service  │  │ Manager  │  │ Manager  │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘   │
│       └─────────────┴─────────────┴─────────────┘         │
├─────────────────────────┬─────────────────────────────────┤
│      Message Bus        │      Resource Manager           │
│    (ZeroMQ/Nanomsg)     │    (Memory, CPU, I/O)           │
├─────────────────────────┴─────────────────────────────────┤
│              System Call Interface                        │
├───────────────────────────────────────────────────────────┤
│                    Microkernel                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │ Scheduler│  │  Memory  │  │   IPC    │  │  Drivers │  │
│  │          │  │ Manager  │  │          │  │ (minimal)│  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
└───────────────────────────────────────────────────────────┘
```

---

## Kernel Architecture

### Microkernel Design

Cule OS uses a true microkernel architecture where only essential services run in kernel space:

| Component | Location | Responsibility |
|-----------|----------|----------------|
| Scheduler | Kernel | Process/thread scheduling |
| Memory Manager | Kernel | Virtual memory, paging |
| IPC | Kernel | Inter-process communication |
| Device Drivers | Kernel | Essential hardware (boot only) |
| File Systems | User Space | VFS operations |
| Network Stack | User Space | TCP/IP, protocols |
| Device Drivers | User Space | Most hardware drivers |

### Kernel Components

#### Scheduler

```
┌─────────────────────────────────────────────────┐
│              Cule Scheduler                     │
├─────────────────────────────────────────────────┤
│  ┌─────────┐  ┌─────────┐  ┌─────────┐         │
│  │  Real   │  │ Normal  │  │  Idle   │         │
│  │  Time   │  │  Queue  │  │  Queue  │         │
│  │ Queue   │  │         │  │         │         │
│  └────┬────┘  └────┬────┘  └────┬────┘         │
│       └─────────────┼─────────────┘             │
│                     ▼                           │
│            ┌─────────────────┐                  │
│            │  Adaptive Core  │                  │
│            │  (ML-based)     │                  │
│            └────────┬────────┘                  │
│                     ▼                           │
│            ┌─────────────────┐                  │
│            │   CPU Cores     │                  │
│            └─────────────────┘                  │
└─────────────────────────────────────────────────┘
```

The adaptive scheduler uses machine learning to predict workload patterns and optimize scheduling decisions:

| Feature | Description |
|---------|-------------|
| Workload Prediction | Predicts CPU/memory usage 100ms ahead |
| Latency Optimization | Prioritizes latency-sensitive tasks |
| Energy Efficiency | Balances performance and power on mobile |
| NUMA Awareness | Optimizes for multi-socket systems |

#### Memory Manager

```
┌─────────────────────────────────────────────────────────┐
│              Memory Management Architecture             │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────┐   │
│  │              Virtual Memory Space                │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐        │   │
│  │  │   Text   │ │   Data   │ │   Heap   │        │   │
│  │  └──────────┘ └──────────┘ └──────────┘        │   │
│  │  ┌────────────────────────────────────────┐     │   │
│  │  │           Memory-mapped I/O             │     │   │
│  │  └────────────────────────────────────────┘     │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐        │   │
│  │  │  Stack   │ │  Shared  │ │  Mapped  │        │   │
│  │  └──────────┘ └──────────┘ └──────────┘        │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Page Table Manager                  │   │
│  │  - 5-level page tables (x86_64)                 │   │
│  │  - Huge pages support (2MB, 1GB)                │   │
│  │  - Kernel same-page merging                     │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Physical Memory                     │   │
│  │  ┌─────────────────────────────────────────┐     │   │
│  │  │        Buddy Allocator (4KB-4MB)        │     │   │
│  │  └─────────────────────────────────────────┘     │   │
│  │  ┌─────────────────────────────────────────┐     │   │
│  │  │        SLAB Allocator (Objects)         │     │   │
│  │  └─────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

#### Inter-Process Communication

Cule OS implements multiple IPC mechanisms:

| Mechanism | Use Case | Latency |
|-----------|----------|---------|
| Message Passing | Service communication | < 1μs |
| Shared Memory | High-bandwidth data | < 100ns |
| Signals | Async notifications | ~1μs |
| Capabilities | Security delegation | < 500ns |

```c
// Message passing example
message_t msg = {
    .type = MSG_REQUEST,
    .sender = getpid(),
    .payload = data,
    .size = len
};

cule_ipc_send(endpoint, &msg, CULE_IPC_NONBLOCK);
```

---

## User Space Architecture

### Service Layer

All non-essential operating system services run in user space as isolated processes:

```
┌─────────────────────────────────────────────────────────┐
│                  Service Layer                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌───────────────┐  ┌───────────────┐                  │
│  │  API Gateway  │  │    Monitor    │                  │
│  │               │  │   Service     │                  │
│  │ - REST/gRPC   │  │               │  ┌──────────┐   │
│  │ - Auth        │  │ - Metrics     │  │  Config  │   │
│  │ - Rate Limit  │  │ - Alerting    │  │ Manager  │   │
│  └───────┬───────┘  └───────┬───────┘  └────┬─────┘   │
│          │                  │               │          │
│          └──────────────────┼───────────────┘          │
│                             │                          │
│  ┌───────────────┐  ┌───────┴───────┐  ┌──────────┐   │
│  │   Network     │  │   Resource    │  │  Storage │   │
│  │   Manager     │  │   Manager     │  │ Manager  │   │
│  │               │  │               │  │          │   │
│  │ - TCP/IP      │  │ - CPU         │  │ - VFS    │   │
│  │ - Firewall    │  │ - Memory      │  │ - Cache  │   │
│  │ - DNS         │  │ - I/O         │  │ - Sync   │   │
│  └───────────────┘  └───────────────┘  └──────────┘   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Service Lifecycle

```
    ┌─────────────┐
    │   Created   │
    └──────┬──────┘
           │ spawn()
           ▼
    ┌─────────────┐
    │ Initializing│
    └──────┬──────┘
           │ init_complete()
           ▼
    ┌─────────────┐
    │   Ready     │◄─────────┐
    └──────┬──────┘          │
           │ health_check()  │
           ▼                 │
    ┌─────────────┐          │
    │  Degraded   │──────────┘
    └──────┬──────┘ recover()
           │ max_retries
           ▼
    ┌─────────────┐
    │   Failed    │
    └──────┬──────┘
           │ restart_policy
           ▼
    ┌─────────────┐
    │ Terminated  │
    └─────────────┘
```

---

## Service Architecture

### API Gateway Service

```
┌─────────────────────────────────────────────────────────┐
│                    API Gateway                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │              HTTP/2 Server                       │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐      │   │
│  │  │  REST    │  │  gRPC    │  │ GraphQL  │      │   │
│  │  │ Handler  │  │ Handler  │  │ Handler  │      │   │
│  │  └────┬─────┘  └────┬─────┘  └────┬─────┘      │   │
│  │       └─────────────┼─────────────┘            │   │
│  └─────────────────────┼──────────────────────────┘   │
│                        ▼                               │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Middleware Stack                    │   │
│  │  - Authentication (JWT, mTLS)                   │   │
│  │  - Authorization (RBAC)                         │   │
│  │  - Rate Limiting (Token bucket)                 │   │
│  │  - Request Validation                           │   │
│  │  - CORS handling                                │   │
│  └─────────────────────┬──────────────────────────┘   │
│                        ▼                               │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Router                              │   │
│  │  - Path-based routing                           │   │
│  │  - Version negotiation                          │   │
│  │  - Load balancing                               │   │
│  └─────────────────────┬──────────────────────────┘   │
│                        ▼                               │
│  ┌─────────────────────────────────────────────────┐   │
│  │           Backend Services                       │   │
│  └─────────────────────────────────────────────────┘   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Monitoring Service

```
┌─────────────────────────────────────────────────────────┐
│                 Monitoring Service                      │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   Metrics    │  │    Logs      │  │   Traces     │  │
│  │  Collector   │  │  Aggregator  │  │  Collector   │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
│         │                 │                  │          │
│         └─────────────────┼──────────────────┘          │
│                           ▼                             │
│  ┌────────────────────────────────────────────────────┐ │
│  │              Time-Series Database                   │ │
│  │              (Built-in / Prometheus)                │ │
│  └─────────────────────────┬──────────────────────────┘ │
│                            ▼                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │  Alerting    │  │ Dashboards   │  │   API        │  │
│  │  Engine      │  │ (Grafana)    │  │  (Query)     │  │
│  └──────────────┘  └──────────────┘  └──────────────┘  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Security Architecture

### Capability-Based Access Control

Cule OS uses capabilities instead of traditional ACLs:

```
┌─────────────────────────────────────────────────────────┐
│              Capability System                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Process A              Kernel              Resource    │
│  ┌─────────┐          ┌─────────┐          ┌────────┐  │
│  │ Capability          │ Capability          │        │  │
│  │ Token:              │ Registry            │ File X │  │
│  │  {                  │                     │        │  │
│  │    resource: "file" │  ┌───────────────┐ │        │  │
│  │    id: "file-x"     │  │  File X Cap   │ │        │  │
│  │    rights: [read]   │  │  - Owner: A   │ │        │  │
│  │    issuer: kernel   │  │  - Rights: RW │ │        │  │
│  │  }                  │  └───────────────┘ │        │  │
│  └────┬────┘          └─────────┘          └────────┘  │
│       │                      ▲                         │
│       │ 1. Present capability│                         │
│       └──────────────────────┘                         │
│                      2. Validate                       │
│                      3. Grant access                   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Security Layers

| Layer | Mechanism | Purpose |
|-------|-----------|---------|
| Hardware | TPM, Secure Boot | Root of trust |
| Kernel | Capabilities, Sandboxing | Process isolation |
| Services | mTLS, RBAC | Service-to-service security |
| API | JWT, OAuth2 | User authentication |
| Data | Encryption at rest | Data protection |

---

## Storage Architecture

### Virtual File System

```
┌─────────────────────────────────────────────────────────┐
│              Virtual File System (VFS)                  │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │              VFS Layer                           │   │
│  │  - Unified interface for all filesystems         │   │
│  │  - Path resolution                               │   │
│  │  - File descriptor management                    │   │
│  └────────────────────┬────────────────────────────┘   │
│                       │                                 │
│       ┌───────────────┼───────────────┐                │
│       ▼               ▼               ▼                │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐            │
│  │  ext4   │    │   ZFS   │    │   NFS   │            │
│  │ Driver  │    │ Driver  │    │ Driver  │            │
│  └────┬────┘    └────┬────┘    └────┬────┘            │
│       │               │               │                 │
│       ▼               ▼               ▼                 │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐            │
│  │  Block  │    │  Pool   │    │ Network │            │
│  │ Device  │    │ Manager │    │ Client  │            │
│  └─────────┘    └─────────┘    └─────────┘            │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### I/O Stack

```
┌─────────────────────────────────────────────────────────┐
│                    I/O Stack                            │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Application                                            │
│       │                                                 │
│       ▼                                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              System Call Interface               │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              VFS Layer                           │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Page Cache                          │   │
│  │              (Write-through / Write-back)        │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Block Layer                         │   │
│  │              - I/O Scheduling                    │   │
│  │              - Request merging                   │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Device Driver                       │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Physical Device                     │   │
│  └─────────────────────────────────────────────────┘   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Network Architecture

### Network Stack

```
┌─────────────────────────────────────────────────────────┐
│                 Network Stack                           │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Application Layer                                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │  HTTP/2, gRPC, WebSocket, Custom Protocols       │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  Transport Layer                                        │
│  ┌─────────────────────────────────────────────────┐   │
│  │  TCP (with BBR congestion control)               │   │
│  │  UDP (with QUIC support)                         │   │
│  │  SCTP                                            │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  Network Layer                                          │
│  ┌─────────────────────────────────────────────────┐   │
│  │  IPv4 / IPv6                                     │   │
│  │  - Routing table management                      │   │
│  │  - Neighbor discovery                            │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  Data Link Layer                                        │
│  ┌─────────────────────────────────────────────────┐   │
│  │  Ethernet, WiFi, Virtual interfaces              │   │
│  │  - ARP / NDP                                     │   │
│  │  - VLAN support                                  │   │
│  └────────────────────┬────────────────────────────┘   │
│                       ▼                                 │
│  Physical Layer                                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │  Network Interface Card (NIC) Drivers            │   │
│  └─────────────────────────────────────────────────┘   │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Performance Characteristics

| Metric | Value | Notes |
|--------|-------|-------|
| Context Switch | ~200ns | Microkernel optimized |
| System Call | ~50ns | Fast path |
| IPC (message) | <1μs | Same node |
| IPC (shared memory) | <100ns | Zero-copy path |
| Boot Time | <500ms | Minimal init |
| Memory Overhead | ~50MB | Base system |

---

*For implementation details, see the [Developer Documentation](https://docs.cule-os.io/dev).*