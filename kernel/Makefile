# Cule OS Real-Time Kernel Makefile
# Target: NVIDIA Jetson AGX Orin (ARM64, PREEMPT_RT)

# Module names
obj-m += cule_rt_scheduler.o
obj-m += cule_neural_server.o
obj-m += cule_memory.o
obj-m += cule_ipc.o

# Source files
cule_rt_scheduler-objs := rt_scheduler.o
cule_neural_server-objs := neural_server.o
cule_memory-objs := memory.o
cule_ipc-objs := ipc.o

# Kernel build configuration
KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build
PWD := $(shell pwd)

# Cross-compilation for Jetson AGX Orin
ARCH ?= arm64
CROSS_COMPILE ?= aarch64-linux-gnu-

# Compiler flags for C11 and RT patches
ccflags-y := -std=gnu11 -Wall -Wextra -Werror
ccflags-y += -DCONFIG_PREEMPT_RT
ccflags-y += -DCONFIG_CULE_RT_SCHEDULER
ccflags-y += -DCONFIG_CULE_NEURAL_SERVER
ccflags-y += -DCONFIG_CULE_MEMORY_PROTECTION
ccflags-y += -DCONFIG_CULE_IPC
ccflags-y += -O2 -ftree-vectorize

# Debug flags (enable for development)
# ccflags-y += -DDEBUG -g

# Include paths
ccflags-y += -I$(PWD)/include

# ARM64 specific optimizations
ccflags-y += -march=armv8.2-a+fp16+dotprod
cflags-y += -mtune=cortex-a78ae

# Kernel module build targets
.PHONY: all clean modules_install dtb dtbo

all: modules

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) clean
	@rm -f *.o *.ko *.mod.c *.mod.o *.order *.symvers
	@rm -rf .tmp_versions .*.cmd

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

# Device tree overlay build
dtbo: cule-jetson-agx-orin.dtbo

%.dtbo: ../dts/%.dts
	cpp -nostdinc -I$(KDIR)/include -I$(KDIR)/arch/$(ARCH)/boot/dts -x assembler-with-cpp $< > $<.tmp
dtc -I dts -O dtb -o $@ $<.tmp
	@rm $<.tmp

# Install device tree overlay
install_dtbo: dtbo
	@echo "Installing device tree overlay..."
	@cp cule-jetson-agx-orin.dtbo /boot/kernel-overlays/
	@echo "Device tree overlay installed. Reboot to apply."

# Load modules (for testing on target)
load:
	@insmod cule_memory.ko || true
	@insmod cule_ipc.ko || true
	@insmod cule_neural_server.ko || true
	@insmod cule_rt_scheduler.ko || true
	@echo "Cule kernel modules loaded"

# Unload modules
unload:
	@rmmod cule_rt_scheduler 2>/dev/null || true
	@rmmod cule_neural_server 2>/dev/null || true
	@rmmod cule_ipc 2>/dev/null || true
	@rmmod cule_memory 2>/dev/null || true
	@echo "Cule kernel modules unloaded"

# Check kernel configuration
check-config:
	@echo "Checking kernel configuration..."
	@zgrep -q CONFIG_PREEMPT_RT /boot/config-$(KVER) && echo "PREEMPT_RT: OK" || echo "PREEMPT_RT: MISSING"
	@zgrep -q CONFIG_HIGH_RES_TIMERS /boot/config-$(KVER) && echo "HIGH_RES_TIMERS: OK" || echo "HIGH_RES_TIMERS: MISSING"
	@zgrep -q CONFIG_CGROUP_SCHED /boot/config-$(KVER) && echo "CGROUP_SCHED: OK" || echo "CGROUP_SCHED: MISSING"
	@zgrep -q CONFIG_RT_GROUP_SCHED /boot/config-$(KVER) && echo "RT_GROUP_SCHED: OK" || echo "RT_GROUP_SCHED: MISSING"

# Print build info
info:
	@echo "Cule OS Kernel Build Configuration"
	@echo "==================================="
	@echo "Kernel version: $(KVER)"
	@echo "Architecture: $(ARCH)"
	@echo "Cross compiler: $(CROSS_COMPILE)"
	@echo "Kernel dir: $(KDIR)"
	@echo "Build dir: $(PWD)"
	@echo ""
	@echo "Modules:"
	@echo "  - cule_rt_scheduler (Real-time scheduler with TT/NIS)"
	@echo "  - cule_neural_server (Neural inference with checkpointing)"
	@echo "  - cule_memory (Memory protection between agents)"
	@echo "  - cule_ipc (Zero-copy IPC)"
