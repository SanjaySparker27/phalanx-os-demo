#!/bin/bash
#
# Cule OS Kernel Build Script
# Builds the kernel for NVIDIA Jetson platforms
#

set -e

KERNEL_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${KERNEL_DIR}/build"
CROSS_COMPILE="${CROSS_COMPILE:-aarch64-none-elf-}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check for cross-compiler
check_toolchain() {
    if ! command -v ${CROSS_COMPILE}gcc &> /dev/null; then
        log_error "Cross-compiler ${CROSS_COMPILE}gcc not found!"
        log_info "Install with: sudo apt-get install gcc-aarch64-linux-gnu"
        exit 1
    fi
    log_info "Using cross-compiler: $(${CROSS_COMPILE}gcc --version | head -1)"
}

# Clean build directory
clean() {
    log_info "Cleaning build directory..."
    rm -rf "${BUILD_DIR}"
}

# Build kernel
build() {
    log_info "Building Cule OS Kernel..."
    
    mkdir -p "${BUILD_DIR}"
    
    cd "${KERNEL_DIR}"
    make clean 2>/dev/null || true
    make -j$(nproc) 2>&1 | tee "${BUILD_DIR}/build.log"
    
    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        log_info "Build successful!"
        log_info "Kernel image: ${BUILD_DIR}/cule-kernel.img"
        log_info "Device trees: ${BUILD_DIR}/*.dtb"
    else
        log_error "Build failed! Check ${BUILD_DIR}/build.log"
        exit 1
    fi
}

# Build device tree only
build_dtb() {
    log_info "Building device tree blobs..."
    
    mkdir -p "${BUILD_DIR}"
    
    for dts in "${KERNEL_DIR}"/dtc/*.dts; do
        if [ -f "$dts" ]; then
            dtb_name=$(basename "$dts" .dts).dtb
            log_info "Compiling $dtb_name..."
            dtc -I dts -O dtb -o "${BUILD_DIR}/${dtb_name}" "$dts" 2>/dev/null || \
                log_warn "Failed to compile $dtb_name (dtc not installed?)"
        fi
    done
}

# Run in QEMU
run_qemu() {
    log_info "Running kernel in QEMU..."
    
    if ! command -v qemu-system-aarch64 &> /dev/null; then
        log_error "QEMU not found! Install with: sudo apt-get install qemu-system-arm"
        exit 1
    fi
    
    if [ ! -f "${BUILD_DIR}/cule-kernel.img" ]; then
        log_error "Kernel image not found. Build first with: $0 build"
        exit 1
    fi
    
    qemu-system-aarch64 \
        -M virt,gic-version=3 \
        -cpu cortex-a78 \
        -smp 8 \
        -m 2048 \
        -kernel "${BUILD_DIR}/cule-kernel.img" \
        -dtb "${BUILD_DIR}/jetson-agx-orin.dtb" 2>/dev/null || true \
        -nographic \
        -serial stdio \
        -no-reboot
}

# Debug with GDB
debug() {
    log_info "Starting QEMU in debug mode..."
    log_info "Connect with: aarch64-none-elf-gdb -ex 'target remote localhost:1234'"
    
    if [ ! -f "${BUILD_DIR}/cule-kernel.elf" ]; then
        log_error "Kernel ELF not found. Build first with: $0 build"
        exit 1
    fi
    
    qemu-system-aarch64 \
        -M virt,gic-version=3 \
        -cpu cortex-a78 \
        -smp 8 \
        -m 2048 \
        -kernel "${BUILD_DIR}/cule-kernel.img" \
        -nographic \
        -serial stdio \
        -s -S &
    
    QEMU_PID=$!
    log_info "QEMU started with PID $QEMU_PID"
    log_info "GDB server listening on localhost:1234"
    
    wait $QEMU_PID
}

# Generate configuration
config() {
    log_info "Generating kernel configuration..."
    
    cat > "${KERNEL_DIR}/.config" << EOF
#
# Cule OS Kernel Configuration
#

CONFIG_ARCH_ARM64=y
CONFIG_JETSON_AGX_ORIN=y
CONFIG_JETSON_AGX_XAVIER=y

#
# Scheduler
#
CONFIG_SCHED_TT=y
CONFIG_SCHED_NIS=y
CONFIG_MAX_TASKS=64
CONFIG_TICK_RATE_HZ=1000

#
# Memory
#
CONFIG_PAGE_SIZE_4KB=y
CONFIG_SLAB_ALLOCATOR=y

#
# Drivers
#
CONFIG_DRIVER_CSI_CAMERA=y
CONFIG_DRIVER_USB_CAMERA=y
CONFIG_DRIVER_GPS=y
CONFIG_DRIVER_IRIDIUM=y
CONFIG_DRIVER_MAVLINK=y

#
# IPC
#
CONFIG_SYSVIPC=y
CONFIG_MESSAGE_QUEUES=y
CONFIG_SHARED_MEMORY=y
CONFIG_SEMAPHORES=y
EOF
    
    log_info "Configuration written to ${KERNEL_DIR}/.config"
}

# Package kernel for deployment
package() {
    log_info "Packaging kernel for deployment..."
    
    local pkg_dir="${BUILD_DIR}/cule-os-$(date +%Y%m%d)"
    mkdir -p "${pkg_dir}"
    
    cp "${BUILD_DIR}/cule-kernel.img" "${pkg_dir}/"
    cp "${BUILD_DIR}/cule-kernel.elf" "${pkg_dir}/" 2>/dev/null || true
    cp "${BUILD_DIR}"/*.dtb "${pkg_dir}/" 2>/dev/null || true
    cp "${KERNEL_DIR}/.config" "${pkg_dir}/config" 2>/dev/null || true
    
    cat > "${pkg_dir}/README.txt" << EOF
Cule OS Kernel Package
======================

Kernel Image: cule-kernel.img
Device Trees: jetson-*.dtb

Installation:
1. Copy cule-kernel.img to /boot/
2. Copy appropriate .dtb to /boot/dtb/
3. Update bootloader configuration

For Jetson AGX Orin:
  - Use jetson-agx-orin.dtb

For Jetson AGX Xavier:
  - Use jetson-agx-xavier.dtb

EOF
    
    tar czf "${pkg_dir}.tar.gz" -C "${BUILD_DIR}" "$(basename ${pkg_dir})"
    log_info "Package created: ${pkg_dir}.tar.gz"
}

# Show usage
usage() {
    cat << EOF
Cule OS Kernel Build Script

Usage: $0 [command]

Commands:
    build       Build the kernel (default)
    clean       Clean build artifacts
    dtb         Build device tree blobs only
    qemu        Run kernel in QEMU emulator
    debug       Start QEMU with GDB server
    config      Generate default configuration
    package     Create deployment package
    help        Show this help message

Environment Variables:
    CROSS_COMPILE   Cross-compiler prefix (default: aarch64-none-elf-)

Examples:
    $0 build                    # Build kernel
    $0 clean && $0 build        # Clean rebuild
    $0 qemu                     # Test in emulator
    CROSS_COMPILE=aarch64-linux-gnu- $0 build

EOF
}

# Main
main() {
    case "${1:-build}" in
        build)
            check_toolchain
            build
            ;;
        clean)
            clean
            ;;
        dtb)
            build_dtb
            ;;
        qemu)
            run_qemu
            ;;
        debug)
            debug
            ;;
        config)
            config
            ;;
        package)
            package
            ;;
        help|--help|-h)
            usage
            ;;
        *)
            log_error "Unknown command: $1"
            usage
            exit 1
            ;;
    esac
}

main "$@"
