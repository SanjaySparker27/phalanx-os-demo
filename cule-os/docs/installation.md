# Installation Guide

This guide covers all methods for installing Cule OS on your hardware or virtual environment.

## Table of Contents

1. [Pre-Installation](#pre-installation)
2. [Installation Methods](#installation-methods)
3. [Post-Installation](#post-installation)
4. [Troubleshooting](#troubleshooting)

---

## Pre-Installation

### Hardware Requirements

#### Minimum Requirements
| Component | Specification |
|-----------|---------------|
| Processor | x86_64 (Intel/AMD) or ARM64, 2 cores |
| Memory | 4 GB RAM |
| Storage | 20 GB available space |
| Network | Internet connection for updates |

#### Recommended Specifications
| Component | Specification |
|-----------|---------------|
| Processor | 4+ cores, 64-bit |
| Memory | 8+ GB RAM |
| Storage | 50+ GB NVMe SSD |
| Network | Gigabit Ethernet |

### Supported Platforms

| Platform | Architecture | Status |
|----------|--------------|--------|
| Bare Metal | x86_64 | ✅ Supported |
| Bare Metal | ARM64 | ✅ Supported |
| VMware | x86_64 | ✅ Supported |
| VirtualBox | x86_64 | ✅ Supported |
| KVM/QEMU | x86_64, ARM64 | ✅ Supported |
| AWS EC2 | x86_64, ARM64 | ✅ Supported |
| Azure | x86_64 | ✅ Supported |
| GCP | x86_64, ARM64 | ✅ Supported |
| Docker | x86_64, ARM64 | ✅ Supported |

### Pre-Installation Checklist

- [ ] Verify hardware meets minimum requirements
- [ ] Backup existing data
- [ ] Obtain installation media (USB/DVD/ISO)
- [ ] Download latest Cule OS image
- [ ] Verify image checksum

---

## Installation Methods

### Method 1: Automated Installation (Recommended)

The automated installer is the quickest way to get Cule OS running.

```bash
# Download and run the installer
curl -fsSL https://get.cule-os.io | bash

# Or with custom options
curl -fsSL https://get.cule-os.io | bash -s -- --edition server --disk /dev/sda
```

#### Available Options

| Option | Description | Default |
|--------|-------------|---------|
| `--edition` | Installation edition: `minimal`, `server`, `desktop` | `server` |
| `--disk` | Target disk device | Auto-detected |
| `--hostname` | System hostname | `cule-os` |
| `--timezone` | System timezone | `UTC` |
| `--username` | Primary user account | `admin` |

### Method 2: Manual Installation

For custom configurations or specific requirements.

#### Step 1: Boot from Installation Media

1. Create bootable USB:
```bash
# Linux/macOS
dd if=cule-os-2.0.iso of=/dev/sdX bs=4M status=progress

# Windows (using Rufus or Etcher)
# Select ISO and USB drive, write in DD mode
```

2. Boot from USB and select "Install Cule OS"

#### Step 2: Disk Partitioning

```bash
# Launch partitioning tool
cule-install partition

# Example: Manual partitioning with cfdisk
cfdisk /dev/sda

# Recommended layout:
# /dev/sda1 - 512MB - EFI System Partition (ESP)
# /dev/sda2 - 2GB   - Boot partition
# /dev/sda3 - Remainder - LVM/Root partition
```

#### Step 3: Base Installation

```bash
# Mount target filesystems
mount /dev/sda3 /mnt
mkdir -p /mnt/boot /mnt/boot/efi
mount /dev/sda2 /mnt/boot
mount /dev/sda1 /mnt/boot/efi

# Install base system
cule-install base /mnt

# Generate fstab
cule-install fstab /mnt

# Install bootloader
cule-install bootloader /mnt
```

#### Step 4: System Configuration

```bash
# Chroot into new system
cule-install chroot /mnt

# Set timezone
cule-config timezone America/New_York

# Set hostname
cule-config hostname my-server

# Create user account
cule-user create admin --sudo

# Exit and unmount
exit
umount -R /mnt
```

### Method 3: Cloud Installation

#### AWS EC2

```bash
# Using AWS CLI
aws ec2 run-instances \
  --image-id ami-0culeos20xxxxxxxxx \
  --instance-type t3.medium \
  --key-name my-key \
  --security-group-ids sg-xxxxxxxxx

# Using Terraform
resource "aws_instance" "cule_os" {
  ami           = "ami-0culeos20xxxxxxxxx"
  instance_type = "t3.medium"
  
  tags = {
    Name = "cule-os-server"
  }
}
```

#### Azure

```bash
# Using Azure CLI
az vm create \
  --resource-group myResourceGroup \
  --name cule-os-vm \
  --image cule-os-2-0 \
  --size Standard_B2s \
  --admin-username admin
```

#### GCP

```bash
# Using gcloud CLI
gcloud compute instances create cule-os-vm \
  --image-project cule-os-public \
  --image-family cule-os-2 \
  --machine-type e2-medium
```

### Method 4: Docker Installation

```bash
# Pull Cule OS container
docker pull cule-os/runtime:2.0

# Run container
docker run -it --privileged \
  --name cule-os-dev \
  cule-os/runtime:2.0

# Or use docker-compose
version: '3.8'
services:
  cule-os:
    image: cule-os/runtime:2.0
    privileged: true
    volumes:
      - ./data:/data
    ports:
      - "8080:8080"
```

---

## Post-Installation

### Initial Setup

```bash
# Login with created credentials
ssh admin@<hostname>

# Update system packages
cule update && cule upgrade

# Verify installation
cule doctor
```

### Network Configuration

```bash
# Configure network interface
cule network setup --interface eth0 --dhcp

# Or static IP
cule network setup \
  --interface eth0 \
  --ip 192.168.1.100/24 \
  --gateway 192.168.1.1 \
  --dns 8.8.8.8,8.8.4.4
```

### Security Hardening

```bash
# Enable firewall
cule firewall enable

# Configure SSH hardening
cule security ssh-harden

# Set up automatic updates
cule update auto --enable --schedule "daily"
```

### Service Management

```bash
# Start essential services
cule service start cule-api
cule service start cule-monitor

# Enable services at boot
cule service enable cule-api
cule service enable cule-monitor

# Check service status
cule service status
```

---

## Troubleshooting

### Installation Issues

#### Boot Problems

| Symptom | Cause | Solution |
|---------|-------|----------|
| "No bootable device" | Incorrect boot order | Check BIOS/UEFI boot priority |
| Kernel panic | Incompatible hardware | Check hardware compatibility list |
| Black screen | Graphics driver issue | Try nomodeset kernel parameter |

#### Disk Issues

```bash
# Check disk health
smartctl -a /dev/sda

# Verify partition table
cule-install verify-disk /dev/sda

# Repair filesystem
cule-install repair /dev/sda3
```

#### Network Issues

```bash
# Test connectivity
ping -c 4 8.8.8.8

# Check network configuration
cule network status

# Reset network stack
cule network reset
```

### Recovery Mode

If the system fails to boot:

```bash
# Boot from installation media
# Select "Recovery Mode"

# Mount existing installation
cule-recovery mount /dev/sda3

# Chroot into system
cule-recovery chroot

# Fix issues, then exit
exit
cule-recovery reboot
```

### Getting Help

If issues persist:

1. Check [Troubleshooting FAQ](https://docs.cule-os.io/troubleshooting)
2. Review installation logs: `/var/log/cule-install.log`
3. Contact support: support@cule-os.io
4. File an issue: [GitHub Issues](https://github.com/cule-os/issues)

---

## Next Steps

After successful installation:

1. [Configure your system](./configuration.md)
2. [Understand the architecture](./architecture.md)
3. [Explore the API](./api.md)
4. Join the community: [Discord](https://discord.gg/cule-os)

---

*For additional installation scenarios, see the [Advanced Installation Guide](https://docs.cule-os.io/advanced-install).*