# Configuration Guide

This comprehensive guide covers all aspects of configuring Cule OS, from basic system settings to advanced service configurations.

## Table of Contents

1. [Configuration Overview](#configuration-overview)
2. [System Configuration](#system-configuration)
3. [Network Configuration](#network-configuration)
4. [Service Configuration](#service-configuration)
5. [Security Configuration](#security-configuration)
6. [Storage Configuration](#storage-configuration)
7. [Advanced Configuration](#advanced-configuration)

---

## Configuration Overview

### Configuration Files Location

| Configuration Type | Path | Description |
|-------------------|------|-------------|
| System | `/etc/cule/config.yaml` | Main system configuration |
| Network | `/etc/cule/network/` | Network interface configs |
| Services | `/etc/cule/services/` | Service-specific configs |
| Security | `/etc/cule/security/` | Security policies and keys |
| User | `~/.cule/config.yaml` | User-specific settings |

### Configuration Format

Cule OS uses YAML for all configuration files:

```yaml
# Example system configuration
version: "2.0"
system:
  hostname: "my-server"
  timezone: "America/New_York"
  locale: "en_US.UTF-8"
  
logging:
  level: "info"
  destination: "file"
  path: "/var/log/cule/"
  rotation:
    max_size: "100MB"
    max_files: 10
```

---

## System Configuration

### Hostname and Identity

```bash
# Set system hostname
cule config set system.hostname "production-server-01"

# View current hostname
cule config get system.hostname

# Set system description
cule config set system.description "Primary Application Server"
```

### Time and Locale

```bash
# Set timezone
cule config set system.timezone "Europe/London"

# List available timezones
cule timezone list

# Set locale
cule config set system.locale "en_GB.UTF-8"

# Configure NTP
cule config set ntp.enabled true
cule config set ntp.servers "pool.ntp.org,time.google.com"
```

### System Limits

```yaml
# /etc/cule/config.yaml
system:
  limits:
    max_processes: 65535
    max_open_files: 1048576
    max_memory: "32GB"
    
  kernel:
    parameters:
      vm.swappiness: 10
      vm.dirty_ratio: 15
      net.core.somaxconn: 65535
```

---

## Network Configuration

### Interface Management

```bash
# List network interfaces
cule network list

# Configure DHCP
cule network config eth0 \
  --type dhcp \
  --metric 100

# Configure static IP
cule network config eth0 \
  --type static \
  --ip 192.168.1.100/24 \
  --gateway 192.168.1.1 \
  --dns "8.8.8.8,8.8.4.4"

# Apply configuration
cule network apply
```

### Network Configuration File

```yaml
# /etc/cule/network/interfaces.yaml
interfaces:
  eth0:
    type: static
    address: 192.168.1.100/24
    gateway: 192.168.1.1
    dns:
      - 8.8.8.8
      - 8.8.4.4
    mtu: 1500
    
  eth1:
    type: dhcp
    metric: 200
    optional: true
```

### Firewall Configuration

```bash
# Enable firewall
cule firewall enable

# Default policies
cule firewall policy --input DROP --forward DROP --output ACCEPT

# Allow SSH
cule firewall allow --port 22 --protocol tcp

# Allow HTTP/HTTPS
cule firewall allow --port 80 --protocol tcp
cule firewall allow --port 443 --protocol tcp

# Create custom rules
cule firewall rule add \
  --name "app-traffic" \
  --source 10.0.0.0/24 \
  --port 8080 \
  --protocol tcp \
  --action ACCEPT
```

### Firewall Configuration File

```yaml
# /etc/cule/security/firewall.yaml
enabled: true
default_policy:
  input: DROP
  forward: DROP
  output: ACCEPT

rules:
  - name: "allow-ssh"
    port: 22
    protocol: tcp
    action: ACCEPT
    
  - name: "allow-http"
    port: 80
    protocol: tcp
    action: ACCEPT
    
  - name: "allow-https"
    port: 443
    protocol: tcp
    action: ACCEPT
    
  - name: "block-malicious"
    source: "192.168.100.0/24"
    action: DROP
```

---

## Service Configuration

### Service Management

```bash
# List all services
cule service list

# Start a service
cule service start <service-name>

# Stop a service
cule service stop <service-name>

# Enable at boot
cule service enable <service-name>

# Disable at boot
cule service disable <service-name>

# View service logs
cule service logs <service-name> --follow
```

### Core Services

#### API Service

```yaml
# /etc/cule/services/api.yaml
enabled: true
bind:
  address: "0.0.0.0"
  port: 8080
  
tls:
  enabled: true
  cert: "/etc/cule/certs/server.crt"
  key: "/etc/cule/certs/server.key"
  
rate_limiting:
  enabled: true
  requests_per_minute: 1000
  burst_size: 100
  
authentication:
  type: "jwt"
  secret_file: "/etc/cule/secrets/jwt.key"
  token_expiry: "24h"
```

#### Monitoring Service

```yaml
# /etc/cule/services/monitor.yaml
enabled: true
metrics:
  enabled: true
  interval: 15
  retention: "30d"
  
alerting:
  enabled: true
  endpoints:
    - type: "webhook"
      url: "https://alerts.example.com/webhook"
    - type: "email"
      smtp_server: "smtp.example.com"
      to: "ops@example.com"
      
thresholds:
  cpu:
    warning: 70
    critical: 90
  memory:
    warning: 80
    critical: 95
  disk:
    warning: 80
    critical: 90
```

### Custom Service Definition

```yaml
# /etc/cule/services/my-app.yaml
name: "my-application"
description: "Custom Application Service"
enabled: true

executable: "/opt/my-app/bin/server"
working_directory: "/opt/my-app"

environment:
  NODE_ENV: "production"
  PORT: "3000"
  
resources:
  cpu_limit: "2.0"
  memory_limit: "4G"
  
restart_policy:
  type: "always"
  max_attempts: 5
  delay: "10s"
  
logging:
  driver: "json-file"
  options:
    max-size: "100m"
    max-file: "5"
```

---

## Security Configuration

### User Management

```bash
# Create user
cule user create john.doe \
  --fullname "John Doe" \
  --groups "developers,deployers" \
  --ssh-key "ssh-ed25519 AAAAC3..."

# Modify user
cule user modify john.doe --add-group "admins"

# Delete user
cule user delete john.doe --remove-home

# List users
cule user list
```

### RBAC Configuration

```yaml
# /etc/cule/security/rbac.yaml
roles:
  admin:
    permissions:
      - "*"
      
  operator:
    permissions:
      - "service:*"
      - "config:read"
      - "logs:read"
      
  developer:
    permissions:
      - "service:read"
      - "config:read"
      - "logs:read"
      - "deploy:write"
      
  viewer:
    permissions:
      - "service:read"
      - "config:read"
      - "logs:read"

users:
  john.doe:
    roles:
      - "developer"
      
  admin:
    roles:
      - "admin"
```

### SSL/TLS Configuration

```bash
# Generate self-signed certificate
cule cert generate --hostname "my-server.local"

# Request Let's Encrypt certificate
cule cert request \
  --domain "example.com" \
  --email "admin@example.com" \
  --agree-tos

# Import existing certificate
cule cert import \
  --cert /path/to/cert.pem \
  --key /path/to/key.pem \
  --name "my-cert"

# Auto-renewal
cule cert auto-renew --enable
```

### Audit Logging

```yaml
# /etc/cule/security/audit.yaml
enabled: true

events:
  - authentication
  - authorization
  - configuration_changes
  - file_access
  - network_activity

destination:
  type: "file"
  path: "/var/log/cule/audit.log"
  rotation:
    max_size: "1GB"
    max_files: 30

alerting:
  suspicious_activity: true
  failed_logins:
    threshold: 5
    window: "5m"
```

---

## Storage Configuration

### Filesystem Management

```bash
# List volumes
cule storage list

# Create volume
cule storage create my-data \
  --size 100G \
  --type ext4

# Mount volume
cule storage mount my-data /data

# Configure auto-mount
cule storage automount my-data --enable
```

### Storage Configuration

```yaml
# /etc/cule/storage/volumes.yaml
volumes:
  system:
    device: "/dev/sda1"
    mount: "/"
    type: "ext4"
    
  data:
    device: "/dev/sdb1"
    mount: "/data"
    type: "xfs"
    options:
      - "noatime"
      - "nodiratime"
      
  logs:
    device: "/dev/sdc1"
    mount: "/var/log"
    type: "ext4"
    max_usage: "80%"
```

### Backup Configuration

```yaml
# /etc/cule/storage/backup.yaml
enabled: true
schedule: "0 2 * * *"  # Daily at 2 AM

sources:
  - "/etc/cule"
  - "/data"
  - "/home"
  
exclusions:
  - "*.tmp"
  - "*.log"
  - "/data/cache"
  
destination:
  type: "s3"
  bucket: "my-backups"
  region: "us-east-1"
  path: "cule-os/{{hostname}}"
  
retention:
  daily: 7
  weekly: 4
  monthly: 12
  
encryption:
  enabled: true
  key_file: "/etc/cule/secrets/backup.key"
```

---

## Advanced Configuration

### Kernel Parameters

```bash
# View current parameters
cule kernel params

# Set parameter (persistent)
cule kernel set vm.swappiness 10

# Set parameter (runtime only)
cule kernel set net.core.somaxconn 65535 --runtime

# Load custom module
cule kernel module load zfs
```

### Custom Kernel Configuration

```yaml
# /etc/cule/kernel/custom.yaml
parameters:
  vm.swappiness: 10
  vm.dirty_ratio: 15
  vm.dirty_background_ratio: 5
  net.core.somaxconn: 65535
  net.ipv4.tcp_fastopen: 3
  
modules:
  - name: "zfs"
    options:
      zfs_arc_max: "8589934592"
      
blacklist:
  - "floppy"
  - "pcspkr"
```

### Cluster Configuration

```yaml
# /etc/cule/cluster/config.yaml
enabled: true
node_id: "node-01"
cluster_name: "production"

discovery:
  type: "consul"
  endpoints:
    - "consul-01.internal:8500"
    - "consul-02.internal:8500"
    
consensus:
  type: "raft"
  peers:
    - "node-01"
    - "node-02"
    - "node-03"
    
replication:
  enabled: true
  mode: "async"
  lag_threshold: "1s"
```

### Environment-Specific Configurations

```bash
# Development
cule config env use development
cule config set logging.level debug
cule config set metrics.enabled false

# Staging
cule config env use staging
cule config set logging.level info

# Production
cule config env use production
cule config set logging.level warning
cule config set security.hardened true
```

---

## Configuration Validation

```bash
# Validate configuration syntax
cule config validate

# Test configuration
cule config test

# Check for security issues
cule security audit

# Preview changes
cule config diff
```

---

## Best Practices

1. **Version Control**: Keep `/etc/cule/` in version control
2. **Backups**: Regularly backup configuration files
3. **Documentation**: Comment complex configurations
4. **Testing**: Test changes in staging before production
5. **Monitoring**: Monitor configuration drift

---

*For more configuration options, see the [Configuration Reference](https://docs.cule-os.io/config-reference).*