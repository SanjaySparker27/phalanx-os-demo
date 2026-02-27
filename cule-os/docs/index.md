# Cule OS Documentation

Welcome to the **Cule OS** documentation — a modern, lightweight operating system designed for performance, security, and developer productivity.

## Quick Start

```bash
# Install Cule OS
curl -fsSL https://get.cule-os.io | bash

# Verify installation
cule --version
```

## Documentation Structure

| Document | Description | Audience |
|----------|-------------|----------|
| [Installation](./installation.md) | Step-by-step installation guide | New Users |
| [Configuration](./configuration.md) | System and service configuration | Administrators |
| [Architecture](./architecture.md) | System design and internals | Developers, Architects |
| [API Reference](./api.md) | Complete API documentation | Developers |

## Core Features

### 🚀 Performance
- **Microkernel Architecture**: Minimal kernel footprint with services running in user space
- **Zero-Copy I/O**: Efficient data transfer mechanisms
- **Adaptive Scheduling**: AI-driven process scheduling for optimal resource utilization

### 🔒 Security
- **Capability-Based Access Control**: Fine-grained permission system
- **Secure Boot**: Cryptographic verification of system components
- **Sandboxing**: Application-level isolation by default

### 🛠 Developer Experience
- **Unified CLI**: Single command-line interface for all operations
- **Hot Reload**: Configuration changes without restart
- **Built-in Observability**: Tracing, metrics, and logging out of the box

## System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 2 cores | 4+ cores |
| RAM | 4 GB | 8+ GB |
| Storage | 20 GB SSD | 50+ GB NVMe |
| Network | 100 Mbps | 1 Gbps |

## Version Compatibility

| Cule OS Version | Kernel | CLI Version | Support Status |
|-----------------|--------|-------------|----------------|
| 2.x (current) | 6.5+ | 2.x | ✅ Active |
| 1.x | 5.15+ | 1.x | ⚠️ Maintenance |
| 0.x | 5.4+ | 0.x | ❌ End of Life |

## Getting Help

- **Documentation**: [docs.cule-os.io](https://docs.cule-os.io)
- **GitHub Issues**: [github.com/cule-os/issues](https://github.com/cule-os/issues)
- **Community Discord**: [discord.gg/cule-os](https://discord.gg/cule-os)
- **Email Support**: support@cule-os.io

## Contributing

We welcome contributions! Please see our [Contributing Guide](https://github.com/cule-os/contributing) for details on:

- Code of Conduct
- Development workflow
- Pull request process
- Commit message conventions

## License

Cule OS is licensed under the **MIT License**. See [LICENSE](./LICENSE) for full details.

---

*Last updated: February 2026*