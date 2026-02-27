# Cule OS Documentation

Welcome to the Cule OS documentation. Cule OS is an innovative operating system for autonomous unmanned vehicles — aerial, ground, and maritime — built with multi-agent intelligence.

## Quick Start

New to Cule OS? Start here:

- **[Getting Started](GETTING_STARTED.md)** — Get up and running in 30 minutes
- **[Installation Guide](installation.md)** — Detailed installation instructions
- **[Quick Start (UAV)](uav/quickstart.md)** — UAV-specific quick start

## Documentation Sections

### 🚁 UAV / Drone (Axon Edition)

Complete documentation for autonomous aerial vehicles.

| Guide | Description |
|-------|-------------|
| [UAV Overview](uav/README.md) | Introduction to Axon UAV Edition |
| [Quick Start](uav/quickstart.md) | 30-minute setup guide |
| [Flight Modes](uav/flight-modes.md) | All flight modes explained |
| [Mission Planning](uav/missions.md) | Create autonomous missions |
| [Safety Procedures](uav/safety.md) | Pre-flight checklists & emergencies |
| [Troubleshooting](uav/troubleshooting.md) | Fix common issues |

### 🔧 Hardware Setup

Configure your hardware for Cule OS.

| Component | Documentation |
|-----------|--------------|
| [Flight Controllers](hardware/flight-controllers.md) | Pixhawk, Cube Orange setup |
| [Companion Computers](hardware/companion-computers.md) | Jetson & Raspberry Pi |
| [Sensors](uav/sensors/) | GPS, IMU, compass, barometer |
| [Wiring](hardware/wiring.md) | Connection diagrams |

### 💻 API & Development

Integrate with Cule OS programmatically.

| Resource | Description |
|----------|-------------|
| [API Reference](api.md) | Python API, ROS2 topics, CLI |
| [Architecture](architecture.md) | System architecture overview |
| [Configuration](configuration.md) | System configuration guide |
| [Examples](../examples/) | Working code examples |

## System Requirements

### Minimum Hardware

- **Companion Computer**: NVIDIA Jetson Nano or Raspberry Pi 4
- **Flight Controller**: Pixhawk 4/5/6 or Cube Orange
- **GPS**: u-blox NEO-M8N or better
- **Storage**: 32GB microSD card (Class 10)
- **Power**: 5V 4A power supply

### Supported Vehicles

- **Multirotor**: Quadcopter, Hexacopter, Octocopter
- **Fixed-Wing**: Standard planes, flying wings, VTOL
- **Ground**: Rovers, crawlers
- **Maritime**: Surface vessels, boats

## Installation

### 1. Download

```bash
# UAV Edition - NVIDIA Jetson
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-jetson-v1.0.img.xz

# UAV Edition - Raspberry Pi
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-rpi-v1.0.img.xz
```

### 2. Flash SD Card

```bash
# Linux/Mac
xzcat cule-os-axon-jetson-v1.0.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Windows: Use BalenaEtcher
```

### 3. Configure

```bash
# SSH into Cule OS
ssh cule@cule-os.local
# Password: culeos

# Run configuration wizard
sudo cule-config
```

## Quick Commands

```bash
# Check system status
cule-status

# Arm the vehicle
cule-arm

# Takeoff (UAV)
cule-takeoff 10

# Change flight mode
cule-mode loiter

# Return to launch
cule-rtl

# Land
cule-land
```

## Community & Support

- **Website**: https://sanjaysparker27.github.io/cule-os
- **GitHub**: https://github.com/SanjaySparker27/cule-os
- **Discussions**: https://github.com/SanjaySparker27/cule-os/discussions
- **Issues**: https://github.com/SanjaySparker27/cule-os/issues

## Contributing

We welcome contributions! See our [Contributing Guide](CONTRIBUTING.md) for details.

## License

Cule OS is proprietary software. See [LICENSE](../LICENSE) for details.

---

**Ready to start?** → [Getting Started Guide](GETTING_STARTED.md)