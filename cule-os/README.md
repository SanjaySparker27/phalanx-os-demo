# Cule OS

An innovative operating system for autonomous unmanned vehicles — aerial, ground, and maritime.

## Overview

Cule OS is a complete autonomous vehicle operating system built with multi-agent intelligence. It integrates real-time control, perception, planning, and execution for unmanned vehicles.

## Editions

| Edition | Vehicle | Status | Documentation |
|---------|---------|--------|---------------|
| **Axon** | UAV (Drones) | ✅ Available | [UAV Docs](docs/uav/) |
| **Terra** | UGV (Rovers) | 🚧 Coming Soon | [UGV Docs](docs/ugv/) |
| **Aqua** | USV (Boats) | 🚧 Coming Soon | [USV Docs](docs/usv/) |
| **Core** | Development | 🚧 Coming Soon | [Core Docs](docs/core/) |

## Quick Start

### Download

```bash
# UAV Edition - NVIDIA Jetson
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-jetson-v1.0.img.xz

# UAV Edition - Raspberry Pi
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-rpi-v1.0.img.xz
```

### Flash to SD Card

```bash
# Linux/Mac
xzcat cule-os-axon-jetson-v1.0.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Windows: Use BalenaEtcher
```

### Configure

```bash
# SSH into Cule OS
ssh cule@cule-os.local
# Password: culeos

# Run configuration
sudo cule-config

# Select vehicle type and options
```

## Documentation

- [Getting Started](docs/GETTING_STARTED.md) - 30-minute setup guide
- [UAV Documentation](docs/uav/) - Drone/quadcopter specific
- [Hardware Setup](docs/hardware/) - Flight controller, sensors, cameras
- [API Reference](docs/api/) - Python and ROS2 APIs
- [Troubleshooting](docs/uav/troubleshooting.md) - Common issues

## Repository Structure

```
cule-os/
├── docs/               # Documentation
├── examples/           # Example code
├── kernel/             # RT kernel modules
├── mally/              # Multi-agent system
├── tests/              # Test suite
├── training/           # ML training tools
└── ui/                 # Dashboard UI
```

## Features

- **Real-Time Control**: 1kHz control loop with deterministic latency
- **Multi-Agent System**: Distributed agent architecture (Mally)
- **Perception**: Computer vision with YOLO and custom models
- **Planning**: Path planning, obstacle avoidance, mission execution
- **Swarm Coordination**: Multi-vehicle coordination
- **Simulation**: Gazebo/Ignition integration
- **Training Pipeline**: End-to-end ML training

## Hardware Support

### Companion Computers
- NVIDIA Jetson (Orin Nano, Xavier NX, AGX)
- Raspberry Pi 4/5
- x86 platforms

### Flight Controllers
- Pixhawk 4/5/6
- Cube Orange
- Custom H743-based boards

### Sensors
- GPS (u-blox NEO-M8N/M9N)
- IMU (ICM-42688, BMI088)
- Cameras (CSI, USB)
- LiDAR (Ouster, Velodyne)

## Community

- **Website**: https://sanjaysparker27.github.io/cule-os
- **Documentation**: https://sanjaysparker27.github.io/cule-os/docs.html
- **GitHub**: https://github.com/SanjaySparker27/cule-os
- **Forum**: https://github.com/sanjaysparker27/cule-os/discussions

## License

Proprietary - See [LICENSE](LICENSE)

---

Built with multi-agent intelligence for the future of autonomy.
