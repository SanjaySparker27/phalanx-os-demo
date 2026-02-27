# Cule OS Documentation

Welcome to the Cule OS documentation portal. Cule OS is an innovative operating system for autonomous unmanned vehicles — aerial, ground, and maritime — built with multi-agent intelligence.

## Quick Links

| Edition | Description | Get Started |
|---------|-------------|-------------|
| **Axon UAV** | Autonomous aerial platform for drones | [UAV Guide](./uav/) |
| **Terra UGV** | Ground platform for rovers | [UGV Guide](./ugv/) |
| **Aqua USV** | Maritime platform for boats | [USV Guide](./usv/) |
| **Core** | Developer edition for researchers | [Core Guide](./core/) |

## Documentation Sections

### 🚁 UAV / Drone Documentation
- [Getting Started Guide](./uav/quickstart.md) - 30-minute quick start
- [UAV Overview](./uav/README.md) - Axon edition details
- [Flight Modes](./uav/flight-modes.md) - All flight modes explained
- [Mission Planning](./uav/missions.md) - Create autonomous missions
- [Safety Procedures](./uav/safety.md) - Critical safety information
- [Troubleshooting](./uav/troubleshooting.md) - Common issues

### 🚙 UGV / Rover Documentation
- [UGV Quick Start](./ugv/quickstart.md) - Get driving in 30 minutes
- [UGV Overview](./ugv/README.md) - Terra edition details
- [Navigation](./ugv/navigation.md) - Path planning and SLAM
- [Teleoperation](./ugv/teleop.md) - Remote control modes

### 🚢 USV / Boat Documentation
- [USV Quick Start](./usv/quickstart.md) - Get sailing in 30 minutes
- [USV Overview](./usv/README.md) - Aqua edition details
- [Maritime Operations](./usv/operations.md) - Water-specific features

### 🔧 Hardware Setup
- [Hardware Overview](./hardware/README.md) - Supported hardware
- [Pixhawk Setup](./hardware/pixhawk.md) - Flight controller configuration
- [Sensors](./hardware/sensors.md) - GPS, IMU, camera, LiDAR
- [Telemetry](./hardware/telemetry.md) - Radio setup
- [Cameras](./hardware/camera.md) - Camera configuration

### 💻 Developer Documentation
- [Core Architecture](./core/architecture.md) - System design
- [Kernel](./core/kernel.md) - RT scheduler and IPC
- [Agent System](./core/agents.md) - Multi-agent framework
- [API Reference](./api/README.md) - Python and ROS2 APIs
- [Contributing](./CONTRIBUTING.md) - How to contribute

### 🎓 Examples & Tutorials
- [Examples](../examples/README.md) - Working code examples
- [Training Guide](./TRAINING.md) - ML model training
- [Simulation](./simulation/README.md) - Gazebo/Ignition setup

## Installation

### Download Cule OS

Choose your platform:

```bash
# UAV Edition (NVIDIA Jetson)
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-jetson-v1.0.img.xz

# UAV Edition (Raspberry Pi)
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-rpi-v1.0.img.xz

# UGV Edition
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-terra-v1.0.img.xz

# USV Edition
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-aqua-v1.0.img.xz
```

### Flash to SD Card

```bash
# Linux/Mac
xzcat cule-os-axon-jetson-v1.0.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Windows (using BalenaEtcher)
# 1. Download and install BalenaEtcher
# 2. Select image file
# 3. Select SD card
# 4. Click Flash
```

### First Boot

1. Insert SD card into Jetson/RPi
2. Connect flight controller via USB
3. Power on
4. Wait 2-3 minutes
5. Connect to WiFi network `Cule-OS-Setup`
6. SSH: `ssh cule@192.168.4.1` (password: `culeos`)

## Quick Start Guides

### UAV (Drone) - 30 Minutes

```bash
# Configure vehicle
sudo cule-config --type uav --frame quad

# Calibrate sensors
sudo cule-calibrate compass
sudo cule-calibrate accel
sudo cule-calibrate radio

# Test in simulation
roslaunch cule_simulation uav.launch

# First flight
cule-arm --mode stabilize
```

[Full UAV Guide →](./uav/quickstart.md)

### UGV (Rover) - 30 Minutes

```bash
# Configure vehicle
sudo cule-config --type ugv --drive skid

# Calibrate sensors
sudo cule-calibrate compass
sudo cule-calibrate accel

# Test teleoperation
cule-teleop --mode keyboard
```

[Full UGV Guide →](./ugv/quickstart.md)

### USV (Boat) - 30 Minutes

```bash
# Configure vehicle
sudo cule-config --type usv --hull mono

# Calibrate sensors
sudo cule-calibrate compass
sudo cule-calibrate accel

# Test in water
cule-arm --mode manual
```

[Full USV Guide →](./usv/quickstart.md)

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                     │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │ Mission  │ │   Swarm  │ │  Vision  │ │  Cloud   │  │
│  │ Planning │ │ Coordination│ │ Processing│ │ Connect │  │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │
├─────────────────────────────────────────────────────────┤
│                   Mally Agent System                     │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │Perception│ │ Planning │ │  Control │ │Communication│ │
│  │  Agent   │ │  Agent   │ │  Agent   │ │   Agent    │  │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │
├─────────────────────────────────────────────────────────┤
│                    Cule Core Kernel                      │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │  RT      │ │   IPC    │ │  Memory  │ │  Device  │  │
│  │Scheduler │ │  System  │ │  Manager │ │  Drivers │  │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │
├─────────────────────────────────────────────────────────┤
│                   Hardware Abstraction                   │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │
│  │  Pixhawk │ │  Camera  │ │  Sensors │ │ Telemetry│  │
│  │  Driver  │ │  Driver  │ │  Driver  │ │  Driver  │  │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │
└─────────────────────────────────────────────────────────┘
```

## API Quick Reference

### Python API

```python
from cule import UAV

# Connect to vehicle
drone = UAV.connect('/dev/ttyUSB0')

# Arm and takeoff
drone.arm()
drone.takeoff(altitude=10)

# Navigate
drone.goto(lat=37.7749, lon=-122.4194, alt=20)

# Return and land
drone.rtl()
drone.land()
```

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cule/telemetry` | `nav_msgs/Odometry` | Position and velocity |
| `/cule/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/cule/camera/image` | `sensor_msgs/Image` | Camera feed |
| `/cule/lidar/scan` | `sensor_msgs/LaserScan` | LiDAR data |

### CLI Commands

```bash
# Vehicle control
cule-arm
cule-disarm
cule-mode <mode>
cule-takeoff <altitude>
cule-land

# Mission management
cule-mission create
cule-mission upload
cule-mission start

# System status
cule-status
cule-diagnose
cule-logs
```

## Community & Support

### Getting Help

- **Documentation:** https://sanjaysparker27.github.io/cule-os/docs
- **Community Forum:** https://discussion.cule-os.io
- **Matrix Chat:** #cule-os:matrix.org
- **GitHub Issues:** https://github.com/SanjaySparker27/cule-os/issues

### Contributing

We welcome contributions! See [CONTRIBUTING.md](./CONTRIBUTING.md) for guidelines.

### License

Cule OS is proprietary software. See [LICENSE](../LICENSE) for details.

---

**Ready to get started?** → [UAV Quick Start](./uav/quickstart.md)
