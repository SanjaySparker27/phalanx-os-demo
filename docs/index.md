# Project Documentation

Complete documentation for the ZEX ATHENA H743 PRO VTOL system with Cule OS companion computer.

## Overview

This project combines:
- **ZEX ATHENA H743 PRO** flight controller (ArduPilot-based)
- **Cule OS** molecular autonomous operating system on NVIDIA Jetson
- **HEXA X + Pusher** configuration (6 motors + 1 pusher motor)
- **Custom firmware** with C++ mode switching logic

## Documentation Structure

### Installation Guides
- [Jetson Setup Guide](installation/jetson-setup.md) - Complete Jetson installation and configuration

### Hardware Documentation
- [Wiring Diagrams](hardware/wiring-diagrams.md) - Complete hardware wiring and connections

### Configuration Guides
- [ArduPilot Configuration](configuration/ardupilot-config.md) - Flight controller parameters and setup

### Procedures
- [Calibration Procedures](calibration/calibration-procedures.md) - ESC, IMU, compass, camera calibration
- [First Flight Checklist](operations/first-flight-checklist.md) - Pre-flight checks and procedures

### Troubleshooting
- [Troubleshooting Guide](troubleshooting/troubleshooting-guide.md) - Common issues and solutions

### API Reference
- [API Documentation](api/api-documentation.md) - MAVLink, Cule OS, and integration APIs

## Quick Start

1. **Hardware Setup**: Follow [Wiring Diagrams](hardware/wiring-diagrams.md)
2. **Jetson Installation**: Follow [Jetson Setup Guide](installation/jetson-setup.md)
3. **Flight Controller Config**: Follow [ArduPilot Configuration](configuration/ardupilot-config.md)
4. **Calibration**: Follow [Calibration Procedures](calibration/calibration-procedures.md)
5. **First Flight**: Follow [First Flight Checklist](operations/first-flight-checklist.md)

## System Architecture

```
                    +---------------------+
                    |   Power System      |
                    |   (6S LiPo)         |
                    +----------+----------+
                               |
           +-------------------+-------------------+
           |                   |                   |
           v                   v                   v
    +--------------+   +---------------+   +--------------+
    |   Motors     |   | Flight Ctrl   |   |   Jetson     |
    |   (7x)       |   | ATHENA H743   |   |  (Cule OS)   |
    +--------------+   +-------+-------+   +------+-------+
                               |                   |
                               v                   v
                       +---------------+   +---------------+
                       |   Sensors     |   |    Agents     |
                       | GPS/Compass   |   | Perception    |
                       | LIDAR         |   | Planning      |
                       | Camera        |   | Control       |
                       +---------------+   +---------------+
```

## Key Features

- **Pure C++ Mode Switching**: No Lua scripts required
- **HEXA X Frame**: 6-motor configuration with redundancy
- **Pusher Motor**: 7th motor controlled by RC8 slider
- **Smart Mode Switch**: RC8 slider switches flight modes (0-16% = Loiter, >16% = Stabilize)
- **AI-Powered Autonomy**: Cule OS with perception and planning agents
- **Real-time Control**: 1kHz control loop with safety guarantees

## Support

For issues and questions:
- Check the [Troubleshooting Guide](troubleshooting/troubleshooting-guide.md)
- Review [ArduPilot Documentation](https://ardupilot.org/copter/)
- Visit [Cule OS Documentation](https://cule-os.io/docs)

## License

- ArduPilot: GPLv3
- Cule OS: Proprietary
- Custom Firmware: As-is for educational purposes
