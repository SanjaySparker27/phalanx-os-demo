# Cule OS - UAV Edition (Axon)

The leading autonomous aerial platform for multirotor, fixed-wing, and VTOL vehicles.

## Overview

Axon UAV Edition provides a complete autonomous flight stack for drones, featuring:

- **Precision Landing**: Computer vision-based landing with 5cm accuracy
- **Obstacle Avoidance**: Real-time LiDAR and camera-based collision avoidance
- **Swarm Coordination**: Multi-drone coordination for coordinated missions
- **GPS-Denied Navigation**: Visual SLAM for indoor and denied environments
- **Mission Planning**: Autonomous waypoint navigation with failsafes

## Supported Airframes

### Multirotor
- **Quadcopters** (X, H, + configurations)
- **Hexacopters** (6 motors)
- **Octocopters** (8 motors)
- **Tricopters** (3 motors + servo)

### Fixed-Wing
- **Standard Planes** (Aileron, Elevator, Rudder)
- **Flying Wings** (Elevons)
- **VTOL Tailsitters**
- **VTOL Tiltrotors**

## Quick Start

### 1. Install Cule OS UAV Edition

```bash
# Download UAV edition image
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-v1.0.img.xz

# Flash to SD card
xzcat cule-os-axon-v1.0.img.xz | sudo dd of=/dev/sdX bs=4M status=progress
```

### 2. Initial Configuration

```bash
# Connect via SSH
ssh cule@cule-os.local

# Run UAV configuration wizard
sudo cule-config-uav

# Select your airframe type
1. Quadcopter X
2. Quadcopter +
3. Hexacopter
4. Fixed-wing
5. VTOL
```

### 3. Hardware Connections

**Minimum Required:**
- Flight Controller (Pixhawk 4/5/6) → USB to Jetson/RPi
- GPS Module → UART on Pixhawk
- Telemetry Radio → Telem1 on Pixhawk
- RC Receiver → RC IN on Pixhawk
- Camera → CSI/USB to Jetson/RPi

### 4. First Flight Checklist

Before first flight:
- [ ] Compass calibration complete
- [ ] Accelerometer calibration complete
- [ ] Radio calibration complete
- [ ] ESC calibration complete
- [ ] GPS lock (3D fix minimum)
- [ ] Battery voltage verified
- [ ] Props secured (remove before calibration!)
- [ ] Safety pilot ready

## Flight Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| **MANUAL** | Direct RC control, no stabilization | Testing, emergencies |
| **STABILIZE** | Self-leveling but manual throttle | Learning, manual flight |
| **ALT_HOLD** | Maintains altitude automatically | Photography, inspection |
| **LOITER** | Holds position and altitude | Waiting, observation |
| **AUTO** | Executes pre-programmed mission | Survey, mapping, delivery |
| **RTL** | Return to launch point | Low battery, signal loss |
| **LAND** | Controlled landing | End of mission |
| **GUIDED** | Ground station control | Follow-me, point-and-click |
| **CIRCLE** | Orbits a point of interest | Inspection, filming |

## Safety Features

### Failsafes
- **Radio Failsafe**: RTL on signal loss
- **Battery Failsafe**: RTL at low voltage
- **Geofence**: Virtual boundaries
- **Crash Detection**: Auto-disarm on impact
- **Parachute Support**: Automatic deployment

### Pre-arm Checks
- INS calibration status
- Compass health
- GPS lock quality
- Battery voltage
- RC signal strength
- Airspeed (for planes)

## Mission Planning

### Using QGroundControl

1. Connect to Cule OS via telemetry
2. Plan → Survey → Create grid
3. Set altitude and speed
4. Upload to vehicle
5. Arm and switch to AUTO

### Using Cule CLI

```bash
# Create waypoint mission
cule-mission create --name survey --altitude 50 --speed 10

# Add waypoints
cule-mission add-waypoint --lat 37.7749 --lon -122.4194 --alt 50
cule-mission add-waypoint --lat 37.7750 --lon -122.4195 --alt 50

# Upload and execute
cule-mission upload
cule-mission start
```

## Swarm Operations

### Setup Swarm

```bash
# On ground station
cule-swarm init --count 4

# Assign drones
cule-swarm add --id 1 --ip 192.168.1.101
cule-swarm add --id 2 --ip 192.168.1.102
```

### Formation Flight

```bash
# Grid formation
cule-swarm formation --type grid --spacing 5

# Circle formation
cule-swarm formation --type circle --radius 10

# Execute mission with swarm
cule-swarm mission --file survey.plan
```

## Troubleshooting

### Common Issues

**GPS Not Locking:**
- Check antenna placement (clear view of sky)
- Wait 5-10 minutes for cold start
- Verify baud rate (typically 115200)

**Compass Errors:**
- Calibrate away from metal objects
- Check for magnetic interference
- Verify compass orientation

**Vibration Issues:**
- Balance propellers
- Check motor mount tightness
- Use vibration dampening

**Camera Not Detected:**
- Check CSI cable connection
- Verify `nvargus-daemon` is running
- Test with `gst-launch-1.0 nvarguscamerasrc`

## API Reference

### Python API

```python
from cule import UAV

# Connect to vehicle
drone = UAV.connect('/dev/ttyUSB0')

# Arm and takeoff
drone.arm()
drone.takeoff(altitude=10)

# Navigate to waypoint
drone.goto(lat=37.7749, lon=-122.4194, alt=20)

# Return to launch
drone.rtl()

# Land
drone.land()
```

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cule/uav/telemetry` | `nav_msgs/Odometry` | Position and velocity |
| `/cule/uav/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/cule/uav/mission` | `cule_msgs/Mission` | Mission status |
| `/cule/camera/image` | `sensor_msgs/Image` | Camera feed |

## Specifications

- **Update Rate**: 400Hz control loop
- **Latency**: <10ms sensor to actuator
- **Position Accuracy**: ±0.3m (GPS), ±0.05m (RTK)
- **Altitude Accuracy**: ±0.1m (baro), ±0.02m (LiDAR)
- **Communication Range**: 1km (915MHz), 10km (long-range)
- **Battery Life**: 20-45 minutes (depending on configuration)

## Next Steps

- [Quick Start Guide](./quickstart.md) - Get flying in 30 minutes
- [Flight Modes](./flight-modes.md) - Detailed mode documentation
- [Mission Planning](./missions.md) - Create autonomous missions
- [Safety Procedures](./safety.md) - Critical safety information
- [Troubleshooting](./troubleshooting.md) - Fix common issues

## Support

- **Documentation**: https://sanjaysparker27.github.io/cule-os/docs/uav
- **Forum**: https://discussion.cule-os.io
- **GitHub Issues**: https://github.com/SanjaySparker27/cule-os/issues
