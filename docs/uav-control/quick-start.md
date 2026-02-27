# Cule OS UAV Control - Quick Start Guide

## Overview

This guide gets you flying with Cule OS UAV control in minutes.

## Prerequisites

### Hardware Requirements
- Flight controller running ArduPilot 4.3+ or PX4 1.14+
- Telemetry radio (optional but recommended)
- RC transmitter with at least 6 channels
- Battery with appropriate voltage for your vehicle

### Software Requirements
- Cule OS 1.0+
- Python 3.8+
- MAVLink libraries

## Installation

```bash
# Install Cule OS UAV control package
pip install cule-os-uav

# Or from source
git clone https://github.com/cule-os/uav-control.git
cd uav-control
pip install -e .
```

## Configuration

### 1. Connect to Your Vehicle

```python
from cule_os.uav import UAVController

# USB connection
uav = UAVController(
    autopilot_type="px4",
    connection_string="/dev/ttyUSB0",  # Linux
    # connection_string="COM3",         # Windows
    # connection_string="/dev/tty.usbmodem1",  # macOS
    baudrate=57600
)

# Or UDP for telemetry radio
uav = UAVController(
    autopilot_type="px4",
    connection_string="udp:127.0.0.1:14550"
)

uav.connect()
```

### 2. Verify Connection

```python
# Check vehicle heartbeat
print(f"Connected to: {uav.autopilot_type}")
print(f"Vehicle type: {uav.vehicle_type}")
print(f"Armed: {uav.is_armed()}")
print(f"Mode: {uav.get_mode()}")

# Get basic telemetry
print(f"Battery: {uav.get_battery()}")
print(f"GPS: {uav.get_gps()}")
print(f"Position: {uav.get_position()}")
```

## Basic Operations

### Arm the Vehicle

```python
# Arm with all safety checks
result = uav.arm()
if result == "MAV_RESULT_ACCEPTED":
    print("Vehicle armed successfully")
else:
    print(f"Arm failed: {result}")

# Check pre-arm status
status = uav.get_prearm_status()
for check, passed in status.items():
    print(f"{check}: {'PASS' if passed else 'FAIL'}")
```

### Takeoff

```python
# Set to guided mode first (if using ArduPilot)
uav.set_mode("GUIDED")

# Takeoff to 10 meters
uav.takeoff(altitude=10)

# Wait for altitude
uav.wait_for_altitude(10, tolerance=0.5)
print("Reached target altitude")
```

### Navigate

```python
# Fly to a GPS coordinate
uav.goto(
    lat=-35.363261,
    lon=149.165230,
    alt=30,
    groundspeed=5  # m/s
)

# Wait to reach waypoint
uav.wait_for_waypoint(lat=-35.363261, lon=149.165230, tolerance=2.0)
```

### Return Home

```python
# Initiate return to launch
uav.return_to_launch()

# Wait for landing
uav.wait_for_landing()

# Disarm
uav.disarm()
```

### Land

```python
# Land at current position
uav.land()

# Wait for landing
uav.wait_for_landing()

# Disarm
uav.disarm()
```

## Flight Mode Reference

### ArduPilot Modes

| Mode | Use Case | GPS Required |
|------|----------|--------------|
| STABILIZE | Manual flight training | No |
| ALT_HOLD | Altitude hold practice | No |
| LOITER | Position hold | Yes |
| GUIDED | GCS control | Yes |
| AUTO | Mission execution | Yes |
| RTL | Return home | Yes |
| LAND | Auto landing | Optional |

### PX4 Modes

| Mode | Use Case | Position Hold |
|------|----------|---------------|
| MANUAL | Full manual | No |
| STABILIZED | Self-leveling | No |
| ALTITUDE | Altitude hold | No |
| POSITION | Position hold | Yes |
| MISSION | Autonomous | Yes |
| RETURN | RTL mode | Yes |
| LAND | Auto landing | Yes |

## Mission Planning

### Simple Waypoint Mission

```python
# Define waypoints
waypoints = [
    {"lat": -35.363261, "lon": 149.165230, "alt": 50},
    {"lat": -35.364261, "lon": 149.166230, "alt": 50},
    {"lat": -35.365261, "lon": 149.167230, "alt": 50},
    {"lat": -35.363261, "lon": 149.165230, "alt": 50},  # Return to start
]

# Upload mission
uav.upload_mission(waypoints)

# Set mode to AUTO
uav.set_mode("AUTO")

# Arm and start
uav.arm()

# Monitor progress
while uav.mission_in_progress():
    current_wp = uav.get_current_waypoint()
    progress = uav.get_mission_progress()
    print(f"Waypoint {current_wp}/{len(waypoints)}: {progress}%")
    time.sleep(1)
```

## Safety Checklist

### Before Each Flight

- [ ] Props secure and undamaged
- [ ] Battery fully charged
- [ ] GPS lock acquired (wait for 3D fix)
- [ ] Pre-arm checks passing
- [ ] RC transmitter batteries charged
- [ ] Telemetry connection stable
- [ ] Home position set correctly
- [ ] RTL altitude appropriate
- [ ] Flight area clear

### Emergency Procedures

#### Loss of Control
```python
# Switch to manual/stabilize
uav.set_mode("STABILIZE")  # ArduPilot
# or
uav.set_mode("STABILIZED")  # PX4

# Reduce throttle if descending
# Attempt controlled descent
```

#### Low Battery
```python
# Immediately RTL
uav.return_to_launch()
```

#### Flyaway
```python
# Emergency kill (if parachute equipped)
uav.terminate_flight()
```

## Troubleshooting

### Connection Issues

```
Problem: Cannot connect to vehicle
Solution: 
- Check cable/connection
- Verify baudrate matches
- Check correct port/COM
- Ensure no other app using port
```

### GPS Issues

```
Problem: No GPS lock
Solution:
- Wait longer (first lock can take minutes)
- Ensure clear view of sky
- Check GPS module connection
- Verify GPS is enabled in parameters
```

### Arm Issues

```
Problem: Cannot arm
Solution:
- Check all pre-arm errors
- Calibrate accelerometer
- Calibrate compass
- Check battery voltage
- Verify in SAFE mode if switch exists
```

## Example: Complete Flight Script

```python
#!/usr/bin/env python3
"""Complete flight example"""

import time
from cule_os.uav import UAVController

def main():
    # Initialize
    uav = UAVController(
        autopilot_type="px4",
        connection_string="udp:127.0.0.1:14550"
    )
    uav.connect()
    
    # Wait for GPS
    print("Waiting for GPS lock...")
    while uav.get_gps()["fix_type"] < 3:
        time.sleep(1)
    print("GPS locked!")
    
    # Pre-flight checks
    print("\nPre-flight status:")
    print(f"Battery: {uav.get_battery()}")
    print(f"Mode: {uav.get_mode()}")
    print(f"Armed: {uav.is_armed()}")
    
    # Arm and takeoff
    print("\nArming...")
    uav.arm()
    
    print("Taking off to 10m...")
    uav.takeoff(10)
    uav.wait_for_altitude(10, tolerance=1.0)
    
    # Simple mission
    print("\nExecuting simple mission...")
    uav.goto(lat=-35.363261, lon=149.165230, alt=20)
    time.sleep(10)
    
    # RTL
    print("\nReturning to launch...")
    uav.return_to_launch()
    uav.wait_for_landing()
    
    # Disarm
    uav.disarm()
    print("\nFlight complete!")

if __name__ == "__main__":
    main()
```

## Next Steps

1. Read the [Comprehensive Guide](comprehensive-guide.md)
2. Review the [API Reference](api-reference.md)
3. Explore [Example Missions](examples/)
4. Configure [Failsafes](failsafe-configuration.md)

## Support

- GitHub Issues: https://github.com/cule-os/uav-control/issues
- Documentation: https://docs.cule-os.io/uav/
- Community Forum: https://discuss.cule-os.io/

---

**Fly safe, fly smart!**
