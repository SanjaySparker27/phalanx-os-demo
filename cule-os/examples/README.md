# Cule OS Examples

This directory contains working examples to help you get started with Cule OS.

## Quick Start Examples

### 1. Basic UAV Mission (Drone)

```bash
# Navigate to examples
cd /opt/cule-os/examples

# Run basic UAV example
python3 basic_uav_mission.py
```

**What it does:**
- Takes off to 10m altitude
- Hovers for 10 seconds
- Lands safely

**Requirements:**
- UAV with ArduPilot/PX4
- GPS lock
- Safety pilot recommended

### 2. UGV Patrol (Rover)

```bash
python3 ugv_patrol.py
```

**What it does:**
- Drives in a square pattern
- Avoids obstacles
- Returns to start

### 3. USV Survey (Boat)

```bash
python3 usv_survey.py
```

**What it does:**
- Surveys a rectangular area
- Logs GPS coordinates
- Returns to dock

## Simulation Examples

Test without real hardware:

```bash
# Start Gazebo simulation
roslaunch cule_simulation gazebo.launch

# Run simulation example
python3 simulation_example.py
```

## Advanced Examples

### Swarm Coordination

```bash
python3 swarm_formation.py --drones 4 --pattern grid
```

### Object Tracking

```bash
python3 object_tracking.py --target person
```

### GPS-Denied Navigation

```bash
python3 visual_odometry.py --enable-slam
```

## Example Code Structure

```python
#!/usr/bin/env python3
"""
Basic Cule OS Mission Example
"""

from cule import Vehicle, Agent
import time

# Connect to vehicle
drone = Vehicle.connect('/dev/ttyUSB0')

# Arm and takeoff
drone.arm()
drone.takeoff(altitude=10)

# Wait
time.sleep(10)

# Land
drone.land()
```

## Running Examples

### With Real Hardware

1. Connect flight controller
2. Power on vehicle
3. Run example script
4. Monitor in QGroundControl

### In Simulation

1. Launch simulator:
   ```bash
   roslaunch cule_simulation uav.launch
   ```

2. Run example with `--sim` flag:
   ```bash
   python3 example.py --sim
   ```

## Safety

⚠️ **IMPORTANT:**
- Always test in simulation first
- Use safety pilot for real flights
- Check propellers are secure
- Maintain line of sight
- Follow local regulations

## Contributing Examples

Have a cool example? Share it!

1. Fork the repository
2. Add your example to `examples/`
3. Include README with instructions
4. Submit pull request

## More Examples

See also:
- `training/examples/` - ML training examples
- `tests/scenarios/` - Test scenarios
- `docs/tutorials/` - Step-by-step tutorials