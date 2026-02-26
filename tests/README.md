# Testing Framework for Robotics Simulation

Comprehensive testing and validation suite for UAV/USV/UGV robotics platforms.

## Directory Structure

```
tests/
├── simulation/          # Gazebo/Ignition simulation environments
│   ├── gazebo/         # Gazebo world files (.world)
│   ├── ignition/       # Ignition/Fortress world files (.sdf)
│   ├── models/         # Vehicle models
│   │   ├── uav/       # Quadcopter models
│   │   ├── usv/       # Boat models
│   │   └── ugv/       # Rover models
│   └── worlds/         # Additional world assets
├── ros2/               # ROS2 integration
│   ├── nodes/          # ROS2 nodes
│   ├── launch/         # Launch files
│   └── config/         # Configuration files
├── scenarios/          # Test scenarios
│   ├── test_takeoff.py
│   ├── test_landing.py
│   ├── test_waypoint.py
│   ├── test_obstacle_avoidance.py
│   └── test_runner.py  # Test suite orchestrator
├── hil/                # Hardware-in-Loop testing
│   └── hil_framework.py
├── benchmarks/         # Performance benchmarks
│   └── performance_benchmarks.py
└── utils/              # Testing utilities
```

## Quick Start

### Run Test Suite
```bash
cd tests/scenarios
python test_runner.py
```

### Run Specific Test
```bash
python test_runner.py --test "UAV Takeoff"
```

### Run HIL Tests
```bash
cd tests/hil
python hil_framework.py
```

### Run Performance Benchmarks
```bash
cd tests/benchmarks
python performance_benchmarks.py
```

## Test Scenarios

### 1. Takeoff Test (UAV)
- Validates vertical takeoff performance
- Measures: time to altitude, overshoot, settling time, stability

### 2. Landing Test (UAV)
- Precision landing with GPS and visual guidance
- Measures: descent time, touchdown velocity, accuracy, drift

### 3. Waypoint Navigation (UAV/USV/UGV)
- Multi-waypoint mission execution
- Measures: path efficiency, cross-track error, waypoint reach rate

### 4. Obstacle Avoidance (UGV/UAV)
- Reactive and planned avoidance
- Measures: clearance distance, collision count, path deviation

## Simulation Environments

### Gazebo Worlds
- `uav_world.world` - Aerial vehicle testing with wind and obstacles
- `usv_world.world` - Marine environment with waves and currents
- `ugv_world.world` - Ground terrain with rocks and tunnels

### Ignition/Fortress Worlds
- Modern SDF format with improved physics
- Same environments as Gazebo, updated format

### Vehicle Models
- **UAV**: Quadcopter with IMU, GPS, camera, sonar
- **USV**: Boat with thrusters, LIDAR, GPS, depth sensor
- **UGV**: 4-wheel rover with LIDAR, RGB-D camera, bumpers

## Hardware-in-Loop (HIL)

Supports testing with real flight controllers:
- MAVLink protocol interface
- Real-time simulation coupling
- Sensor/actuator loop closure
- Supports PX4/ArduPilot

### Usage
```python
from hil.hil_framework import MAVLinkHILInterface

hil = MAVLinkHILInterface("udp:127.0.0.1:14550")
hil.start()
# Run tests...
hil.stop()
```

## Performance Benchmarks

### Metrics Measured
- **Latency**: Function/ROS callback latency (μs)
- **Throughput**: Message/data throughput (msgs/sec, MB/s)
- **Resources**: CPU usage, memory consumption
- **Simulation**: Real-time factor, physics step time

### Running Benchmarks
```python
from benchmarks.performance_benchmarks import BenchmarkRunner, LatencyBenchmark

runner = BenchmarkRunner()
runner.add_benchmark(LatencyBenchmark())
report = runner.run_all()
```

## ROS2 Integration

### Bridge Node
Publishes simulated sensor data:
- `/imu/data` - IMU measurements
- `/gps/fix` - GPS position
- `/odometry` - State estimation
- `/camera/image_raw` - Camera feed
- `/scan` - LIDAR data

Subscribes to commands:
- `/cmd_vel` - Velocity commands
- `/cmd_pose` - Position commands

### Launch Example
```bash
ros2 launch test_simulation uav_simulation.launch.py
```

## Adding New Tests

1. Create test file in `scenarios/`
2. Implement test class inheriting from `unittest.TestCase`
3. Register in `test_runner.py`:

```python
suite.register_test("My Test", my_test_function)
```

## Requirements

- ROS2 Humble/Iron
- Gazebo 11+ or Ignition Fortress
- Python 3.8+
- NumPy, PyTorch (for ML components)

## Validation Criteria

| Test | Pass Criteria |
|------|---------------|
| Takeoff | < 15s to altitude, < 2m overshoot, > 80% stability |
| Landing | Touchdown < 1 m/s, accuracy < 0.5m |
| Waypoint | 100% waypoint reach, > 80% path efficiency |
| Obstacle | 0 collisions, > 0.5m clearance |
| Performance | RT factor > 0.9, latency < 100μs |
