# LiDAR Integration

Light Detection and Ranging (LiDAR) provides precise 3D spatial mapping for obstacle avoidance, SLAM, and autonomous navigation.

## LiDAR Overview

### LiDAR Types

| Type | Range | Resolution | Update Rate | Use Case |
|------|-------|------------|-------------|----------|
| 2D Scanning | 10-40m | 0.25°-1° | 10-40Hz | Obstacle avoidance, 2D SLAM |
| 3D Solid-State | 10-100m | 0.1°-0.4° | 10-20Hz | Small UAVs, tight spaces |
| 3D Mechanical | 50-300m | 0.1°-0.4° | 5-20Hz | Mapping, autonomous driving |
| Flash | 5-50m | 160x120 | 1-5Hz | Small size, simple integration |
| FMCW | 200m+ | High | 10Hz | Long range, velocity measurement |

### Supported LiDARs

| Model | Type | Range | FOV | Rate | Interface |
|-------|------|-------|-----|------|-----------|
| RPLIDAR A1 | 2D | 12m | 360° | 5.5Hz | UART |
| RPLIDAR A2 | 2D | 18m | 360° | 10Hz | UART |
| RPLIDAR A3 | 2D | 25m | 360° | 10Hz | UART |
| YDLIDAR X4 | 2D | 10m | 360° | 7Hz | UART |
| YDLIDAR X2L | 2D | 8m | 360° | 7Hz | UART |
| Ouster OS1-32 | 3D | 120m | 360°x45° | 10Hz | Ethernet |
| Ouster OS1-64 | 3D | 120m | 360°x45° | 10Hz | Ethernet |
| Ouster OS0-128 | 3D | 50m | 360°x90° | 10Hz | Ethernet |
| Livox Mid-360 | 3D | 100m | 360°x59° | 10Hz | Ethernet |
| Livox Avia | 3D | 450m | 70.4°x77.2° | 10Hz | Ethernet |
| Velodyne Puck | 3D | 100m | 360°x30° | 10Hz | Ethernet |
| Velodyne VLP-16 | 3D | 100m | 360°x30° | 10Hz | Ethernet |
| Livox Horizon | 3D | 260m | 81.7°x25.1° | 10Hz | Ethernet |
| Benewake TF03 | Solid | 180m | 1° | 1kHz | UART/CAN |
| LightWare SF45 | Solid | 50m | Adjustable | 50Hz | UART/I2C |

## 2D LiDAR (RPLIDAR)

### RPLIDAR A1/A2/A3

```
RPLIDAR 2D Scanning Pattern
══════════════════════════════════════════════════════════════════

                  360° Scan
                     │
        ═════════════╪══════════════
                     │
                    ╱ │ ╲
                   ╱  │  ╲
                  ╱   │   ╲
                 ╱    ●    ╲
                ╱    FC     ╲
               ╱              ╲
              ╱                ╲

Scan Characteristics:
- 360° horizontal field of view
- Single plane (2D)
- Rotating mirror mechanism
- ~0.25° angular resolution (A3)
- 10Hz rotation rate

Output: Array of (angle, distance, intensity)
Points per scan: ~720-1440
```

### Wiring

```
RPLIDAR Connection
══════════════════════════════════════════════════════════════════

     ┌─────────────┐                      ┌─────────────┐
     │   RPLIDAR   │                      │  Companion  │
     │    A2       │                      │  Computer   │
     │             │                      │  (Pi/Jetson)│
     │             │                      │             │
     │  VCC (5V)   ●──────────────────────●  5V         │
     │             │       Red            │             │
     │  GND        ●──────────────────────●  GND        │
     │             │       Black          │             │
     │  TX         ●──────────────────────●  RX (UART)  │
     │             │       Green          │             │
     │  RX         ●──────────────────────●  TX (UART)  │
     │             │       Yellow         │             │
     │  MOTOCTL    ●──────────────────────●  GPIO       │
     │             │       Blue           │             │
     └─────────────┘                      └─────────────┘

Pinout (7-pin connector):
Pin 1: VCC 5V (Red)
Pin 2: GND (Black)
Pin 3: TX (Green) - Data from LiDAR
Pin 4: RX (Yellow) - Commands to LiDAR
Pin 5: MOTOCTL (Blue) - Motor control (PWM)
Pin 6: GND (Black) - Motor ground
Pin 7: VMOTO (Red) - Motor power (5V)

Power Consumption:
- Idle: ~350mA @ 5V
- Scanning: ~450mA @ 5V
- Peak: ~600mA @ 5V
```

### Software Setup

```bash
# 1. Install RPLIDAR SDK
git clone https://github.com/Slamtec/rplidar_sdk.git
cd rplidar_sdk/sdk
make

# 2. Add udev rule (for USB-UART adapter)
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", \
    ATTRS{idProduct}=="ea60", MODE="0666", \
    GROUP="dialout", SYMLINK+="rplidar"' | \
    sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules

# 3. Test with SDK tools
./output/Linux/Release/ultra_simple --channel \
    --serial /dev/ttyUSB0 --baud 115200

# 4. ROS2 driver (if using ROS2)
sudo apt install ros-$ROS_DISTRO-rplidar-ros
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200

# 5. Cule OS integration
sudo cule-config --sensor lidar --model rplidar-a2
```

### Cule OS Integration

```python
from cule import LiDAR2D

# Initialize RPLIDAR
lidar = LiDAR2D.connect('/dev/ttyUSB0', baudrate=115200)

# Start scanning
lidar.start_motor()
lidar.start_scan()

# Get scan data
scan = lidar.get_scan()

# Process scan
for point in scan:
    angle = point.angle  # degrees
    distance = point.distance  # mm
    quality = point.quality
    print(f"Angle: {angle:.1f}°, Distance: {distance}mm")

# Stop
lidar.stop_scan()
lidar.stop_motor()
lidar.disconnect()
```

## 3D LiDAR (Ouster)

### Ouster OS1

```
Ouster OS1-64 Scanning Pattern
══════════════════════════════════════════════════════════════════

Vertical FOV: 45° (-22.5° to +22.5°)
Horizontal FOV: 360°
Channels: 32 or 64 vertical beams

Side View:
                    ╱│╲
                   ╱ │ ╲
                  ╱  │  ╲
                 ╱   │   ╲
                ╱    ●    ╲    ← +22.5° (top)
               ╱    LiDAR   ╲
              ╱              ╲
             ╱                ╲
            ╱                  ╲
           ╱                    ╲
          ╱______________________╲  ← -22.5° (bottom)

Top View (360° rotation):

              ╱╲    ╱╲    ╱╲
             ╱  ╲  ╱  ╲  ╱  ╲
            ╱    ╲╱    ╲╱    ╲
           │    360° Scan     │
            ╲    ╱╲    ╱╲    ╱
             ╲  ╱  ╲  ╱  ╲  ╱
              ╲╱    ╲╱    ╲╱

Output: Point cloud (x, y, z, intensity, timestamp)
Points per frame: ~32,000 (OS1-32) / ~65,000 (OS1-64)
```

### Wiring

```
Ouster OS1 Connection
══════════════════════════════════════════════════════════════════

     ┌─────────────┐                      ┌─────────────┐
     │   Ouster    │                      │  Network    │
     │    OS1      │                      │   Switch    │
     │             │                      │             │
     │  RJ45       ●══════════════════════●  Port 1     │
     │  (Ethernet) │       Cat6           │             │
     │             │                      │             │
     │  Power      ●────┐                 │  Port 2     │
     │  (48V PoE)  │    │                 └──────┬──────┘
     └─────────────┘    │                        │
                        │         ┌──────────────┤
                        │         │              │
                        ▼         ▼              ▼
                   ┌────────┐ ┌────────┐  ┌──────────┐
                   │  48V   │ │  24V   │  │ Companion│
                   │  PoE   │ │  BEC   │  │ Computer │
                   │Injector│ │ (FC)   │  │ (via ETH)│
                   └────────┘ └────────┘  └──────────┘

Power Requirements:
- Input: 48V PoE or 24V DC
- Consumption: 14-20W typical
- Startup: Up to 40W

Network Configuration:
- Default IP: 192.168.1.1 (LiDAR)
- Host IP: 192.168.1.100
- Subnet: 255.255.255.0
- Port: 7502 (data)
```

### Network Configuration

```bash
# 1. Set static IP on host
sudo ip addr add 192.168.1.100/24 dev eth0

# 2. Verify connection
ping 192.168.1.1

# 3. Install Ouster SDK
pip install ouster-sdk

# 4. Test connection
ouster-cli config --sensor 192.168.1.1

# 5. View data
ouster-cli source 192.168.1.1 viz

# 6. Cule OS configuration
sudo cule-config --sensor lidar --model ouster-os1 --ip 192.168.1.1
```

### Cule OS Integration

```python
from cule import LiDAR3D

# Initialize Ouster LiDAR
lidar = LiDAR3D.connect('192.168.1.1')

# Configure
config = {
    'mode': '1024x10',  # Resolution and rate
    'udp_dest': '192.168.1.100',
    'udp_port_lidar': 7502,
    'udp_port_imu': 7503
}
lidar.configure(config)

# Start streaming
lidar.start()

# Get point cloud
cloud = lidar.get_cloud()

# Access data
points = cloud.points  # Nx3 array (x, y, z)
intensities = cloud.intensities  # N array
timestamps = cloud.timestamps  # N array

# Process with Open3D
import open3d as o3d
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Voxel downsampling
pcd_down = pcd.voxel_down_sample(voxel_size=0.05)

# Stop
lidar.stop()
lidar.disconnect()
```

## 3D LiDAR (Livox)

### Livox Mid-360

```
Livox Mid-360 Pattern
══════════════════════════════════════════════════════════════════

Coverage Pattern:
- 360° horizontal
- -7° to +52° vertical (forward)
- 360° x 59° total FOV

Unique Feature:
- Non-repetitive scanning
- High density in center
- Solid-state (no moving parts)

FOV Visualization:

              ╱╲    High density
             ╱  ╲   center region
            ╱    ╲
           ╱  ●   ╲    ← Forward
          ╱  UAV   ╲
         ╱          ╲
        ╱____________╲
        360° coverage

Range: 10m @ 80% reflectivity
      40m @ 10% reflectivity
      100m @ 80% reflectivity

Update Rate: 10Hz (fixed)
Points per second: 200,000
```

### Wiring

```
Livox Mid-360 Connection
══════════════════════════════════════════════════════════════════

     ┌─────────────┐                      ┌─────────────┐
     │  Livox      │                      │   Network   │
     │  Mid-360    │                      │   Switch    │
     │             │                      │             │
     │  RJ45       ●══════════════════════●  Port 1     │
     │  (Ethernet) │       Cat6           │             │
     │             │                      │             │
     │  Power      ●────┐                 │  Port 2     │
     │  (12-30V)   │    │                 └──────┬──────┘
     └─────────────┘    │                        │
                        │                        │
                        ▼                        ▼
                   ┌────────┐            ┌──────────┐
                   │  12V   │            │ Companion│
                   │  BEC   │            │ Computer │
                   │ (5A+)  │            │ (via ETH)│
                   └────────┘            └──────────┘

Power Requirements:
- Input: 12-30V DC
- Consumption: 8-10W typical
- Peak: 15W

Installation:
- Mount with clear view forward and down
- Keep away from prop wash
- Temperature: -20°C to 60°C
```

### Software Setup

```bash
# 1. Install Livox SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j4
sudo make install

# 2. Configure IP
# Edit config file
sudo nano /usr/local/etc/livox/livox_config.json
{
    "host_ip": "192.168.1.100",
    "broadcast_code": "<your_broadcast_code>"
}

# 3. Build ROS2 driver
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd .. && colcon build

# 4. Launch
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 5. Cule OS configuration
sudo cule-config --sensor lidar --model livox-mid-360
```

## LiDAR for Obstacle Avoidance

### Configuration

```bash
# ArduPilot obstacle avoidance setup

# Enable proximity sensor
param set PRX_TYPE 2  # LightWare SF45c
param set PRX_TYPE 4  # RPLidarA2
param set PRX_TYPE 5  # Range finders

# For 360° LiDAR
param set AVOID_ENABLE 7  # All avoidance
param set AVOID_MARGIN 2  # 2m margin
param set AVOID_DIST_MAX 10  # 10m max

# Bendy ruler path planning
param set OA_TYPE 1  # BendyRuler
param set OA_BR_LOOKAHEAD 5  # 5m lookahead
param set OA_BR_CONT_RATIO 1.5  # Contour following

# Dijkstra path planning
param set OA_TYPE 2  # Dijkstra
param set OA_DIJKSTRA_MAX 1000  # Max nodes
```

### Cule OS Obstacle Avoidance

```python
from cule import ObstacleAvoidance
from cule import LiDAR2D

# Initialize LiDAR
lidar = LiDAR2D.connect('/dev/ttyUSB0')

# Initialize avoidance system
oa = ObstacleAvoidance()
oa.set_lidar(lidar)

# Configuration
oa.set_safety_margin(2.0)  # meters
oa.set_max_range(20.0)  # meters

# In navigation loop
while navigating:
    # Get processed obstacles
    obstacles = oa.get_obstacles()
    
    # Check path is clear
    if not oa.is_path_clear(target_position):
        # Get alternative path
        path = oa.plan_path(current_pos, target_pos)
        
    # Or use reactive avoidance
    avoidance_vector = oa.compute_avoidance_vector()
    velocity_cmd += avoidance_vector
```

## LiDAR for SLAM

### SLAM Configuration

```bash
# Install SLAM packages
# For 2D (RPLIDAR):
sudo apt install ros-$ROS_DISTRO-slam-toolbox

# For 3D (Ouster/Livox):
sudo apt install ros-$ROS_DISTRO-fast-lio
# or
sudo apt install ros-$ROS_DISTRO-lio-sam

# Launch 2D SLAM
ros2 launch slam_toolbox online_sync_launch.py

# Launch 3D SLAM (Livox)
ros2 launch fast_lio mapping.launch.py
```

### Cule OS SLAM Integration

```python
from cule import SLAM, LiDAR2D

# Initialize LiDAR
lidar = LiDAR2D.connect('/dev/ttyUSB0')

# Initialize SLAM
slam = SLAM.lidar_2d()
slam.set_lidar(lidar)

# Start mapping
slam.start_mapping()

# Get position in map
while True:
    pose = slam.get_pose()
    x, y, theta = pose.x, pose.y, pose.theta
    
    map_data = slam.get_map()
    # Use for navigation

# Save map
slam.save_map('my_map.yaml')
```

## Multi-LiDAR Setup

```
Multi-LiDAR Configuration
══════════════════════════════════════════════════════════════════

              ┌─────────────────┐
              │  Companion      │
              │  Computer       │
              │                 │
              │  USB0   USB1   ETH0│
              └─┬───────┬───────┬─┘
                │       │       │
                │       │       └────────┐
                │       │                │
                ▼       ▼                ▼
           ┌────────┐ ┌────────┐  ┌──────────┐
           │RPLIDAR │ │RPLIDAR │  │ Ouster   │
           │  A2    │ │  A2    │  │  OS1     │
           │(Front) │ │ (Rear) │  │ (Top)    │
           └────────┘ └────────┘  └──────────┘

Configuration:
- Front LiDAR: /dev/ttyUSB0 - Obstacle avoidance
- Rear LiDAR: /dev/ttyUSB1 - Rear coverage
- Top LiDAR: 192.168.1.1 - 3D mapping

Benefits:
- 360° coverage without blind spots
- Redundancy
- Different LiDARs for different tasks
```

## Power and Mounting Considerations

### Power Budget

| LiDAR | Voltage | Current | Power | Peak |
|-------|---------|---------|-------|------|
| RPLIDAR A1 | 5V | 450mA | 2.25W | 3W |
| RPLIDAR A2 | 5V | 450mA | 2.25W | 3W |
| Ouster OS1 | 48V PoE | 300mA | 14W | 40W |
| Livox Mid-360 | 12V | 700mA | 8.4W | 15W |
| YDLIDAR X4 | 5V | 350mA | 1.75W | 2.5W |

### Vibration Isolation

```
LiDAR Mounting Best Practices
══════════════════════════════════════════════════════════════════

Requirements:
1. Rigid mounting for mechanical LiDARs
2. Vibration isolation for IMU-based units
3. Clear field of view
4. Away from prop wash
5. Weather protection (if outdoor)

Vibration Damping:
- Gel dampers for high-frequency noise
- Foam pads for lower frequencies
- Mass dampers for resonance control

Mounting Materials:
- Carbon fiber (rigid, lightweight)
- 3D printed PETG (damping, cheap)
- Aluminum (rigid, durable)
```

## Troubleshooting

### Common Issues

```bash
# RPLIDAR not spinning
# Check motor power (separate from logic)
# VMOTO should be 5V

# Ouster connection timeout
# Check network configuration
# Verify 192.168.1.x subnet
ping 192.168.1.1

# Livox not detected
# Check broadcast code
# Verify firewall rules
sudo ufw allow from 192.168.1.0/24

# No points in scan
# Check LiDAR health in web interface
# Verify no physical obstructions
```

### Health Monitoring

```python
from cule import LiDARHealth

# Monitor LiDAR health
health = LiDARHealth()

while True:
    status = health.check('192.168.1.1')
    
    if status.temperature > 70:
        print("Warning: LiDAR overheating")
        
    if status.packet_loss > 5:
        print("Warning: Network issues")
        
    if status.motor_rpm < 300:
        print("Warning: Motor speed low")
        
    time.sleep(5)
```

## See Also

- [Sensors](./sensors.md) - General sensor documentation
- [Rangefinders](./rangefinders.md) - Single-point distance sensors
- [SLAM](./slam.md) - Simultaneous Localization and Mapping
- [Obstacle Avoidance](./obstacle-avoidance.md) - Navigation safety
