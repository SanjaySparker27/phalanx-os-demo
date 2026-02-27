# MAVLink Protocol

## Overview

MAVLink (Micro Air Vehicle Link) is a lightweight messaging protocol for communicating with drones and between drone components. Cule-OS implements MAVLink 2.0 for ground control, companion computers, and inter-vehicle communication.

## Protocol Basics

### MAVLink Versions

```
MAVLink 1.0 (Legacy):
- 8-byte header
- 255 max payload bytes
- Basic message set
- Limited field types

MAVLink 2.0 (Current):
- 10-byte header
- 255 max payload bytes
- Larger message set
- Signed messages (security)
- Compatibility with MAVLink 1

Cule-OS uses MAVLink 2.0 by default with 1.0 fallback
```

### Message Structure

```
MAVLink 2 Packet Format:

┌─────────┬────────┬────────┬────────┬─────────┬────────┬────────┬────────┬────────┐
│  STX    │ LEN    │INC/FLG│ INC/FLG│ SEQ     │ SYS ID │ COMP ID│ MSG ID │ PAYLOAD │
│0xFD     │        │        │        │         │        │        │        │         │
│1 byte   │1 byte  │1 byte  │1 byte  │1 byte   │1 byte  │1 byte  │3 bytes │0-255 b  │
└─────────┴────────┴────────┴────────┴─────────┴────────┴────────┴────────┴────────┘

┌────────┬────────┬────────┐
│SIGNATURE (if signing enabled)│
│13 bytes                      │
└──────────────────────────────┘

Total: 12 + payload + signature (optional) bytes

Field descriptions:
STX:      Start byte (0xFD for MAVLink 2)
LEN:      Payload length
INC/FLG:  Incompatibility flags
INC/FLG:  Compatibility flags
SEQ:      Packet sequence number (0-255)
SYS ID:   System ID (1-255, 0=broadcast)
COMP ID:  Component ID (see table below)
MSG ID:   Message ID (3 bytes in MAVLink 2)
PAYLOAD:  Message-specific data
SIGNATURE: Authentication (optional)
```

### Component IDs

```
Common Component IDs:

1   MAV_COMP_ID_AUTOPILOT1    # Main autopilot
0   MAV_COMP_ID_ALL           # Broadcast
1   MAV_COMP_ID_GPS           # GPS
2   MAV_COMP_ID_MISSIONPLANNER # Ground station
3   MAV_COMP_ID_QGROUNDCONTROL # QGroundControl
4   MAV_COMP_ID_MAVIOSD       # OSD
5   MAV_COMP_ID_CAMERA        # Camera
6   MAV_COMP_ID_BRUSHLESSPUMP # Pump controller
7   MAV_COMP_ID_SERVO1-8      # Servos
8   MAV_COMP_ID_ODID           # Remote ID

Companion Computer IDs:
195 MAV_COMP_ID_PATHPLANNER   # Path planner
196 MAV_COMP_ID_OBSTACLE_AVOIDANCE
197 MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
198 MAV_COMP_ID_PAIRING_MANAGER

Peripheral IDs:
154 MAV_COMP_ID_GIMBAL        # Gimbal
155 MAV_COMP_ID_LOG           # Logging
156 MAV_COMP_ID_ADSB          # ADS-B
157 MAV_COMP_ID_OA            # Obstacle avoidance
158 MAV_COMP_ID_PERIPHERAL    # Generic peripheral
```

## Cule-OS MAVLink Configuration

### Serial Configuration

```
Enable MAVLink on a serial port:

SERIAL1_PROTOCOL  2       # MAVLink 2
SERIAL1_BAUD      921600  # High speed

Protocol options:
0 = None
1 = MAVLink 1
2 = MAVLink 2
3 = FrSky D
4 = FrSky SPort
5 = GPS
...

Common baud rates:
57600   # Standard telemetry
115200  # Fast telemetry
921600  # High-speed companion
1500000 # Maximum speed (short cables)
```

### MAVLink Stream Configuration

```
Control message rates per stream:

SR0_xxx parameters (USB port):
SR0_ADSB         5       # ADS-B stream rate (Hz)
SR0_EXTRA1       10      # Attitude
SR0_EXTRA2       10      # VFR_HUD
SR0_EXTRA3       2       # AHRS, HWSTATUS, etc.
SR0_EXT_STAT     2       # Extended status
SR0_POSITION     3       # Position
SR0_RAW_CTRL     0       # Raw control
SR0_RAW_SENS     2       # Raw sensors
SR0_RC_CHAN      5       # RC channels

SR1_xxx parameters (Telemetry radio):
SR1_ADSB         2       # Lower rates for radio
SR1_EXTRA1       4
SR1_EXTRA2       4
...

Disable stream:
SR0_ADSB         0       # Disable ADS-B stream
```

### Message Signing

```
Enable MAVLink signing for security:

Set signing key:
MAVLINK_SIGNING_KEY  "your-secret-key-here"

Behavior:
- Outgoing messages signed
- Unsigned incoming messages rejected (optional)
- Replay attack protection

Use case:
- Commercial operations
- Security-sensitive applications
- Prevent unauthorized control
```

## Common Message Types

### Heartbeat (MSG_ID #0)

```
Purpose: Indicate system status and type

Message fields:
mavlink_heartbeat_t:
  type:         MAV_TYPE_QUADROTOR
  autopilot:    MAV_AUTOPILOT_PX4
  base_mode:    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
                MAV_MODE_FLAG_STABILIZE_ENABLED
  custom_mode:  0  # Mode-specific
  system_status: MAV_STATE_ACTIVE

Standard rate: 1 Hz (required)
```

### System Status (MSG_ID #1)

```
Purpose: Report system health and battery

Key fields:
- onboard_control_sensors_present
- onboard_control_sensors_enabled
- onboard_control_sensors_health
- load: CPU load (%)
- voltage_battery: mV
- current_battery: cA
- battery_remaining: %
- drop_rate_comm: Dropped packets
- errors_comm: Communication errors
- errors_count1-4: Subsystem errors
```

### Attitude (MSG_ID #30)

```
Purpose: Report vehicle attitude

Key fields:
- time_boot_ms: Timestamp
- roll: Roll angle (rad)
- pitch: Pitch angle (rad)
- yaw: Yaw angle (rad)
- rollspeed: Roll rate (rad/s)
- pitchspeed: Pitch rate (rad/s)
- yawspeed: Yaw rate (rad/s)

Standard rate: 10-50 Hz
```

### Global Position (MSG_ID #33)

```
Purpose: GPS position and navigation data

Key fields:
- time_boot_ms: Timestamp
- lat: Latitude (degE7)
- lon: Longitude (degE7)
- alt: Altitude MSL (mm)
- relative_alt: Relative altitude (mm)
- vx: Ground X speed (cm/s)
- vy: Ground Y speed (cm/s)
- vz: Ground Z speed (cm/s)
- hdg: Heading (cdeg)

Standard rate: 1-10 Hz
```

### Command Long (MSG_ID #76)

```
Purpose: Send commands to vehicle

Structure:
- target_system: Target system ID
- target_component: Target component
- command: MAV_CMD enum
- confirmation: 0=first attempt, 1+ retry
- param1-7: Command-specific parameters

Example: Arm/Disarm
command: MAV_CMD_COMPONENT_ARM_DISARM (400)
param1: 1 (arm) or 0 (disarm)
param2: 21196 (force disarm magic number)
```

## MAVLink Commands

### Common MAV_CMDs

```
Navigation Commands:
MAV_CMD_NAV_WAYPOINT          # Navigate to waypoint
MAV_CMD_NAV_TAKEOFF           # Takeoff to altitude
MAV_CMD_NAV_LAND              # Land at location
MAV_CMD_NAV_RETURN_TO_LAUNCH  # Return to home
MAV_CMD_NAV_LOITER_TIME       # Loiter for time
MAV_CMD_NAV_LOITER_TURNS      # Loiter for turns

Condition Commands:
MAV_CMD_CONDITION_DELAY       # Wait for time
MAV_CMD_CONDITION_DISTANCE    # Wait for distance
MAV_CMD_CONDITION_YAW         # Wait for heading

Do Commands:
MAV_CMD_DO_SET_MODE           # Set flight mode
MAV_CMD_DO_JUMP               # Jump to waypoint
MAV_CMD_DO_CHANGE_SPEED       # Change speed
MAV_CMD_DO_SET_CAM_TRIGG_DIST # Camera trigger
MAV_CMD_DO_GRIPPER            # Actuate gripper
MAV_CMD_DO_PARACHUTE          # Deploy parachute

Preflight Commands:
MAV_CMD_PREFLIGHT_CALIBRATION # Calibrate sensors
MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS
MAV_CMD_PREFLIGHT_STORAGE     # Save/Load params
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

Mission Commands:
MAV_CMD_MISSION_START         # Start mission
MAV_CMD_MISSION_CLEAR_ALL     # Clear mission
MAV_CMD_MISSION_SET_CURRENT   # Set current WP
```

### Command Acknowledgment

```
After sending COMMAND_LONG:

Vehicle responds with COMMAND_ACK (MSG_ID #77)

Fields:
- command: Echo of command ID
- result: Result code

Result codes:
MAV_RESULT_ACCEPTED = 0      # Command executed
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED = 4
MAV_RESULT_IN_PROGRESS = 5   # Long-running command
```

## Mission Protocol

### Mission Upload

```
Protocol flow:

1. GCS sends MISSION_COUNT
   - target_system
   - target_component
   - count: Number of waypoints
   - mission_type: 0=flight plan

2. Vehicle sends MISSION_REQUEST_INT
   - seq: Requested sequence number

3. GCS sends MISSION_ITEM_INT
   - seq: Sequence number
   - frame: Coordinate frame
   - command: MAV_CMD
   - param1-4: Command params
   - x: Latitude (degE7)
   - y: Longitude (degE7)
   - z: Altitude

4. Repeat 2-3 for all items

5. Vehicle sends MISSION_ACK
   - type: MAV_MISSION_ACCEPTED
```

### Mission Download

```
Protocol flow:

1. GCS sends MISSION_REQUEST_LIST

2. Vehicle sends MISSION_COUNT
   - count: Number of items

3. GCS sends MISSION_REQUEST_INT for seq 0

4. Vehicle sends MISSION_ITEM_INT

5. Repeat 3-4 for all items

6. GCS sends MISSION_ACK
```

## Parameter Protocol

### Parameter Read

```
Read all parameters:

1. GCS sends PARAM_REQUEST_LIST

2. Vehicle sends PARAM_VALUE for each parameter
   - param_id: Parameter name (16 chars)
   - param_value: Float value
   - param_type: MAV_PARAM_TYPE
   - param_count: Total parameters
   - param_index: Index of this param

Read single parameter:
1. GCS sends PARAM_READ with param_id
2. Vehicle responds with PARAM_VALUE
```

### Parameter Write

```
Set parameter:

1. GCS sends PARAM_SET
   - target_system
   - target_component
   - param_id: Parameter name
   - param_value: New value
   - param_type: Data type

2. Vehicle sends PARAM_VALUE (confirmation)
   - Echo of new value

3. Parameter persists if acknowledged
```

## MAVLink Over UDP/TCP

### UDP Configuration

```
MAVLink over UDP (typical for WiFi/Ethernet):

On Flight Controller:
SERIAL5_PROTOCOL  2       # MAVLink 2
SERIAL5_BAUD      [N/A for network]
NET_P1_TYPE       1       # MAVLink server
NET_P1_PROTOCOL   2       # MAVLink 2
NET_P1_PORT       14550   # Standard port

Network settings:
NET_IPADDR0-3     [IP address]
NET_NETMASK0-3    [Netmask]
NET_GWADDR0-3     [Gateway]

QGroundControl connection:
- UDP to [FC IP]:14550
- Auto-connect on startup
```

### TCP Configuration

```
MAVLink over TCP (more reliable):

On Flight Controller:
NET_P1_TYPE       2       # MAVLink TCP client
NET_P1_PROTOCOL   2
NET_P1_PORT       5760    # TCP port
NET_IPADDR0-3     [GCS IP]  # Connect to GCS

Or server mode:
NET_P1_TYPE       3       # MAVLink TCP server
NET_P1_PORT       5760    # Listen port
```

## Companion Computer Integration

### Offboard Control

```
MAVLink for companion computer control:

Setup:
SERIAL4_PROTOCOL  2       # MAVLink 2
SERIAL4_BAUD      921600  # High speed

Offboard mode activation:
- Set flight mode to OFFBOARD
- Send SET_POSITION_TARGET_LOCAL_NED at >2Hz

Message: SET_POSITION_TARGET_LOCAL_NED (#84)
Fields:
- time_boot_ms
- coordinate_frame: MAV_FRAME_LOCAL_NED
- type_mask: What to control
- x, y, z: Position (m)
- vx, vy, vz: Velocity (m/s)
- afx, afy, afz: Acceleration
- yaw, yaw_rate
```

### MAVROS (ROS Integration)

```
MAVLink bridge to ROS:

1. Install MAVROS:
   sudo apt install ros-$ROS_DISTRO-mavros

2. Launch MAVROS:
   roslaunch mavros px4.launch

3. Configure connection:
   fcu_url: /dev/ttyACM0:921600
   gcs_url: udp://@192.168.1.100

4. Access topics:
   /mavros/state
   /mavros/global_position/global
   /mavros/local_position/pose
   /mavros/imu/data
```

## Debugging MAVLink

### MAVLink Inspector

```
QGroundControl MAVLink Inspector:
Tools → MAVLink Inspector

Shows:
- All received messages
- Message rates (Hz)
- System and component IDs
- Message contents

Use for:
- Verifying message flow
- Debugging missing data
- Checking message rates
- Protocol analysis
```

### Console Commands

```
MAVLink console commands:

mavlink status          # Show MAVLink status
mavlink stream          # Configure streams
mavlink boot            # Show boot messages
mavlink check           # Check message rates

Set stream rates:
mavlink stream -d /dev/ttyS1 -s ATTITUDE -r 50
```

## Reference

- [MAVLink Developer Guide](https://mavlink.io)
- [MAVLink Message Definitions](https://mavlink.io/en/messages/common.html)
- [Cule-OS MAVLink Implementation](https://github.com/cule-os/mavlink)
