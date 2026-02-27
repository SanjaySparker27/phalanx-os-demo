# Cule OS UAV Control System - Comprehensive Documentation

## Table of Contents
1. [Overview](#overview)
2. [Supported Autopilot Systems](#supported-autopilot-systems)
3. [MAVLink Protocol Integration](#mavlink-protocol-integration)
4. [Flight Modes Reference](#flight-modes-reference)
5. [Safety & Failsafe Systems](#safety--failsafe-systems)
6. [Operation Procedures](#operation-procedures)
7. [API Reference](#api-reference)
8. [Configuration](#configuration)

---

## Overview

Cule OS provides a comprehensive UAV control subsystem designed for professional drone operations. It integrates with industry-standard autopilot systems (ArduPilot and PX4) via the MAVLink protocol, offering a unified interface for controlling multirotors, fixed-wing aircraft, VTOL vehicles, and other unmanned systems.

### Key Features

- **Multi-Autopilot Support**: Compatible with ArduPilot and PX4 flight stacks
- **Full MAVLink 2.0 Compliance**: Supports all standard MAVLink messages and commands
- **Real-time Control**: Sub-50ms command latency for critical operations
- **Safety-First Design**: Built-in failsafe mechanisms and geofencing
- **Mission Planning**: Waypoint-based autonomous mission execution
- **Telemetry Monitoring**: Real-time vehicle status and health monitoring

---

## Supported Autopilot Systems

### ArduPilot

ArduPilot is a mature, open-source autopilot system supporting multiple vehicle types.

#### Vehicle Types Supported
| Vehicle | Description |
|---------|-------------|
| Copter | Multirotors (quad, hexa, octo, etc.) |
| Plane | Fixed-wing aircraft |
| Rover | Ground vehicles |
| Sub | Underwater vehicles |
| Blimp | Lighter-than-air vehicles |
| AntennaTracker | Ground-based antenna tracking |

#### Architecture
ArduPilot uses a monolithic architecture with:
- **Scheduler**: 400Hz main loop for critical tasks
- **Attitude Control**: PID-based controllers with feedforward
- **Navigation**: EKF-based state estimation
- **Failsafe**: Multiple independent failsafe triggers

#### Key Parameters (ArduPilot)
```
FLTMODE1 - FLTMODE6    # Flight mode channel mapping
ARMING_CHECK           # Pre-arm safety checks
BATT_LOW_VOLT          # Battery low voltage threshold
FS_THR_ENABLE          # Throttle failsafe action
WPNAV_SPEED            # Waypoint navigation speed
```

### PX4

PX4 is a professional-grade autopilot with a modular, reactive architecture.

#### System Architecture
```
┌─────────────────────────────────────────────────────────┐
│                    PX4 Flight Stack                     │
├─────────────────────────────────────────────────────────┤
│  Estimators │  Controllers  │  Navigator  │  Commander  │
│   (EKF2)    │ (Rate/Attitude│  (Mission)  │   (State)   │
│             │ /Position)    │             │             │
├─────────────────────────────────────────────────────────┤
│                    uORB Message Bus                     │
├─────────────────────────────────────────────────────────┤
│  Drivers  │  Middleware   │  Simulation │   uXRCE-DDS   │
│ (Sensors) │   (uORB)      │  (SITL/HITL)│   (ROS2)      │
└─────────────────────────────────────────────────────────┘
```

#### Key Features
- **Reactive Architecture**: Publish-subscribe messaging via uORB
- **Modular Design**: Each module runs independently
- **High Update Rates**: IMU at 1kHz, control at 250Hz
- **Multi-Platform**: NuttX, Linux, macOS, Windows support

#### Key Parameters (PX4)
```
COM_FLTMODE1 - COM_FLTMODE6    # Flight mode slots
SYS_AUTOSTART                  # Airframe configuration
MPC_XY_CRUISE                  # Horizontal cruise speed
MPC_Z_VEL_MAX_UP              # Maximum ascent velocity
COM_RC_IN_MODE                # RC input mode
```

---

## MAVLink Protocol Integration

MAVLink is the micro air vehicle communication protocol used for communication between the ground control station, companion computer, and autopilot.

### Protocol Versions

| Version | Status | Features |
|---------|--------|----------|
| MAVLink 1 | Legacy | Basic messaging, limited fields |
| MAVLink 2 | Current | Message signing, larger payloads, extensions |

### Core Message Categories

#### 1. System Status Messages
```
HEARTBEAT (0)           - System presence and type
SYS_STATUS (1)          - Sensor and subsystem status
SYSTEM_TIME (2)         - Unix timestamp
PING (4)                - Latency measurement
```

#### 2. Command Messages
```
COMMAND_LONG (76)       - Command with float params
COMMAND_INT (75)        - Command with int params (navigation)
COMMAND_ACK (77)        - Command acknowledgment
COMMAND_CANCEL (80)     - Cancel long-running command
```

#### 3. Mission Protocol
```
MISSION_ITEM (39)       - Waypoint definition
MISSION_ITEM_INT (73)   - Waypoint with int coordinates
MISSION_REQUEST (40)    - Request mission item
MISSION_COUNT (44)      - Mission item count
MISSION_ACK (47)        - Mission operation ack
```

#### 4. Navigation Messages
```
GLOBAL_POSITION_INT (33)     - Global position (lat/lon/alt)
LOCAL_POSITION_NED (32)      - Local position (NED frame)
ATTITUDE (30)                - Attitude (quaternion/rates)
GPS_RAW_INT (24)             - GPS raw data
VFR_HUD (74)                 - HUD data (airspeed, alt, etc.)
```

### Command Protocol

#### Command Structure
Commands use `MAV_CMD` enumeration with up to 7 parameters:

```c
// COMMAND_LONG format
struct COMMAND_LONG {
    uint16_t command;       // MAV_CMD id
    uint8_t target_system;
    uint8_t target_component;
    float param1;           // Command-specific
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
};
```

#### Essential Commands

| Command | ID | Description | Parameters |
|---------|-----|-------------|------------|
| MAV_CMD_NAV_WAYPOINT | 16 | Navigate to waypoint | Delay, accept radius, pass radius |
| MAV_CMD_NAV_TAKEOFF | 22 | Takeoff to altitude | Min pitch, yaw, latitude, longitude, altitude |
| MAV_CMD_NAV_LAND | 21 | Land at location | Abort altitude, land mode |
| MAV_CMD_NAV_RETURN_TO_LAUNCH | 20 | Return to launch | - |
| MAV_CMD_COMPONENT_ARM_DISARM | 400 | Arm/disarm vehicle | 1=arm, 0=disarm, force |
| MAV_CMD_DO_SET_MODE | 176 | Set flight mode | Mode base, custom mode, submode |
| MAV_CMD_GET_HOME_POSITION | 410 | Get home position | - |
| MAV_CMD_DO_REPOSITION | 192 | Reposition vehicle | Speed, bitmask, yaw, lat, lon, alt |

#### Command Acknowledgment

```c
enum MAV_RESULT {
    MAV_RESULT_ACCEPTED = 0,           // Command accepted
    MAV_RESULT_TEMPORARILY_REJECTED = 1, // Try again later
    MAV_RESULT_DENIED = 2,              // Command denied
    MAV_RESULT_UNSUPPORTED = 3,         // Command not supported
    MAV_RESULT_FAILED = 4,              // Command failed
    MAV_RESULT_IN_PROGRESS = 5,         // Command in progress
    MAV_RESULT_CANCELLED = 6            // Command cancelled
};
```

### Message Signing (MAVLink 2)

For secure operations, MAVLink 2 supports message signing:

```
Sign Key: 48-byte secret key
Link ID: 0-255 (identifies signing link)
Timestamp: Microseconds since boot
Signature: 6-byte HMAC-SHA256
```

---

## Flight Modes Reference

### ArduPilot Flight Modes (Copter)

| Mode | Alt Ctrl | Pos Ctrl | GPS Req | Description |
|------|----------|----------|---------|-------------|
| **STABILIZE** | - | + | No | Self-leveling, manual throttle |
| **ALT_HOLD** | s | + | No | Altitude hold, self-leveling |
| **LOITER** | s | s | Yes | Position and altitude hold |
| **POSHOLD** | s | + | Yes | Loiter with manual override |
| **AUTO** | A | A | Yes | Mission execution |
| **GUIDED** | A | A | Yes | GCS commanded waypoints |
| **RTL** | A | A | Yes | Return to launch |
| **SMART_RTL** | A | A | Yes | RTL via recorded path |
| **LAND** | A | s | Opt | Automatic landing |
| **ACRO** | - | - | No | Rate control, no limits |
| **SPORT** | s | s | Yes | Alt hold, rate attitude |
| **DRIFT** | - | + | Yes | Coordinated turns |
| **FLIP** | A | A | Yes | Automated flip |
| **BRAKE** | A | A | Yes | Emergency stop |
| **CIRCLE** | s | A | Yes | Orbit around point |
| **FOLLOW** | s | A | Yes | Follow another vehicle |
| **AUTOTUNE** | s | A | Yes | Auto PID tuning |
| **THROW** | A | A | Yes | Throw launch mode |

**Legend**: `-`=Manual, `+`=Stabilized, `s`=Semi-auto, `A`=Auto

### ArduPilot Flight Modes (Plane)

| Mode | Roll | Pitch | Throttle | GPS | Description |
|------|------|-------|----------|-----|-------------|
| **MANUAL** | - | - | - | No | Full manual control |
| **FBWA** | s | s | - | No | Fly-by-wire A (stabilized) |
| **FBWB** | s | A | SPD | No | FBWA + altitude hold |
| **CRUISE** | A | A | SPD | Yes | FBWB + course tracking |
| **STABILIZE** | + | + | - | No | Wing leveling |
| **ACRO** | + | + | - | No | Rate mode |
| **TRAINING** | + | + | - | No | Limits with manual control |
| **AUTOTUNE** | s | s | - | No | Learn tuning in flight |
| **AUTO** | A | A | A | Yes | Mission mode |
| **LOITER** | A | A | A | Yes | Circle hold |
| **RTL** | A | A | A | Yes | Return to launch |
| **TAKEOFF** | A | A | A | Yes | Auto takeoff |
| **LAND** | A | A | A | Yes | Auto landing |
| **GUIDED** | A | A | A | Yes | GCS control |
| **CIRCLE** | A | A | A | Yes | Continuous circle |
| **THERMAL** | A | A | A | Yes | Thermal soaring |
| **QSTABILIZE** | + | + | - | No | VTOL stabilize |
| **QHOVER** | s | A | SPD | No | VTOL hover |
| **QLOITER** | s | s | A | Yes | VTOL position hold |
| **QLAND** | A | A | A | Yes | VTOL landing |

### PX4 Flight Modes

#### Multicopter Modes

| Mode | Description | Position | Altitude | Manual |
|------|-------------|----------|----------|--------|
| **Manual/Stabilized** | Self-leveling | - | - | Attitude |
| **Altitude** | Altitude hold | - | Hold | Attitude |
| **Position** | Position hold | Hold | Hold | Velocity |
| **Position Slow** | Slow position hold | Hold | Hold | Slow velocity |
| **Acro** | Rate control | - | - | Rates |
| **Orbit** | Circle around point | Auto | Auto | - |
| **Hold** | Position hold at point | Hold | Hold | - |
| **Return** | Return to launch | Auto | Auto | - |
| **Mission** | Execute mission | Auto | Auto | - |
| **Land** | Precision landing | Auto | Auto | - |
| **Takeoff** | Auto takeoff | Auto | Auto | - |
| **Offboard** | External control | Ext | Ext | Ext |
| **Follow Me** | Follow GCS position | Auto | Auto | - |

#### Fixed-Wing Modes

| Mode | Description |
|------|-------------|
| **Manual** | Direct control surface control |
| **Stabilized** | Self-leveling, coordinated turns |
| **Altitude** | Altitude and heading hold |
| **Position** | Position hold with loiter |
| **Acro** | Rate control for aerobatics |
| **Mission** | Waypoint navigation |
| **Return** | Return to launch |
| **Land** | Precision landing |
| **Takeoff** | Auto takeoff |
| **Hold** | Loiter at current position |
| **Offboard** | External control input |

#### VTOL Modes

VTOL vehicles support both multicopter and fixed-wing modes, with automatic transition:

```
Transition Forward:  MC Mode → FW Mode
Transition Back:     FW Mode → MC Mode
```

---

## Safety & Failsafe Systems

### ArduPilot Failsafes

#### 1. Throttle Failsafe (Radio)
Triggered when RC signal is lost.

```
Parameters:
- FS_THR_ENABLE: 0=Disabled, 1=Enabled always RTL, 2=Enabled Continue Mission, 3=Enabled always Land
- FS_THR_VALUE: PWM threshold (default: 975)
```

**Actions Available:**
- Disabled
- Always RTL
- Continue with Mission
- Always Land
- SmartRTL or RTL
- SmartRTL or Land

#### 2. Battery Failsafe
Triggered when battery voltage/capacity falls below threshold.

```
Parameters:
- BATT_LOW_VOLT: Low voltage threshold
- BATT_CRT_VOLT: Critical voltage threshold
- BATT_LOW_MAH: Low capacity threshold (mAh)
- FS_BATT_ENABLE: Action on low battery
- FS_BATT_MAH: Action on low capacity
```

**Actions:**
- None
- Land
- RTL
- SmartRTL
- Terminate (Parachute)

#### 3. GCS Failsafe
Triggered when ground control station heartbeat is lost.

```
Parameters:
- FS_GCS_ENABLE: Enable GCS failsafe
- FS_GCS_TIMEOUT: Timeout in seconds (default: 5)
```

**Actions:**
- Disabled
- Always RTL
- Continue Mission
- Always Land

#### 4. EKF Failsafe
Triggered when EKF navigation accuracy degrades.

```
Parameters:
- FS_EKF_ACTION: 0=Disabled, 1=Switch to Land, 2=RTL, 3=Land
- FS_EKF_THRESH: Threshold for triggering (0.6=strict, 0.8=default, 1.0=relaxed)
```

#### 5. Vibration Failsafe
Triggered by excessive vibration.

```
Parameters:
- FS_VIBE_ENABLE: Enable/disable
```

#### 6. Terrain Failsafe
Triggered when terrain data is unavailable.

```
Parameters:
- TERRAIN_FOLLOW: Enable terrain following
```

### PX4 Failsafes

#### Commander Failsafe States

```c
enum FailsafeState {
    FAILSAFE_STATE_NONE = 0,
    FAILSAFE_STATE_RC = 1,
    FAILSAFE_STATE_LINK = 2,
    FAILSAFE_STATE_GPS = 3,
    FAILSAFE_STATE_BATTERY = 4,
    FAILSAFE_STATE_GEOFENCE = 5,
    FAILSAFE_STATE_TERMINATE = 6
};
```

#### Key Safety Parameters

```
# RC Loss
COM_RC_LOSS_T:        RC loss timeout (default: 0.5s)
COM_RC_OVERRIDE:      Allow RC override in offboard

# Data Link Loss
COM_DL_LOSS_T:        Data link loss timeout
COM_DLL_ACT:          Data link loss action

# Battery
BAT_CRIT_THR:         Critical battery threshold (%)
BAT_EMERGEN_THR:      Emergency battery threshold (%)
BAT_LOW_THR:          Low battery threshold (%)
COM_LOW_BAT_ACT:      Low battery action

# Position Loss
COM_POSCTL_NAVL:      Position control navigation loss
EKF2_GPS_CTRL:        GPS control mode

# Geofence
GF_ACTION:            Geofence breach action
GF_ALTMODE:           Geofence altitude mode
GF_MAX_VER_DIST:      Max vertical distance
GF_MAX_HOR_DIST:      Max horizontal distance
```

#### Failsafe Action Hierarchy

1. **Termination** (highest priority)
2. **Battery Emergency**
3. **Geofence Breach**
4. **RC Loss**
5. **Data Link Loss**
6. **GPS/Position Loss**
7. **Battery Low**

### Geofencing

#### ArduPilot Geofence

```
Parameters:
- FENCE_ENABLE: Enable geofence
- FENCE_TYPE: Bitmask of types (0=Alt, 1=Circle, 2=Polygon)
- FENCE_ACTION: 0=Report, 1=RTL/Loiter, 2=Land, 3=Terminate
- FENCE_ALT_MAX: Maximum altitude
- FENCE_RADIUS: Circular fence radius
- FENCE_MARGIN: Breach margin
```

#### PX4 Geofence

```
Parameters:
- GF_ACTION: 0=None, 1=Warning, 2=Hold, 3=Return, 4=Terminate
- GF_ALTMODE: 0=WGS84, 1=AGL
- GF_SOURCE: 0=Globalpos, 1=GPS
- GF_COUNT: -1=Unlimited, 0=Disabled
- GF_MAX_HOR_DIST: Max horizontal distance from home
- GF_MAX_VER_DIST: Max vertical distance from home
```

### Arming/Disarming Safety

#### ArduPilot

```
Parameters:
ARMING_CHECK: Pre-arm check bitmask
  - Bit 0: All
  - Bit 1: Barometer
  - Bit 2: Compass
  - Bit 3: GPS
  - Bit 4: INS (IMU)
  - Bit 5: Parameters
  - Bit 6: RC channels
  - Bit 7: Board voltage
  - Bit 8: Battery level
  - Bit 9: Airspeed
  - Bit 10: Logging
  - Bit 11: Hardware safety switch
  - Bit 12: GPS configuration
  - Bit 13: System
  - Bit 14: Mission
  - Bit 15: Rangefinder
  - Bit 16: Camera
  - Bit 17: Aux auth
  - Bit 18: Visual odometry
  - Bit 19: FFT
  - Bit 20: OGND (optical flow)

ARMING_REQUIRE: 0=Disabled, 1=Required before throttle
ARMING_RUDDER: Rudder arming (0=Disabled, 1=Enabled, 2=Only in ACRO)
```

#### PX4

```
Parameters:
COM_ARM_AUTH: Arm authorization method
COM_ARM_AUTH_ID: Authorization method ID
COM_ARM_CHK_ESCS: Check ESCs on arm
COM_ARM_HDL_TAKEOFF: Hand launch detection
COM_ARM_HDL_TAKEOFF_T: Hand launch timeout
COM_ARM_IMU_ACC: Accel magnitude deviation limit
COM_ARM_IMU_GYR: Gyro magnitude limit
COM_ARM_MAG_ANG: Max compass/POS heading diff
COM_ARM_MAG_STR: Compass strength check
COM_ARM_SDCARD: SD card check
COM_ARM_SWISBTN: Arm switch is button
COM_ARM_WO_GPS: Arm without GPS
COM_DISARM_LAND: Auto-disarm after land (seconds)
COM_DISARM_PRFLT: Pre-flight disarm timeout
COM_REARM_GRACE: Re-arm grace period
```

---

## Operation Procedures

### Pre-Flight Checklist

#### 1. Planning Phase
- [ ] Verify NOTAMs and airspace restrictions
- [ ] Check weather conditions (wind, visibility, precipitation)
- [ ] Plan flight path and alternate routes
- [ ] Identify emergency landing sites
- [ ] Notify relevant parties (if required)

#### 2. Equipment Check
- [ ] Visual inspection of airframe (cracks, damage, loose parts)
- [ ] Propeller condition and security
- [ ] Battery charge level and connections
- [ ] Gimbal/camera mounting and functionality
- [ ] GPS module clear view of sky
- [ ] RC transmitter batteries charged
- [ ] Ground station software updated
- [ ] Telemetry radio range tested

#### 3. Software Configuration
- [ ] Verify correct airframe configuration
- [ ] Check flight mode switch mapping
- [ ] Verify home point is set
- [ ] Confirm geofence is configured
- [ ] Test failsafe settings in simulation
- [ ] Validate mission waypoints
- [ ] Check RTL altitude is appropriate

#### 4. Pre-Arm Checks
- [ ] IMU calibration complete
- [ ] Compass calibration current
- [ ] GPS lock acquired (minimum satellites)
- [ ] Radio link established
- [ ] Telemetry connected
- [ ] Battery voltage nominal
- [ ] Pre-arm checks passing (listen for tones)
- [ ] Motors spinning in correct direction

#### 5. Takeoff Area
- [ ] Area clear of people and obstacles
- [ ] Takeoff surface level and clear
- [ ] Safe landing approach available
- [ ] Visual line of sight maintained
- [ ] Emergency access clear

### Flight Operations

#### Standard Flight Profile

```
1. ARM VEHICLE
   ├── Verify all checks pass
   ├── Confirm GPS lock
   └── Arm via switch or GCS

2. TAKEOFF
   ├── Vertical ascent to hover altitude
   ├── Verify stable hover
   ├── Check control response
   └── Confirm telemetry healthy

3. MISSION EXECUTION
   ├── Navigate to first waypoint
   ├── Monitor progress and status
   ├── Adjust for wind/conditions
   └── Maintain VLOS when possible

4. RTL/RETURN
   ├── Initiate return mode
   ├── Monitor approach
   └── Prepare for landing

5. LANDING
   ├── Position over landing zone
   ├── Controlled descent
   ├── Touchdown detection
   └── Disarm after landing confirmed
```

#### Emergency Procedures

##### 1. Loss of Control
1. Switch to manual/stabilize mode if available
2. Reduce throttle to minimum if descending
3. Attempt to regain orientation
4. Initiate controlled descent
5. Execute emergency landing

##### 2. Flyaway
1. Switch to RTL immediately
2. If RTL fails, switch to manual
3. Attempt to regain control
4. If all else fails, use kill switch (parachute if equipped)

##### 3. Low Battery
1. Abort mission immediately
2. Initiate RTL
3. If insufficient power for RTL, land immediately at safe location
4. Do not attempt to continue flight

##### 4. GPS Loss
1. Switch to altitude/altitude hold mode
2. Fly manually back to launch area
3. Land as soon as safe
4. Do not continue autonomous operations

##### 5. RC Link Loss
1. Verify failsafe activation (should RTL)
2. Do not attempt to reconnect while vehicle is returning
3. Wait for RTL completion or failsafe action
4. Investigate cause before next flight

### Post-Flight Procedures

1. **Immediate Actions**
   - Disarm vehicle
   - Remove props (if applicable)
   - Power off systems
   - Secure aircraft

2. **Data Management**
   - Download flight logs
   - Review telemetry data
   - Check for errors/warnings
   - Backup mission files

3. **Inspection**
   - Visual check for damage
   - Motor temperature check
   - Battery voltage check
   - Props for nicks/cracks
   - Connectors for wear

4. **Documentation**
   - Log flight details
   - Note any anomalies
   - Record battery cycles
   - Update maintenance log

---

## API Reference

### Cule OS UAV Control API

#### Initialization

```python
from cule_os.uav import UAVController

# Initialize controller
uav = UAVController(
    autopilot_type="px4",  # or "ardupilot"
    connection_string="udp:127.0.0.1:14550",
    system_id=1,
    component_id=1
)

# Connect to vehicle
uav.connect()
```

#### Arming/Disarming

```python
# Arm vehicle
result = uav.arm()
# Returns: MAV_RESULT_ACCEPTED on success

# Arm with checks override (emergency only)
result = uav.arm(force=True)

# Disarm vehicle
result = uav.disarm()
```

#### Flight Mode Control

```python
# Set flight mode
uav.set_mode("LOITER")     # ArduPilot
uav.set_mode("POSCTL")     # PX4 Position
uav.set_mode("ALTCTL")     # PX4 Altitude
uav.set_mode("STABILIZE")  # ArduPilot Stabilize

# Get current mode
current_mode = uav.get_mode()
```

#### Takeoff and Landing

```python
# Takeoff to altitude (meters)
uav.takeoff(altitude=50)

# Wait for takeoff complete
uav.wait_for_altitude(50, tolerance=1.0)

# Land at current position
uav.land()

# Precision land (if equipped)
uav.land(precision=True)
```

#### Waypoint Navigation

```python
# Navigate to GPS coordinate
uav.goto(
    lat=-35.363261,
    lon=149.165230,
    alt=100,
    groundspeed=10
)

# Upload mission
waypoints = [
    {"lat": -35.363261, "lon": 149.165230, "alt": 50},
    {"lat": -35.364261, "lon": 149.166230, "alt": 50},
    {"lat": -35.365261, "lon": 149.167230, "alt": 50}
]
uav.upload_mission(waypoints)

# Start mission
uav.start_mission()

# Pause mission
uav.pause_mission()

# Resume mission
uav.resume_mission()
```

#### Return to Launch

```python
# Initiate RTL
uav.return_to_launch()

# RTL with options
uav.return_to_launch(
    altitude=100,  # RTL altitude
    land=True      # Auto-land at home
)
```

#### Telemetry Queries

```python
# Get position
position = uav.get_position()
# Returns: {"lat": float, "lon": float, "alt": float, "relative_alt": float}

# Get attitude
attitude = uav.get_attitude()
# Returns: {"roll": rad, "pitch": rad, "yaw": rad}

# Get battery status
battery = uav.get_battery()
# Returns: {"voltage": V, "current": A, "remaining": %, "mah_consumed": mAh}

# Get GPS status
gps = uav.get_gps()
# Returns: {"satellites": int, "fix_type": int, "hdop": float, "vdop": float}

# Get all vehicle state
state = uav.get_state()
```

#### Geofencing

```python
# Configure circular geofence
uav.set_geofence(
    type="circle",
    radius=500,      # meters
    max_alt=150      # meters
)

# Configure polygon geofence
uav.set_geofence(
    type="polygon",
    points=[
        {"lat": -35.363, "lon": 149.165},
        {"lat": -35.364, "lon": 149.166},
        {"lat": -35.365, "lon": 149.167},
        {"lat": -35.363, "lon": 149.165}
    ],
    max_alt=150
)

# Enable/disable geofence
uav.enable_geofence(True)
uav.enable_geofence(False)
```

#### Failsafe Configuration

```python
# Configure RC failsafe
uav.set_failsafe(
    type="rc_loss",
    action="rtl",     # rtl, land, continue, terminate
    timeout=1.0       # seconds
)

# Configure battery failsafe
uav.set_failsafe(
    type="battery",
    action="rtl",
    low_voltage=14.0,   # Volts
    critical_voltage=13.5
)

# Configure GCS failsafe
uav.set_failsafe(
    type="gcs_loss",
    action="rtl",
    timeout=5.0
)
```

### MAVLink Command Constants

```python
# MAV_CMD values
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_LOITER_TURNS = 18
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND_LOCAL = 23
MAV_CMD_NAV_TAKEOFF_LOCAL = 24
MAV_CMD_NAV_FOLLOW = 25
MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30
MAV_CMD_NAV_LOITER_TO_ALT = 31
MAV_CMD_DO_FOLLOW = 32
MAV_CMD_DO_FOLLOW_REPOSITION = 33
MAV_CMD_DO_ORBIT = 34
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_DO_JUMP = 177
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_SET_HOME = 179
MAV_CMD_DO_SET_PARAMETER = 180
MAV_CMD_DO_SET_RELAY = 181
MAV_CMD_DO_REPEAT_RELAY = 182
MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_DO_REPEAT_SERVO = 184
MAV_CMD_DO_FLIGHTTERMINATION = 185
MAV_CMD_DO_GO_AROUND = 191
MAV_CMD_DO_REPOSITION = 192
MAV_CMD_DO_PAUSE_CONTINUE = 193
MAV_CMD_DO_SET_REVERSE = 194
MAV_CMD_DO_SET_ROI_LOCATION = 195
MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196
MAV_CMD_DO_SET_ROI_NONE = 197
MAV_CMD_DO_CONTROL_VIDEO = 200
MAV_CMD_DO_SET_ROI = 201
MAV_CMD_DO_DIGICAM_CONFIGURE = 202
MAV_CMD_DO_DIGICAM_CONTROL = 203
MAV_CMD_DO_MOUNT_CONFIGURE = 204
MAV_CMD_DO_MOUNT_CONTROL = 205
MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206
MAV_CMD_DO_FENCE_ENABLE = 207
MAV_CMD_DO_PARACHUTE = 208
MAV_CMD_DO_MOTOR_TEST = 209
MAV_CMD_DO_INVERTED_FLIGHT = 210
MAV_CMD_DO_GRIPPER = 211
MAV_CMD_DO_AUTOTUNE_ENABLE = 212
MAV_CMD_NAV_SET_YAW_SPEED = 213
MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214
MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220
MAV_CMD_DO_GUIDED_MASTER = 221
MAV_CMD_DO_GUIDED_LIMITS = 222
MAV_CMD_DO_ENGINE_CONTROL = 223
MAV_CMD_DO_SET_MISSION_CURRENT = 224
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_GET_HOME_POSITION = 410
MAV_CMD_START_RX_PAIR = 500
MAV_CMD_GET_MESSAGE_INTERVAL = 510
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512
MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
MAV_CMD_REQUEST_CAMERA_INFORMATION = 521
MAV_CMD_REQUEST_CAMERA_SETTINGS = 522
MAV_CMD_REQUEST_STORAGE_INFORMATION = 525
MAV_CMD_STORAGE_FORMAT = 526
MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527
MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528
MAV_CMD_RESET_CAMERA_SETTINGS = 529
MAV_CMD_SET_CAMERA_MODE = 530
MAV_CMD_SET_CAMERA_ZOOM = 531
MAV_CMD_SET_CAMERA_FOCUS = 532
MAV_CMD_JUMP_TAG = 600
MAV_CMD_DO_JUMP_TAG = 601
MAV_CMD_PARAM_TRANSACTION = 900
MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001
MAV_CMD_IMAGE_START_CAPTURE = 2000
MAV_CMD_IMAGE_STOP_CAPTURE = 2001
MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002
MAV_CMD_DO_TRIGGER_CONTROL = 2003
MAV_CMD_CAMERA_TRACK_POINT = 2004
MAV_CMD_CAMERA_TRACK_RECTANGLE = 2005
MAV_CMD_CAMERA_STOP_TRACKING = 2010
MAV_CMD_VIDEO_START_CAPTURE = 2500
MAV_CMD_VIDEO_STOP_CAPTURE = 2501
MAV_CMD_VIDEO_START_STREAMING = 2502
MAV_CMD_VIDEO_STOP_STREAMING = 2503
MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504
MAV_CMD_REQUEST_VIDEO_STREAM_STATUS = 2505
MAV_CMD_LOGGING_START = 2510
MAV_CMD_LOGGING_STOP = 2511
MAV_CMD_AIRFRAME_CONFIGURATION = 2520
MAV_CMD_CONTROL_HIGH_LATENCY = 2600
MAV_CMD_PANORAMA_CREATE = 2800
MAV_CMD_DO_VTOL_TRANSITION = 3000
MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001
MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000
MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001
MAV_CMD_CONDITION_DELAY = 112
MAV_CMD_CONDITION_CHANGE_ALT = 113
MAV_CMD_CONDITION_DISTANCE = 114
MAV_CMD_CONDITION_YAW = 115
MAV_CMD_CONDITION_LAST = 159
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_SET_HOME = 179
```

---

## Configuration

### Parameter Management

#### Reading Parameters

```python
# Get single parameter
param_value = uav.get_param("MPC_XY_CRUISE")

# Get multiple parameters
params = uav.get_params([
    "MPC_XY_CRUISE",
    "MPC_Z_VEL_MAX_UP",
    "COM_RC_LOSS_T"
])
```

#### Writing Parameters

```python
# Set parameter
uav.set_param("MPC_XY_CRUISE", 8.0)

# Set multiple parameters
uav.set_params({
    "MPC_XY_CRUISE": 8.0,
    "MPC_Z_VEL_MAX_UP": 3.0,
    "COM_RC_LOSS_T": 1.0
})

# Save to persistent storage
uav.save_params()
```

### Calibration

```python
# Accelerometer calibration
uav.calibrate_accel()

# Compass calibration
uav.calibrate_compass()

# Gyro calibration
uav.calibrate_gyro()

# Level calibration
uav.calibrate_level()

# ESC calibration
uav.calibrate_esc()

# Barometer calibration
uav.calibrate_baro()
```

### Airframe Configuration

```python
# Configure for quadcopter
uav.configure_airframe("quad_x")

# Configure for hexacopter
uav.configure_airframe("hex_x")

# Configure for fixed-wing
uav.configure_airframe("fixed_wing")

# Configure for VTOL
uav.configure_airframe("vtol_quad_tailsitter")
```

### Sensor Configuration

```python
# GPS configuration
uav.configure_gps(
    type="ublox",
    baudrate=115200,
    protocol="nmea"
)

# Compass configuration
uav.configure_compass(
    primary=0,
    use_gps_yaw=True
)

# Rangefinder configuration
uav.configure_rangefinder(
    type="lidar_lite",
    min_distance=0.2,
    max_distance=40.0
)

# Optical flow configuration
uav.configure_optical_flow(
    type="px4flow",
    fusion_mode=1
)
```

---

## References

### Official Documentation

- **ArduPilot**: https://ardupilot.org/
- **PX4**: https://docs.px4.io/
- **MAVLink**: https://mavlink.io/
- **MAVSDK**: https://mavsdk.mavlink.io/

### Protocol Specifications

- **MAVLink XML Definitions**: https://github.com/mavlink/mavlink/tree/master/message_definitions
- **MAVLink Microservices**: https://mavlink.io/en/services/

### Hardware Compatibility

See `hardware-compatibility.md` for supported flight controllers and peripherals.

---

## Version Information

- **Document Version**: 1.0.0
- **Cule OS Version**: 1.0+
- **ArduPilot Support**: 4.3+
- **PX4 Support**: 1.14+
- **MAVLink Version**: 2.0

---

*This documentation is part of the Cule OS project. For updates and contributions, visit the project repository.*
