# MAVLink Message Reference

## Overview

This reference provides a quick lookup for commonly used MAVLink messages in Cule-OS operations.

## Essential Messages

### System Status Messages

| ID | Message | Rate | Description |
|----|---------|------|-------------|
| 0 | HEARTBEAT | 1 Hz | System presence and type |
| 1 | SYS_STATUS | 1 Hz | System health and battery |
| 2 | SYSTEM_TIME | 1 Hz | UTC time from GPS |
| 4 | PING | On req | Latency check |
| 193 | STATUSTEXT | Event | Status messages |

### Navigation Messages

| ID | Message | Rate | Description |
|----|---------|------|-------------|
| 24 | GPS_RAW_INT | 1 Hz | Raw GPS data |
| 27 | RAW_IMU | 1-8 Hz | Raw IMU readings |
| 29 | SCALED_PRESSURE | 1 Hz | Barometer data |
| 30 | ATTITUDE | 10-50 Hz | Vehicle attitude |
| 31 | ATTITUDE_QUATERNION | 10 Hz | Attitude quaternion |
| 32 | LOCAL_POSITION_NED | 10 Hz | Local position |
| 33 | GLOBAL_POSITION_INT | 1-10 Hz | GPS position |
| 34 | RC_CHANNELS_SCALED | 2 Hz | RC inputs |
| 35 | RC_CHANNELS_RAW | 2 Hz | Raw RC inputs |
| 36 | SERVO_OUTPUT_RAW | 2 Hz | Servo/PWM outputs |
| 74 | VFR_HUD | 4 Hz | Basic flight data |
| 116 | SCALED_IMU2 | 2 Hz | Second IMU |
| 125 | POWER_STATUS | 1 Hz | Power supply status |
| 147 | BATTERY_STATUS | 1 Hz | Battery info |
| 241 | VIBRATION | 1 Hz | IMU vibration levels |
| 242 | HOME_POSITION | 1 Hz | Home location |
| 243 | SET_HOME_POSITION | On req | Set home |

### Extended Status

| ID | Message | Rate | Description |
|----|---------|------|-------------|
| 22 | EXTENDED_SYS_STATE | 1 Hz | VTOL/fixed-wing state |
| 25 | GPS_STATUS | 1 Hz | GPS satellite info |
| 62 | NAV_CONTROLLER_OUTPUT | 2 Hz | Navigation info |
| 125 | POWER_STATUS | 1 Hz | Power rails |
| 147 | BATTERY_STATUS | 1 Hz | Battery cells |
| 253 | STATUSTEXT_LONG | Event | Long messages |

## Message Details

### HEARTBEAT (#0)

```
Frequency: 1 Hz (Required)
Purpose: Indicate system presence

Fields:
type              uint8_t    MAV_TYPE (quad, plane, etc.)
autopilot         uint8_t    MAV_AUTOPILOT (PX4, ArduPilot)
base_mode         uint8_t    System mode bitmask
custom_mode       uint32_t   Autopilot-specific mode
system_status     uint8_t    MAV_STATE
mavlink_version   uint8_t    Protocol version

Example values:
type = MAV_TYPE_QUADROTOR (2)
autopilot = MAV_AUTOPILOT_PX4 (12)
base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED (16)
system_status = MAV_STATE_ACTIVE (4)
```

### SYS_STATUS (#1)

```
Frequency: 1 Hz
Purpose: System health and power

Fields:
onboard_control_sensors_present   uint32_t  Sensors present
onboard_control_sensors_enabled   uint32_t  Sensors enabled
onboard_control_sensors_health    uint32_t  Sensors healthy
load                              uint16_t  CPU load (d%)
voltage_battery                   uint16_t  Battery voltage (mV)
current_battery                   int16_t   Current (cA)
battery_remaining                 int8_t    Remaining %
drop_rate_comm                    uint16_t  Drop rate (d%)
errors_comm                       uint16_t  Comm errors

Sensor health bits:
MAV_SYS_STATUS_SENSOR_3D_GYRO       (1)
MAV_SYS_STATUS_SENSOR_3D_ACCEL      (2)
MAV_SYS_STATUS_SENSOR_3D_MAG        (4)
MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE (8)
MAV_SYS_STATUS_SENSOR_GPS           (32)
MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW  (64)
MAV_SYS_STATUS_VISION_POSITION      (128)
MAV_SYS_STATUS_SENSOR_LASER_POSITION (256)
MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH (512)
MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL (1024)
MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION (2048)
MAV_SYS_STATUS_SENSOR_YAW_POSITION (4096)
MAV_SYS_STATUS_SENSOR_ALTITUDE_CONTROL (8192)
```

### ATTITUDE (#30)

```
Frequency: 10-50 Hz
Purpose: Vehicle orientation

Fields:
time_boot_ms    uint32_t    Timestamp (ms since boot)
roll            float       Roll angle (rad)
pitch           float       Pitch angle (rad)
yaw             float       Yaw angle (rad)
rollspeed       float       Roll rate (rad/s)
pitchspeed      float       Pitch rate (rad/s)
yawspeed        float       Yaw rate (rad/s)

Example:
roll = 0.05 (2.9 degrees)
pitch = -0.10 (-5.7 degrees)
yaw = 1.57 (90 degrees, east)
```

### GLOBAL_POSITION_INT (#33)

```
Frequency: 1-10 Hz
Purpose: Global position and velocity

Fields:
time_boot_ms    uint32_t    Timestamp (ms)
lat             int32_t     Latitude (degE7)
lon             int32_t     Longitude (degE7)
alt             int32_t     Altitude MSL (mm)
relative_alt    int32_t     Altitude above home (mm)
vx              int16_t     Ground X speed (cm/s)
vy              int16_t     Ground Y speed (cm/s)
vz              int16_t     Ground Z speed (cm/s)
hdg             uint16_t    Heading (cdeg)

Example:
lat = 336452000 (33.6452000° N)
lon = -1178425000 (-117.8425000° W)
alt = 100000 (100m MSL)
relative_alt = 50000 (50m above home)
```

### BATTERY_STATUS (#147)

```
Frequency: 1 Hz
Purpose: Detailed battery information

Fields:
id                  uint8_t     Battery ID
temperature         int16_t     Temperature (cdegC)
voltages            uint16_t[10] Cell voltages (mV)
current_battery     int16_t     Current (cA)
current_consumed    int32_t     Consumed (mAh)
energy_consumed     int32_t     Energy (hJ)
battery_remaining   int8_t      Remaining %
time_remaining      int32_t     Seconds remaining
charge_state        uint8_t     MAV_BATTERY_CHARGE_STATE

Charge states:
MAV_BATTERY_CHARGE_STATE_UNDEFINED = 0
MAV_BATTERY_CHARGE_STATE_OK = 1
MAV_BATTERY_CHARGE_STATE_LOW = 2
MAV_BATTERY_CHARGE_STATE_CRITICAL = 3
MAV_BATTERY_CHARGE_STATE_EMERGENCY = 4
MAV_BATTERY_CHARGE_STATE_FAILED = 5
MAV_BATTERY_CHARGE_STATE_UNHEALTHY = 6
MAV_BATTERY_CHARGE_STATE_CHARGING = 7
```

### VIBRATION (#241)

```
Frequency: 1 Hz
Purpose: IMU vibration monitoring

Fields:
time_usec       uint64_t    Timestamp (microseconds)
vibration_x     float       X-axis vibration level
vibration_y     float       Y-axis vibration level
vibration_z     float       Z-axis vibration level
clipping_0      uint32_t    Clip count IMU 0
clipping_1      uint32_t    Clip count IMU 1
clipping_2      uint32_t    Clip count IMU 2

Good values: < 30 m/s/s for X/Y, < 15 m/s/s for Z
```

## Command Messages

### COMMAND_LONG (#76)

```
Purpose: Send commands to vehicle

Fields:
target_system       uint8_t     Target system ID
target_component    uint8_t     Target component
command             uint16_t    Command ID
confirmation        uint8_t     0=first, 1+=retry
param1-7            float       Command parameters

Common commands:
ARM_DISARM (400):
  param1: 1=arm, 0=disarm, 21196=force disarm

DO_SET_MODE (176):
  param1: Mode base
  param2: Custom mode
  param3: Submode

NAV_TAKEOFF (22):
  param1: Min pitch
  param7: Altitude

NAV_LAND (21):
  param4: Desired yaw angle
  param5: Latitude
  param6: Longitude
  param7: Altitude

PREFLIGHT_CALIBRATION (241):
  param1: Gyro
  param2: Magnetometer
  param3: Pressure
  param4: Radio
  param5: Accelerometer
  param6: Compass/Motor
  param7: Airspeed

REQUEST_AUTOPILOT_CAPABILITIES (520):
  param1: 1=request
```

### COMMAND_ACK (#77)

```
Purpose: Acknowledge command receipt

Fields:
command     uint16_t    Command being acknowledged
result      uint8_t     MAV_RESULT
progress    uint8_t     Progress percentage (optional)
result_param2   int32_t Result-specific
target_system   uint8_t Target system

Results:
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_TEMPORARILY_REJECTED = 1
MAV_RESULT_DENIED = 2
MAV_RESULT_UNSUPPORTED = 3
MAV_RESULT_FAILED = 4
MAV_RESULT_IN_PROGRESS = 5
```

## Mission Messages

### MISSION_ITEM_INT (#73)

```
Purpose: Single waypoint definition

Fields:
target_system       uint8_t
seq                 uint16_t    Sequence number
frame               uint8_t     MAV_FRAME
command             uint16_t    MAV_CMD
current             uint8_t     1=current WP
autocontinue        uint8_t     Auto-continue
param1-4            float       Command params
x                   int32_t     Latitude (degE7)
y                   int32_t     Longitude (degE7)
z                   float       Altitude (m)

Coordinate frames:
MAV_FRAME_GLOBAL = 0              # Global, alt MSL
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3 # Global, alt relative
MAV_FRAME_LOCAL_NED = 1           # Local coordinates
```

### MISSION_COUNT (#44)

```
Purpose: Announce number of mission items

Fields:
target_system       uint8_t
count               uint16_t    Number of items
mission_type        uint8_t     0=flight plan
```

## Parameter Messages

### PARAM_VALUE (#22)

```
Purpose: Parameter value report

Fields:
param_id        char[16]    Parameter name
param_value     float       Value
param_type      uint8_t     MAV_PARAM_TYPE
param_count     uint16_t    Total parameters
param_index     uint16_t    This parameter index

Types:
MAV_PARAM_TYPE_UINT8 = 1
MAV_PARAM_TYPE_INT8 = 2
MAV_PARAM_TYPE_UINT16 = 3
MAV_PARAM_TYPE_INT16 = 4
MAV_PARAM_TYPE_UINT32 = 5
MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_UINT64 = 7
MAV_PARAM_TYPE_INT64 = 8
MAV_PARAM_TYPE_REAL32 = 9
MAV_PARAM_TYPE_REAL64 = 10
```

### PARAM_SET (#23)

```
Purpose: Set parameter value

Fields:
target_system       uint8_t
target_component    uint8_t
param_id            char[16]
param_value         float
param_type          uint8_t
```

## Custom Cule-OS Messages

### Extended Messages

Cule-OS may implement vendor-specific extensions:

```
#33000-33100 reserved for Cule-OS custom messages

Example extensions:
- Extended sensor data
- Custom telemetry
- Proprietary status
- Debug information
```

### Message Rate Guidelines

```
Recommended stream rates for different links:

Telemetry Radio (57600 baud):
- ATTITUDE: 4 Hz
- GLOBAL_POSITION_INT: 1 Hz
- SYS_STATUS: 1 Hz
- BATTERY_STATUS: 1 Hz

WiFi/Ethernet (high bandwidth):
- ATTITUDE: 50 Hz
- GLOBAL_POSITION_INT: 10 Hz
- LOCAL_POSITION_NED: 50 Hz
- All other: 10 Hz

USB (debug):
- ATTITUDE: 250 Hz
- RAW_IMU: 200 Hz
- DEBUG_VECT: 50 Hz
```

## Reference

- [MAVLink Message IDs](https://mavlink.io/en/messages/common.html)
- [MAVLink XML Definitions](https://github.com/mavlink/mavlink/tree/master/message_definitions/v1.0)
