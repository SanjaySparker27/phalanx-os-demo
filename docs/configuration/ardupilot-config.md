# ArduPilot Configuration Guide

Complete configuration guide for ArduPilot on the ATHENA H743 PRO flight controller with HEXA X + Pusher configuration.

## Table of Contents
- [Initial Setup](#initial-setup)
- [Frame Configuration](#frame-configuration)
- [RC Configuration](#rc-configuration)
- [Flight Modes](#flight-modes)
- [Motor Configuration](#motor-configuration)
- [Sensor Configuration](#sensor-configuration)
- [Battery Configuration](#battery-configuration)
- [Failsafe Settings](#failsafe-settings)
- [Tuning Parameters](#tuning-parameters)

---

## Initial Setup

### Step 1: Flash Firmware

1. Download the custom firmware: `arducopter_dakefph743pro.apj`
2. Connect FC via USB to computer
3. Open Mission Planner or QGroundControl
4. Go to **Initial Setup** → **Install Firmware**
5. Select **Load custom firmware** → Choose `.apj` file
6. Wait for flash to complete

### Step 2: Load Parameter File

1. Connect FC to Mission Planner
2. Go to **Config/Tuning** → **Full Parameter List**
3. Click **Load from file** → Select `athena_h743pro_params.param`
4. Click **Write Params** 
5. Reboot FC when prompted

---

## Frame Configuration

### HEXA X Frame Setup

```
Parameter: FRAME_CLASS = 2      # Hexa
Parameter: FRAME_TYPE = 13      # X configuration
```

### Motor Layout Verification

Verify motor positions match this diagram:

```
          Front
            ^
            |
       M5       M6
         \     /
          \   /
           \ /
      M3 -- X -- M4
           /|
          / |
         /  |
    M1 ----- M2
        
      [M7 Pusher]
```

---

## RC Configuration

### Channel Mapping

| Channel | Function | Parameter | Notes |
|---------|----------|-----------|-------|
| CH1 | Roll | RC1_MIN/MAX | Right stick horizontal |
| CH2 | Pitch | RC2_MIN/MAX | Right stick vertical |
| CH3 | Throttle | RC3_MIN/MAX | Left stick vertical |
| CH4 | Yaw | RC4_MIN/MAX | Left stick horizontal |
| CH5 | Aux Mode | RC5_OPTION | 2nd mode switch (optional) |
| CH6 | Tuning | RC6_OPTION | In-flight tuning |
| CH7 | Aux | RC7_OPTION | Camera trigger, etc. |
| CH8 | **Mode Switch + Pusher** | RC8_OPTION = 0 | Primary control |

### RC8 Slider Configuration

```
# RC8 (Left Slider) - Controls both mode and pusher motor
RC8_MIN = 1000
RC8_MAX = 2000
RC8_TRIM = 1000
RC8_DZ = 0
RC8_OPTION = 0      # Do Nothing (handled by firmware)

# Mode Switching Threshold
# 0-16% slider = Loiter Mode
# >16% slider = Stabilize Mode

# Pusher Motor Control
SERVO7_FUNCTION = 51    # RCIN8 passthrough
SERVO7_MIN = 1000
SERVO7_MAX = 2000
SERVO7_TRIM = 1000
```

### Calibrate RC

1. Connect transmitter and power on
2. In Mission Planner: **Initial Setup** → **Mandatory Hardware** → **Radio Calibration**
3. Click **Calibrate Radio**
4. Move all sticks and switches through full range
5. Click **Complete** when finished

---

## Flight Modes

### Mode Configuration

```
# Flight mode channel
FLTMODE_CH = 8      # Use RC8 for mode switching

# Available modes (0-6 positions)
FLTMODE1 = 5        # Loiter (POSHOLD)
FLTMODE2 = 5        # Loiter
FLTMODE3 = 0        # Stabilize
FLTMODE4 = 0        # Stabilize
FLTMODE5 = 0        # Stabilize
FLTMODE6 = 0        # Stabilize

# Initial mode on boot
INITIAL_MODE = 5    # Start in Loiter
```

### Mode Descriptions

| Mode | Value | Description | Use Case |
|------|-------|-------------|----------|
| Stabilize | 0 | Manual with self-leveling | Takeoff/landing |
| Alt Hold | 2 | Altitude hold | Hovering |
| Loiter | 5 | GPS position hold | Station keeping |
| RTL | 6 | Return to launch | Emergency |
| Auto | 3 | Waypoint mission | Autonomous flight |
| Land | 9 | Auto landing | Landing sequence |

---

## Motor Configuration

### HEXA X Motor Outputs

```
# Main hex motors
SERVO1_FUNCTION = 33    # Motor1
SERVO2_FUNCTION = 34    # Motor2
SERVO3_FUNCTION = 35    # Motor3
SERVO4_FUNCTION = 36    # Motor4
SERVO5_FUNCTION = 37    # Motor5
SERVO6_FUNCTION = 38    # Motor6

# Pusher motor (on RC8)
SERVO7_FUNCTION = 51    # RCIN8 passthrough
SERVO7_MIN = 1000
SERVO7_MAX = 2000

# Spare outputs
SERVO8_FUNCTION = 0     # Disabled (or gimbal)
```

### Motor Direction Test

1. Remove all propellers
2. Connect battery
3. In Mission Planner: **Initial Setup** → **Optional Hardware** → **Motor Test**
4. Test each motor individually at low throttle
5. Verify rotation direction:
   - CW motors: Clockwise
   - CCW motors: Counter-clockwise
6. If wrong, swap any two motor wires on ESC

---

## Sensor Configuration

### IMU Configuration

```
# Internal IMUs (Dual ICM42688)
INS_ENABLE_MASK = 3     # Both IMUs enabled
INS_GYRO_FILTER = 20    # Gyro filter at 20Hz
INS_ACCEL_FILTER = 20   # Accel filter at 20Hz

# Vibration isolation
INS_HNTCH_ENABLE = 1    # Enable notch filter
INS_HNTCH_FREQ = 80     # Notch frequency (ESC freq)
INS_HNTCH_BW = 20       # Notch bandwidth
```

### Compass Configuration

```
# External compass (on GPS module)
COMPASS_EXTERNAL = 1    # External compass
COMPASS_ORIENT = 0      # Arrow forward
COMPASS_USE = 1         # Use for yaw
COMPASS_AUTODEC = 1     # Auto declination

# Calibration required after installation
```

### Barometer

```
# Internal barometer
BARO_PRIMARY = 0        # Use first barometer
BARO_GND_TEMP = 0       # Auto ground temperature
BARO_ALT_OFFSET = 0     # Altitude offset
```

### GPS Configuration

```
# Serial port for GPS
SERIAL1_PROTOCOL = 5    # GPS
SERIAL1_BAUD = 115      # 115200 baud

# GPS settings
GPS_TYPE = 1            # Auto detect
GPS_AUTO_SWITCH = 1     # Auto switch GPS
GPS_RATE_MS = 200       # 5Hz update rate
GPS_GNSS_MODE = 67      # GPS + GLONASS + Galileo
```

---

## Battery Configuration

### Voltage Monitoring

```
# Battery monitor
BATT_MONITOR = 4        # Analog voltage and current
BATT_VOLT_PIN = 11      # Voltage pin
BATT_CURR_PIN = 10      # Current pin

# Calibration (adjust for your hardware)
BATT_VOLT_MULT = 16.0   # Voltage multiplier
BATT_AMP_PERVLT = 83.3  # Current per volt

# Battery specs
BATT_CAPACITY = 22000   # mAh (adjust to your battery)
BATT_CRT_VOLT = 18.0    # Critical voltage (6S)
BATT_LOW_VOLT = 19.8    # Low voltage (6S)
BATT_FS_LOW_ACT = 2     # RTL on low battery
BATT_FS_CRT_ACT = 1     # Land on critical
```

### Cell Count

```
# 6S LiPo (22.2V nominal)
# Full charge: 25.2V (4.2V/cell)
# Storage: 22.8V (3.8V/cell)
# Empty: 21.0V (3.5V/cell)

BATT_LOW_VOLT = 19.8    # 3.3V per cell
BATT_CRT_VOLT = 18.0    # 3.0V per cell
```

---

## Failsafe Settings

### Radio Failsafe

```
# Throttle failsafe
FS_THR_ENABLE = 1       # Enabled
FS_THR_VALUE = 975      # Trigger below 975 PWM

# Action on failsafe
FS_THR_ENABLE = 1       # RTL then Land

# GCS failsafe
FS_GCS_ENABLE = 1       # RTL on GCS timeout
FS_GCS_TIMEOUT = 5      # 5 second timeout
```

### Battery Failsafe

```
BATT_FS_LOW_ACT = 2     # RTL on low battery
BATT_FS_CRT_ACT = 1     # Land on critical
```

### EKF Failsafe

```
# Position estimation failsafe
FS_EKF_ACTION = 1       # Land
FS_EKF_THRESH = 0.8     # EKF variance threshold
```

---

## Tuning Parameters

### PID Tuning (Start Conservative)

```
# Rate roll/pitch
ATC_RAT_RLL_P = 0.135
ATC_RAT_RLL_I = 0.135
ATC_RAT_RLL_D = 0.0036
ATC_RAT_RLL_FILT = 20

ATC_RAT_PIT_P = 0.135
ATC_RAT_PIT_I = 0.135
ATC_RAT_PIT_D = 0.0036
ATC_RAT_PIT_FILT = 20

# Rate yaw
ATC_RAT_YAW_P = 0.18
ATC_RAT_YAW_I = 0.018
ATC_RAT_YAW_D = 0.0
ATC_RAT_YAW_FILT = 2.5
```

### Angle P Gains

```
ATC_ANG_RLL_P = 4.5
ATC_ANG_PIT_P = 4.5
ATC_ANG_YAW_P = 4.5
```

### Loiter Tuning

```
# Loiter speed
WPNAV_LOIT_SPEED = 1250     # cm/s
WPNAV_LOIT_JERK = 1000      # cm/s/s/s

# Loiter control
LOIT_ACC_MAX = 250          # cm/s/s
LOIT_BRK_ACCEL = 250        # cm/s/s
LOIT_BRK_JERK = 500         # cm/s/s/s
```

### Hexa + Pusher Specific

```
# Pusher motor mixing
MOT_PWM_MIN = 1000
MOT_PWM_MAX = 2000
MOT_SPIN_ARM = 0.05         # 5% on arming
MOT_SPIN_MIN = 0.10         # 10% minimum
MOT_SPIN_MAX = 0.95         # 95% maximum

# Yaw headroom for pusher
MOT_YAW_HEADROOM = 200      # PWM headroom
```

---

## Complete Parameter File

Save as `athena_h743pro_params.param`:

```
# ATHENA H743 PRO Configuration
# HEXA X with Pusher Motor
# RC8 (left slider) controls pusher motor and flight mode switching
# 0-16% slider = Loiter, >16% = Stabilize

# Frame Configuration
FRAME_CLASS,2
FRAME_TYPE,13

# Flight Mode Channel (RC8)
FLTMODE_CH,8

# RC8 Configuration (left slider)
RC8_MIN,1000
RC8_MAX,2000
RC8_TRIM,1000
RC8_REVERSED,0
RC8_DZ,0
RC8_OPTION,0

# Pusher Motor Configuration
SERVO7_FUNCTION,51
SERVO7_MIN,1000
SERVO7_MAX,2000
SERVO7_TRIM,1000
SERVO7_REVERSED,0

# Hexa Motor Configuration (M1-M6)
SERVO1_FUNCTION,33
SERVO2_FUNCTION,34
SERVO3_FUNCTION,35
SERVO4_FUNCTION,36
SERVO5_FUNCTION,37
SERVO6_FUNCTION,38
SERVO8_FUNCTION,0

# Flight Modes (6-position switch fallback)
FLTMODE1,5
FLTMODE2,5
FLTMODE3,0
FLTMODE4,0
FLTMODE5,0
FLTMODE6,0

# Safety Settings
ARMING_CHECK,0
FS_THR_ENABLE,1
FS_THR_VALUE,975

# Battery Settings
BATT_MONITOR,4
BATT_VOLT_PIN,11
BATT_CURR_PIN,10
BATT_VOLT_MULT,16.0
BATT_AMP_PERVLT,83.3

# RC Input
SERIAL5_PROTOCOL,23
SERIAL5_BAUD,115

# Telemetry
SERIAL2_PROTOCOL,2
SERIAL2_BAUD,921

# GPS
SERIAL1_PROTOCOL,5
SERIAL1_BAUD,115

# OSD
SERIAL4_PROTOCOL,42

# Compass (external I2C)
COMPASS_EXTERNAL,1

# Initial Mode (Loiter)
INITIAL_MODE,5

# Throttle Configuration
THR_DZ,100
PILOT_THR_BHV,0

# Logging
LOG_BITMASK,65535
```

---

## Verification Checklist

After configuration, verify:

- [ ] RC channels respond correctly in Mission Planner
- [ ] Mode switches with RC8 slider (0-16% = Loiter, >16% = Stabilize)
- [ ] Pusher motor responds to RC8
- [ ] GPS shows 3D fix
- [ ] Compass heading accurate
- [ ] Battery voltage reading correct
- [ ] All motors spin in correct direction
- [ ] Failsafe triggers on throttle cut
- [ ] All pre-arm checks pass

---

## References

- [ArduPilot HEXA X Frame](https://ardupilot.org/copter/docs/connect-escs-and-motors.html#hexacopter)
- [ArduPilot Parameter List](https://ardupilot.org/copter/docs/parameters.html)
- [Mission Planner Setup](https://ardupilot.org/planner/docs/mission-planner-overview.html)
