# Pixhawk Setup

Complete guide for configuring Pixhawk flight controllers with Cule OS.

## Supported Pixhawk Models

| Model | Status | Features |
|-------|--------|----------|
| Pixhawk 6X | ✅ Full | Triple IMU, dual CAN, Ethernet |
| Pixhawk 6C | ✅ Full | Single IMU, cost-effective |
| Pixhawk 5X | ✅ Full | Triple IMU, vibration isolation |
| Pixhawk 4 | ✅ Full | Legacy, widely used |
| Pixhawk 4 Mini | ✅ Full | Compact version |
| Cube Orange+ | ✅ Full | ProfiCNC design, triple IMU |
| Cube Orange | ✅ Full | Proven design |

## Pixhawk 6X

### Hardware Overview

```
Pixhawk 6X Block Diagram
══════════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────────┐
│                        STM32H753                                │
│                    ARM Cortex-M7 @ 480MHz                       │
│                        2MB Flash                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  IMU (Triple Redundant)                                         │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐            │
│  │ ICM-20649    │ │ ICM-42688-P  │ │ ICM-45686    │            │
│  │ (Primary)    │ │ (Secondary)  │ │ (Tertiary)   │            │
│  └──────────────┘ └──────────────┘ └──────────────┘            │
│                                                                  │
│  Barometer: DPS310 (I2C)                                        │
│  Magnetometer: BMM150 (I2C)                                     │
│  FRAM: 2MB (non-volatile storage)                               │
│                                                                  │
│  Interfaces:                                                    │
│  ├─ 16x PWM Outputs (Main + Aux)                                │
│  ├─ 5x UART (TELEM1-4, GPS)                                     │
│  ├─ 3x I2C                                                      │
│  ├─ 2x CAN FD                                                   │
│  ├─ 1x Ethernet (100M)                                         │
│  ├─ 1x SPI (external)                                          │
│  ├─ 1x USB-C                                                   │
│  └─ 1x SD Card                                                 │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### Connector Pinout Details

#### Main Output (J1 - 10-pin)

| Pin | Color | Signal | Default | Protocol |
|-----|-------|--------|---------|----------|
| 1 | Red | VCC | 5V | - |
| 2 | Black | GND | Ground | - |
| 3 | White | PWM1 | Motor 1 | DShot/PWM |
| 4 | White | PWM2 | Motor 2 | DShot/PWM |
| 5 | White | PWM3 | Motor 3 | DShot/PWM |
| 6 | White | PWM4 | Motor 4 | DShot/PWM |
| 7 | White | PWM5 | Motor 5/Servo | DShot/PWM |
| 8 | White | PWM6 | Motor 6/Servo | DShot/PWM |
| 9 | White | PWM7 | Motor 7/Servo | DShot/PWM |
| 10 | White | PWM8 | Motor 8/Servo | DShot/PWM |

#### Auxiliary Output (J2 - 8-pin)

| Pin | Color | Signal | Default | Protocol |
|-----|-------|--------|---------|----------|
| 1 | Red | VCC | 5V | - |
| 2 | Black | GND | Ground | - |
| 3 | White | AUX1 | Servo/GPIO | PWM |
| 4 | White | AUX2 | Servo/GPIO | PWM |
| 5 | White | AUX3 | Servo/GPIO | PWM |
| 6 | White | AUX4 | Servo/GPIO | PWM |
| 7 | White | AUX5 | Servo/GPIO | PWM |
| 8 | White | AUX6 | Servo/GPIO | PWM |

#### GPS1 Port (J3 - 6-pin JST-GH)

| Pin | Color | Signal | Direction |
|-----|-------|--------|-----------|
| 1 | Red | 5V | Power out |
| 2 | Black | GND | Ground |
| 3 | Green | TX | FC → GPS |
| 4 | Yellow | RX | GPS → FC |
| 5 | Blue | SCL | I2C Clock |
| 6 | White | SDA | I2C Data |

#### TELEM1/2/3/4 (6-pin JST-GH)

| Pin | Color | Signal | Notes |
|-----|-------|--------|-------|
| 1 | Red | 5V | Power out |
| 2 | Black | GND | Ground |
| 3 | Green | TX | FC → Device |
| 4 | Yellow | RX | Device → FC |
| 5 | Blue | CTS | Flow control |
| 6 | White | RTS | Flow control |

## Firmware Installation

### ArduPilot Installation

```bash
# Method 1: Web-based (Recommended)
# 1. Connect Pixhawk via USB
# 2. Open https://firmware.ardupilot.org
# 3. Select vehicle type (Copter/Plane/Rover)
# 4. Click "Upload Firmware"

# Method 2: Command line
# Download firmware for your board
wget https://firmware.ardupilot.org/Copter/stable/Pixhawk6X/arducopter.apj

# Upload using mavproxy
mavproxy.py --master=/dev/ttyACM0
# In MAVProxy console:
flash arducopter.apj

# Method 3: QGroundControl
# 1. Open QGC
# 2. Connect Pixhawk
# 3. Go to Firmware tab
# 4. Select ArduPilot Stable
# 5. Follow prompts
```

### PX4 Installation (Alternative)

```bash
# PX4 is supported but ArduPilot recommended for Cule OS

# Download PX4 firmware
wget https://github.com/PX4/PX4-Autopilot/releases/download/v1.14.0/pixhawk6x_default.px4

# Flash using QGroundControl
# Or command line:
px4-uploader pixhawk6x_default.px4
```

## Initial Configuration

### Frame Type Selection

```bash
# Connect via MAVProxy or QGC
mavproxy.py --master=/dev/ttyACM0

# Set frame class (multicopter)
param set FRAME_CLASS 1  # Quad
param set FRAME_CLASS 2  # Hexa
param set FRAME_CLASS 3  # Octa
param set FRAME_CLASS 4  # OctaQuad

# Set frame type (motor arrangement)
param set FRAME_TYPE 1   # X configuration
param set FRAME_TYPE 0   # + configuration

# For custom frame
param set FRAME_CLASS 15  # Scripting frame
```

### Serial Port Configuration

```bash
# TELEM1 - Companion Computer
param set SERIAL1_BAUD 921  # 921600
param set SERIAL1_PROTOCOL 2  # MAVLink2

# TELEM2 - Telemetry Radio
param set SERIAL2_BAUD 57   # 57600
param set SERIAL2_PROTOCOL 2  # MAVLink2

# GPS1
param set SERIAL3_BAUD 115  # 115200
param set SERIAL3_PROTOCOL 5  # GPS

# TELEM3 - Peripherals
param set SERIAL4_BAUD 57
param set SERIAL4_PROTOCOL 2

# TELEM4 - OSD/Display
param set SERIAL5_BAUD 57
param set SERIAL5_PROTOCOL 1  # MAVLink1
```

## Calibration

### Accelerometer Calibration

```bash
# Using Cule OS
cule-calibrate accel

# Or manually via MAVProxy
# Place vehicle on each of 6 faces:
# 1. Level
# 2. Left side down
# 3. Right side down
# 4. Nose down
# 5. Nose up
# 6. Upside down

# Command:
accelcal
```

### Compass Calibration

```bash
# Using Cule OS
cule-calibrate compass

# Or manually
# 1. Hold level, rotate 360°
# 2. Nose up 45°, rotate 360°
# 3. Nose down 45°, rotate 360°
# 4. Left side down, rotate 360°
# 5. Right side down, rotate 360°

# Check calibration quality
param show COMPASS_OFS*
# Values should be < 500
```

### Radio Calibration

```bash
# Connect RC receiver to RCIN port
# Open radio calibration in QGC

# Or via MAVProxy
calibrate rc

# Move all sticks and switches through full range
# Verify endpoints in QGC
```

### ESC Calibration

```bash
# For PWM ESCs
param set MOT_PWM_TYPE 0  # Normal PWM

# Calibration procedure:
# 1. Remove props
# 2. Power on transmitter, throttle to max
# 3. Power on vehicle
# 4. Wait for beeps
# 5. Lower throttle to minimum
# 6. Wait for confirmation beeps

# For DShot (no calibration needed)
param set MOT_PWM_TYPE 6  # DShot600
param set MOT_PWM_TYPE 7  # DShot1200
```

## Advanced Configuration

### PID Tuning

```bash
# Default PID values usually work well
# For tuning, use Cule OS autotune:
cule-autotune start

# Or manual adjustment
# Rate PIDs
param set ATC_ANG_RLL_P 6.0   # Roll angle P
param set ATC_RAT_RLL_P 0.135 # Roll rate P
param set ATC_RAT_RLL_I 0.135 # Roll rate I
param set ATC_RAT_RLL_D 0.0036 # Roll rate D

param set ATC_ANG_PIT_P 6.0   # Pitch angle P
param set ATC_RAT_PIT_P 0.135 # Pitch rate P
param set ATC_RAT_PIT_I 0.135 # Pitch rate I
param set ATC_RAT_PIT_D 0.0036 # Pitch rate D

param set ATC_ANG_YAW_P 6.0   # Yaw angle P
param set ATC_RAT_YAW_P 0.18  # Yaw rate P
param set ATC_RAT_YAW_I 0.018 # Yaw rate I
```

### Notch Filtering

```bash
# Enable harmonic notch filter
param set INS_HNTCH_ENABLE 1

# Set base frequency (motor frequency at hover)
# Example: 4S quad with 2300KV motors, 5" props
# Hover throttle ~40%, frequency ~120Hz
param set INS_HNTCH_FREQ 120

# Bandwidth
param set INS_HNTCH_BW 60

# Attenuation
param set INS_HNTCH_ATT 40

# Reference throttle
param set INS_HNTCH_REF 0.4

# For dynamic notch
param set INS_HNTCH_MODE 1  # Throttle based
param set INS_HNTCH_MODE 4  # RPM based (requires ESC telemetry)
```

## Safety Configuration

### Arming Checks

```bash
# Enable all arming checks
param set ARMING_CHECK 1  # All checks

# Or select specific checks
param set ARMING_CHECK 72  # Baro + Compass + GPS

# Individual check bits:
# 1 = All
# 2 = Barometer
# 4 = Compass
# 8 = GPS
# 16 = INS (IMU)
# 32 = Parameters
# 64 = RC
# 128 = Board voltage
# 256 = Battery level
# 512 = Airspeed
# 1024 = Logging available
# 2048 = Hardware safety switch
# 4096 = GPS configuration
# 8192 = System
```

### Failsafe Settings

```bash
# RC Failsafe
param set FS_THR_ENABLE 1  # RTL on RC loss
param set FS_THR_ENABLE 2  # Continue mission
param set FS_THR_ENABLE 3  # Land

# GCS Failsafe
param set FS_GCS_ENABLE 1  # RTL
param set FS_GCS_ENABLE 2  # Continue
param set FS_GCS_ENABLE 3  # Land

# Battery Failsafe
param set BATT_FS_LOW_ACT 2  # RTL
param set BATT_FS_CRT_ACT 1  # Land

# EKF Failsafe
param set FS_EKF_ACTION 1  # Land
param set FS_EKF_THRESH 0.8  # Tolerance
```

## Cule OS Integration

### MAVLink Connection

```bash
# Cule OS auto-detects Pixhawk
# Verify connection
cule-status

# Output:
# Flight Controller: Pixhawk 6X
# Firmware: ArduCopter 4.4.4
# Connection: MAVLink2 @ 921600 baud
# Heartbeat: OK

# View live telemetry
cule-telemetry

# Access MAVLink shell
cule-mavlink-shell
```

### Python API

```python
from cule import FlightController

# Connect to Pixhawk
fc = FlightController.connect('/dev/ttyACM0', baud=921600)

# Get vehicle state
state = fc.get_state()
print(f"Armed: {state.armed}")
print(f"Mode: {state.mode}")
print(f"Battery: {state.battery_voltage}V")

# Arm vehicle
fc.arm()

# Change mode
fc.set_mode('LOITER')
fc.set_mode('GUIDED')

# Send velocity command
fc.set_velocity(vx=1.0, vy=0.0, vz=0.0)

# Takeoff
fc.takeoff(altitude=10)

# Navigate to waypoint
fc.goto(lat=-35.363261, lon=149.165230, alt=20)

# Return to launch
fc.rtl()

# Land
fc.land()

# Disarm
fc.disarm()
```

## Troubleshooting

### Boot Issues

```bash
# Pixhawk not detected
ls /dev/ttyACM*
# Should show /dev/ttyACM0

# Check USB connection
dmesg | grep -i "usb.*acm"

# Force bootloader mode
# Connect USB while holding safety button

# Check for solid LED on boot
# No LED = power issue
# Flashing red = boot error
```

### Sensor Issues

```bash
# Check IMU health
cule-fc-status --sensors

# IMU inconsistency
# Solution: Recalibrate accelerometer

# Compass variance
param show COMPASS_OFS*
# Values > 500 indicate problem
# Solution: Recalibrate away from metal

# GPS no fix
# Ensure clear view of sky
# Wait 2-3 minutes for cold start
# Check antenna connection
```

### Parameter Reset

```bash
# Reset all parameters to default
param reset
reboot

# Or reset specific groups
param reset INS_  # IMU params
param reset ATC_  # Attitude control
param reset COMPASS_  # Compass
```

## See Also

- [Flight Controllers](./flight-controllers.md) - General FC overview
- [DAKEFPV H743 Pro](./dakefpv-h743.md) - Alternative FC
- [MAVLink Setup](./mavlink.md) - Protocol configuration
- [Tuning](./tuning.md) - Advanced tuning guide
