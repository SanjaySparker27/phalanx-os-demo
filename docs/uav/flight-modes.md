# Flight Modes Guide

Complete guide to flight modes available on the ATHENA H743 PRO with custom firmware.

## Table of Contents
- [Overview](#overview)
- [Flight Mode Reference](#flight-mode-reference)
- [Mode Switching Logic](#mode-switching-logic)
- [Mode Characteristics](#mode-characteristics)
- [Recommended Mode Progression](#recommended-mode-progression)
- [Mode-Specific Tuning](#mode-specific-tuning)

---

## Overview

The ATHENA H743 PRO supports multiple flight modes optimized for different operational scenarios. With the custom firmware, flight mode switching is controlled via **RC8 (left slider)** with automatic switching at the 16% threshold.

### Primary Modes

| Mode | ID | Description | Use Case |
|------|-----|-------------|----------|
| **Stabilize** | 0 | Manual flight with self-leveling | Acrobatic flying, manual control |
| **Alt Hold** | 2 | Altitude hold with manual horizontal control | Hovering, filming |
| **Loiter** | 5 | GPS position and altitude hold | Station keeping, photography |
| **RTL** | 6 | Return to launch position | Emergency recovery |
| **Auto** | 3 | Autonomous waypoint navigation | Surveys, mapping |
| **Land** | 9 | Automatic landing sequence | Precision landing |
| **Circle** | 7 | Orbit around point of interest | Inspection, filming |
| **Drift** | 11 | Coordinated turns | Smooth cinematic flight |
| **Sport** | 13 | Aggressive manual mode | Fast acrobatic flight |
| **Guided** | 4 | Position control via GCS/Companion | Autonomous operations |
| **Follow** | 6 | Follow a target | Dynamic tracking |

---

## Flight Mode Reference

### Stabilize (Mode 0)

**Full manual control with angle stabilization.**

```
RC Input → Angle Rate → Motor Mix
```

**Characteristics:**
- Pilot controls roll/pitch angles directly
- Self-leveling when sticks centered
- No altitude hold (throttle controls thrust directly)
- No position hold
- Maximum pilot authority

**Best For:**
- Acrobatic flying
- Aggressive maneuvers
- Manual takeoff/landing
- Training (with experienced pilot)

**Parameters:**
```
ATC_ANG_RLL_P = 4.5     # Roll angle P gain
ATC_ANG_PIT_P = 4.5     # Pitch angle P gain
ATC_ANG_YAW_P = 4.5     # Yaw angle P gain
```

**Usage with Custom Firmware:**
```
RC8 Slider > 16% → Stabilize Mode + Pusher Active
```

---

### Alt Hold (Mode 2)

**Altitude hold with manual horizontal control.**

**Characteristics:**
- Maintains altitude automatically
- Pilot controls roll/pitch for horizontal movement
- Yaw control is manual
- Throttle stick centered = hold current altitude

**Best For:**
- Hovering in place
- Aerial photography
- Smooth altitude transitions
- Learning orientation without altitude management

**Parameters:**
```
PILOT_THR_BHV = 0       # Throttle behavior (0 = center = alt hold)
THR_DZ = 100            # Throttle deadzone
PSC_ACCZ_P = 0.3        # Altitude hold P gain
PSC_ACCZ_I = 0.5        # Altitude hold I gain
```

**PID Tuning:**
```
# If altitude oscillates:
PSC_ACCZ_P = 0.2        # Reduce P
PSC_ACCZ_I = 0.3        # Reduce I

# If slow to respond:
PSC_ACCZ_P = 0.5        # Increase P
PSC_ACCZ_I = 0.8        # Increase I
```

---

### Loiter (Mode 5)

**GPS position and altitude hold.**

**Characteristics:**
- Maintains position using GPS
- Maintains altitude using barometer
- Pilot controls can nudge position
- Automatically compensates for wind

**Best For:**
- Station keeping
- Long-duration hovering
- Aerial photography/videography
- Waiting for commands

**Requirements:**
- GPS 3D Fix (8+ satellites recommended)
- HDOP < 2.0
- Compass calibrated and healthy

**Parameters:**
```
WPNAV_LOIT_SPEED = 1250     # Maximum loiter speed (cm/s)
WPNAV_LOIT_JERK = 1000      # Jerk limit (cm/s/s/s)
LOIT_ACC_MAX = 250          # Maximum acceleration (cm/s/s)
LOIT_BRK_ACCEL = 250        # Braking acceleration (cm/s/s)
LOIT_BRK_JERK = 500         # Braking jerk (cm/s/s/s)
LOIT_BRK_DELAY = 1          # Braking delay (seconds)
```

**Tuning for Wind:**
```
# High wind conditions:
WPNAV_LOIT_SPEED = 1500     # Allow faster corrections
LOIT_ACC_MAX = 400          # More aggressive acceleration
LOIT_BRK_ACCEL = 400        # Stronger braking
```

**Usage with Custom Firmware:**
```
RC8 Slider 0-16% → Loiter Mode (Pusher Idle)
```

---

### RTL - Return to Launch (Mode 6)

**Automatic return to home position.**

**Characteristics:**
- Flies back to takeoff position
- Climbs to RTL_ALT first, then returns
- Lands automatically at home
- Can be interrupted by mode change

**Best For:**
- Emergency recovery
- Low battery return
- Lost orientation recovery
- End of mission

**Parameters:**
```
RTL_ALT = 1500              # RTL altitude (15 meters)
RTL_ALT_FINAL = 0           # Final approach altitude (0 = land)
RTL_SPEED = 0               # RTL speed (0 = WPNAV_SPEED)
RTL_CONE_SLOPE = 3          # RTL cone slope
RTL_CLIMB_MIN = 0           # Minimum climb before return
RTL_LOIT_TIME = 5000        # Loiter time before land (ms)
```

**Behavior Flow:**
```
1. Climb to RTL_ALT (if below)
2. Turn toward home position
3. Fly to home at RTL_SPEED
4. Loiter at home for RTL_LOIT_TIME
5. Land (if RTL_ALT_FINAL = 0)
```

**Safety Notes:**
- Ensure RTL_ALT is above all obstacles
- RTL uses GPS - verify GPS health before relying on it
- Can be overridden by switching to another mode

---

### Auto (Mode 3)

**Autonomous waypoint mission execution.**

**Characteristics:**
- Executes pre-programmed mission
- Follows waypoints in sequence
- Supports multiple command types
- Can be paused/resumed

**Best For:**
- Survey missions
- Mapping
- Automated inspections
- Repeatable flight patterns

**Mission Commands:**
```
NAV_WAYPOINT        - Fly to position
NAV_LOITER_TIME     - Loiter for time
NAV_LOITER_TURNS    - Orbit waypoint
NAV_RETURN_TO_LAUNCH - Return home
NAV_LAND            - Land at position
NAV_TAKEOFF         - Takeoff to altitude
NAV_SPLINE_WAYPOINT - Spline path waypoint
NAV_GUIDED_ENABLE   - Enable guided mode
NAV_DELAY           - Delay execution
NAV_PAYLOAD_PLACE   - Payload placement
DO_JUMP             - Jump to waypoint
DO_CHANGE_SPEED     - Change speed
DO_SET_ROI          - Set region of interest
DO_MOUNT_CONTROL    - Control camera gimbal
```

**Parameters:**
```
WPNAV_SPEED = 1000          # Default waypoint speed (cm/s)
WPNAV_SPEED_UP = 250        # Climb speed (cm/s)
WPNAV_SPEED_DN = 150        # Descent speed (cm/s)
WPNAV_ACCEL = 100           # Acceleration (cm/s/s)
WPNAV_ACCEL_Z = 100         # Vertical acceleration
WPNAV_RFND_USE = 1          # Use rangefinder for terrain
```

---

### Land (Mode 9)

**Automatic landing sequence.**

**Characteristics:**
- Descends at LAND_SPEED
- Levels aircraft before touchdown
- Disarms after landing
- Can be interrupted

**Best For:**
- Precision landings
- End of flight
- Battery conservation

**Parameters:**
```
LAND_SPEED = 50             # Landing speed (cm/s)
LAND_ALT_LOW = 1000         # Low altitude threshold (cm)
LAND_REPOSITION = 0         # Allow repositioning during land
PILOT_THR_BHV = 0           # Throttle behavior
```

**Behavior:**
```
1. Descend at LAND_SPEED
2. Below LAND_ALT_LOW: Slow to half speed
3. Detect touchdown (baro + accel)
4. Disarm motors after LAND_DISARMDELAY
```

---

### Circle (Mode 7)

**Orbit around point of interest.**

**Characteristics:**
- Flies circle around target point
- Adjustable radius and speed
- Can orbit current position or specified point

**Best For:**
- Point inspection
- Cinematic shots
- Surveillance

**Parameters:**
```
CIRCLE_RADIUS = 1000        # Circle radius (cm)
CIRCLE_RATE = 20            # Rotation rate (deg/s)
CIRCLE_CONTROL = 0          # Control behavior
```

**Controls:**
```
Roll stick: Change radius
Pitch stick: Change altitude
Yaw stick: Face direction
```

---

### Guided (Mode 4)

**Position control via ground station or companion computer.**

**Characteristics:**
- Accepts position/velocity setpoints
- Used by autonomous systems
- Supports guided takeoff/landing

**Best For:**
- Companion computer control
- Offboard processing
- Dynamic mission updates
- Cule OS integration

**MAVLink Commands:**
```python
# Set position target
SET_POSITION_TARGET_LOCAL_NED
SET_POSITION_TARGET_GLOBAL_INT

# Set attitude target
SET_ATTITUDE_TARGET

# Set velocity
SET_POSITION_TARGET_LOCAL_NED (type_mask for velocity)
```

**Cule OS Integration:**
```python
from cule_os.agents import ControlAgent

control = ControlAgent()
control.set_setpoint(
    position=[10, 5, -5],    # x, y, z (NED frame)
    velocity=[2, 0, 0],
    yaw=0
)
```

---

### Sport (Mode 13)

**Aggressive manual mode with rate control.**

**Characteristics:**
- Angle-limited acrobatic mode
- Rate-based control
- Altitude hold optional
- Fast response

**Best For:**
- Fast flying
- Sport aerobatics
- Experienced pilots

**Parameters:**
```
ACRO_RP_P = 4.5             # Rate P gain
ACRO_YAW_P = 4.5            # Yaw rate P
ACRO_BAL_PITCH = 1.0        # Balance pitch
ACRO_BAL_ROLL = 1.0         # Balance roll
ACRO_TRAINER = 0            # Training limits (0 = unlimited)
```

---

## Mode Switching Logic

### Custom Firmware Mode Switch

The custom firmware implements intelligent mode switching on **RC8 (left slider)**:

```cpp
// Mode switching threshold
if (slider_percent <= 16) {
    mode = LOITER;      // GPS position hold
    pusher = IDLE;      // Pusher motor off
} else {
    mode = STABILIZE;   // Manual control
    pusher = ACTIVE;    // Pusher motor responds to throttle
}
```

### Switching Behavior

| Slider Position | Flight Mode | Pusher State | Use Case |
|----------------|-------------|--------------|----------|
| 0-16% | Loiter | Idle | Hover, position hold |
| 16-100% | Stabilize | Active | Manual flight, forward thrust |

### Debouncing

The firmware includes 100ms debouncing to prevent rapid mode switches:

```cpp
static uint32_t last_check_ms = 0;
if (now_ms - last_check_ms < 100) return;  // 10Hz max
```

---

## Mode Characteristics

### Control Authority Matrix

| Mode | Roll/Pitch | Throttle | Yaw | Position | Altitude |
|------|------------|----------|-----|----------|----------|
| Stabilize | Angle | Thrust | Rate | No | No |
| Alt Hold | Angle | Altitude | Rate | No | Yes |
| Loiter | Velocity | Altitude | Rate | Yes | Yes |
| RTL | Auto | Auto | Auto | Yes | Yes |
| Auto | Auto | Auto | Auto | Yes | Yes |
| Guided | Velocity/Pos | Altitude | Rate/Pos | Yes | Yes |
| Land | Auto | Auto | Rate | No | Auto |
| Circle | Velocity | Altitude | Rate | Yes (orbit) | Yes |

### Required Sensors

| Mode | Gyro | Accel | Compass | GPS | Baro |
|------|------|-------|---------|-----|------|
| Stabilize | ✓ | ✓ | - | - | - |
| Alt Hold | ✓ | ✓ | - | - | ✓ |
| Loiter | ✓ | ✓ | ✓ | ✓ | ✓ |
| RTL | ✓ | ✓ | ✓ | ✓ | ✓ |
| Auto | ✓ | ✓ | ✓ | ✓ | ✓ |
| Guided | ✓ | ✓ | ✓ | ✓ | ✓ |
| Land | ✓ | ✓ | - | - | ✓ |
| Circle | ✓ | ✓ | ✓ | ✓ | ✓ |

---

## Recommended Mode Progression

### For New Pilots

```
1. Stabilize (manual) - Learn basic orientation
2. Alt Hold - Add altitude management
3. Loiter - Add position hold
4. RTL - Emergency recovery
5. Auto - Waypoint missions
```

### For Experienced Pilots

```
1. Loiter - GPS operations
2. Guided - Companion computer
3. Auto - Autonomous missions
4. Circle - Inspection
5. Sport - Fast maneuvers
```

### VTOL Specific (HEXA + Pusher)

```
Takeoff:  Loiter → Climb to altitude
Cruise:   Stabilize (>16%) → Forward flight with pusher
Hover:    Loiter (<16%) → Position hold
Landing:  Loiter → Descend → Land mode
Emergency: RTL (any time)
```

---

## Mode-Specific Tuning

### Loiter Tuning

```
# Smooth loiter (cinematics)
WPNAV_LOIT_SPEED = 500
WPNAV_LOIT_JERK = 500
LOIT_ACC_MAX = 100

# Aggressive loiter (windy conditions)
WPNAV_LOIT_SPEED = 2000
WPNAV_LOIT_JERK = 2000
LOIT_ACC_MAX = 500
```

### Alt Hold Tuning

```
# Smooth altitude (filming)
PSC_ACCZ_P = 0.2
PSC_ACCZ_I = 0.3

# Responsive altitude (survey)
PSC_ACCZ_P = 0.5
PSC_ACCZ_I = 0.8
```

### Auto Mode Tuning

```
# Smooth waypoint following
WPNAV_SPEED = 500
WPNAV_ACCEL = 50

# Fast survey missions
WPNAV_SPEED = 1500
WPNAV_ACCEL = 200
```

---

## Mode Fallback Behavior

### GPS Loss (in Loiter/RTL/Auto)

```
GPS Lost → Switch to Alt Hold → Pilot must take control
```

### Compass Error

```
Compass Error → EKF failsafe → Land or Alt Hold
```

### Low Battery

```
Low Battery → RTL (if enabled) → Land at home
```

---

## Quick Reference Card

```
Mode        | Stick = Position? | Throttle = Alt? | Needs GPS?
------------|-------------------|-----------------|------------
Stabilize   | No (rate)         | No (thrust)     | No
Alt Hold    | No (rate)         | Yes             | No
Loiter      | Yes               | Yes             | Yes
RTL         | Auto              | Auto            | Yes
Auto        | Auto              | Auto            | Yes
Guided      | Yes (GCS)         | Yes             | Yes
Land        | No                | Auto            | No
Circle      | Orbit             | Yes             | Yes
```

---

## References

- [ArduPilot Flight Modes](https://ardupilot.org/copter/docs/flight-modes.html)
- [Flight Mode Setup](https://ardupilot.org/copter/docs/ac_flightmodes.html)
- [Tuning Guide](https://ardupilot.org/copter/docs/tuning.html)
