# Cule OS UAV Flight Modes

Complete guide to all flight modes available in Cule OS Axon Edition.

## Mode Overview

| Mode | Stability | Position Hold | Altitude Hold | Autopilot | Use Case |
|------|-----------|---------------|---------------|-----------|----------|
| **MANUAL** | ✗ | ✗ | ✗ | ✗ | Testing, emergencies |
| **STABILIZE** | ✓ | ✗ | ✗ | ✗ | Learning, manual flight |
| **ALT_HOLD** | ✓ | ✗ | ✓ | ✗ | Photography, inspection |
| **LOITER** | ✓ | ✓ | ✓ | ✗ | Waiting, observation |
| **AUTO** | ✓ | ✓ | ✓ | ✓ | Survey, mapping, delivery |
| **RTL** | ✓ | ✓ | ✓ | ✓ | Return home, failsafe |
| **LAND** | ✓ | ✗ | ✓ | ✓ | Controlled landing |
| **GUIDED** | ✓ | ✓ | ✓ | ✓ | Ground station control |
| **CIRCLE** | ✓ | ✓ | ✓ | ✓ | Inspection, filming |
| **FOLLOW** | ✓ | ✓ | ✓ | ✓ | Follow-me mode |

---

## Manual Modes

### MANUAL Mode
**Direct control with no stabilization.**

- No self-leveling
- No altitude hold
- Raw RC input to motors
- **Use only for testing or emergencies**

```bash
cule-mode manual
```

**When to use:**
- Testing motor directions
- Emergency recovery
- Debugging control issues

**Warning:** Drone will not self-level. Requires expert piloting skills.

---

### STABILIZE Mode
**Self-leveling with manual throttle control.**

- Roll/Pitch: Self-leveling when sticks released
- Yaw: Rate control (no heading hold)
- Throttle: Direct motor control
- No altitude hold

```bash
cule-mode stabilize
```

**When to use:**
- Learning to fly
- Manual acrobatics
- Testing new hardware

**Controls:**
- Center sticks = level hover
- Throttle stick controls climb/descent rate
- Must manually maintain altitude

---

## Assisted Modes

### ALT_HOLD Mode
**Altitude hold with manual position control.**

- Roll/Pitch: Self-leveling
- Yaw: Rate control
- Altitude: Automatically maintained
- Position: Manual (drifts with wind)

```bash
cule-mode alt_hold
```

**When to use:**
- Aerial photography
- Building inspection
- Learning altitude control

**Center throttle behavior:**
- Stick centered = maintain current altitude
- Stick up = climb
- Stick down = descend
- Deadband: 10% around center

**Parameters:**
```bash
# Climb rate limits
cule-param set PILOT_SPEED_UP 250  # cm/s
cule-param set PILOT_SPEED_DN 150  # cm/s

# Altitude deadzone
cule-param set THR_DZ 100  # PWM deadzone
```

---

### LOITER Mode
**Position and altitude hold with GPS.**

- Roll/Pitch: Position hold when sticks released
- Yaw: Rate control
- Altitude: Automatically maintained
- Position: GPS hold (within 2-3m accuracy)

```bash
cule-mode loiter
```

**When to use:**
- Waiting for commands
- Emergency hover
- Battery swap preparation
- Photography

**Requirements:**
- GPS 3D fix
- HDOP < 2.5
- Compass healthy

**Parameters:**
```bash
# Position hold aggressiveness
cule-param set LOITER_SPEED 500      # cm/s max speed
cule-param set LOITER_ACCEL 100      # cm/s² acceleration
cule-param set LOITER_BRK_ACCEL 200  # braking acceleration
```

---

## Autonomous Modes

### AUTO Mode
**Execute pre-programmed mission.**

- Follows uploaded waypoint mission
- Automatically transitions between waypoints
- Executes commands at each waypoint
- RTL on mission complete or failsafe

```bash
# Upload mission
cule-mission upload mission.plan

# Start mission
cule-mode auto
```

**Mission Commands:**
| Command | Description |
|---------|-------------|
| `WAYPOINT` | Navigate to coordinates |
| `TAKEOFF` | Climb to specified altitude |
| `LAND` | Land at coordinates |
| `RTL` | Return to launch |
| `DELAY` | Wait for time/distance |
| `DO_SET_ROI` | Point camera at location |
| `DO_DIGICAM_CONTROL` | Trigger camera |
| `DO_JUMP` | Loop to another command |

**Parameters:**
```bash
# Default speed in AUTO
cule-param set WP_SPEED 500       # cm/s

# Radius to consider waypoint reached
cule-param set WP_RADIUS 200      # cm
```

---

### RTL Mode (Return to Launch)
**Return to home position and land.**

- Climbs to RTL altitude
- Flies directly to launch point
- Descends and lands automatically

```bash
cule-mode rtl
```

**When triggered:**
- Manual command
- Radio failsafe
- Battery failsafe
- Geofence breach

**Parameters:**
```bash
# RTL altitude (above terrain)
cule-param set RTL_ALT 1500       # 15 meters

# Final approach altitude
cule-param set RTL_ALT_FINAL 0    # 0 = land immediately

# RTL speed
cule-param set RTL_SPEED 0        # 0 = use WP_SPEED

# RTL climb min
cule-param set RTL_CLIMB_MIN 0    # climb before RTL
```

---

### LAND Mode
**Controlled landing at current position.**

- Descends at controlled rate
- Maintains position (if GPS available)
- Disarms automatically on touchdown

```bash
cule-mode land
```

**Parameters:**
```bash
# Land speed
cule-param set LAND_SPEED 50      # cm/s descent

# Final land speed (last 10m)
cule-param set LAND_SPEED_HIGH 100
```

---

### GUIDED Mode
**Ground station or companion computer control.**

- Accepts position targets from companion computer
- Used for follow-me, object tracking
- Real-time waypoint updates

```bash
cule-mode guided

# Send position target
cule-guided goto --lat 37.7749 --lon -122.4194 --alt 20
```

**Python API:**
```python
from cule import UAV

drone = UAV.connect('/dev/ttyUSB0')
drone.mode = 'GUIDED'
drone.goto(37.7749, -122.4194, 20)
```

---

## Advanced Modes

### CIRCLE Mode
**Orbit a point of interest.**

- Circles around a center point
- Adjustable radius and speed
- Camera always points at center

```bash
cule-mode circle --radius 10 --speed 3
```

**Parameters:**
```bash
# Circle radius
cule-param set CIRCLE_RADIUS 1000   # cm

# Circle rate (negative = clockwise)
cule-param set CIRCLE_RATE 2        # deg/s

# Circle control mode
cule-param set CIRCLE_CONTROL 0     # 0=pilot can adjust
```

---

### FOLLOW Mode
**Follow a ground target.**

- Tracks GPS position of target device
- Maintains relative position/altitude
- Useful for follow-me shots

```bash
# Enable follow mode
cule-mode follow

# Set target device
cule-follow target --id phone_001
```

**Requirements:**
- Target device with GPS
- Telemetry connection
- Cule Follow app on target

---

## Mode Switching

### Via RC Transmitter
Configure flight mode switch on transmitter:

```bash
# View current mode channel
cule-param show FLTMODE_CH

# Set flight mode 1 (low position)
cule-param set FLTMODE1 0    # STABILIZE

# Set flight mode 2 (mid position)
cule-param set FLTMODE2 5    # LOITER

# Set flight mode 3 (high position)
cule-param set FLTMODE3 3    # AUTO
```

### Via Command Line
```bash
# Switch mode
cule-mode <MODE_NAME>

# Examples:
cule-mode stabilize
cule-mode loiter
cule-mode rtl
cule-mode auto
```

### Via Python API
```python
from cule import UAV

drone = UAV.connect('/dev/ttyUSB0')

# Get current mode
print(drone.mode)  # 'STABILIZE'

# Set mode
drone.mode = 'LOITER'

# Or use method
drone.set_mode('AUTO')
```

---

## Failsafe Behavior

When failsafe triggers, mode changes automatically:

| Failsafe | Default Action | Configurable |
|----------|----------------|--------------|
| Radio loss | RTL | Yes |
| Battery low | RTL | Yes |
| GCS loss | Continue | Yes |
| GPS loss | LAND/ALT_HOLD | Yes |
| Geofence | RTL/LAND | Yes |

```bash
# Configure radio failsafe
cule-param set FS_THR_ENABLE 1   # 0=disabled, 1=RTL, 2=Land, 3=Continue
cule-param set FS_THR_VALUE 975  # PWM threshold

# Configure battery failsafe
cule-param set FS_BATT_ENABLE 1  # 1=RTL, 2=Land
cule-param set FS_BATT_VOLTAGE 14.0  # 4S battery

# Configure GCS failsafe
cule-param set FS_GCS_ENABLE 1   # 1=RTL, 2=Continue, 3=Land
```

---

## Mode Transitions

Safe mode switching order for beginners:

```
STABILIZE → ALT_HOLD → LOITER → AUTO
                    ↓
                  RTL (emergency)
```

**Important:**
- Can switch from any mode to any mode
- RTL can interrupt AUTO missions
- MANUAL should be avoided unless necessary

---

## Best Practices

### For Beginners
1. Start in STABILIZE, get comfortable
2. Progress to ALT_HOLD
3. Then LOITER for position hold
4. Finally AUTO for missions

### For Missions
1. Always test in LOITER first
2. Upload mission, verify in QGC
3. Arm in LOITER, verify GPS
4. Switch to AUTO
5. Monitor throughout

### Emergency Procedures
- **Lost control** → Switch to RTL immediately
- **GPS fails** → Switch to ALT_HOLD, land manually
- **Compass error** → Switch to STABILIZE, land manually
- **Battery critical** → AUTO lands (don't fight it)

---

## Troubleshooting

### Can't switch to LOITER
```bash
# Check GPS status
cule-status | grep GPS

# Required: 3D fix, 6+ satellites, HDOP < 2.5
```

### Can't switch to AUTO
```bash
# Check mission uploaded
cule-mission list

# Verify home position set
cule-param show HOME
```

### Drone drifts in LOITER
- Calibrate compass: `sudo cule-calibrate compass`
- Check for magnetic interference
- Verify GPS accuracy

---

## API Reference

### Check Current Mode
```bash
cule-mode
# Output: Current mode: LOITER
```

### List Available Modes
```bash
cule-mode --list
```

### Get Mode Details
```bash
cule-mode --info LOITER
```
