# Calibration Procedures

Complete calibration guide for the ATHENA H743 PRO flight controller and all connected sensors.

## Table of Contents
- [Pre-Calibration Setup](#pre-calibration-setup)
- [RC Radio Calibration](#rc-radio-calibration)
- [Accelerometer Calibration](#accelerometer-calibration)
- [Compass Calibration](#compass-calibration)
- [ESC Calibration](#esc-calibration)
- [Motor Test and Verification](#motor-test-and-verification)
- [Camera Calibration](#camera-calibration)
- [Post-Calibration Verification](#post-calibration-verification)

---

## Pre-Calibration Setup

### Required Tools

| Tool | Purpose |
|------|---------|
| Mission Planner or QGroundControl | Calibration software |
| USB cable | FC connection |
| Laptop/PC | Ground station |
| Propeller removal tool | Safety |
| Spirit level | FC leveling |
| Compass calibration rig | Large open space |

### Safety Precautions

1. **REMOVE ALL PROPELLERS** before any calibration
2. Disconnect battery from ESCs during FC calibration
3. Keep area clear of metal objects during compass calibration
4. Do not wear metal jewelry during calibration
5. Ensure FC is securely mounted during accelerometer calibration

### Initial Connection

1. Connect FC to computer via USB
2. Open Mission Planner
3. Select correct COM port and baud rate (115200)
4. Click **Connect**
5. Verify heartbeat messages in Messages tab

---

## RC Radio Calibration

### Prerequisites

- Transmitter bound to receiver
- All switches in default positions
- Fresh batteries in transmitter

### Calibration Steps

1. **Navigate to Calibration Page**
   ```
   Mission Planner → Initial Setup → Mandatory Hardware → Radio Calibration
   ```

2. **Verify Channel Mapping**
   - Move each stick and watch corresponding bar
   - Verify directions:
     - Right stick right → CH1 increases
     - Right stick up → CH2 increases  
     - Left stick up → CH3 increases
     - Left stick right → CH4 increases

3. **Start Calibration**
   - Click **Calibrate Radio**
   - Confirm warning about removing props

4. **Move All Controls**
   - Move right stick in full circles (both axes)
   - Move left stick in full circles (both axes)
   - Move throttle stick full range multiple times
   - Toggle all 3-position switches
   - Move slider through full range (RC8)

5. **Complete Calibration**
   - Click **Click when Done**
   - Verify all bars show green with center positions

### RC8 Slider Verification

```
# Verify RC8 (left slider) range
Slider Position    Expected Value
    0%          →   ~1000 PWM
   50%          →   ~1500 PWM
  100%          →   ~2000 PWM
```

### RC Configuration Parameters

After calibration, verify these parameters:

```
RC1_MIN = ~1000      RC1_MAX = ~2000
RC2_MIN = ~1000      RC2_MAX = ~2000
RC3_MIN = ~1000      RC3_MAX = ~2000
RC4_MIN = ~1000      RC4_MAX = ~2000
RC5_MIN = ~1000      RC5_MAX = ~2000
RC6_MIN = ~1000      RC6_MAX = ~2000
RC7_MIN = ~1000      RC7_MAX = ~2000
RC8_MIN = ~1000      RC8_MAX = ~2000
```

---

## Accelerometer Calibration

### Prerequisites

- FC firmly mounted in final position
- Vehicle level and stable
- No vibration during calibration
- USB connection stable

### Calibration Steps

1. **Navigate to Calibration Page**
   ```
   Mission Planner → Initial Setup → Mandatory Hardware → Accel Calibration
   ```

2. **Calibrate Level**
   - Place vehicle perfectly level
   - Use spirit level on frame
   - Click **Calibrate Level**
   - Wait for completion

3. **6-Point Calibration**
   - Click **Calibrate Accel**
   - Follow prompts for each orientation:

   ```
   Position Sequence:
   1. Level (normal flight position)
   2. Left side down
   3. Right side down
   4. Nose down
   5. Nose up
   6. Upside down
   ```

4. **Verification**
   - Go to **Flight Data** tab
   - Check artificial horizon
   - Should be level when vehicle is level

### Accelerometer Parameters

```
# After calibration, these values should be non-zero
INS_ACCOFFS_X = <calibrated>
INS_ACCOFFS_Y = <calibrated>
INS_ACCOFFS_Z = <calibrated>
INS_ACCSCAL_X = <calibrated>
INS_ACCSCAL_Y = <calibrated>
INS_ACCSCAL_Z = <calibrated>
```

---

## Compass Calibration

### Prerequisites

- Large open space (at least 10m from vehicles, buildings)
- No metal objects nearby
- Remove watches, phones, keys
- Away from power lines

### Calibration Steps

1. **Navigate to Calibration Page**
   ```
   Mission Planner → Initial Setup → Mandatory Hardware → Compass
   ```

2. **Enable Compasses**
   - Check **Use this compass** for external compass
   - Set orientation to **None** (arrow forward)

3. **Start Calibration**
   - Click **Start** or **Large Vehicle MagCal**
   - For large vehicles, use **Large Vehicle MagCal** with throttle method

4. **Perform Calibration Dance**
   
   ```
   Large Vehicle MagCal (throttle method):
   1. Arm vehicle
   2. Raise throttle to full
   3. Move vehicle through all axes:
      - Pitch: Nose up, nose down
      - Roll: Left, right
      - Yaw: Full 360° rotations
   4. Lower throttle to minimum
   5. Disarm
   ```

   ```
   Onboard Calibration (smaller vehicles):
   1. Click Start
   2. Rotate vehicle around all axes
   3. Watch progress bar fill
   4. Complete when progress reaches 100%
   ```

5. **Verification**
   - Check compass heading in Flight Data
   - Should point to magnetic north when nose points north
   - Verify compass doesn't jump when tilting

### Compass Parameters

```
COMPASS_EXTERNAL = 1      # External compass
COMPASS_ORIENT = 0        # Forward
COMPASS_USE = 1           # Use for yaw

# Calibration offsets (will be auto-populated)
COMPASS_OFS_X = <calibrated>
COMPASS_OFS_Y = <calibrated>
COMPASS_OFS_Z = <calibrated>
COMPASS_DIA_X = <calibrated>
COMPASS_DIA_Y = <calibrated>
COMPASS_DIA_Z = <calibrated>
```

### Interference Check

After calibration:
1. Power on all systems (motors armed but not spinning)
2. Check compass interference:
   ```
   Mission Planner → Flight Data → Status tab
   Look for mx, my, mz values
   ```
3. Values should remain stable
4. If jumping, relocate compass away from power cables

---

## ESC Calibration

### Prerequisites

- Props removed
- Battery connected
- USB connected
- Safety switch accessible

### Method 1: All at Once (Recommended)

1. **Set ESC Calibration Parameter**
   ```
   Mission Planner → Config/Tuning → Full Parameter List
   Set ESC_CALIBRATION = 3
   Write params and reboot
   ```

2. **Perform Calibration**
   ```
   1. Disconnect USB
   2. Connect battery (ESCs will beep)
   3. Hold throttle to maximum on transmitter
   4. Press safety switch
   5. Wait for musical tone (full throttle recognized)
   6. Lower throttle to minimum
   7. Wait for confirmation beep
   8. Calibration complete
   ```

3. **Verify**
   ```
   Set ESC_CALIBRATION = 0
   Test motors at low throttle
   ```

### Method 2: Individual ESC Calibration

For ESCs that don't support auto-calibration:

1. Connect each ESC signal wire to throttle channel (CH3)
2. Power on with throttle high
3. Wait for beep sequence
4. Lower throttle to minimum
5. Wait for confirmation
6. Repeat for all 7 ESCs

### ESC Parameters

```
MOT_PWM_MIN = 1000      # Minimum PWM
MOT_PWM_MAX = 2000      # Maximum PWM
MOT_SPIN_ARM = 0.05     # Arming spin (5%)
MOT_SPIN_MIN = 0.10     # Minimum spin (10%)
MOT_SPIN_MAX = 0.95     # Maximum spin (95%)
```

---

## Motor Test and Verification

### Prerequisites

- Props removed
- Battery connected
- USB connected
- Safe area cleared

### Motor Test Procedure

1. **Navigate to Motor Test**
   ```
   Mission Planner → Initial Setup → Optional Hardware → Motor Test
   ```

2. **Safety Settings**
   - Set **Throttle %** to low (5-10%)
   - Set **Duration** to 2 seconds
   - Keep **Motor Count** at 1

3. **Test Each Motor**
   ```
   Click motor buttons A through F (M1-M6)
   Each motor should spin briefly
   
   Test motor G (M7 pusher)
   Verify responds to RC8 slider
   ```

### Motor Direction Verification

| Motor | Expected Direction | Blade Type |
|-------|-------------------|------------|
| M1 | CCW | Pusher |
| M2 | CW | Tractor |
| M3 | CW | Tractor |
| M4 | CCW | Pusher |
| M5 | CCW | Pusher |
| M6 | CW | Tractor |
| M7 (Pusher) | CW | Pusher |

**To reverse motor direction:**
- Swap any two of the three motor wires
- OR change `MOT_PWM_TYPE` if using DShot

### Pusher Motor Test

1. Set RC8 slider to 50%
2. Arm vehicle
3. Raise throttle slightly
4. Verify M7 spins with throttle
5. Lower RC8 slider to 0%
6. Verify M7 stops

---

## Camera Calibration

### Prerequisites

- Camera connected to Jetson
- Camera focused
- Calibration pattern (checkerboard)

### Camera Calibration Steps

1. **Capture Calibration Images**
   ```bash
   # On Jetson
   cd ~/camera_calibration
   
   # Start camera capture
   python3 capture_images.py --camera /dev/video0
   
   # Show checkerboard pattern to camera
   # Capture 20-30 images with different angles
   ```

2. **Run Calibration**
   ```bash
   # Generate calibration file
   python3 calibrate_camera.py --input ./images --output camera_calibration.yaml
   
   # Parameters generated:
   # - Camera matrix (intrinsics)
   # - Distortion coefficients
   # - RMS error
   ```

3. **Verify Calibration**
   ```bash
   # Test undistortion
   python3 test_undistort.py --calibration camera_calibration.yaml
   
   # Check reprojection error < 0.5 pixels
   ```

### Camera Parameters File

```yaml
# camera_calibration.yaml
camera_name: imx477
image_width: 4056
image_height: 3040
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
```

---

## Post-Calibration Verification

### Pre-Flight Checks

1. **RC Verification**
   ```
   - All sticks respond correctly
   - Mode switch changes mode (check HUD)
   - Pusher motor responds to RC8
   - Failsafe triggers on TX off
   ```

2. **Sensor Verification**
   ```
   - Artificial horizon level
   - Compass points north
   - GPS has 3D fix (8+ satellites)
   - No compass interference
   - Barometer reading reasonable
   ```

3. **Motor Verification**
   ```
   - All motors spin in correct direction
   - Motor order correct (M1-M6)
   - Pusher motor responds to slider
   - No unusual vibrations
   ```

### Calibration Log

Record calibration values:

```
Calibration Date: _______________
Calibrated By: _______________

RC Calibration:
- RC8 Min: _____ PWM
- RC8 Max: _____ PWM

Accelerometer:
- X Offset: _____
- Y Offset: _____
- Z Offset: _____

Compass:
- X Offset: _____
- Y Offset: _____
- Z Offset: _____

ESC:
- Calibration Method: ____________
- Min PWM: _____
- Max PWM: _____

Camera:
- RMS Error: _____ pixels
- Calibration File: ____________
```

---

## Troubleshooting Calibration Issues

### Compass Calibration Fails

| Symptom | Cause | Solution |
|---------|-------|----------|
| Can't reach 100% | Interference | Move away from metal/EMI |
| Values don't save | Bad compass | Check wiring |
| Heading jumps | Power interference | Relocate compass |

### Accelerometer Calibration Fails

| Symptom | Cause | Solution |
|---------|-------|----------|
| "Bad accel health" | Vibration | Isolate FC better |
| Level offset | Wrong position | Use spirit level |

### ESC Calibration Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| ESCs don't beep | Wrong protocol | Check MOT_PWM_TYPE |
| Different end points | Mixed ESCs | Calibrate individually |
| Motor twitching | Bad signal | Check PWM wires |

---

## References

- [ArduPilot RC Calibration](https://ardupilot.org/copter/docs/common-radio-control-calibration.html)
- [ArduPilot Accelerometer Calibration](https://ardupilot.org/copter/docs/common-accelerometer-calibration.html)
- [ArduPilot Compass Calibration](https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html)
- [ArduPilot ESC Calibration](https://ardupilot.org/copter/docs/esc-calibration.html)
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
