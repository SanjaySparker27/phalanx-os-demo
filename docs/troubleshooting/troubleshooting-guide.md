# Troubleshooting Guide

Comprehensive troubleshooting guide for the ATHENA H743 PRO flight controller and HEXA X + Pusher VTOL system.

## Table of Contents
- [Quick Diagnostic Flowchart](#quick-diagnostic-flowchart)
- [Power Issues](#power-issues)
- [RC/Communication Issues](#rccommunication-issues)
- [Sensor Issues](#sensor-issues)
- [Motor/ESC Issues](#motoresc-issues)
- [Flight Mode Issues](#flight-mode-issues)
- [Companion Computer Issues](#companion-computer-issues)
- [Software/Firmware Issues](#softwarefirmware-issues)
- [Performance Issues](#performance-issues)

---

## Quick Diagnostic Flowchart

```
START
  |
  v
+------------------+
| FC powers on?    |
+--------+---------+
         |
    No   |   Yes
         v
+--------+---------+
| Check battery    |        +------------------+
| connections      |        | Check LEDs       |
| and voltage      |        | and messages     |
+------------------+        +--------+---------+
                                     |
                              OK     |   Error
                                     v
                            +--------+---------+
                            | Run pre-arm      |
                            | checks           |
                            +--------+---------+
                                     |
                            Pass     |   Fail
                                     v
                            +--------+---------+
                            | Check specific   |
                            | error below      |
                            +------------------+
```

---

## Power Issues

### FC Won't Power On

| Symptom | Check | Solution |
|---------|-------|----------|
| No LEDs | Battery connection | Check XT60 connector |
| No LEDs | Battery voltage | Charge/replace battery |
| No LEDs | Polarity | Verify red/black correct |
| Dim LED | Low battery | Charge to >22V |
| Blinking red | Power supply issue | Check BEC output |

**Diagnostic Steps:**
```
1. Measure battery voltage at connector
2. Check voltage at FC power input
3. Verify 5V on FC power rail
4. Check for blown fuse
```

### Battery Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Voltage drops fast | Old battery | Replace battery |
| Cell imbalance | Bad cell | Balance charge or replace |
| Won't charge | Deep discharge | Use NiMH mode briefly |
| Swelling | Overcharge/heat | DISPOSE SAFELY |

**Battery Error Codes:**
```
ERR: BATT VOLTAGE - Check battery voltage reading
ERR: BATT CAPACITY - Check mAh consumed vs capacity
ERR: BATT FS - Battery failsafe triggered
```

---

## RC/Communication Issues

### No RC Input

| Symptom | Check | Solution |
|---------|-------|----------|
| No bars in MP | Transmitter off | Power on transmitter |
| No bars in MP | Wrong channel map | Check RC_MAP parameters |
| Jittery input | Interference | Change frequency |
| Limited range | Antenna position | Orient vertically |

**Diagnostic Commands:**
```bash
# In MAVProxy
rc 3          # Check throttle channel
rc 8          # Check mode/pusher channel
status        # Check RC status
```

### Telemetry Issues

| Symptom | Check | Solution |
|---------|-------|----------|
| No link | Radio off | Power on ground radio |
| Weak signal | Distance | Move closer |
| Dropped packets | Interference | Change NETID |
| Slow updates | Baud rate | Match SERIALx_BAUD |

**Radio Configuration:**
```
Ground and Air must match:
- NETID: 25 (same both sides)
- Frequency: 915 MHz (US) / 433 MHz (EU)
- Baud rate: 57 (57600)
```

### Mode Switch Not Working

| Symptom | Check | Solution |
|---------|-------|----------|
| Mode not changing | Wrong FLTMODE_CH | Verify set to 8 |
| Mode not changing | RC8 not calibrated | Re-calibrate RC |
| Wrong modes shown | FLTMODE1-6 params | Set to 0 or 5 |

**Verify Mode Switch:**
```
1. Connect to Mission Planner
2. Go to Flight Data -> Status
3. Watch 'flightmode' field
4. Move RC8 slider
5. Should show: Stabilize (0-16%) or Loiter (>16%)
```

---

## Sensor Issues

### GPS Problems

| Symptom | Cause | Solution |
|---------|-------|----------|
| No GPS | No antenna | Connect GPS antenna |
| No fix | Indoors | Move outside |
| Slow fix | Cold start | Wait 2-5 minutes |
| Drifting | Poor reception | Relocate GPS |
| Wrong position | Wrong coordinate frame | Check GPS_TYPE |

**GPS Diagnostic:**
```bash
# In MAVProxy
gps status      # Check satellites and fix type
gps raw         # Raw NMEA/UBX messages
```

**Parameter Check:**
```
GPS_TYPE = 1        # Auto detect
GPS_AUTO_SWITCH = 1 # Auto switch GPS
SERIAL1_PROTOCOL = 5
SERIAL1_BAUD = 115
```

### Compass Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| Compass error | Interference | Move away from power |
| Wrong heading | Calibration | Re-do compass cal |
| Toilet bowling | Compass/GPS mismatch | Check orientation |
| Yaw drift | Magnetic interference | Shield or relocate |

**Compass Health Check:**
```
1. Check COMPASS_HEALTH in messages
2. Verify COMPASS_USE = 1
3. Check for magnetic interference:
   - Power on motors (don't spin)
   - Watch compass heading
   - Should remain stable
```

### IMU/Accelerometer Issues

| Symptom | Cause | Solution |
|---------|-------|----------|
| "Bad accel health" | High vibration | Add damping |
| Horizon drift | Bad calibration | Re-calibrate accel |
| "Inconsistent compass" | Yaw error | Check compass cal |
| EKF errors | Sensor mismatch | Verify orientations |

**Vibration Check:**
```
Mission Planner -> Flight Data -> Vibration
Acceptable: < 30 m/s/s
Marginal: 30-60 m/s/s
Bad: > 60 m/s/s
```

---

## Motor/ESC Issues

### Motor Not Spinning

| Symptom | Check | Solution |
|---------|-------|----------|
| No response | Arming | Arm vehicle first |
| No response | Safety switch | Press safety switch |
| No response | PWM wires | Check connections |
| Weak response | ESC calibration | Re-calibrate ESCs |
| Wrong direction | Phase wires | Swap any two wires |

**Motor Test Procedure:**
```
1. Remove props
2. Connect battery
3. Mission Planner -> Motor Test
4. Test each motor at 5-10%
5. Verify correct motor and direction
```

### ESC Beep Codes

| Beep Pattern | Meaning |
|--------------|---------|
| 1 long, 2 short | Normal startup |
| Continuous beep | No throttle signal |
| 3 short | Low voltage |
| 5 short | Overheating |

### Pusher Motor Issues

| Symptom | Check | Solution |
|---------|-------|----------|
| No response | SERVO7_FUNCTION | Set to 51 (RCIN8) |
| No response | RC8 calibration | Re-calibrate RC |
| Wrong direction | ESC wires | Swap any two |
| Not proportional | RC8 range | Check MIN/MAX |

**Pusher Verification:**
```
1. Set RC8 to 0%
2. Arm and raise throttle slightly
3. M7 should not spin
4. Increase RC8 to 50%
5. M7 should spin with throttle
6. M7 should stop when RC8 back to 0%
```

---

## Flight Mode Issues

### Can't Enter Desired Mode

| Symptom | Check | Solution |
|---------|-------|----------|
| Mode rejected | Pre-arm checks | Pass all checks |
| Mode rejected | GPS required | Wait for 3D fix |
| Mode rejected | EKF unhealthy | Check EKF status |
| Wrong mode | RC8 threshold | Check slider position |

**Mode Requirements:**
```
Mode          Requirements
Stabilize     None
Alt Hold      Barometer healthy
Loiter        GPS 3D fix + Compass
RTL           GPS 3D fix
Auto          GPS 3D fix + Mission loaded
Land          None (or GPS for precision)
```

### EKF Errors

| Error | Meaning | Solution |
|-------|---------|----------|
| EKF variance | Position error | Check GPS/compass |
| EKF failsafe | Position lost | Switch to manual |
| Compass variance | Heading error | Re-calibrate compass |
| Velocity variance | Speed error | Check GPS quality |

**EKF Status:**
```bash
# In MAVProxy
status EKF_STATUS        # Check EKF health
param show EKF_*         # Check EKF parameters
```

---

## Companion Computer Issues

### Jetson Won't Boot

| Symptom | Check | Solution |
|---------|-------|----------|
| No power LED | Power supply | Check 5V/4A supply |
| No display | HDMI connection | Check cable |
| Boot loop | SD card | Re-flash OS |
| Can't SSH | Network | Check IP address |

**Jetson Diagnostics:**
```bash
# On Jetson
sudo tegrastats          # Check GPU/temp
nvidia-smi               # GPU status
dmesg | tail -50         # Boot messages
```

### MAVLink Connection Issues

| Symptom | Check | Solution |
|---------|-------|----------|
| No heartbeat | UART connection | Check TX/RX wires |
| No heartbeat | Wrong device | Check /dev/ttyTHS* |
| No heartbeat | Baud rate | Match 921600 |
| Connection drops | Power | Check 5V supply |

**MAVLink Test:**
```bash
# Test UART connection
stty -F /dev/ttyTHS1 921600
cat /dev/ttyTHS1 | xxd   # Should see MAVLink packets

# Start MAVProxy
mavproxy.py --master=/dev/ttyTHS1 --baudrate 921600
```

### Cule OS Issues

| Symptom | Check | Solution |
|---------|-------|----------|
| Service failed | Configuration | Check cule-os.conf |
| Agent offline | Dependencies | Install missing packages |
| High CPU | Inference load | Reduce model size |
| Camera error | GStreamer | Check pipeline |

**Cule OS Diagnostics:**
```bash
# Check services
sudo systemctl status cule-*
cule-status

# View logs
sudo journalctl -u cule-agent -f

# Test camera
nvarguscamerasrc ! nvoverlaysink
```

---

## Software/Firmware Issues

### Firmware Flash Failed

| Symptom | Cause | Solution |
|---------|-------|----------|
| Flash error | Wrong file | Use .apj not .hex |
| Bootloader missing | First flash | Flash bootloader first |
| Verification fail | Bad USB | Try different cable/port |
| Timeout | Driver issue | Install STM32 driver |

**Recovery Flash:**
```
1. Hold BOOT button on FC
2. Connect USB
3. Use STM32CubeProgrammer
4. Erase chip
5. Flash bootloader
6. Flash firmware
```

### Parameter Reset

| Symptom | Cause | Solution |
|---------|-------|----------|
| Defaults loaded | Bad params | Reload from file |
| Unexpected behavior | Wrong params | Compare with reference |
| Corruption | Power loss | Backup regularly |

**Parameter Backup/Restore:**
```
Backup:
1. Connect to Mission Planner
2. Config/Tuning -> Full Parameter List
3. Save to File

Restore:
1. Load from File
2. Write Params
3. Reboot FC
```

---

## Performance Issues

### Drifting in Loiter

| Symptom | Cause | Solution |
|---------|-------|----------|
| Position drift | GPS accuracy | Improve GPS reception |
| Position drift | Wind | Normal, wait for correction |
| Toilet bowl | Compass error | Re-calibrate compass |
| Oscillation | PID tuning | Reduce P gains |

### Vibrations

| Symptom | Cause | Solution |
|---------|-------|----------|
| High X/Y vibration | Unbalanced props | Balance props |
| High Z vibration | Motor issue | Check motor mounts |
| All axes high | Frame resonance | Stiffen frame |
| Intermittent | Loose screw | Check all hardware |

**Vibration Reduction:**
```
1. Balance all props
2. Check prop tightness
3. Verify motor mount security
4. Add damping to FC mount
5. Check for bent motor shafts
```

### Battery Consumption

| Symptom | Cause | Solution |
|---------|-------|----------|
| Short flight time | Old battery | Replace battery |
| Short flight time | High current | Check for drag |
| Voltage sag | High C draw | Use higher C battery |
| Unbalanced cells | Bad charger | Replace charger |

---

## Getting Help

### Information to Gather

When seeking help, provide:

1. **Logs:**
   - Dataflash log (.bin from SD card)
   - Tlogs from ground station
   - Parameter file

2. **System Info:**
   - Firmware version
   - Hardware type (ATHENA H743 PRO)
   - Parameter changes from default

3. **Problem Description:**
   - What happened
   - When it happened
   - Steps to reproduce

### Resources

- [ArduPilot Discuss Forums](https://discuss.ardupilot.org/)
- [ArduPilot Discord](https://discord.gg/ardupilot)
- [Mission Planner Docs](https://ardupilot.org/planner/)
- [Cule OS Support](https://cule-os.io/support)

### Log Analysis

```
Key things to check in logs:
- Vibration levels (VIBE)
- GPS accuracy (GPS)
- RC input (RCIN)
- Mode changes (MODE)
- EKF status (NKF1-NKF9)
- Battery voltage (BAT)
- Motor outputs (RCOU)
```

---

## Emergency Procedures

### Immediate Actions

```
1. SWITCH TO STABILIZE - Take manual control
2. REDUCE THROTTLE - Descend safely
3. FIND CLEAR AREA - Avoid obstacles
4. LAND IMMEDIATELY - Don't risk further
5. DISARM - Cut motors on ground
```

### Contact Information

```
Emergency Contacts:
- Local Emergency: 911
- Flying Site Manager: _______________
- Project Lead: _______________
- Technical Support: _______________
```
