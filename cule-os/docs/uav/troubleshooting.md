# Cule OS UAV - Troubleshooting Guide

Common issues and solutions for Cule OS UAV Edition.

## Quick Diagnostics

```bash
# Run full system check
cule-diagnose

# Check all systems
cule-status

# View recent errors
cule-logs --errors
```

---

## Hardware Issues

### Flight Controller Not Detected

**Symptoms:**
- `cule-status` shows "Flight Controller: Not connected"
- No telemetry data
- Cannot arm

**Solutions:**
```bash
# 1. Check USB connection
lsusb | grep STMicroelectronics
# Should show: STMicroelectronics Virtual COM Port

# 2. Check serial ports
ls /dev/ttyUSB*
ls /dev/ttyACM*

# 3. Check permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# 4. Test with direct USB connection
# Try different USB cable
# Try different USB port

# 5. Restart cule services
sudo systemctl restart cule-os
```

---

### GPS Not Locking

**Symptoms:**
- GPS: No Fix or 2D Fix only
- 0-5 satellites
- Cannot enter LOITER mode

**Solutions:**
```bash
# Check GPS status
cule-status | grep GPS

# View GPS details
cule-gps status

# Check baud rate (default 115200 for u-blox)
cule-param show GPS_TYPE
cule-param show SERIAL4_BAUD
```

**Physical Checks:**
- GPS antenna has clear sky view
- No metal objects near antenna
- GPS cable securely connected
- Wait 5-10 minutes (cold start)

**Common Fixes:**
```bash
# Force GPS type
cule-param set GPS_TYPE 1  # Auto detect
cule-param set GPS_TYPE 2  # u-blox

# Check baud rate
cule-param set SERIAL4_BAUD 115

# Reset GPS
cule-gps reset
```

---

### Compass Errors

**Symptoms:**
- "Bad compass health" warning
- Toilet bowling (circling while hovering)
- Erratic yaw behavior

**Solutions:**
```bash
# Check compass health
cule-status | grep Compass

# View compass interference
cule-compass-interference
```

**Calibration:**
```bash
# Calibrate away from metal objects
sudo cule-calibrate compass

# Follow prompts and rotate drone in all axes
```

**Common Causes:**
- Flying near metal structures
- Compass too close to power wires
- Magnetic interference from motors
- External compass not oriented correctly

**Parameter Adjustments:**
```bash
# Enable compass learning
cule-param set COMPASS_LEARN 1

# Use only external compass
cule-param set COMPASS_USE 0
cule-param set COMPASS_USE2 1
cule-param set COMPASS_USE3 0
```

---

### Camera Not Detected

**Symptoms:**
- Camera not in `v4l2-ctl --list-devices`
- Black screen in QGroundControl
- `cule-camera` commands fail

**Solutions:**

**USB Camera:**
```bash
# Check USB devices
lsusb

# Check video devices
ls /dev/video*

# Test with ffmpeg
ffmpeg -f v4l2 -i /dev/video0 -vframes 1 test.jpg
```

**CSI Camera (Jetson):**
```bash
# Check nvargus daemon
sudo systemctl status nvargus-daemon

# Test gstreamer
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink

# Check CSI connection
# Blue side of ribbon cable faces heatsink
```

**Configuration:**
```bash
# Set camera device
cule-param set CAM_DEVICE /dev/video0

# Set resolution
cule-param set CAM_WIDTH 1920
cule-param set CAM_HEIGHT 1080

# Restart camera service
sudo systemctl restart cule-camera
```

---

## Software Issues

### Cannot Arm

**Symptoms:**
- Pre-arm check fails
- Arming denied message
- Motors don't start

**Solutions:**
```bash
# Check why arming failed
cule-prearm-check

# Common causes:
# - Compass not calibrated
# - Accelerometer not calibrated
# - GPS lock not achieved
# - RC not calibrated
# - Safety switch engaged
```

**Force Arming (Emergency Only):**
```bash
cule-arm --force
# Warning: Bypasses safety checks
```

---

### Drift in LOITER Mode

**Symptoms:**
- Drone drifts while in LOITER
- Position hold not precise
- Toilet bowling

**Causes & Fixes:**

**GPS Accuracy:**
```bash
# Check GPS HDOP
cule-gps status | grep HDOP
# Should be < 2.5

# Check satellite count
cule-gps status | grep satellites
# Should be 8+
```

**Vibration:**
```bash
# Check vibration levels
cule-status | grep vibration
# Should be < 30

# Balance propellers
# Check motor mount tightness
# Add vibration dampening
```

**Compass Interference:**
```bash
# Check compass health
cule-status | grep Compass
# Calibrate compass
sudo cule-calibrate compass
```

---

### Telemetry Connection Lost

**Symptoms:**
- QGroundControl disconnects
- "Communication lost" message
- No live data

**Solutions:**

**Check Radio:**
```bash
# Check radio connection
cule-status | grep Telemetry

# Check serial port
cule-param show SERIAL1_BAUD
cule-param show SERIAL1_PROTOCOL
```

**Common Fixes:**
- Check antenna connections
- Verify both radios have same settings
- Check for interference sources
- Reduce telemetry range (move closer)

**Parameters:**
```bash
# Set telemetry baud rate
cule-param set SERIAL1_BAUD 57  # 57600

# Set MAVLink protocol
cule-param set SERIAL1_PROTOCOL 1
```

---

### High CPU Usage

**Symptoms:**
- System lag
- Video stuttering
- Slow response to commands

**Diagnosis:**
```bash
# Check CPU usage
top
htop

# Check specific processes
ps aux | grep cule

# Monitor GPU (Jetson)
tegrastats
```

**Solutions:**
```bash
# Reduce camera resolution
cule-param set CAM_WIDTH 1280
cule-param set CAM_HEIGHT 720

# Reduce frame rate
cule-param set CAM_FPS 15

# Disable unnecessary features
cule-config --obstacle-avoidance off
cule-config --visual-slam off
```

---

## Flight Issues

### Motors Won't Start

**Symptoms:**
- Arming succeeds but motors don't spin
- Props don't turn

**Solutions:**

**Safety Checks:**
```bash
# Check arming status
cule-status | grep Armed

# Check RC throttle position
cule-status | grep RC
# Throttle must be at minimum

# Check safety switch
# Some FCs require physical switch
```

**ESC Calibration:**
```bash
# Calibrate ESCs
sudo cule-calibrate esc

# Or manually:
# 1. Remove props
# 2. Connect battery
# 3. Hold throttle max + yaw right
# 4. Wait for beeps
# 5. Release sticks
```

---

### Erratic Flight Behavior

**Symptoms:**
- Oscillations
- Unstable hover
- Wobbling

**Solutions:**

**PID Tuning:**
```bash
# View current PID values
cule-param show ATC_ANG_RLL_P
cule-param show ATC_RAT_RLL_P

# Reduce P gains if oscillating
cule-param set ATC_ANG_RLL_P 4.5
cule-param set ATC_ANG_PIT_P 4.5
```

**Vibration:**
```bash
# Check vibration levels
cule-status | grep vibration

# Balance props
# Check motor mounts
# Add dampening
```

---

### Battery Drains Too Fast

**Symptoms:**
- Flight time much shorter than expected
- Voltage drops quickly
- Battery gets hot

**Solutions:**

**Check Current Draw:**
```bash
# Monitor current
cule-telemetry | grep current

# Check battery settings
cule-param show BATT_MONITOR
cule-param show BATT_AMP_PERVLT
```

**Common Causes:**
- Old/degraded battery
- Too much payload
- High wind conditions
- Aggressive flying
- Motor/prop mismatch

**Battery Health:**
```bash
# Check cell voltages
cule-status | grep Battery

# Internal resistance check
# Should be < 20mΩ per cell
```

---

## Network Issues

### Cannot SSH to Cule OS

**Symptoms:**
- `ssh cule@cule-os.local` times out
- Cannot connect to WiFi

**Solutions:**

**Check Network:**
```bash
# Find IP address
# Connect monitor and keyboard
cule-network status

# Or check router admin page
```

**WiFi Configuration:**
```bash
# Reconfigure WiFi
sudo cule-config --wifi

# Or edit directly
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
```

**Direct Connection:**
- Connect HDMI monitor
- Connect USB keyboard
- Login: cule / culeos

---

### Video Stream Not Working

**Symptoms:**
- No video in QGroundControl
- GStreamer errors
- Black screen

**Solutions:**

**Test Pipeline:**
```bash
# Test camera
gst-launch-1.0 v4l2src ! xvimagesink

# Test with nvargus (Jetson)
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink
```

**Check RTSP:**
```bash
# Check if stream is active
cule-camera status

# Restart stream
sudo systemctl restart cule-camera
```

**Network Bandwidth:**
```bash
# Check network speed
iperf3 -c server_ip

# Reduce stream quality if needed
cule-param set CAM_BITRATE 1000000
```

---

## Getting Help

### Collect Diagnostics

```bash
# Generate diagnostic report
cule-diagnose --full --output report.txt

# Collect logs
cule-logs download --all

# System info
cule-version
cule-status
```

### Support Channels

- **Documentation:** https://sanjaysparker27.github.io/cule-os/docs
- **Forum:** https://discussion.cule-os.io
- **GitHub Issues:** https://github.com/SanjaySparker27/cule-os/issues
- **Matrix Chat:** #cule-os:matrix.org

### Report Template

When reporting issues:

```
**Hardware:**
- Flight Controller: Pixhawk ___
- Companion Computer: Jetson ___ / Raspberry Pi ___
- GPS: ___
- Camera: ___

**Software:**
- Cule OS Version: ___
- Flight Stack: ArduPilot ___ / PX4 ___

**Issue Description:**
___

**Steps to Reproduce:**
1. ___
2. ___

**Expected Behavior:**
___

**Actual Behavior:**
___

**Logs:**
[Attach cule-diagnose output]
```
