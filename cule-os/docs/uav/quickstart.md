# Cule OS UAV - Quick Start Guide

**Get your drone flying autonomously in 30 minutes**

## Prerequisites

### Required Hardware
| Component | Recommended | Alternative |
|-----------|-------------|-------------|
| Flight Controller | Pixhawk 6X | Pixhawk 4, Cube Orange |
| Companion Computer | NVIDIA Jetson Orin Nano | Raspberry Pi 5 |
| GPS | u-blox NEO-M9N | u-blox NEO-M8N |
| Telemetry | Holybro SiK 915MHz | RFD900x |
| Camera | Raspberry Pi Camera v3 | USB Webcam |
| RC System | FrSky Taranis | Radiomaster TX16S |

### Software Requirements
- Cule OS Axon Edition flashed to SD card
- QGroundControl (Windows/Mac/Linux)
- MicroSD card (64GB Class 10)

---

## Step 1: Flash Cule OS (5 minutes)

```bash
# Download latest release
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-axon-v1.0.img.xz

# Identify your SD card (be careful!)
lsblk

# Flash to SD card (replace sdX with your device)
sudo xzcat cule-os-axon-v1.0.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Sync and eject
sync
sudo eject /dev/sdX
```

---

## Step 2: Hardware Assembly (10 minutes)

### Connection Diagram
```
┌─────────────────┐     USB      ┌─────────────────┐
│  NVIDIA Jetson  │◄────────────►│   Pixhawk 6X    │
│   Orin Nano     │              │  Flight Control │
└────────┬────────┘              └────────┬────────┘
         │                                │
         │ CSI                            │ UART
         │                                │
    ┌────▼────┐                      ┌────▼────┐
    │  Camera │                      │   GPS   │
    └─────────┘                      └────┬────┘
                                          │
                                     ┌────▼────┐
                                     │Telemetry│
                                     │ Radio   │
                                     └────┬────┘
                                          │
                                     ┌────▼────┐
                                     │Ground   │
                                     │Station  │
                                     └─────────┘
```

### Wiring Steps
1. **Connect Flight Controller to Jetson**
   - USB-C to USB-A cable
   - Verify connection: `lsusb | grep STMicroelectronics`

2. **Connect GPS Module**
   - GPS → Pixhawk GPS1 port
   - Place antenna on mast, away from electronics

3. **Connect Telemetry Radio**
   - Radio → Pixhawk TELEM1 port
   - Note: 57600 baud default

4. **Connect Camera**
   - CSI ribbon cable to Jetson CSI port
   - Blue side facing heatsink

---

## Step 3: First Boot & Configuration (10 minutes)

### Boot Up
1. Insert SD card into Jetson
2. Connect flight controller USB
3. Power on (12V-5V buck converter recommended)
4. Wait 2-3 minutes for first boot
5. Connect to WiFi network `Cule-OS-Setup`

### Initial Configuration

```bash
# SSH into Cule OS
ssh cule@192.168.4.1
# Password: culeos

# Run configuration wizard
sudo cule-config

# Select options:
# 1. Vehicle Type: UAV (Drone)
# 2. Frame: Quadcopter X
# 3. Flight Stack: ArduPilot
# 4. GPS: u-blox NEO-M9N
# 5. Telemetry: 915MHz
```

### Configure WiFi
```bash
sudo cule-config --wifi

# Enter your network details:
SSID: YourNetworkName
Password: YourPassword
```

---

## Step 4: Sensor Calibration (10 minutes)

**⚠️ REMOVE PROPELLERS BEFORE CALIBRATION**

### Compass Calibration
```bash
sudo cule-calibrate compass

# Follow on-screen instructions:
# 1. Hold drone level, rotate 360°
# 2. Tilt nose down, rotate 360°
# 3. Tilt left side down, rotate 360°
# 4. Hold nose up, rotate 360°
```

### Accelerometer Calibration
```bash
sudo cule-calibrate accel

# Place drone in each position:
# 1. Level on table
# 2. Left side down
# 3. Right side down
# 4. Nose down
# 5. Nose up
# 6. Upside down
```

### Radio Calibration
```bash
sudo cule-calibrate radio

# Move all sticks and switches through full range
# Verify all channels respond in QGroundControl
```

### ESC Calibration
```bash
sudo cule-calibrate esc

# Follow prompts - motors will beep during calibration
```

---

## Step 5: Verify Setup (5 minutes)

### Check System Status
```bash
cule-status

# Expected output:
✓ Cule OS Kernel: v1.0.0
✓ Flight Controller: Connected (/dev/ttyUSB0)
✓ GPS: 3D Fix (12 satellites)
✓ Compass: Healthy
✓ Accelerometer: Healthy
✓ Gyroscope: Healthy
✓ Barometer: Healthy
✓ Battery: 16.8V (4S LiPo)
✓ Telemetry: Connected
✓ Camera: /dev/video0 (1920x1080@30fps)
```

### Test in QGroundControl
1. Open QGroundControl on ground station
2. Connect via telemetry radio
3. Verify:
   - Position on map
   - Compass heading correct
   - Battery voltage accurate
   - RC channels respond

---

## Step 6: First Flight Test

### Pre-Flight Checklist
- [ ] Props securely attached (check rotation direction!)
- [ ] Battery fully charged (16.8V for 4S)
- [ ] GPS lock (3D fix, HDOP < 2.0)
- [ ] Safety pilot ready with RC transmitter
- [ ] Flight area clear of people/obstacles
- [ ] Weather conditions suitable (< 15mph wind)

### Maiden Flight Procedure

**1. ARM in STABILIZE mode**
```bash
# Via command line
cule-arm --mode stabilize

# Or use RC: throttle down + yaw right for 2 seconds
```

**2. Test Manual Control**
- Throttle up slowly until hovering (~50%)
- Test pitch, roll, yaw responses
- Verify drone stays level

**3. Test Altitude Hold**
```bash
cule-mode alt_hold
```
- Release throttle stick (center)
- Drone should maintain altitude

**4. Test Loiter (Position Hold)**
```bash
cule-mode loiter
```
- Release all sticks (center)
- Drone should hold position

**5. First Mission (Optional)**
```bash
# Simple takeoff-hover-land mission
cule-mission simple-test
```

**6. Return to Launch**
```bash
cule-mode rtl
```

**7. Land and Disarm**
```bash
cule-land
cule-disarm
```

---

## Next Steps

### Learn More
- [Flight Modes](./flight-modes.md) - Master all flight modes
- [Mission Planning](./missions.md) - Create autonomous missions
- [Safety Procedures](./safety.md) - Critical safety information

### Advanced Features
```bash
# Enable obstacle avoidance
cule-config --obstacle-avoidance on

# Enable precision landing
cule-config --precision-landing on

# Set up follow-me mode
cule-config --follow-me on
```

### Common Commands
```bash
# Check flight controller parameters
cule-param list

# View live telemetry
cule-telemetry

# View camera feed
cule-camera view

# Download flight logs
cule-logs download
```

---

## Troubleshooting

### Drone won't arm
```bash
# Check why
cule-prearm-check

# Common fixes:
sudo cule-calibrate accel  # Re-calibrate
sudo cule-calibrate compass  # Re-calibrate compass
```

### GPS not locking
- Move outside with clear sky view
- Wait 5 minutes (cold start)
- Check GPS cable connection
- Verify baud rate: `cule-param show GPS_TYPE`

### Compass errors
```bash
# Check interference
cule-compass-interference

# Calibrate away from metal objects
sudo cule-calibrate compass
```

### Camera not working
```bash
# Check camera detected
v4l2-ctl --list-devices

# Test gstreamer pipeline
gst-launch-1.0 nvarguscamerasrc ! nvoverlaysink
```

---

## Emergency Procedures

### Immediate Disarm (Crash)
```bash
# Emergency stop all motors
cule-disarm --force
```

### Lost Communication
- Drone automatically enters RTL after 5 seconds
- If battery low, executes emergency land
- Set failsafe actions: `cule-config --failsafe rtl`

### Flyaway Recovery
1. Switch to STABILIZE mode immediately
2. Take manual control
3. Fly back visually
4. Land and disarm
5. Check GPS/compass calibration

---

## Support

**Need help?**
- 📖 Full Docs: https://sanjaysparker27.github.io/cule-os/docs
- 💬 Forum: https://discussion.cule-os.io
- 🐛 Issues: https://github.com/SanjaySparker27/cule-os/issues
- 💻 Matrix Chat: #cule-os:matrix.org

**Happy flying! 🚁**
