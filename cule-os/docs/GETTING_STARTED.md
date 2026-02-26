# Getting Started with Cule OS

Welcome! This guide will help you get Cule OS running on your vehicle in 30 minutes.

## What You Need

### Hardware
- **Computer:** NVIDIA Jetson OR Raspberry Pi 4/5
- **Vehicle:** UAV (drone), UGV (rover), or USV (boat)
- **Flight Controller:** Pixhawk (ArduPilot/PX4 compatible)
- **Camera:** USB webcam or CSI camera (Raspberry Pi Camera)
- **GPS:** u-blox NEO-M8N or better (optional but recommended)

### For UAV (Drone)
- Multirotor or fixed-wing frame
- Motors + ESCs
- Battery (3S-6S LiPo)
- RC transmitter

### For UGV (Rover)
- Chassis with motors
- Motor driver
- Battery
- RC transmitter or gamepad

### For USV (Boat)
- Waterproof hull
- Thrusters
- Waterproof electronics enclosure
- Battery

## Quick Start (10 Minutes)

### Step 1: Download Cule OS

Choose your edition:

```bash
# For NVIDIA Jetson (best performance)
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-jetson-v1.0.img.xz

# For Raspberry Pi 4/5
wget https://sanjaysparker27.github.io/cule-os/releases/cule-os-rpi-v1.0.img.xz
```

### Step 2: Flash to SD Card

**Using BalenaEtcher (Easiest):**
1. Download [BalenaEtcher](https://www.balena.io/etcher/)
2. Select your downloaded .img.xz file
3. Insert SD card (32GB or larger)
4. Click "Flash"

**Using Command Line:**
```bash
# Linux/Mac
xzcat cule-os-jetson-v1.0.img.xz | sudo dd of=/dev/sdX bs=4M status=progress

# Windows (using WSL)
wsl xzcat cule-os-jetson-v1.0.img.xz | wsl dd of=/dev/sdX bs=4M
```

### Step 3: First Boot

1. Insert SD card into Jetson/RPi
2. Connect flight controller via USB
3. Connect camera
4. Power on
5. Wait 2-3 minutes for first boot

### Step 4: Connect to Cule OS

**Via Network:**
```bash
# Find IP address
# Default hostname: cule-os
ssh cule@cule-os.local
# Password: culeos
```

**Via Direct Connection:**
- Connect monitor and keyboard
- Login: `cule` / Password: `culeos`

### Step 5: Configure Your Vehicle

```bash
# Run configuration wizard
sudo cule-config

# Select your vehicle type
1. UAV (Drone)
2. UGV (Rover)  
3. USV (Boat)
```

**For UAV:**
```bash
sudo cule-config --type uav --frame quad --motors 4
```

**For UGV:**
```bash
sudo cule-config --type ugv --drive skid --wheels 4
```

**For USV:**
```bash
sudo cule-config --type usv --hull mono --thrusters 2
```

### Step 6: Calibrate Sensors

```bash
# Calibrate IMU
sudo cule-calibrate imu

# Calibrate camera
sudo cule-calibrate camera

# Calibrate GPS (if equipped)
sudo cule-calibrate gps
```

### Step 7: Test in Simulation

```bash
# Launch simulation
roslaunch cule_simulation test.launch

# In another terminal, test autonomous mode
cule-test-mode --mode autonomous --duration 30
```

### Step 8: First Flight/Drive/Sail

**SAFETY FIRST:**
1. Test in MANUAL mode first
2. Have safety pilot ready with RC transmitter
3. Start in ASSISTED mode (AI helps, human controls)
4. Only then try AUTONOMOUS mode

```bash
# Start Cule OS agents
sudo systemctl start cule-os

# Check status
cule-status

# Expected output:
# ✓ Kernel Module: Loaded
# ✓ Perception Agent: Running
# ✓ Planning Agent: Running
# ✓ Control Agent: Running
# ✓ Autopilot: Connected
```

## Next Steps

- [Read the full documentation](index.md)
- [Connect your sensors](hardware.md)
- [Train custom AI models](TRAINING.md)
- [Join the community](https://discussion.cule-os.io)

## Troubleshooting

### Can't Connect to WiFi
```bash
# Use cule-os-config tool
sudo cule-config --wifi
```

### Camera Not Detected
```bash
# Check camera
v4l2-ctl --list-devices

# If not listed, check connection and reboot
sudo reboot
```

### Flight Controller Not Connecting
```bash
# Check connection
lsusb

# Should show: STMicroelectronics (Pixhawk)
# If not, check USB cable
```

## Getting Help

- **Documentation:** https://sanjaysparker27.github.io/cule-os/docs
- **Community Forum:** https://discussion.cule-os.io
- **Matrix Chat:** #cule-os:matrix.org
- **GitHub Issues:** https://github.com/SanjaySparker27/cule-os/issues

## Video Tutorials

Coming soon! For now, see examples/ directory in the repository.

---

**Welcome to autonomous futures! 🚁🚙🚢**