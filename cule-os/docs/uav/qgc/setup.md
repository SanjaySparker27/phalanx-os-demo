# QGroundControl Setup

## Overview

QGroundControl (QGC) is the ground control station for Cule-OS, providing vehicle configuration, mission planning, flight monitoring, and log analysis capabilities.

## Installation

### System Requirements

```
Minimum Requirements:
- OS: Windows 10, macOS 10.14, Ubuntu 18.04
- CPU: 64-bit dual-core 2 GHz
- RAM: 4 GB
- Storage: 500 MB
- Display: 1280x720

Recommended:
- CPU: Quad-core 2.5 GHz+
- RAM: 8 GB+
- Display: 1920x1080+
- SSD storage
```

### Download and Install

```
Official download: https://qgroundcontrol.com

Windows:
1. Download QGroundControl-installer.exe
2. Run installer
3. Follow prompts
4. Launch from Start menu

macOS:
1. Download QGroundControl.dmg
2. Open DMG
3. Drag to Applications
4. Launch from Applications

Linux (Ubuntu):
1. Download AppImage
2. chmod +x QGroundControl.AppImage
3. ./QGroundControl.AppImage

Or install from repository:
sudo apt install qgroundcontrol
```

## Initial Setup

### First Launch

```
1. Launch QGroundControl
2. Accept terms of use
3. Select language
4. Choose units:
   - Metric or Imperial
   - Apply to all measurements

5. Configure video:
   - Auto-detect or manual
   - Select video source (if applicable)

6. Set default location:
   - Type location name
   - Or use current position
```

### Connection Setup

```
USB Connection (most common):
1. Connect vehicle via USB
2. QGC should auto-detect
3. Vehicle appears in toolbar
4. Ready to configure

Serial Connection:
1. Connect telemetry radio
2. Select COM port in QGC
3. Set baud rate (usually 57600)
4. Connect

TCP Connection:
1. Go to Application Settings → Comm Links
2. Add new link
3. Type: TCP
4. Host: [vehicle IP]
5. Port: 5760 or 14550
6. Connect

UDP Connection:
1. Add UDP link
2. Port: 14550 (standard)
3. Auto-connect enabled
4. Vehicle broadcasts to this port
```

## Vehicle Configuration

### Firmware Flashing

```
1. Connect vehicle via USB
2. Go to Vehicle Setup → Firmware
3. Select "Advanced settings" if needed
4. Choose firmware type:
   - Standard (latest stable)
   - Beta (testing)
   - Custom file

5. For Cule-OS custom firmware:
   - Select "Custom firmware file"
   - Browse to cule-os.px4 file
   - Click OK

6. Wait for flash to complete
7. Vehicle will reboot
8. Reconnect QGC
```

### Airframe Selection

```
1. Go to Vehicle Setup → Airframe
2. Select airframe type:
   - Quadrotor X
   - Quadrotor +
   - Hexarotor X
   - Octarotor X
   - Fixed-wing
   - VTOL
   - etc.

3. Click Apply and Restart
4. Vehicle reboots with new configuration

Custom Airframe:
- If not in list, use closest match
- Fine-tune parameters manually
- Or create custom mixers
```

### Sensor Calibration

```
Accelerometer:
1. Vehicle Setup → Sensors
2. Select Accelerometer
3. Follow prompts for 6 positions:
   - Level
   - Left
   - Right
   - Nose down
   - Nose up
   - Upside down
4. Keep vehicle still during each

Gyroscope:
1. Select Gyroscope
2. Keep vehicle still
3. Wait for completion

Compass:
1. Select Compass
2. Rotate vehicle in all directions
3. Cover all orientations
4. Wait for 100%
5. Check for interference warnings

Level Horizon:
1. Place on level surface
2. Select Level Horizon
3. Wait for completion
```

### Radio Calibration

```
1. Vehicle Setup → Radio
2. Turn on transmitter
3. Verify transmitter bound to receiver
4. Click Calibrate
5. Follow prompts:
   - Move throttle up/down
   - Move roll left/right
   - Move pitch up/down
   - Move yaw left/right
   - Move mode switch
6. Click Next to save

Troubleshooting:
- If no response, check binding
- Verify receiver powered
- Check channel mapping
```

### Flight Modes

```
1. Vehicle Setup → Flight Modes
2. Select mode switch channel
3. Configure modes:
   - Position 1: Stabilize/Manual
   - Position 2: Altitude
   - Position 3: Position Hold
   - Position 4: Return
   - Position 5: Mission
   - Position 6: Offboard

4. Test switch positions
5. Verify mode changes in display
```

### Power Setup

```
1. Vehicle Setup → Power
2. Select battery type:
   - LiPo
   - Li-ion
   - LiFe

3. Configure cells:
   - Number of cells
   - Full voltage per cell
   - Empty voltage per cell

4. Set capacity:
   - Battery capacity in mAh
   - Reserved capacity

5. Configure failsafe:
   - Low battery action
   - Critical battery action
   - Return voltage

6. Calibrate voltage:
   - Measure with multimeter
   - Enter measured value
   - QGC calculates multiplier
```

## Safety Setup

### Failsafe Configuration

```
1. Vehicle Setup → Safety

2. RC Loss:
   - Set timeout (0.5s default)
   - Select action:
     * Return to Launch
     * Land
     * Hold

3. Data Link Loss:
   - Set timeout
   - Select action

4. Geofence:
   - Enable fence
   - Set max altitude
   - Set max radius
   - Select action on breach

5. Battery:
   - Low level warning
   - Critical level action
   - Emergency level action
```

### Return to Launch

```
1. Vehicle Setup → Safety → Return to Launch
2. Set return altitude:
   - Above obstacles
   - Consider terrain

3. Set loiter time at home
4. Enable/confirm land at home
5. Test RTL switch
```

## Tuning

### Basic Tuning

```
1. Vehicle Setup → PID Tuning
2. Select tuning axis:
   - Roll
   - Pitch
   - Yaw

3. Adjust gains:
   - P gain (responsiveness)
   - I gain (error correction)
   - D gain (damping)

4. Test flight between changes
5. Incremental adjustments only
```

### Auto-Tuning

```
Some versions support auto-tune:
1. Go to PID Tuning → Autotune
2. Select axes to tune
3. Arm and hover
4. Activate autotune
5. System will excite and measure
6. Save parameters when complete

⚠️ Use only in open area with space
```

## Mission Planning

### Creating Missions

```
1. Click Plan tab
2. Click takeoff waypoint
3. Add waypoints:
   - Click on map
   - Drag to position
   - Set altitude

4. Add actions:
   - Delay
   - Camera trigger
   - Change speed
   - Change altitude
   - ROI (region of interest)

5. Add landing pattern
6. Upload to vehicle
7. Verify on Flight view
```

### Survey Missions

```
1. Plan → Pattern → Survey
2. Draw polygon on map
3. Configure:
   - Camera type
   - Overlap (front/side)
   - Altitude
   - Ground speed

4. Generate grid
5. Review waypoints
6. Upload to vehicle
```

### Geofence

```
1. Plan → Geofence
2. Draw inclusion zone (must stay in)
3. Draw exclusion zones (must avoid)
4. Set altitude limits
5. Configure breach action
6. Upload to vehicle
```

## Flight Monitoring

### Main Flight Display

```
Instrument panel shows:
- Artificial horizon
- Compass heading
- Altitude (relative and MSL)
- Ground speed
- Airspeed (if equipped)
- Climb rate
- Battery voltage and %
- GPS status and satellites
- Flight mode
- Throttle position

Map view shows:
- Vehicle position
- Flight track
- Waypoints
- Home position
- Geofence
- Mission progress
```

### Video Display

```
Enable video:
1. Click video icon in toolbar
2. Configure video source:
   - Auto-detect
   - RTSP stream
   - TCP stream
   - UDP stream

3. Set URL (if manual):
   - rtsp://192.168.1.10:8554/live
   - udp://0.0.0.0:5600

4. Adjust display:
   - Picture in picture
   - Full screen
   - Recording
```

## Log Analysis

### Download Logs

```
1. Vehicle Setup → Analyze → Log Download
2. List available logs
3. Select logs to download
4. Choose destination folder
5. Click Download

Logs stored on SD card:
- /fs/microsd/log/
- .ulg format
```

### Review Logs

```
1. Analyze → Log Analysis
2. Load log file
3. View plots:
   - Attitude
   - Rates
   - Position
   - Battery
   - Vibration
   - Setpoints

4. Check for:
   - Errors
   - Warnings
   - Performance issues
   - Anomalies
```

### GeoTag Images

```
1. Analyze → GeoTag Images
2. Select log file
3. Select image folder
4. Match timestamp offset
5. Process images
6. Save geotagged images
```

## Advanced Features

### Parameters

```
Access full parameter list:
1. Vehicle Setup → Parameters
2. Search by name
3. Edit values
4. Save to vehicle

Export/Import:
- Save to file for backup
- Load from file
- Compare parameters
```

### Console

```
Access MAVLink console:
1. Analyze → MAVLink Console
2. Type commands:
   - commander status
   - gps status
   - sensors
   - param show
   - uorb top
   - work_queue status

3. View system messages
4. Debug in real-time
```

### Custom Widgets

```
Add custom instrument panels:
1. Settings → Instrument Panel
2. Add/remove widgets
3. Rearrange layout
4. Save configuration
```

## Troubleshooting

### Connection Issues

```
Problem: Vehicle not connecting

Solutions:
1. Check USB cable (data, not charge-only)
2. Try different USB port
3. Install drivers (Windows)
4. Check port permissions (Linux)
5. Verify correct port selected
6. Restart QGC and vehicle
```

### Firmware Flash Failed

```
Solutions:
1. Enter bootloader manually:
   - Hold BOOT button
   - Connect USB
   - Release BOOT

2. Try different USB port
3. Use different cable
4. Check for driver issues
5. Try QGC daily build
```

### Parameter Upload Failed

```
Solutions:
1. Reboot vehicle
2. Reconnect QGC
3. Try smaller batches
4. Check for read-only parameters
5. Verify parameter file format
```

## Reference

- [QGroundControl User Guide](https://docs.qgroundcontrol.com)
- [QGroundControl GitHub](https://github.com/mavlink/qgroundcontrol)
- [Daily Builds](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html)
