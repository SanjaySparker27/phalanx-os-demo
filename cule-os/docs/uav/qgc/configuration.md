# QGroundControl Configuration

## Overview

This guide covers advanced QGroundControl configuration options for optimal operation with Cule-OS vehicles.

## Application Settings

### General Settings

```
Menu: Q (top left) → Application Settings → General

Offline Maps:
- Download maps for areas of operation
- Select zoom levels (higher = more detail)
- Cache size management

Telemetry Logging:
- Enable auto-save
- Set log location
- Maximum log size

AutoConnect:
- USB: Enable for direct connection
- Pixhawk: Detect Cule-OS/PX4
- SiK Radio: Telemetry radios
- UDP: Network connections

Video:
- Source: UDP/RTSP/TCP
- UDP port: 5600 (typical)
- Aspect ratio
- Recording directory

Fly View:
- Show additional instruments
- Map scale
- Guided action confirmations
- Keep map centered
```

### Units

```
Menu: Application Settings → Units

Horizontal Distance:
- Metric (meters)
- Imperial (feet)

Vertical Distance:
- Metric (meters)
- Imperial (feet)

Area:
- Metric (m², hectares)
- Imperial (ft², acres)

Speed:
- Metric (m/s, km/h)
- Imperial (ft/s, mph, knots)

Temperature:
- Celsius
- Fahrenheit

Mixing units:
- Can set independently
- Example: metric distance, knots speed
```

### Offline Maps

```
Menu: Application Settings → Offline Maps

Download Maps:
1. Click Add
2. Name the map set
3. Navigate to area on map
4. Select zoom levels:
   - 1-10: Regional overview
   - 11-15: Local detail
   - 16-20: Street level
5. Click Download

Storage:
- Shows cache usage
- Clear old maps
- Manage storage location

Usage:
- Maps available offline
- Useful in remote areas
- Plan missions without internet
```

## Comm Link Configuration

### Serial Links

```
Menu: Application Settings → Comm Links → Add

Type: Serial

Settings:
- Port: COM3 (Windows) or /dev/ttyUSB0 (Linux)
- Baud Rate: 57600 (standard telemetry)
- Data Bits: 8
- Stop Bits: 1
- Parity: None
- Flow Control: None

Advanced:
- High latency link (satellite)
- Ping testing
- Auto-connect on start
```

### UDP Links

```
Type: UDP

Settings:
- Port: 14550 (standard)
- Host: 0.0.0.0 (listen all)
- Auto-connect: Enabled

Multiple vehicles:
- Use port 14550 for primary
- Use port 14551 for secondary
- Each vehicle to different port
```

### TCP Links

```
Type: TCP

Settings:
- Host: 192.168.1.10 (vehicle IP)
- Port: 5760 (standard)
- Auto-connect: As needed

Use for:
- WiFi connections
- Companion computers
- Long-range links
```

### Link Priorities

```
QGC prioritizes links:
1. USB (highest priority)
2. TCP/UDP
3. Serial telemetry

When USB connected:
- Telemetry link becomes backup
- USB used for primary control
- Both links active
```

## MAVLink Configuration

### Message Rates

```
Menu: Vehicle Setup → MAVLink

Standard Rates:
- Stream 0 (USB): High rate
- Stream 1 (radio): Reduced rate

Custom rates:
Set per-stream message rates
Example for telemetry radio:
- Raw sensors: 2 Hz
- Extended status: 2 Hz
- RC channels: 2 Hz
- Position: 2 Hz
- Extra 1: 4 Hz
- Extra 2: 4 Hz
- Extra 3: 2 Hz
```

### MAVLink Forwarding

```
Forward to secondary GCS:

1. Enable forwarding in QGC
2. Set target IP and port
3. Secondary GCS receives copy

Use cases:
- Instructor monitoring
- Multiple observers
- Distributed operations
```

## Video Configuration

### Video Sources

```
Auto-Detect:
- QGC tries common ports
- UDP 5600, 5601
- RTSP streams

Manual Configuration:
Source: UDP Video Stream
UDP Port: 5600
Aspect Ratio: Auto/16:9/4:3

Source: RTSP Video Stream
RTSP URL: rtsp://192.168.1.10:8554/live

Source: TCP Video Stream
TCP URL: tcp://192.168.1.10:5000
```

### Recording Settings

```
Video Recording:
- Enable recording
- Set directory
- File format: .mkv
- Auto-start on arm (optional)

Screenshot:
- Hotkey: Ctrl+S
- Save location
- Include telemetry overlay
```

### GStreamer Pipeline

```
Advanced video configuration:

Custom pipeline example:
v4l2src device=/dev/video0 ! video/x-h264 !
h264parse ! decodebin ! videoconvert !
autovideosink

For hardware decoding:
v4l2src ! video/x-h264 ! h264parse !
nvh264dec ! glimagesink
```

## Plan View Configuration

### Default Altitude

```
Menu: Plan → Gear icon → Default Mission Altitude

Set default altitude for:
- Takeoff
- Waypoints
- Survey patterns

Recommended:
- Above obstacles
- Below legal limits
- Account for terrain
```

### Survey Settings

```
Camera database:
- Select camera model
- Or add custom camera
- Focal length
- Sensor size
- Resolution

Overlap:
- Front overlap: 80%
- Side overlap: 70%
- Adjust for terrain

Ground speed:
- Match to conditions
- Consider wind
- Battery consumption
```

### Pattern Settings

```
Spiral:
- Center point
- Radius
- Loiter time
- Altitude change

Survey:
- Polygon based
- Camera triggering
- Automatic grid

Corridor Scan:
- Polyline based
- Width settings
- Multiple passes
```

## Fly View Configuration

### Instrument Panel

```
Menu: Fly → Settings (gear icon)

Widgets to show:
- Compass
- Attitude indicator
- Telemetry values
- Battery status
- GPS status
- RC signal
- Flight timer
- Video

Layout:
- Arrange as desired
- Resize widgets
- Save layout
```

### Guided Actions

```
Confirmations:
- Confirm guided takeoff
- Confirm guided land
- Confirm go to location
- Confirm change altitude

Safety:
- Prevent accidental commands
- Require confirmation for critical actions
```

### Map Display

```
Options:
- Street map
- Satellite imagery
- Terrain elevation
- Hybrid

Overlays:
- Airspace
- Traffic
- Weather
- TFRs (temporary flight restrictions)
```

## Analyze Configuration

### Log Download

```
Settings:
- Download path
- Auto-download on connect
- Delete after download

Format:
- .tlog (telemetry logs)
- .ulg (vehicle logs)
```

### MAVLink Console

```
Preferences:
- Font size
- Color scheme
- Scrollback buffer
- Command history
```

### Log Analysis

```
Tools:
- Flight Plot (external)
- PX4 Tools
- ULog analysis

Configuration:
- Default plot types
- Value ranges
- Export settings
```

## Customization

### Custom Branding

```
For custom builds:

1. Fork QGC repository
2. Modify resources:
   - Icons
   - Colors
   - Logo
   - Splash screen

3. Build custom version
4. Distribute to team
```

### Custom QML Widgets

```
Add custom instrument widgets:

1. Create QML file
2. Place in Widgets folder
3. Register in code
4. Available in instrument panel
```

### Toolbar Customization

```
Modify toolbar:
- Add/remove buttons
- Reorder items
- Custom actions
- Plugin architecture
```

## Backup and Restore

### Settings Backup

```
Settings location:

Windows:
%APPDATA%\QGroundControl.org\

macOS:
~/Library/Preferences/com.qgroundcontrol.qgroundcontrol.plist

Linux:
~/.config/QGroundControl.org/

Backup:
- Copy entire folder
- Store securely
- Version control
```

### Parameter Backup

```
From Vehicle Setup → Parameters:
1. Click Save to File
2. Choose location
3. Name with date
4. Store securely

Restore:
1. Click Load from File
2. Select backup
3. Review changes
4. Apply to vehicle
```

### Mission Backup

```
Save missions:
1. In Plan view
2. File → Save
3. Choose .plan format
4. Include all waypoints

Organize:
- By location
- By date
- By mission type
```

## Performance Tuning

### Graphics Performance

```
For low-end systems:

1. Disable video recording
2. Reduce map detail
3. Disable 3D view
4. Lower update rates
5. Close unused instruments

For high-end systems:
1. Enable all instruments
2. Full video quality
3. Maximum map detail
4. 3D terrain enabled
```

### Network Performance

```
Slow connections:
- Reduce stream rates
- Use TCP for reliability
- Enable compression
- Limit video quality

Fast connections:
- Maximize stream rates
- Use UDP for latency
- Full video bandwidth
```

## Troubleshooting

### Settings Reset

```
Reset to defaults:

Windows:
Delete %APPDATA%\QGroundControl.org\

macOS:
Delete plist file and:
~/Library/Preferences/QGroundControl.org/

Linux:
Delete ~/.config/QGroundControl.org/

Or use command line:
--reset-settings
```

### Performance Issues

```
Slow UI:
1. Update graphics drivers
2. Reduce display scaling
3. Close other applications
4. Increase RAM allocation

Connection lag:
1. Check link quality
2. Reduce message rates
3. Use wired connection
4. Update radio firmware
```

## Reference

- [QGroundControl Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_view.html)
- [Custom Build Guide](https://dev.qgroundcontrol.com)
