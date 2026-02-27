# Cule OS UAV - Mission Planning

Create, upload, and execute autonomous flight missions.

## Overview

Cule OS supports full autonomous waypoint missions including:
- Survey patterns for mapping
- Delivery routes
- Inspection paths
- Search and rescue grids
- Cinematography waypoints

---

## Quick Start

### Simple Survey Mission

```bash
# Create a rectangular survey
cule-mission survey \
  --bounds "37.7749,-122.4194,37.7759,-122.4184" \
  --altitude 50 \
  --overlap 80

# Upload to drone
cule-mission upload

# Execute
cule-mode auto
```

---

## Mission Planning Methods

### 1. QGroundControl (Recommended)

Graphical mission planning with point-and-click interface.

**Steps:**
1. Open QGroundControl
2. Connect to vehicle
3. Click **Plan** tab
4. Add waypoints on map
5. Set altitude for each
6. Upload to vehicle
7. Switch to AUTO mode

**Survey Tool:**
1. Plan → Survey → Draw polygon
2. Set altitude and overlap
3. Set camera trigger distance
4. Generate waypoints
5. Upload

### 2. Cule CLI

Command-line mission creation for automation.

```bash
# Create new mission
cule-mission create --name "delivery_route"

# Add takeoff
cule-mission add-takeoff --alt 20

# Add waypoints
cule-mission add-waypoint --lat 37.7749 --lon -122.4194 --alt 30
cule-mission add-waypoint --lat 37.7759 --lon -122.4184 --alt 30
cule-mission add-waypoint --lat 37.7769 --lon -122.4174 --alt 30

# Add land
cule-mission add-land --lat 37.7749 --lon -122.4194

# Save and upload
cule-mission save
cule-mission upload
```

### 3. Python API

Programmatic mission creation.

```python
from cule import UAV, Mission, Waypoint

# Connect to drone
drone = UAV.connect('/dev/ttyUSB0')

# Create mission
mission = Mission(name="inspection")

# Add waypoints
mission.add_takeoff(altitude=20)
mission.add_waypoint(
    lat=37.7749, 
    lon=-122.4194, 
    alt=25,
    delay=5  # Wait 5 seconds
)
mission.add_waypoint(
    lat=37.7759, 
    lon=-122.4184, 
    alt=25,
    roi=(37.7754, -122.4189)  # Point camera here
)
mission.add_land()

# Upload and execute
drone.upload_mission(mission)
drone.set_mode('AUTO')
drone.arm()
```

### 4. File Import

Import missions from standard formats.

```bash
# Import QGroundControl plan
cule-mission import survey.plan

# Import MAVProxy waypoint file
cule-mission import waypoints.txt --format mavproxy

# Import CSV
cule-mission import waypoints.csv --format csv
```

**CSV Format:**
```csv
lat,lon,alt,command,delay
37.7749,-122.4194,20,TAKEOFF,0
37.7759,-122.4184,30,WAYPOINT,0
37.7769,-122.4174,30,WAYPOINT,5
37.7749,-122.4194,0,LAND,0
```

---

## Mission Commands

### Navigation Commands

| Command | Description | Parameters |
|---------|-------------|------------|
| **TAKEOFF** | Climb to altitude | altitude |
| **WAYPOINT** | Navigate to point | lat, lon, alt, delay |
| **SPLINE_WAYPOINT** | Smooth curved path | lat, lon, alt |
| **LOITER_TIME** | Hold for time | time, radius |
| **LOITER_TURNS** | Orbit location | turns, radius |
| **RTL** | Return to launch | altitude |
| **LAND** | Land at location | lat, lon |

### Camera Commands

| Command | Description | Parameters |
|---------|-------------|------------|
| **DO_SET_ROI** | Point camera | lat, lon, alt |
| **DO_DIGICAM_CONTROL** | Trigger camera | on/off |
| **DO_SET_CAM_TRIGG_DIST** | Auto-trigger | distance |
| **DO_MOUNT_CONTROL** | Gimbal control | pitch, roll, yaw |

### Utility Commands

| Command | Description | Parameters |
|---------|-------------|------------|
| **DELAY** | Wait | time_sec |
| **DO_JUMP** | Jump to command | command#, repeat |
| **DO_CHANGE_SPEED** | Change speed | speed, accel |
| **DO_SET_RELAY** | Control relay | relay#, on/off |
| **DO_PARACHUTE** | Deploy parachute | enable/disable |

---

## Mission Examples

### Agricultural Survey

```python
from cule import Mission

mission = Mission("field_survey")

# Takeoff
mission.add_takeoff(altitude=30)

# Survey pattern
mission.add_survey(
    polygon=[
        (37.7749, -122.4194),
        (37.7749, -122.4094),
        (37.7649, -122.4094),
        (37.7649, -122.4194)
    ],
    altitude=30,
    overlap=80,
    sidelap=70,
    trigger_distance=20
)

# Return
mission.add_rtl()
mission.upload()
```

### Building Inspection

```bash
# Create inspection mission around building
cule-mission create --name "building_inspection"
cule-mission add-takeoff --alt 20

# Circle building at 3 levels
cule-mission add-loiter --lat 37.7749 --lon -122.4194 --alt 30 --time 60 --radius 20
cule-mission add-loiter --lat 37.7749 --lon -122.4194 --alt 20 --time 60 --radius 20
cule-mission add-loiter --lat 37.7749 --lon -122.4194 --alt 10 --time 60 --radius 20

cule-mission add-land
cule-mission upload
```

### Delivery Route

```python
mission = Mission("delivery")
mission.add_takeoff(altitude=30)

# Transit to delivery zone
mission.add_waypoint(lat=37.7849, lon=-122.4294, alt=50)

# Descend for drop
mission.add_waypoint(lat=37.7849, lon=-122.4294, alt=10, delay=3)
mission.add_command("DO_SET_SERVO", channel=9, pwm=1500)  # Drop payload
mission.add_delay(2)
mission.add_command("DO_SET_SERVO", channel=9, pwm=1000)  # Reset servo

# Return
mission.add_rtl(altitude=50)
mission.upload()
```

### Search Pattern

```bash
# Lawnmower search pattern
cule-mission search \
  --bounds "37.7749,-122.4194,37.7849,-122.4094" \
  --altitude 40 \
  --spacing 50 \
  --overlap 10
```

---

## Advanced Features

### Dynamic Waypoints

Update waypoints during flight:

```python
# Modify next waypoint in flight
drone.update_waypoint(
    index=3,
    lat=37.7759,
    lon=-122.4184,
    alt=40
)
```

### Conditional Commands

```python
# Only execute if battery > 50%
mission.add_conditional(
    condition="battery > 50",
    command="DO_DIGICAM_CONTROL",
    trigger=True
)
```

### Event Triggers

```bash
# Trigger on geofence entry
cule-mission add-trigger \
  --type geofence \
  --action "DO_DIGICAM_CONTROL" \
  --params "trigger=1"

# Trigger on altitude
cule-mission add-trigger \
  --type altitude \
  --value 100 \
  --action "DO_CHANGE_SPEED" \
  --params "speed=500"
```

---

## Mission Monitoring

### Real-time Status

```bash
# Watch mission progress
cule-mission monitor

# Output:
# Mission: survey_field
# Progress: 12/45 waypoints
# Current: WAYPOINT (37.7752, -122.4189, 50m)
# Next: WAYPOINT (37.7755, -122.4184, 50m)
# ETA: 8 minutes
```

### Python Monitoring

```python
from cule import UAV

drone = UAV.connect('/dev/ttyUSB0')

# Mission progress callback
def on_progress(current, total, waypoint):
    pct = (current / total) * 100
    print(f"Progress: {pct:.1f}% - At waypoint {current}")

drone.on_mission_progress = on_progress

# Mission complete callback
def on_complete():
    print("Mission complete!")
    drone.rtl()

drone.on_mission_complete = on_complete
```

---

## Safety Features

### Geofence

```bash
# Set cylindrical geofence
cule-geofence set \
  --center "37.7749,-122.4194" \
  --radius 500 \
  --max-alt 120 \
  --action RTL
```

### Return-to-Launch

```bash
# Configure RTL behavior during missions
cule-param set RTL_ALT 5000        # 50m RTL altitude
cule-param set RTL_LOIT_TIME 5000  # Loiter 5s before landing
```

### Battery Failsafe

```bash
# RTL at 25% battery
cule-param set FS_BATT_ENABLE 1
cule-param set FS_BATT_VOLTAGE 14.0
cule-param set FS_BATT_MAH 1000
```

---

## Best Practices

### Pre-Mission Checklist

- [ ] Verify GPS lock (6+ satellites)
- [ ] Check battery voltage
- [ ] Verify mission uploaded correctly
- [ ] Test in QGroundControl simulator first
- [ ] Set geofence boundaries
- [ ] Configure failsafe actions
- [ ] Brief safety pilot

### During Mission

- [ ] Monitor battery percentage
- [ ] Watch for traffic/obstacles
- [ ] Stay in telemetry range
- [ ] Be ready to switch to RTL
- [ ] Log all anomalies

### Post-Mission

- [ ] Download flight logs
- [ ] Review camera images
- [ ] Analyze flight data
- [ ] Note any issues

---

## Troubleshooting

### Mission Won't Start
```bash
# Check pre-arm
cule-prearm-check

# Verify home position
cule-param show HOME_LAT
cule-param show HOME_LNG

# Check GPS
cule-status | grep GPS
```

### Skipping Waypoints
```bash
# Increase waypoint radius
cule-param set WP_RADIUS 300  # 3 meters
```

### Not Triggering Camera
```bash
# Check trigger distance
cule-param show CAM_TRIGG_DIST

# Verify camera connected
cule-camera status
```

---

## File Formats

### QGroundControl (.plan)
```json
{
  "fileType": "Plan",
  "geoFence": {...},
  "mission": {
    "items": [
      {
        "command": 22,
        "params": [0,0,0,0,0,0,20],
        "type": "SimpleItem"
      }
    ]
  }
}
```

### MAVLink Waypoints (.txt)
```
QGC WPL 120
0	1	0	16	0	0	0	0	37.7749	-122.4194	0	1
1	0	3	22	0	0	0	0	0	0	20	1
2	0	3	16	0	0	0	0	37.7759	-122.4184	30	1
```

### Cule Native (.cule)
```yaml
name: survey_mission
version: 1.0
waypoints:
  - {type: takeoff, alt: 20}
  - {type: waypoint, lat: 37.7749, lon: -122.4194, alt: 30}
  - {type: land}
```
