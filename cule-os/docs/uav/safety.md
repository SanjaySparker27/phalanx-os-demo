# Cule OS UAV - Safety Procedures

**Critical safety information for autonomous drone operations.**

⚠️ **READ THIS BEFORE FLYING**

---

## Legal Requirements

### Regulations (India)
- **NPNT** (No Permission No Takeoff) - Required for most operations
- **DGCA** approval for commercial operations
- **UAOP** (Unmanned Aircraft Operator Permit) for commercial use
- Height limit: 120 meters (400 feet)
- Visual Line of Sight (VLOS) required
- No flying near airports (5km radius)
- No flying over crowds or sensitive areas

### Check Before Every Flight
- [ ] Drone registration completed
- [ ] Flight permission obtained (if required)
- [ ] Insurance active
- [ ] Weather conditions suitable

---

## Pre-Flight Safety Checklist

### Equipment Check
- [ ] Battery voltage > 16.0V (4S) or > 12.0V (3S)
- [ ] Propellers securely attached (check CW/CCW direction)
- [ ] No visible damage to frame or arms
- [ ] Camera and gimbal secured
- [ ] GPS antenna clear of obstructions
- [ ] Telemetry radio antenna connected
- [ ] SD card inserted (for logging)

### System Check
- [ ] GPS: 3D fix with 8+ satellites
- [ ] Compass: Normal variance (< 10%)
- [ ] Accelerometer: Calibrated
- [ ] Gyroscope: Stable readings
- [ ] Battery: Healthy, no swelling
- [ ] RC: All channels responding
- [ ] Telemetry: Link established

### Environment Check
- [ ] Wind speed < 10 m/s (22 mph)
- [ ] Visibility > 1 km
- [ ] No precipitation
- [ ] No thunderstorms within 10km
- [ ] Clear of obstacles (trees, power lines, buildings)
- [ ] No people or animals in flight area
- [ ] Emergency landing zone identified

---

## Emergency Procedures

### Immediate Actions

#### Loss of Control
1. **Switch to STABILIZE mode** - Take manual control
2. **Land immediately** - Find clear area
3. **Disarm** - Cut power if crashing

```bash
# Emergency disarm (all motors stop immediately)
cule-disarm --force
```

#### Flyaway
1. **Switch to RTL** - Return home immediately
2. **If RTL fails, switch to STABILIZE** - Manual recovery
3. **Note wind direction** - May be carrying drone
4. **Head toward launch point** - Manual return

#### Battery Critical
1. **Let AUTO mode handle it** - Drone will RTL then land
2. **DO NOT fight the descent** - Drone knows battery best
3. **Prepare for landing** - Clear landing zone
4. **If far from home** - Switch to LAND mode immediately

#### GPS Failure
1. **Switch to ALT_HOLD** - Manual position control
2. **Fly visually** - Land immediately
3. **DO NOT switch to LOITER** - Requires GPS
4. **If lost, use RTL** - May work if GPS recovers

#### Compass Error
1. **Switch to STABILIZE** - Disable compass use
2. **Fly manually** - Use visual orientation
3. **Land immediately** - Compass is critical
4. **Calibrate before next flight** - `sudo cule-calibrate compass`

---

## Failsafe Configuration

### Radio Failsafe
```bash
# Enable RTL on signal loss
cule-param set FS_THR_ENABLE 1

# Set failsafe threshold (PWM microseconds)
cule-param set FS_THR_VALUE 975

# Timeout before failsafe triggers
cule-param set FS_THR_TIMEOUT 1
```

### Battery Failsafe
```bash
# RTL at low battery
cule-param set FS_BATT_ENABLE 1

# Warning voltage (4S battery)
cule-param set FS_BATT_VOLTAGE 14.4

# Critical voltage
cule-param set FS_BATT_VOLTAGE 13.8

# Minimum capacity (mAh)
cule-param set FS_BATT_MAH 1000
```

### Geofence
```bash
# Enable geofence
cule-param set FENCE_ENABLE 1

# Circular boundary (meters from home)
cule-param set FENCE_RADIUS 500

# Maximum altitude (meters)
cule-param set FENCE_ALT_MAX 120

# Action when breached (0=report, 1=RTL, 2=land)
cule-param set FENCE_ACTION 1
```

---

## Weather Guidelines

### Safe Conditions
| Parameter | Limit | Notes |
|-----------|-------|-------|
| Wind Speed | < 10 m/s | Gusts up to 12 m/s |
| Visibility | > 1 km | Clear line of sight |
| Temperature | 0°C to 40°C | Battery performance affected |
| Precipitation | None | Water damage risk |
| Cloud Base | > 150m | Maintain VLOS |

### Dangerous Conditions - DO NOT FLY
- ⚡ Thunderstorms or lightning
- 🌪️ Wind gusts > 15 m/s
- 🌧️ Heavy rain
- 🌫️ Fog (visibility < 500m)
- ❄️ Icing conditions
- 🌡️ Extreme heat (> 45°C)

---

## Maintenance Safety

### Before Maintenance
1. **Remove propellers** - Never work with props on
2. **Disconnect battery** - Main power off
3. **Wait 30 seconds** - Capacitors discharge
4. **Use ESD protection** - Static sensitive components

### Battery Safety
- Store at 50% charge (storage voltage)
- Never leave charging unattended
- Use fireproof battery bag
- Inspect for swelling before each flight
- Dispose of damaged batteries properly

### Propeller Safety
- **ALWAYS** remove before calibration
- Check for cracks or chips
- Verify CW/CCW orientation
- Torque to specifications
- Replace if damaged

---

## Operational Safety

### Pilot Responsibilities
- Maintain visual line of sight at all times
- Keep emergency controller ready
- Monitor battery and system status
- Yield to manned aircraft
- Respect privacy of others

### Safety Pilot
- Must have manual override capability
- Briefed on emergency procedures
- Able to take control instantly
- Positioned for best visibility

### Bystander Safety
- Minimum 30m distance from takeoff/landing
- No overflight of unprotected people
- Clear area before arming
- Announce intentions verbally

---

## Incident Response

### Crash Procedures
1. **Disarm immediately** - Cut power
2. **Check for fire** - Battery fire risk
3. **Secure area** - Keep people away
4. **Document damage** - Photos/video
5. **Download logs** - `cule-logs download`
6. **Report if required** - Per regulations

### Lost Drone
1. **Last known GPS position** - Check telemetry log
2. **RTL may have engaged** - Check home location
3. **Search pattern** - Grid search from last position
4. **Telemetry range** - Walk with ground station

### Emergency Contacts
```
Local Emergency: 112
Cule OS Support: support@cule-os.io
Community Forum: https://discussion.cule-os.io
```

---

## Training Progression

### Beginner → Intermediate → Advanced

**Beginner (First 10 flights):**
- Fly only in STABILIZE mode
- Low altitude (< 10m)
- Open areas only
- Safety pilot required
- No autonomous modes

**Intermediate (10-50 flights):**
- ALT_HOLD and LOITER modes
- Medium altitude (< 50m)
- Simple waypoint missions
- Solo flight permitted

**Advanced (50+ flights):**
- Full autonomous missions
- High altitude operations
- Complex maneuvers
- Swarm operations

---

## Safety Checklist Template

Print and use before every flight:

```
DATE: _________ PILOT: _________ LOCATION: _________

PRE-FLIGHT:
□ Battery voltage: _____V (min 16.0V)
□ GPS satellites: _____ (min 8)
□ Propellers secure: CW / CCW correct
□ Control surfaces: Free movement
□ Camera: Recording
□ Telemetry: Connected

WEATHER:
□ Wind: _____ m/s (< 10)
□ Visibility: _____ m (> 1000)
□ Precipitation: None
□ Temperature: _____°C

EMERGENCY:
□ Landing zone identified
□ Safety pilot briefed
□ Emergency procedure reviewed
□ Phone charged

ARMING CHECK:
□ Area clear of people
□ Final compass check
□ Mode selected: _________

POST-FLIGHT:
□ Battery remaining: _____%
□ Download logs: Yes / No
□ Damage check: Pass / Fail
□ Next maintenance: _________
```

---

## Resources

### Official Documentation
- [Getting Started](./quickstart.md)
- [Flight Modes](./flight-modes.md)
- [Emergency Procedures](./emergency.md)

### Regulatory
- DGCA CAR Section 3, Series X Part I & II
- Local aviation authority guidelines

### Training
- Cule OS Simulator: `cule-sim launch`
- QGroundControl simulator
- Hands-on training courses

---

**Remember: Safety is your responsibility. When in doubt, don't fly.**

⚠️ **Violating safety procedures can result in crashes, injury, or legal consequences.**
