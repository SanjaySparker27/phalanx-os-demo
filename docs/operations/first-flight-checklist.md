# First Flight Checklist

Comprehensive pre-flight checklist for the HEXA X + Pusher VTOL system.

## Table of Contents
- [Pre-Flight Preparation](#pre-flight-preparation)
- [Hardware Inspection](#hardware-inspection)
- [Power System Checks](#power-system-checks)
- [Software Verification](#software-verification)
- [Sensor Verification](#sensor-verification)
- [Communication Checks](#communication-checks)
- [Final Checks](#final-checks)
- [Takeoff Procedure](#takeoff-procedure)
- [Emergency Procedures](#emergency-procedures)

---

## Pre-Flight Preparation

### Weather Conditions

- [ ] Wind speed < 15 mph (for first flight, < 10 mph)
- [ ] No precipitation
- [ ] Visibility > 1 mile
- [ ] Temperature within battery operating range (15C - 45C)
- [ ] No thunderstorms within 5 miles

### Site Selection

- [ ] Open area > 100m x 100m
- [ ] No people within 50m
- [ ] No obstacles within 30m
- [ ] Flat, level surface for takeoff/landing
- [ ] Clear approach and departure paths
- [ ] Permission to fly at location

### Required Equipment

| Item | Quantity | Status |
|------|----------|--------|
| Fully charged flight battery | 1 | [ ] |
| Transmitter (charged) | 1 | [ ] |
| Ground station laptop | 1 | [ ] |
| Telemetry radio (ground) | 1 | [ ] |
| Spare props | 2 sets | [ ] |
| Prop wrench | 1 | [ ] |
| First aid kit | 1 | [ ] |
| Fire extinguisher (LiPo rated) | 1 | [ ] |
| Phone/emergency contact | 1 | [ ] |

---

## Hardware Inspection

### Airframe

- [ ] No cracks or damage to frame
- [ ] All screws tight (check with wrench)
- [ ] Arms securely attached
- [ ] Landing gear secure and level
- [ ] Payload properly secured
- [ ] No loose wires or components

### Motors and Props

- [ ] Props installed correctly (CW/CCW as marked)
- [ ] Prop nuts tight (nylon lock nuts)
- [ ] No cracks or nicks on props
- [ ] Motors spin freely (no grinding)
- [ ] Motor wires secured to frame
- [ ] No debris in motor bells

### Prop Installation Check

```
CCW Props (Counter-Clockwise)    CW Props (Clockwise)
   On motors: M1, M4, M5, M7        On motors: M2, M3, M6
   
   Leading edge curves               Leading edge curves
   toward rotation direction         toward rotation direction
   
   When looking from top:            When looking from top:
   Prop label faces UP               Prop label faces UP
```

### Flight Controller

- [ ] FC firmly mounted with vibration dampening
- [ ] Arrow pointing forward
- [ ] GPS/Compass arrow aligned with FC
- [ ] All connectors secure
- [ ] SD card inserted (for logging)
- [ ] Buzzer connected

### Companion Computer (Jetson)

- [ ] Jetson powered on
- [ ] Ethernet cable connected (if using)
- [ ] Camera lens clean
- [ ] LIDAR clear of obstructions
- [ ] All USB connections secure

---

## Power System Checks

### Battery Inspection

- [ ] Battery fully charged (25.2V for 6S)
- [ ] No swelling or damage
- [ ] Balance leads intact
- [ ] Main leads not frayed
- [ ] XT90 connector clean
- [ ] Battery straps secure

### Voltage Verification

```
Check with multimeter:
- Cell 1: _____ V (3.8-4.2V)
- Cell 2: _____ V (3.8-4.2V)
- Cell 3: _____ V (3.8-4.2V)
- Cell 4: _____ V (3.8-4.2V)
- Cell 5: _____ V (3.8-4.2V)
- Cell 6: _____ V (3.8-4.2V)
- Total: _____ V (22.8-25.2V)

Cell voltage deviation < 0.1V: [ ] Yes
```

### Power-Up Sequence

1. [ ] Power on transmitter first
2. [ ] Connect battery to vehicle
3. [ ] Listen for normal startup tones
4. [ ] Wait for GPS lock (solid blue LED)
5. [ ] Check FC LED: Blue = ready
6. [ ] Connect ground station telemetry

---

## Software Verification

### Ground Station Connection

- [ ] Mission Planner connected to vehicle
- [ ] Telemetry link established (>90% health)
- [ ] Heartbeat messages showing
- [ ] Parameter list loads successfully
- [ ] No error messages in console

### Flight Mode Verification

```
Test mode switching with RC8 slider:

Position       Expected Mode
0-16%          Loiter
>16%           Stabilize

Test Result: _______________
```

- [ ] Mode changes displayed in HUD
- [ ] Pusher motor responds to RC8 slider

### Pre-Arm Checks

In Mission Planner, check these are green:

- [ ] GPS: 3D Fix (8+ satellites)
- [ ] Compass: Healthy
- [ ] INS: Healthy
- [ ] Radio: Calibrated
- [ ] Battery: >22V

If any red, do NOT arm. Investigate and fix.

### Parameter Verification

Quick check of critical parameters:

```
FRAME_CLASS = 2         [ ] Verified
FRAME_TYPE = 13         [ ] Verified
FLTMODE_CH = 8          [ ] Verified
SERVO7_FUNCTION = 51    [ ] Verified
ARMING_CHECK = 0        [ ] Verified (or all pass)
```

---

## Sensor Verification

### GPS

- [ ] 3D Fix (not 2D)
- [ ] HDOP < 2.0
- [ ] Satellites: 8+ visible, 6+ used
- [ ] Position accurate (compare to known location)

### Compass

- [ ] Heading accurate (point nose north)
- [ ] No interference when tilting
- [ ] Stable values in Status tab

### Accelerometer

- [ ] Artificial horizon level when vehicle level
- [ ] Responds correctly to tilting

### Barometer

- [ ] Altitude reading reasonable
- [ ] Changes when lifting vehicle

---

## Communication Checks

### RC Link

- [ ] All channels responding
- [ ] Full range of motion (1000-2000)
- [ ] No jitter/stickiness
- [ ] Failsafe position set (throttle low, mode RTL)

### Telemetry Link

- [ ] Ground station receiving data
- [ ] Packet loss < 5%
- [ ] RSSI > -80 dBm
- [ ] Data rate stable

### Jetson Companion

- [ ] SSH accessible
- [ ] MAVProxy running (if used)
- [ ] Cule OS services active
- [ ] Camera feed visible (if applicable)

---

## Final Checks

### Pre-Arm Sequence

1. [ ] Throttle stick at minimum
2. [ ] RC8 slider at 0% (Loiter mode)
3. [ ] Arm switch in correct position
4. [ ] Press safety switch (if equipped)
5. [ ] Arm vehicle (hold arm switch or rudder right)
6. [ ] Verify "Armed" message
7. [ ] Motors should spin slowly (if MOT_SPIN_ARM enabled)

### Motor Test (Brief)

```
With vehicle armed but disarmed motors:
- Gently raise throttle to 10%
- All motors should spin smoothly
- Listen for unusual sounds
- Check for vibrations

Result: _______________
```

### Flight Plan

- [ ] Flight objective clear
- [ ] Maximum altitude set
- [ ] Return point identified
- [ ] Emergency landing zone identified
- [ ] Flight time limit set (battery capacity - 20%)

---

## Takeoff Procedure

### Initial Takeoff (First Flight)

1. **Position Check**
   - [ ] Vehicle on level surface
   - [ ] 5m clear in all directions
   - [ ] Takeoff into wind

2. **Takeoff Sequence**
   ```
   a) Arm vehicle
   b) Mode: Stabilize (RC8 >16%)
   c) Slowly increase throttle
   d) At 30% throttle, check lift-off
   e) Hover at 2-3 meters
   f) Check for stable hover
   g) Test basic controls (pitch, roll, yaw)
   h) Test mode switch to Loiter
   i) Land gently
   ```

3. **Post-Takeoff Checks** (while hovering)
   - [ ] Vehicle stable, not drifting
   - [ ] No unusual vibrations
   - [ ] Controls responsive
   - [ ] Battery voltage stable
   - [ ] Telemetry connected

### Normal Takeoff (Subsequent Flights)

```
1. Arm in Loiter mode (RC8 <16%)
2. Wait for GPS position hold confirmation
3. Slowly increase throttle
4. Vehicle should maintain position
5. Climb to desired altitude
6. Monitor for stable flight
```

---

## Emergency Procedures

### Loss of RC Signal

```
1. Vehicle will enter RTL (Return to Launch)
2. Do NOT re-take control immediately
3. Wait for RTL to complete
4. If needed, switch mode to regain control
```

### Loss of GPS

```
1. Switch to Stabilize or Alt Hold mode
2. Fly manually back to launch
3. Do NOT use Loiter or Auto mode
4. Land as soon as possible
```

### Low Battery

```
1. Vehicle will RTL automatically
2. If too far, manually fly back
3. Land immediately
4. Do NOT continue flying
```

### Motor Failure (HEXA advantage)

```
HEXA can fly with one motor out:
1. Vehicle will auto-compensate
2. Reduce throttle
3. Fly back to launch
4. Land immediately
```

### Emergency Landing

```
1. Find clear area
2. Switch to Stabilize mode
3. Reduce throttle gradually
4. Land gently
5. Disarm immediately
```

---

## Post-Flight

### Immediate Actions

1. [ ] Disarm vehicle
2. [ ] Disconnect battery
3. [ ] Power off transmitter
4. [ ] Download logs from SD card
5. [ ] Inspect vehicle for damage
6. [ ] Note any anomalies

### Log Review

```
Check in Mission Planner:
- Vibration levels
- GPS accuracy
- Battery consumption
- Mode changes
- Error messages
```

### Battery Storage

```
- Charge/discharge to storage voltage (3.8V/cell)
- Store in cool, dry place
- Use fireproof bag
- Check monthly voltage
```

---

## Checklist Sign-Off

```
Flight Date: _______________
Pilot: _______________
Location: _______________

Pre-flight checks completed: [ ] Yes  [ ] No
Weather conditions acceptable: [ ] Yes  [ ] No
Site conditions acceptable: [ ] Yes  [ ] No
All systems verified: [ ] Yes  [ ] No

CLEARED FOR FLIGHT: [ ] YES  [ ] NO

Notes: _____________________________________________
___________________________________________________

Pilot Signature: _______________
Observer Signature: _______________
```

---

## References

- [ArduPilot Pre-Flight Checklist](https://ardupilot.org/copter/docs/flying-arducopter.html)
- [FAA Drone Regulations](https://www.faa.gov/uas/)
- [Local Aviation Authority Guidelines]
