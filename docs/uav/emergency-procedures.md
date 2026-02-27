# Emergency Procedures

Critical procedures for handling UAV emergencies during flight operations.

## Table of Contents
- [Emergency Classification](#emergency-classification)
- [Loss of Control Link](#loss-of-control-link)
- [Loss of GPS Signal](#loss-of-gps-signal)
- [Motor Failure](#motor-failure)
- [Low Battery](#low-battery)
- [Flyaway Recovery](#flyaway-recovery)
- [Loss of Orientation](#loss-of-orientation)
- [Compass/Gyro Failure](#compassgyro-failure)
- [Vibration Issues](#vibration-issues)
- [Emergency Landing](#emergency-landing)
- [Crash Response](#crash-response)
- [Pre-Flight Emergency Checks](#pre-flight-emergency-checks)

---

## Emergency Classification

### Severity Levels

| Level | Description | Response Time | Example |
|-------|-------------|---------------|---------|
| **CRITICAL** | Immediate danger to people/property | < 5 seconds | Motor failure at low altitude |
| **HIGH** | Loss of control or navigation | < 10 seconds | GPS loss in Loiter mode |
| **MEDIUM** | Degraded performance | < 30 seconds | Single motor failure on HEXA |
| **LOW** | Warning condition | < 60 seconds | Low battery warning |

### Emergency Indicators

**Visual (LED Status):**
```
Fast Red Flashing    → Critical error
Slow Red Flashing    → Warning condition
Yellow Flashing      → GPS not ready
Blue Flashing        → Waiting for arming
Solid Green          → Armed and ready
```

**Audio (Buzzer Patterns):**
```
Continuous beep      → Critical battery
Double beep          → GPS lost
Triple beep          → Compass error
Long tone            → Armed
```

**Ground Station Messages:**
```
"Failsafe active"    → Failsafe triggered
"EKF variance"       → Position estimation error
"Bad GPS health"     → GPS quality degraded
"Crash detected"     → Impact detected
```

---

## Loss of Control Link

### RC Signal Loss (Radio Failsafe)

**Detection:**
- No response to stick inputs
- Ground station shows "No RC receiver"
- FC enters failsafe mode automatically

**Automatic Response (Pre-Configured):**
```
1. FS_THR_ENABLE = 1 → RTL activated
2. Vehicle returns to launch position
3. Lands automatically
```

**Pilot Recovery Procedure:**

```
IMMEDIATE ACTIONS (0-10 seconds):
1. Check transmitter power (ON?)
2. Check transmitter antenna orientation
3. Move to higher ground for line of sight
4. Do NOT turn off transmitter

RECOVERY ACTIONS (10-30 seconds):
1. Wait for RTL to complete
2. If RTL in progress, allow it to continue
3. Once vehicle returns, regain control
4. Land normally

MANUAL OVERRIDE:
If RTL is not appropriate:
1. Change flight mode on transmitter (if link restored)
2. Take manual control
3. Fly visually back
```

**Prevention:**
```
RC Setup:
- FS_THR_ENABLE = 1        (Enable throttle failsafe)
- FS_THR_VALUE = 975       (Trigger threshold)
- Range check before flight (>500m)
- Fresh transmitter batteries
- Antenna orientation vertical
```

### Ground Station Link Loss

**Detection:**
- "No heartbeat" messages
- Telemetry timeout warnings
- Last known position stale

**Automatic Response:**
```
FS_GCS_ENABLE = 1          (RTL on GCS timeout)
FS_GCS_TIMEOUT = 5         (5 second timeout)
```

**Recovery:**
```
1. Check telemetry radio power
2. Check antenna connections
3. Reboot ground station software
4. Vehicle will RTL automatically
5. Do NOT power cycle vehicle
```

---

## Loss of GPS Signal

### Detection

```
Ground Station Indicators:
- "GPS Glitch" or "Bad GPS health"
- HDOP > 2.0
- Satellites < 6
- EKF position variance high
```

### Immediate Response

**If in Loiter/RTL/Auto Mode:**

```
CRITICAL (< 3 seconds):
1. Switch to Stabilize or Alt Hold mode
   - Use RC8 slider >16% for Stabilize
2. Take manual control immediately

RECOVERY (3-30 seconds):
3. Fly visually back to launch
4. Climb for better GPS reception
5. Monitor for GPS restoration
6. Once GPS restored, can resume GPS modes

LANDING (30+ seconds):
7. If GPS not restored, land manually
8. Use Alt Hold for altitude control
9. Land in Stabilize if comfortable
```

**If in Stabilize/Alt Hold Mode:**

```
Continue flight as normal:
1. GPS loss does not affect manual modes
2. Monitor for GPS restoration
3. Avoid switching to GPS-dependent modes
4. Land at earliest opportunity
```

### HEXA X Advantage

```
HEXA can fly without GPS:
- Manual modes work perfectly
- No position hold capability
- Requires pilot skill for navigation
- Can fly home using visual reference
```

### GPS Glitch Protection

```
Parameters:
GPS_GLITCH_RADIUS = 200     # Glitch detection radius (cm)
GPS_DELAY_MS = 2000         # Delay before declaring glitch
EKF_GLITCH_RADIUS = 500     # EKF glitch threshold

EKF Actions on GPS Loss:
1. Switch to dead reckoning
2. Use inertial sensors only
3. Position estimate degrades over time
4. Switches to failsafe after timeout
```

### Emergency GPS Recovery

```
If GPS lost at altitude:
1. Switch to Alt Hold
2. Circle slowly to acquire satellites
3. Check for interference sources
4. Move away from:
   - Tall buildings
   - Power lines
   - Radio towers
   - Underground structures
```

---

## Motor Failure

### HEXA X Redundancy

```
Motor Layout:
      M5    M6
        \  /
         \/
    M3--X--M4
         /\
        /  \
      M1    M2

Failure Tolerance:
- 1 motor failed: Can maintain flight
- 2 motors failed: May be controllable
- 3+ motors failed: Emergency landing required
```

### Single Motor Failure Response

**Detection:**
```
Symptoms:
- Vehicle yaws/rolls unexpectedly
- Increased motor noise on remaining motors
- Ground station shows motor current imbalance
- "Motor failure detected" message
```

**Response Procedure:**

```
IMMEDIATE (0-2 seconds):
1. Do NOT panic - HEXA can fly on 5 motors
2. Reduce throttle slightly
3. Switch to Stabilize mode if not already
4. Prepare for reduced performance

STABILIZATION (2-10 seconds):
5. Center all sticks
6. Allow FC to compensate automatically
7. Vehicle will increase power on opposite motor
8. Expect slower response and drift

RECOVERY (10-30 seconds):
9. Fly slowly back to launch
10. Avoid aggressive maneuvers
11. Maintain altitude - descending easier than climbing
12. Plan for immediate landing

LANDING (30+ seconds):
13. Approach landing zone slowly
14. Switch to Land mode if GPS available
15. Otherwise manual landing in Stabilize
16. Be prepared for harder landing
```

### Double Motor Failure

```
If opposite motors fail (M1+M4 or M2+M5 or M3+M6):
→ Immediate emergency landing
→ May have minimal control
→ Prepare for crash

If adjacent motors fail:
→ Likely uncontrollable
→ Prepare for crash
→ Initiate emergency stop if approaching people
```

### Emergency Motor Stop

```
To immediately stop all motors:

Option 1: Disarm
- Throttle minimum + Rudder full right (for 2 seconds)
- OR use arm/disarm switch

Option 2: Emergency stop
- Set MOT_SPIN_ARMED = 0 (if accessible via GCS)

WARNING: Only use if:
- Vehicle is crashing
- Danger to people/property
- Complete loss of control
```

---

## Low Battery

### Battery Monitoring

```
Warning Levels:
- Low Battery:    19.8V (3.3V/cell) → RTL
- Critical:       18.0V (3.0V/cell) → Land immediately
- Emergency:      17.4V (2.9V/cell) → Emergency land
```

### Low Battery Response

```
Detection:
- Ground station "Low battery" warning
- Voltage display turning red
- Audible beeps from FC
- Automatic RTL initiated

Response:
1. If RTL started: ALLOW IT TO CONTINUE
2. Do NOT cancel RTL unless absolutely necessary
3. Monitor voltage during RTL
4. If voltage drops too fast, manually land sooner

Manual Override:
If too far to return:
1. Switch to Stabilize/Alt Hold
2. Find nearest safe landing spot
3. Land immediately
4. Do NOT try to stretch flight
```

### Critical Battery Response

```
Detection:
- "Critical battery" message
- Rapid voltage drop
- Reduced power output

Response:
1. LAND IMMEDIATELY
2. Any mode - just get down
3. Prefer Land mode if available
4. Manual control if necessary
5. Expect sudden power loss

WARNING:
- Lithium batteries can be damaged below 3.0V/cell
- Risk of fire if over-discharged
- Battery may be unusable after deep discharge
```

---

## Flyaway Recovery

### Causes of Flyaway

```
Common Causes:
1. Compass interference (magnetic declination)
2. GPS glitch with wrong position
3. Vibration affecting IMU
4. Wind exceeding vehicle capability
5. Pilot disorientation
6. Automatic mode with wrong waypoint
```

### Immediate Response

```
RECOGNITION:
- Vehicle flying away despite controls
- Not responding to stick inputs correctly
- Heading wrong direction
- Increasing distance from pilot

IMMEDIATE ACTIONS:
1. Switch to STABILIZE mode immediately
   - RC8 slider >16%
2. Take manual control
3. Orient vehicle visually
4. Fly directly toward yourself

If you cannot see vehicle:
1. Use ground station map
2. Note heading indicator
3. Fly toward home position
4. Reduce altitude to increase visibility
```

### Compass-Induced Flyaway

```
Symptoms:
- Vehicle flies in circles
- Yaw drift during flight
- Wrong heading displayed

Response:
1. Switch to Stabilize
2. Fly using visual reference only
3. Ignore compass heading
4. Land immediately
5. Recalibrate compass before next flight
```

### GPS-Induced Flyaway

```
Symptoms:
- RTL flies wrong direction
- Position hold drifts significantly
- Loiter mode moves vehicle

Response:
1. Switch to non-GPS mode (Stabilize/Alt Hold)
2. Fly manually using visual reference
3. Land immediately
4. Check for GPS interference
```

---

## Loss of Orientation

### Spatial Disorientation

```
Common in:
- High altitude flight
- Distance flight
- Low light conditions
- Similar terrain features
```

### Recovery Technique

```
1. Switch to LOITER mode (if GPS available)
   - RC8 slider <16%
2. Stop all stick inputs
3. Let vehicle stabilize
4. Check ground station map
5. Orient using:
   - Sun position
   - Ground features
   - Ground station heading indicator
6. Fly toward home position on map
```

### Visual Reference Technique

```
If no GPS:
1. Switch to Alt Hold
2. Make gentle control inputs
3. Observe vehicle response
4. "Push" vehicle away to see response
5. Once oriented, fly toward yourself
6. Use ground station video if available
```

---

## Compass/Gyro Failure

### Compass Failure

```
Detection:
- "Compass variance" error
- Erratic yaw behavior
- Toilet bowling in Loiter
- Wrong heading indication

Response:
1. Switch to Stabilize mode
2. Fly without compass assistance
3. Manual yaw control
4. Land immediately
5. Check for:
   - Metal objects near compass
   - Power lines
   - Magnetic interference
```

### Gyro/Accel Failure

```
Detection:
- "Bad gyro health"
- "Bad accel health"
- Erratic attitude
- Impossible to hover
- EKF errors

Response:
1. Land immediately
2. May be uncontrollable
3. Prepare for crash
4. Do NOT attempt to fly again
5. Hardware inspection required
```

---

## Vibration Issues

### High Vibration Effects

```
Symptoms:
- Altitude hold oscillation
- Position drift
- Motor output saturation
- Reduced flight time
- IMU clipping
```

### In-Flight Response

```
1. Switch to Stabilize mode
2. Reduce throttle
3. Land immediately
4. Do NOT continue flight
5. Post-flight inspection:
   - Check prop balance
   - Check motor mounts
   - Check for loose screws
```

---

## Emergency Landing

### Emergency Landing Sites

```
Priority (best to worst):
1. Original launch site
2. Clear open field
3. Road (without traffic)
4. Parking lot
5. Roof (flat)
6. Water (if equipped with floats)
7. Trees (last resort)

AVOID:
- Crowded areas
- Power lines
- Water (without floats)
- Steep terrain
- Moving vehicles
```

### Emergency Landing Procedure

```
1. Identify landing site
2. Announce "Emergency landing"
3. Approach at safe altitude
4. Switch to Land mode (if GPS available)
5. If no GPS, manual landing:
   a. Approach into wind
   b. Reduce altitude gradually
   c. Slow forward speed
   d. Level vehicle before touchdown
   e. Cut throttle on contact
   f. Disarm immediately
```

### Forced Landing (No Power)

```
If complete power loss:
1. Pitch for best glide (slight nose down)
2. Choose landing site
3. Avoid obstacles
4. Level before impact
5. Throttle cut before touchdown
6. Brace for impact
```

---

## Crash Response

### Immediate Actions

```
1. Ensure safety of people first
2. Approach cautiously
3. Disarm if not already
4. Disconnect battery
5. Check for fire/smoke
6. Secure the area
```

### Post-Crash Procedures

```
Safety:
- Wait 5 minutes for battery stabilization
- Handle LiPo with care
- Watch for swelling/heat
- Have fire extinguisher ready

Documentation:
- Photograph scene
- Note weather conditions
- Record battery voltage
- Download flight logs
- Preserve evidence

Vehicle Assessment:
- Check for structural damage
- Check motor/ESC damage
- Check electronics
- Do NOT power on if damaged
```

### Fire Response

```
LiPo Battery Fire:
1. Evacuate area
2. Call emergency services
3. Use Class D fire extinguisher
4. If no extinguisher: 
   - Sand or dirt to smother
   - Do NOT use water directly
   - Water from distance to cool surrounding area
5. Stay upwind of smoke
6. Battery may reignite
```

---

## Pre-Flight Emergency Checks

### Failsafe Verification

```
Before each flight:

1. Radio Failsafe Test:
   - Arm vehicle in Loiter
   - Switch to Stabilize
   - Turn off transmitter
   - Verify RTL initiates
   - Turn transmitter back on
   - Regain control

2. GPS Failsafe Test:
   - Enable GPS debug
   - Verify HDOP < 2.0
   - Check satellite count > 8
   - Verify position accuracy

3. Battery Failsafe Test:
   - Verify voltage reading accurate
   - Check failsafe thresholds set
   - Test low voltage alarm
```

### Emergency Equipment

```
Required:
- Fire extinguisher (LiPo rated)
- First aid kit
- Cell phone
- Transmitter with full battery
- Spare props
- Tool kit

Recommended:
- Fireproof battery bag
- Smoke detector
- Emergency landing pad
- GPS tracker
```

---

## Emergency Contact Information

```
Emergency Services: 911
Local Air Traffic Control: _______
Flight Operations Manager: _______
Insurance Provider: _______

MAVLink Emergency Frequencies:
- 915 MHz (Region 2)
- 868 MHz (Region 1)
```

---

## Quick Reference: Emergency Actions

```
EMERGENCY          | IMMEDIATE ACTION              | PREVENTION
-------------------|-------------------------------|---------------------------
RC Loss            | RTL auto/Wait for return      | Range check, fresh batteries
GPS Loss           | Switch to Stabilize           | Pre-flight GPS check
Motor Failure      | Reduce throttle, land         | Pre-flight motor test
Low Battery        | Allow RTL or land now         | Voltage check, timer
Flyaway            | Switch to Stabilize           | Compass cal, GPS check
Disorientation     | Switch to Loiter, stop inputs | Stay within visual range
Compass Error      | Switch to Stabilize, land     | Compass cal, interference check
Vibration          | Land immediately              | Prop balance, tight screws
Crash              | Safety first, disconnect batt | Pre-flight inspection
```

---

## References

- [ArduPilot Failsafe Documentation](https://ardupilot.org/copter/docs/failsafe-landing-page.html)
- [Emergency Procedures Guide](https://ardupilot.org/copter/docs/emergency-procedures.html)
- [FAA Emergency Procedures](https://www.faa.gov/uas/)
