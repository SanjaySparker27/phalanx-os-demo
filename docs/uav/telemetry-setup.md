# Telemetry Setup Guide

Complete guide to telemetry systems, radio configuration, and data link optimization for the ATHENA H743 PRO.

## Table of Contents
- [Overview](#overview)
- [Radio Hardware](#radio-hardware)
- [SiK Radio Configuration](#sik-radio-configuration)
- [MAVLink Configuration](#mavlink-configuration)
- [Ground Station Setup](#ground-station-setup)
- [Companion Computer Telemetry](#companion-computer-telemetry)
- [Long-Range Telemetry](#long-range-telemetry)
- [Multi-Vehicle Setup](#multi-vehicle-setup)
- [Troubleshooting](#troubleshooting)

---

## Overview

### Telemetry Functions

```
Telemetry provides:
├── Real-time flight data
│   ├── Position (GPS)
│   ├── Attitude
│   ├── Battery status
│   └── System health
├── Command uplink
│   ├── Mode changes
│   ├── Parameter updates
│   └── Mission upload
├── Data logging
│   ├── Flight logs
│   ├── Diagnostics
│   └── Analytics
└── Video integration
    ├── FPV feed
    └── Gimbal control
```

### Frequency Bands

| Band | Frequency | Range | Use Case |
|------|-----------|-------|----------|
| 915 MHz | 902-928 MHz | 1-10 km | Americas, Australia |
| 868 MHz | 868-870 MHz | 1-8 km | Europe |
| 433 MHz | 433-435 MHz | 5-20 km | Long range, license required |
| 2.4 GHz | 2.4-2.5 GHz | 0.5-2 km | WiFi telemetry |

---

## Radio Hardware

### SiK Radio (Standard)

```
Specifications:
- Frequency: 915 MHz / 868 MHz / 433 MHz
- Output Power: 100 mW (20 dBm) standard
- Data Rate: 57.6 kbps - 250 kbps
- Range: 300m - 10km (depending on antennas)
- Protocol: MAVLink
- Configuration: AT commands
```

**Components:**
- Air module (connects to flight controller)
- Ground module (connects to ground station)
- Antennas (usually included)

### Connection to Flight Controller

```
ATHENA H743 PRO Serial Port Mapping:

Serial1 (UART1)  → GPS
Serial2 (UART2)  → TELEMETRY (recommended)
Serial3 (UART3)  → Reserve
Serial4 (UART4)  → OSD
Serial5 (UART5)  → RC input (protocol)
Serial6 (UART6)  → Reserve
Serial7 (UART7)  → USB

Telemetry Wiring:
FC TX2 (Pin)  → Air Radio RX
FC RX2 (Pin)  → Air Radio TX
FC 5V/GND     → Air Radio Power
```

### ArduPilot Configuration

```
# Serial port for telemetry
SERIAL2_PROTOCOL = 2        # MAVLink2
SERIAL2_BAUD = 921          # 921600 baud

# Stream rates (messages per second)
SR2_ADSB = 0
SR2_EXT_STAT = 2            # Extended status (2 Hz)
SR2_EXTRA1 = 4              # Attitude (4 Hz)
SR2_EXTRA2 = 4              # VFR_HUD (4 Hz)
SR2_EXTRA3 = 2              # AHRS (2 Hz)
SR2_PARAMS = 10             # Parameters (10 Hz)
SR2_POSITION = 2            # Position (2 Hz)
SR2_RAW_CTRL = 0
SR2_RAW_SENS = 2            # Raw sensors (2 Hz)
SR2_RC_CHAN = 2             # RC channels (2 Hz)
```

---

## SiK Radio Configuration

### Factory Reset

```
Connect ground radio to computer via USB

Using Mission Planner:
1. Initial Setup → Optional Hardware → Sik Radio
2. Click "Load Settings"
3. Click "Reset to Defaults"
4. Click "Save Settings"

Or using AT terminal:
ATZ         # Reset to factory
AT&W        # Save settings
```

### Optimal Settings

```
# Mission Planner Sik Radio Config

Local Settings (Ground Radio):
═══════════════════════════════
Format:            MAVLink
Air Speed:         128        # 128 kbps
ECC:               Enable     # Error correction
MAVLink:           Enable
Opportunistic Resend: Enable
Max Window:        80

Remote Settings (Air Radio):
═══════════════════════════════
Format:            MAVLink
Air Speed:         128
ECC:               Enable
MAVLink:           Enable
Min Frequency:     915000     # 915 MHz
Max Frequency:     928000
Num Channels:      50
Duty Cycle:        100
LBT RSSI:          0
Max Window:        80
```

### AT Command Reference

```
Basic Commands:
ATI         # Show radio version
ATI2        # Show board type
ATI3        # Show board frequency
ATI4        # Show board version
ATI5        # Show all parameters
ATI6        # Show TDM timing
ATI7        # Show RSSI

Configuration:
ATS0?       # Get serial speed
ATS0=57     # Set serial speed to 57.6k
ATS1?       # Get air speed
ATS1=128    # Set air speed to 128k
ATS2?       # Get net ID
ATS2=25     # Set network ID to 25
ATS3?       # Get TX power
ATS3=20     # Set TX power to 20 (100mW)
ATS4?       # Get ECC
ATS4=1      # Enable ECC
ATS5?       # Get MAVLink mode
ATS5=1      # Enable MAVLink framing

Frequency (915 MHz):
ATS8=915000     # Min frequency (Hz)
ATS9=928000     # Max frequency (Hz)
ATS10=50        # Number of channels

Save and Reboot:
AT&W        # Save settings
ATZ         # Reboot radio
```

### Frequency Configuration by Region

```
# Americas / Australia (915 MHz)
ATS8=902000
ATS9=928000
ATS10=50

# Europe (868 MHz)
ATS8=868000
ATS9=870000
ATS10=5

# Long Range (433 MHz) - License Required
ATS8=433050
ATS9=434790
ATS10=10
```

---

## MAVLink Configuration

### Protocol Versions

```
MAVLink 1:
- Older protocol
- Smaller packets
- Less features
- Compatible with older GCS

MAVLink 2:
- Modern protocol
- Larger packet size (255 bytes vs 263)
- Better error checking
- Required for new features
- Recommended for all new setups
```

**Configuration:**
```
SERIAL2_PROTOCOL = 2    # MAVLink2
```

### Stream Rate Optimization

```
# Conservative (long range, low bandwidth)
SR2_POSITION = 1        # 1 Hz position
SR2_EXTRA1 = 2          # 2 Hz attitude
SR2_EXTRA2 = 1          # 1 Hz HUD
SR2_EXTRA3 = 1          # 1 Hz AHRS
SR2_RAW_SENS = 0        # Disable raw sensors
SR2_EXT_STAT = 1        # 1 Hz extended status

# Balanced (standard use)
SR2_POSITION = 2
SR2_EXTRA1 = 4
SR2_EXTRA2 = 4
SR2_EXTRA3 = 2
SR2_RAW_SENS = 2
SR2_EXT_STAT = 2

# High bandwidth (short range, diagnostics)
SR2_POSITION = 4
SR2_EXTRA1 = 10
SR2_EXTRA2 = 10
SR2_EXTRA3 = 4
SR2_RAW_SENS = 4
SR2_EXT_STAT = 4
```

### Custom MAVLink Messages

```python
# Custom telemetry message example
import pymavlink.mavutil as mavutil

# Define custom message
class CustomTelemetry:
    def __init__(self):
        self.battery_temp = 0
        self.motor_temps = [0, 0, 0, 0, 0, 0]
        self.pusher_pwm = 0
    
    def pack(self):
        return struct.pack('<I6Hh',
            int(time.time() * 1000),  # timestamp
            *self.motor_temps,         # 6 motor temps
            self.pusher_pwm            # pusher motor pwm
        )

# Send at 1 Hz
while True:
    custom_msg = CustomTelemetry()
    custom_msg.battery_temp = read_battery_temp()
    custom_msg.motor_temps = read_motor_temps()
    custom_msg.pusher_pwm = get_pusher_pwm()
    
    master.mav.named_value_float_send(
        int(time.time() * 1000),
        b'CUSTOM01',
        custom_value
    )
    time.sleep(1)
```

---

## Ground Station Setup

### Mission Planner Configuration

```
Connection Setup:
1. Select COM port (SiK radio)
2. Set baud rate: 57600
3. Click Connect

Display Settings:
- Flight Data tab: Enable desired instruments
- Configuration → Planner:
  - Set update rate: 4 Hz
  - Enable speech: optional
  - Check "Connect on startup"

Telemetry Logs:
- Auto-enable: tlogs folder
- Review: Ctrl+F → "Review Log"
```

### QGroundControl Setup

```
Connection:
1. Application Settings → Comm Links
2. Add new link
3. Type: Serial
4. Port: Select SiK radio COM port
5. Baud: 57600
6. Protocol: MAVLink

Auto-Connect:
- Enable "Auto-Connect on Start"
- Add link to auto-connect list

Video Setup (if using):
1. Application Settings → Video
2. Source: Auto/UDP/RTSP
3. UDP Port: 5600 (typical)
```

### Multiple Ground Stations

```
# Primary GCS: Full control
# Secondary GCS: Monitoring only

Configuration:
SERIAL2_OPTIONS = 0     # Default
FOLLOW_ME_ENABLE = 0    # Disable conflicting features

Use UDP broadcast for multiple GCS:
SERIAL2_PROTOCOL = 2
SERIAL2_BAUD = 921600

On companion computer:
mavproxy.py --master=/dev/ttyTHS1 \
            --baudrate=921600 \
            --out=udp:192.168.1.255:14550 \
            --out=udp:192.168.1.255:14551
```

---

## Companion Computer Telemetry

### Jetson Integration

```
Hardware Connection:
Jetson UART (/dev/ttyTHS1) → Level Shifter → FC Serial2

Wiring:
Jetson TX (Pin 8)  → Level Shifter → FC RX2
Jetson RX (Pin 10) → Level Shifter → FC TX2
Jetson GND (Pin 6) → FC GND

Note: Jetson is 3.3V logic, FC is typically 5V tolerant
Use level shifter for safety
```

### MAVProxy Setup

```bash
# Install MAVProxy
sudo pip install mavproxy

# Basic connection
mavproxy.py --master=/dev/ttyTHS1 --baudrate 921600

# With ground station output
mavproxy.py --master=/dev/ttyTHS1 \
            --baudrate 921600 \
            --out=udp:192.168.1.100:14550 \
            --aircraft MyDrone

# Multi-output configuration
mavproxy.py --master=/dev/ttyTHS1 \
            --baudrate 921600 \
            --out=udp:127.0.0.1:14550 \
            --out=tcpin:0.0.0.0:5760 \
            --out=/dev/ttyUSB0:57600 \
            --daemon
```

### Cule OS Telemetry Integration

```python
# telemetry_handler.py
from cule_os.agents import CommunicationAgent
from pymavlink import mavutil

class TelemetryHandler:
    def __init__(self, device='/dev/ttyTHS1', baud=921600):
        self.master = mavutil.mavlink_connection(device, baud=baud)
        self.master.wait_heartbeat()
        
        # Communication agent
        self.comm = CommunicationAgent()
        
    def telemetry_loop(self):
        while True:
            msg = self.master.recv_match(blocking=True)
            
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                position = {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.relative_alt / 1000
                }
                self.comm.publish_position(position)
                
            elif msg.get_type() == 'ATTITUDE':
                attitude = {
                    'roll': msg.roll,
                    'pitch': msg.pitch,
                    'yaw': msg.yaw
                }
                self.comm.publish_attitude(attitude)
                
            elif msg.get_type() == 'SYS_STATUS':
                battery = {
                    'voltage': msg.voltage_battery / 1000,
                    'current': msg.current_battery / 100,
                    'remaining': msg.battery_remaining
                }
                self.comm.publish_battery(battery)

# Run
handler = TelemetryHandler()
handler.telemetry_loop()
```

---

## Long-Range Telemetry

### RFD900 Radio Setup

```
High-Range Radio:
- Frequency: 902-928 MHz
- Power: 1W (30 dBm)
- Range: 10-40 km
- Configuration: Via AT or Mission Planner

Settings:
Air Speed: 64 kbps        # Lower for range
Max Window: 131           # Larger window
Encryption: Optional
```

### Antenna Optimization

```
Air Radio Antenna:
- Type: Dipole or Cloverleaf
- Gain: 2-3 dBi
- Mount: Vertical, away from electronics
- Quality: Low SWR (<1.5)

Ground Radio Antenna:
- Type: Yagi or high-gain dipole
- Gain: 6-9 dBi
- Mount: Elevated, clear line of sight
- Tracking: Point toward aircraft

Directional Setup:
- Yagi antenna on ground
- Antenna tracker for long missions
- Automatic or manual pointing
```

### Range Optimization Checklist

```
1. Radio Power:
   [ ] Set to maximum legal limit
   [ ] Check duty cycle regulations
   [ ] Verify thermal management

2. Antenna:
   [ ] Quality antennas on both ends
   [ ] Correct polarization (vertical)
   [ ] No damage or corrosion
   [ ] Secure connections

3. Installation:
   [ ] Away from other electronics
   [ ] Clear line of sight
   [ ] Vertical orientation
   [ ] Not touching carbon fiber

4. Environment:
   [ ] Clear line of sight
   [ ] Minimal interference
   [ ] Ground station elevated
   [ ] Avoid flying behind obstacles

5. Configuration:
   [ ] ECC enabled
   [ ] Optimal air speed
   [ ] Frequency hopping enabled
   [ ] Correct region settings
```

---

## Multi-Vehicle Setup

### Network ID Configuration

```
Each vehicle needs unique NET_ID:

Vehicle 1:
ATS2=1          # Network ID 1

Vehicle 2:
ATS2=2          # Network ID 2

Vehicle 3:
ATS2=3          # Network ID 3

Ground radio:
ATS2=0          # Receive all (broadcast)
```

### MAVProxy Multi-Vehicle

```bash
# Vehicle 1
mavproxy.py --master=/dev/ttyUSB0:57600 \
            --aircraft=Vehicle1 \
            --out=udp:127.0.0.1:14550

# Vehicle 2 (in new terminal)
mavproxy.py --master=/dev/ttyUSB1:57600 \
            --aircraft=Vehicle2 \
            --out=udp:127.0.0.1:14551

# Connect Mission Planner to UDP ports
```

---

## Troubleshooting

### Common Issues

**No Connection:**
```
Symptoms:
- "No heartbeat" message
- Cannot connect

Solutions:
1. Check baud rate matches
2. Verify correct COM port
3. Check radio LED status
4. Verify radio pairing (same NET_ID)
5. Check cable connections
6. Test with default settings
```

**Intermittent Connection:**
```
Symptoms:
- Connection drops
- High packet loss
- Slow updates

Solutions:
1. Check antenna connections
2. Verify antenna orientation
3. Reduce stream rates
4. Enable ECC
5. Reduce air speed for range
6. Check for interference
7. Verify power supply (noise)
```

**Short Range:**
```
Symptoms:
- Connection lost at short distance
- Poor link quality

Solutions:
1. Check antenna quality
2. Verify antenna mounting
3. Increase TX power
4. Reduce air speed
5. Check for obstacles
6. Verify frequency band
7. Check coaxial cables
```

### Diagnostic Commands

```
Check RSSI:
ATI7        # Show RSSI and noise floor

Good RSSI: > -80 dBm
Fair RSSI: -80 to -90 dBm
Poor RSSI: < -90 dBm

Check Error Rate:
ATI6        # Show TDM timing and errors

Good: < 1% error rate
Fair: 1-5% error rate
Poor: > 5% error rate
```

### LED Status Codes

```
Radio LED Patterns:

Rapid Red Blink:     No connection to other radio
Slow Red Blink:      Transmitting data
Solid Red:           Boot mode / Firmware update
Green Blink:         Receiving data
Solid Green:         Paired and connected

Sequence at Power:
1. Red on (boot)
2. Red blink (searching)
3. Green blink (paired)
4. Green solid (ready)
```

---

## Telemetry Best Practices

```
Pre-Flight:
1. Power on ground station first
2. Verify connection at aircraft
3. Check link quality > 90%
4. Verify all data streaming
5. Test range before takeoff

During Flight:
1. Monitor link quality
2. Watch for packet loss
3. Keep ground antenna pointed
4. Avoid obstacles between antennas
5. Have backup control (RC ready)

Post-Flight:
1. Save telemetry logs
2. Analyze for issues
3. Check max distance achieved
4. Review any dropouts
5. Plan improvements
```

---

## References

- [SiK Radio Documentation](https://ardupilot.org/copter/docs/common-sik-telemetry-radio.html)
- [MAVLink Protocol](https://mavlink.io/)
- [MAVProxy Documentation](https://ardupilot.org/mavproxy/)
- [Long Range Telemetry](https://ardupilot.org/copter/docs/common-rfd900.html)
