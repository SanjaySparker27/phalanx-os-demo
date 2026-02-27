# Telemetry Radios

Long-range communication systems for ground control and vehicle monitoring.

## Telemetry Overview

Telemetry radios provide a data link between the vehicle and ground control station (GCS), enabling:
- Real-time flight monitoring
- Parameter configuration
- Mission planning and upload
- Video streaming (with appropriate radios)
- Remote control (as backup)

## Radio Types

### Frequency Bands

| Band | Frequency | Range | Penetration | Use Case |
|------|-----------|-------|-------------|----------|
| 433MHz | 433-435 MHz | 15-30km | Excellent | Long range, Europe |
| 915MHz | 902-928 MHz | 10-20km | Very Good | Americas, medium range |
| 2.4GHz | 2.400-2.485 GHz | 5-15km | Good | Global, video capable |
| 5.8GHz | 5.725-5.850 GHz | 2-8km | Poor | Video only, short range |

### Radio Comparison

| Radio | Band | Range | Rate | Power | Features | Price |
|-------|------|-------|------|-------|----------|-------|
| RFD900x | 900MHz | 40km | 250kbps | 1W | Bi-directional, configurable | $$ |
| RFD900+ | 900MHz | 20km | 250kbps | 500mW | Reliable, proven | $$ |
| SiK 915MHz | 915MHz | 5-10km | 57kbps | 100mW | Open source, cheap | $ |
| SiK 433MHz | 433MHz | 10-15km | 57kbps | 100mW | Europe compliant | $ |
| Herelink | 2.4GHz | 20km | HD Video | 1W | All-in-one, Android | $$$ |
| 4G/LTE | Cellular | Unlimited | 10+ Mbps | Variable | Long range, cloud | $$/mo |
| WiFi | 2.4/5GHz | 1-2km | 54+ Mbps | 100mW | Short range, high bandwidth | $ |

## RFD900x

### Specifications

```
RFD900x Long-Range Radio
══════════════════════════════════════════════════════════════════

Frequency: 902-928 MHz (configurable)
Output Power: 100mW - 1000mW (30dBm)
Air Data Rate: 4-250 kbps
Serial Data Rate: 2400-921600 bps
Range: Up to 40km (line of sight)
Supply Voltage: 5V
Current Draw: 1.5A peak (TX), 50mA (RX)
Antenna: SMA connector, dipole or directional
Weight: 25g

Features:
- Frequency hopping spread spectrum
- AES encryption
- Store-and-forward
- Multi-point network capable
- MAVLink aware
```

### Pinout

| Pin | Function | Notes |
|-----|----------|-------|
| 1 | GND | Ground |
| 2 | 5V | Power input |
| 3 | TX | Serial output (to FC RX) |
| 4 | RX | Serial input (from FC TX) |
| 5 | CTS | Flow control (optional) |
| 6 | RTS | Flow control (optional) |

### Configuration

```bash
# Connect to radio via USB
# Use RFD Tools or Mission Planner

# Essential settings:
# - Baud rate: 57600 (match FC TELEM2)
# - Air speed: 128 (higher = faster but shorter range)
# - Max window: 131 (throughput optimization)
# - Tx power: 20-30 (country dependent)

# Frequency settings:
# - Min frequency: 915000
# - Max frequency: 928000
# - Num channels: 50

# Save and reboot
```

## SiK Radio

### Specifications

```
SiK Telemetry Radio (3DR/Holybro)
══════════════════════════════════════════════════════════════════

Frequency: 433MHz or 915MHz (region dependent)
Output Power: 100mW (20dBm)
Air Data Rate: 57 kbps fixed
Serial Data Rate: 57600 default
Range: 5-10km (line of sight)
Supply Voltage: 5V
Current Draw: 500mA peak
Antenna: 2dBi dipole
Weight: 15g

Features:
- Open source firmware
- Configurable via AT commands
- RSSI reporting
- Low cost
```

### AT Command Configuration

```bash
# Connect radio via USB-UART adapter
# Open terminal at 57600 baud

# Enter AT mode
+++

# Check version
ATI
# RFD SiK 1.9 on HM-TRP

# View all settings
ATI5

# Change air speed (affects range)
ATS4=64   # 64k air speed

# Change TX power
ATS5=20   # 20dBm (100mW)

# Change frequency
ATS3=915000  # Center frequency

# Save and reboot
AT&W
ATZ
```

## Herelink

### Specifications

```
Herelink HD Video & Control System
══════════════════════════════════════════════════════════════════

Frequency: 2.4GHz (5.8GHz WiFi for local)
Range: 20km (FCC), 12km (CE)
Video: 1080p60, 720p90
Latency: <110ms
Control: 16 channels + wheel
Output Power: 1W (30dBm)
Ground Unit: Android-based controller
Air Unit: 88x48x18mm, 75g

Features:
- HD video transmission
- Joystick control
- QGroundControl built-in
- RTP/RTSP streaming
- Solex TX support
- Dual band WiFi
```

### Setup

```
Herelink Connection Diagram
══════════════════════════════════════════════════════════════════

Air Unit (On Vehicle)                Ground Unit (Controller)
┌─────────────────────┐              ┌─────────────────────┐
│  2.4GHz Radio       │◄════════════►│  2.4GHz Radio       │
│  HDMI Input         │              │  5.5" Display       │
│  UART/SBUS Output   │──────────────►│  HDMI Output        │
│  5V Power           │              │  Android OS         │
└─────────────────────┘              │  QGC Pre-installed  │
        │                            └─────────────────────┘
        │                                     │
        ├──────────────┬──────────────┐       │
        │              │              │       │
        ▼              ▼              ▼       ▼
   ┌─────────┐   ┌─────────┐   ┌─────────┐  USB
   │  HDMI   │   │  UART   │   │  SBUS   │  PC
   │ Camera  │   │  FC     │   │  FC     │
   └─────────┘   └─────────┘   └─────────┘

Connection:
1. Air unit UART → FC TELEM port (MAVLink)
2. Air unit HDMI → Camera HDMI
3. Air unit SBUS → FC RC input
4. Power: 5V/2A

Pairing:
1. Hold pair button on both units
2. Wait for solid green LED
3. Connection established
```

## 4G/LTE Telemetry

### Overview

```
4G/LTE Telemetry Architecture
══════════════════════════════════════════════════════════════════

Vehicle                               Internet               Ground
─────────────────────────────────────────────────────────────────

┌─────────────┐                      ┌─────────────┐      ┌─────────────┐
│   4G Modem  │                      │   Cloud     │      │    GCS      │
│  (Quectel)  │◄────────────────────►│   Server    │◄────►│  (QGC/MP)   │
│  USB/UART   │     Cellular         │  (VPN/RTSP) │      │  Laptop     │
└──────┬──────┘                      └─────────────┘      └─────────────┘
       │
       └─────────────────┐
                         │
              ┌──────────┼──────────┐
              │          │          │
              ▼          ▼          ▼
        ┌─────────┐ ┌─────────┐ ┌─────────┐
        │  FC     │ │Companion│ │ Video   │
        │ MAVLink │ │Computer │ │Encoder  │
        └─────────┘ └─────────┘ └─────────┘

Benefits:
- Unlimited range (cell coverage)
- High bandwidth (video streaming)
- Cloud integration
- Multiple vehicle monitoring

Requirements:
- 4G modem (USB or HAT)
- SIM card with data plan
- Cloud server or direct VPN
```

### Configuration

```bash
# Using wwan0 (Quectel EC25)
# 1. Install modem manager
sudo apt install modemmanager

# 2. Check modem detection
mmcli -L

# 3. Connect to network
sudo mmcli -m 0 --simple-connect="apn=internet"

# 4. Configure routing
sudo dhclient wwan0

# 5. Setup MAVLink router
mavlink-routerd -e <cloud_server_ip>:14550 \
                -e 127.0.0.1:14550 \
                /dev/ttyAMA0:921600

# Cloud server forwards to GCS
```

## MAVLink Setup

### Serial Configuration

```bash
# ArduPilot serial port setup
# For telemetry radio on TELEM1:

# Set baud rate
param set SERIAL1_BAUD 57  # 57600

# Set protocol
param set SERIAL1_PROTOCOL 1  # MAVLink1
param set SERIAL1_PROTOCOL 2  # MAVLink2 (recommended)

# Flow control (if supported)
param set SERIAL1_OPTIONS 0  # No flow control
param set SERIAL1_OPTIONS 8  # CTS/RTS enabled

# Reboot to apply
reboot
```

### Stream Rates

```bash
# Control MAVLink message rates
# SR1_ = Stream rate for SERIAL1

# Extended status (battery, etc.)
param set SR1_EXT_STAT 2  # 2Hz

# RC channels
param set SR1_RC_CHAN 2   # 2Hz

# Raw sensors
param set SR1_RAW_SENS 0  # Disabled (saves bandwidth)

# Position
param set SR1_POSITION 3  # 3Hz

# Extra1 (attitude)
param set SR1_EXTRA1 10   # 10Hz

# Extra2 (vfr hud)
param set SR1_EXTRA2 2    # 2Hz

# Extra3 (ahrs, etc.)
param set SR1_EXTRA3 2    # 2Hz
```

## Radio Range Testing

```
Range Testing Procedure
══════════════════════════════════════════════════════════════════

1. Ground Test
   - Verify link at 10m distance
   - Check RSSI in GCS
   - Send/receive test commands

2. Range Test
   - Move to 100m, verify link
   - Move to 500m, verify link
   - Move to 1km, verify link
   - Note RSSI at each distance

3. Flight Test
   - Start at close range
   - Gradually increase distance
   - Monitor RSSI and packet loss
   - Establish safe operating range

RSSI Guidelines:
- > -70dBm: Excellent
- -70 to -80dBm: Good
- -80 to -90dBm: Fair
- < -90dBm: Poor (return to closer range)

Packet Loss:
- < 1%: Excellent
- 1-5%: Acceptable
- > 5%: Investigate interference
```

## Troubleshooting

### Connection Issues

```bash
# No connection
# Check:
# 1. Baud rate match (radio and FC)
# 2. Correct serial port
# 3. TX/RX not swapped
# 4. Power to radio (5V)
# 5. Ground connection

# Check serial connection
mavproxy.py --master=/dev/ttyAMA0 --baudrate=57600

# View radio status in GCS
# Look for:
# - HEARTBEAT messages
# - SYS_STATUS messages
# - RSSI reporting
```

### Range Issues

```
Poor Range Solutions
══════════════════════════════════════════════════════════════════

1. Antenna orientation
   - Keep vertical polarization
   - Avoid pointing null at vehicle
   - Ground antenna should be elevated

2. Antenna upgrade
   - Use directional antenna on ground
   - High-gain dipole (5-8dBi)
   - Tracking antenna for long range

3. Power settings
   - Increase TX power (legal limits)
   - Check for low power mode

4. Interference
   - Change frequency/channels
   - Move away from WiFi sources
   - Check for harmonic interference

5. Air data rate
   - Lower rate for longer range
   - Trade speed for distance
```

## See Also

- [MAVLink Setup](./mavlink.md) - Protocol configuration
- [Wiring Diagrams](./wiring.md) - Connection diagrams
- [Flight Controllers](./flight-controllers.md) - FC integration
