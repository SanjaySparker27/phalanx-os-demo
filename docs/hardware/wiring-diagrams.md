# Hardware Wiring Diagrams

Complete wiring documentation for ZEX ATHENA H743 PRO flight controller with Jetson companion computer and full sensor suite.

## Table of Contents
- [System Overview](#system-overview)
- [Flight Controller Wiring](#flight-controller-wiring)
- [Jetson Companion Computer Wiring](#jetson-companion-computer-wiring)
- [Motor Connections](#motor-connections)
- [Sensor Wiring](#sensor-wiring)
- [Power Distribution](#power-distribution)
- [Telemetry and Communication](#telemetry-and-communication)
- [Camera Integration](#camera-integration)

---

## System Overview

### Block Diagram

```
                    +---------------------------------------------------------+
                    |                    POWER SYSTEM                          |
                    |  +----------+    +------------+    +--------------+     |
                    |  | Battery  |--->| Power Dist |--->|  ESCs (7x)   |     |
                    |  |  6S-12S  |    |   Module   |    +--------------+     |
                    |  +----------+    +-----+------+                         |
                    |                        |                                 |
                    |                        v                                 |
                    |                   +----------+                          |
                    |                   |   BEC    |                          |
                    |                   | 5V/12V   |                          |
                    |                   +----+-----+                          |
                    +------------------------+--------------------------------+
                                             |
                    +------------------------+--------------------------------+
                    |                        |                               |
                    |    +-------------------+-------------------+          |
                    |    |         ATHENA H743 PRO               |          |
                    |    |        (Flight Controller)            |          |
                    |    |  +-------------+  +---------------+  |          |
                    |    |  |   STM32H743 |  |  Dual ICM42688 |  |          |
                    |    |  |  @480MHz   |  |     IMUs       |  |          |
                    |    |  +-------------+  +---------------+  |          |
                    |    |  +-------------+  +---------------+  |          |
                    |    |  |  SPL06 Baro |  |  AT7456E OSD  |  |          |
                    |    |  +-------------+  +---------------+  |          |
                    |    +----------+----------------------------+          |
                    |               |                                       |
                    |    FC SYSTEM  |                                       |
                    +---------------+---------------------------------------+
                                    |
         +--------------------------+--------------------------+
         |                          |                          |
         v                          v                          v
+-----------------+    +---------------------+    +---------------------+
|  MOTOR SYSTEM   |    |  COMPANION COMPUTER |    |   SENSOR SYSTEM     |
| +-----++-----+  |    |                     |    |                     |
| |M1-M6||M7   |  |    |   NVIDIA Jetson     |    |  +---------------+  |
| |HEXA ||Pusher|  |    |   (AGX Orin/NX)     |    |  |  GPS/RTK      |  |
| +-----++-----+  |    |                     |    |  |  u-blox F9P   |  |
|                 |    |  +---------------+  |    |  +---------------+  |
|  7x ESCs        |    |  |  CUDA Cores   |  |    |                     |
|  40A-60A each   |    |  |  AI Inference |  |    |  +---------------+  |
|                 |    |  +---------------+  |    |  |   LIDAR       |  |
+-----------------+    |  +---------------+  |    |  |  Ouster OS1   |  |
                       |  |  Perception   |  |    |  +---------------+  |
                       |  |  Agent (CNN)  |  |    |                     |
                       |  +---------------+  |    |  +---------------+  |
                       |  +---------------+  |    |  |   Camera      |  |
                       |  |  Planning     |  |    |  |  IMX477 12MP |  |
                       |  |  Agent (MPC)  |  |    |  +---------------+  |
                       |  +---------------+  |    |                     |
                       +---------------------+    +---------------------+
```

---

## Flight Controller Wiring

### ATHENA H743 PRO Pinout

```
+-----------------------------------------------------------------------------+
|                         ATHENA H743 PRO PINOUT                              |
+-----------------------------------------------------------------------------+
|                                                                             |
|   POWER INPUT                              USB-C (DEBUG/FLASH)             |
|   +---------+                              +---------+                     |
|   |  VBAT+  |------------------------------|   USB   |                     |
|   |  VBAT-  |                              +---------+                     |
|   |   GND   |                                                               |
|   | CURRENT |                                                               |
|   +---------+                                                               |
|                                                                             |
|   MOTOR OUTPUTS (JST GH)                                                    |
|   +-----------------------------------------------------------------------+ |
|   | M1  M2  M3  M4  M5  M6  M7  M8  M9  M10  M11  M12  M13               | |
|   | PA0 PA1 PA2 PA3 PE9 PE11 PC8 PC9 PF6  PF7  PA10  PA8  PA11            | |
|   +-----------------------------------------------------------------------+ |
|                                                                             |
|   UART PORTS                                                                |
|   +-----------+-----------+-----------+-----------+-----------+            |
|   |  UART1    |  UART2    |  UART3    |  UART4    |  UART5    |            |
|   |  GPS      |  Telem    |  ESC Tel  |  OSD/VTX  |  RC Input |            |
|   | PA10/RX   | PD6/RX    | PD9/RX    | PB9/RX    | PC7/RX    |            |
|   | PA9/TX    | PD5/TX    | PD8/TX    | PB8/TX    | PC6/TX    |            |
|   +-----------+-----------+-----------+-----------+-----------+            |
|   +-----------+-----------+-----------+-----------+                        |
|   |  UART6    |  UART7    |  UART8    |  CAN1     |                        |
|   |  Spare    |  Spare    |  Spare    |  CAN Bus  |                        |
|   +-----------+-----------+-----------+-----------+                        |
|                                                                             |
|   I2C/SPI                                                                   |
|   +---------------+---------------+-------------------+                    |
|   |    I2C1       |    I2C2      |     SPI1          |                    |
|   |  Compass/Ext  |  Baro/Int    |  External sensors |                    |
|   | PB7/PB8       |  PB11/PB10   |  PA5/PA6/PA7      |                    |
|   +---------------+---------------+-------------------+                    |
|                                                                             |
|   ADC/DAC                                                                   |
|   +---------+---------+---------+---------+                                 |
|   |  ADC1   |  ADC2   |  ADC3   |  DAC1   |                                 |
|   | PC4     | PC5     | PC0     | PA4     |                                 |
|   +---------+---------+---------+---------+                                 |
|                                                                             |
|   LED/SWITCH                                                                |
|   +---------------+---------------+                                         |
|   |  LED Strip    |  Safety SW    |                                         |
|   |  PC6          |  PC13         |                                         |
|   +---------------+---------------+                                         |
|                                                                             |
+-----------------------------------------------------------------------------+
```

---

## Motor Connections

### HEXA X + Pusher Configuration

```
                              FRONT
                                ^
                                |
                         M5(CCW)   M6(CW)
                           \       /
                            \     /
                             \   /
                              \ /
                               X
                              / \
                             /   \
                            /     \
                    M3(CW) /       \ M4(CCW)
                          |    M7   |
                          | Pusher  |
                   M1(CCW)|         |M2(CW)
                          +---------+
```

### Motor Connection Table

| Motor | FC Output | Wire Color | ESC Signal | Direction | Blade Type |
|-------|-----------|------------|------------|-----------|------------|
| **M1** | Output 1 | White | PWM | CCW | Pusher |
| **M2** | Output 2 | Red | PWM | CW | Tractor |
| **M3** | Output 3 | Black | PWM | CW | Tractor |
| **M4** | Output 4 | Yellow | PWM | CCW | Pusher |
| **M5** | Output 5 | Green | PWM | CCW | Pusher |
| **M6** | Output 6 | Blue | PWM | CW | Tractor |
| **M7** | Output 7 | Purple | PWM | CW | Pusher |

### ESC Wiring (Per Motor)

```
+---------------+
|     ESC       |
|  +---------+  |
|  | Signal  |--+--> FC Motor Output
|  |  (PWM)  |  |     (White wire)
|  +---------+  |
|  |  Ground |--+--> FC GND / Power GND
|  |  (Black)|  |
|  +---------+  |
|  |   5V    |--+--> FC 5V (Optional, for BEC ESCs)
|  |  (Red)  |  |     Note: Remove if FC has separate BEC
|  +---------+  |
+-------+-------+
        |
        | 3-phase wires to motor
        | (A, B, C phases - any order,
        |  swap any two to reverse direction)
        v
   +---------+
   |  Motor  |
   +---------+
```

---

## Jetson Companion Computer Wiring

### Jetson to Flight Controller Connection

```
+---------------------------------------------------------------+
|                    JETSON <-> FC CONNECTION                     |
+---------------------------------------------------------------+
|                                                               |
|   JETSON AGX ORIN                    ATHENA H743 PRO         |
|   +---------------------+           +----------------------+ |
|   |                     |           |                      | |
|   |  UART1 (ttyTHS0)    |           |   UART2 (Telem2)     | |
|   |  +---------------+  |           |   +---------------+  | |
|   |  | TX (Pin 203)  |--+-----------+-->| RX (PD6)      |  | |
|   |  | RX (Pin 205)  |<--+-----------+---| TX (PD5)      |  | |
|   |  | GND (Pin 199) |--+-----------+---| GND           |  | |
|   |  | 5V (Pin 201)  |--+-----------+---| 5V (Optional) |  | |
|   |  +---------------+  |           |   +---------------+  | |
|   |                     |           |                      | |
|   +---------------------+           +----------------------+ |
|                                                               |
|   NOTE: Cross-connect TX<->RX                                   |
|   JETSON TX -> FC RX                                           |
|   JETSON RX <- FC TX                                           |
|                                                               |
+---------------------------------------------------------------+
```

### Jetson Pinout Reference

```
+-----------------------------------------------------------------------------+
|                        JETSON AGX ORIN HEADER J30                           |
+-----------------------------------------------------------------------------+
|                                                                             |
|   UART (ttyTHS0) - J30 Pins                                                 |
|   +---------------------------------------------------------------------+   |
|   |  Pin 203: UART1_TX                                                  |   |
|   |  Pin 205: UART1_RX                                                  |   |
|   |  Pin 199: GND                                                       |   |
|   |  Pin 201: 5V                                                        |   |
|   +---------------------------------------------------------------------+   |
|                                                                             |
|   I2C (i2c-0) - J30 Pins                                                    |
|   +---------------------------------------------------------------------+   |
|   |  Pin 189: I2C0_SCL                                                  |   |
|   |  Pin 191: I2C0_SDA                                                  |   |
|   |  Pin 199: GND                                                       |   |
|   |  Pin 201: 5V                                                        |   |
|   +---------------------------------------------------------------------+   |
|                                                                             |
|   SPI (spi0) - J30 Pins                                                     |
|   +---------------------------------------------------------------------+   |
|   |  Pin 206: SPI0_SCK                                                  |   |
|   |  Pin 204: SPI0_MISO                                                 |   |
|   |  Pin 202: SPI0_MOSI                                                 |   |
|   |  Pin 200: SPI0_CS0                                                  |   |
|   +---------------------------------------------------------------------+   |
|                                                                             |
|   CAN Bus - J30 Pins                                                        |
|   +---------------------------------------------------------------------+   |
|   |  Pin 180: CAN0_TX                                                   |   |
|   |  Pin 182: CAN0_RX                                                   |   |
|   |  Pin 184: CAN1_TX                                                   |   |
|   |  Pin 186: CAN1_RX                                                   |   |
|   +---------------------------------------------------------------------+   |
|                                                                             |
+-----------------------------------------------------------------------------+
```

---

## Sensor Wiring

### GPS Module (u-blox ZED-F9P)

```
+--------------------------------------------------------------------+
|                    GPS MODULE CONNECTION                            |
+--------------------------------------------------------------------+
|                                                                     |
|   ZED-F9P                          ATHENA H743 PRO                 |
|   +---------------+                +---------------+               |
|   |               |                |               |               |
|   |  VCC (5V)    |----------------| 5V           |               |
|   |  GND         |----------------| GND          |               |
|   |  TX (UART)   |----------------| RX1 (PA10)   |               |
|   |  RX (UART)   |----------------| TX1 (PA9)    |               |
|   |  SCL (I2C)   |----------------| SCL1 (PB8)   |               |
|   |  SDA (I2C)   |----------------| SDA1 (PB7)   |               |
|   |  PPS         |----------------| ADC1 (PC4)   |               |
|   |  SAFE        |----------------| 3.3V/NC      |               |
|   |               |                |               |               |
|   +---------------+                +---------------+               |
|                                                                     |
|   I2C Connection for Compass:                                       |
|   - GPS module contains magnetometer (typically IST8310 or QMC5883) |
|   - Connect SCL/SDA to FC I2C1                                      |
|                                                                     |
+--------------------------------------------------------------------+
```

### LIDAR (Ouster OS1-64)

```
+--------------------------------------------------------------------+
|                    LIDAR CONNECTION                                 |
+--------------------------------------------------------------------+
|                                                                     |
|   Ouster OS1                       Jetson AGX Orin                 |
|   +---------------+                +---------------+               |
|   |               |                |               |               |
|   |  24V Power   |<---------------| Power Dist    |               |
|   |  (2A max)    |                | (24V rail)    |               |
|   |               |                |               |               |
|   |  Ethernet    |<---------------| RJ45 Port     |               |
|   |  (Gigabit)   |                |               |               |
|   |               |                |               |               |
|   |  Sync In     |<---------------| GPIO (Pin 206)|               |
|   |  (PPS)       |                |               |               |
|   |               |                |               |               |
|   +---------------+                +---------------+               |
|                                                                     |
|   Network Configuration:                                            |
|   - LIDAR Default IP: 192.168.1.200                                 |
|   - Jetson IP: 192.168.1.100                                        |
|   - Subnet: 255.255.255.0                                           |
|                                                                     |
|   Ouster Studio access: http://192.168.1.200                        |
|                                                                     |
+--------------------------------------------------------------------+
```

### Camera (Raspberry Pi IMX477 or Arducam)

```
+--------------------------------------------------------------------+
|                    CAMERA CONNECTION                                |
+--------------------------------------------------------------------+
|                                                                     |
|   CSI Camera                       Jetson AGX Orin                 |
|   +---------------+                +---------------+               |
|   |               |                |               |               |
|   |  CSI-2 4-lane|<---------------| CSI Port 0/1  |               |
|   |  (15-pin)    |                | (30-pin)      |               |
|   |               |                |               |               |
|   |  3.3V Power  |<---------------| 3.3V          |               |
|   |  (from FC)   |                | (Regulated)   |               |
|   |               |                |               |               |
|   +---------------+                +---------------+               |
|                                                                     |
|   CSI Pinout (15-pin to 30-pin adapter):                           |
|   +--------+--------+------------------------------+               |
|   | Cam    | Jetson | Function                     |               |
|   +--------+--------+------------------------------+               |
|   | Pin 1  | Pin 19 | GND                          |               |
|   | Pin 2  | Pin 20 | CAM_D0_N                     |               |
|   | Pin 3  | Pin 22 | CAM_D0_P                     |               |
|   | Pin 4  | Pin 21 | GND                          |               |
|   | Pin 5  | Pin 24 | CAM_D1_N                     |               |
|   | Pin 6  | Pin 26 | CAM_D1_P                     |               |
|   | Pin 7  | Pin 25 | GND                          |               |
|   | Pin 8  | Pin 28 | CAM_CLK_N                    |               |
|   | Pin 9  | Pin 30 | CAM_CLK_P                    |               |
|   | Pin 10 | Pin 29 | GND                          |               |
|   | Pin 11 | Pin 32 | CAM_D2_N                     |               |
|   | Pin 12 | Pin 34 | CAM_D2_P                     |               |
|   | Pin 13 | Pin 33 | GND                          |               |
|   | Pin 14 | Pin 36 | CAM_D3_N                     |               |
|   | Pin 15 | Pin 38 | CAM_D3_P                     |               |
|   +--------+--------+------------------------------+               |
|                                                                     |
+--------------------------------------------------------------------+
```

---

## Power Distribution

### Power Architecture

```
+-----------------------------------------------------------------------------+
|                         POWER DISTRIBUTION SYSTEM                           |
+-----------------------------------------------------------------------------+
|                                                                             |
|   Battery (6S LiPo - 22.2V nominal)                                         |
|   +---------------------------------------------------------------------+   |
|   |                         XT90 Connector                              |   |
|   +---------------------------------+-----------------------------------+   |
|                                     |                                       |
|                                     v                                       |
|   +---------------------------------------------------------------------+   |
|   |                    POWER DISTRIBUTION MODULE                        |   |
|   |  +-------------+  +-------------+  +-------------+                  |   |
|   |  | 7x XT60     |  | 12V BEC     |  | 5V BEC      |                  |   |
|   |  | Outputs     |  | 10A         |  | 10A         |                  |   |
|   |  | (for ESCs)  |  | (VTX/LIDAR) |  | (FC/Jetson) |                  |   |
|   |  +------+------+  +------+------+  +------+------+                  |   |
|   +--------+---------------+---------------+--------------------------+   |
|            |               |               |                             |
|            v               v               v                             |
|       +----------+     +----------+     +----------+                       |
|       |  ESCs    |     |   12V    |     |    5V    |                       |
|       |  (7x)    |     |  Devices |     |  Devices |                       |
|       | 40A each |     |          |     |          |                       |
|       +----------+     +----------+     +----------+                       |
|                                                                             |
|   12V Rail Distribution:                                                    |
|   +- LIDAR (24V via step-up): 2A                                           |
|   +- VTX: 1A                                                               |
|   +- LED Strip: 2A                                                         |
|   +- Spare: 5A                                                             |
|                                                                             |
|   5V Rail Distribution:                                                     |
|   +- Flight Controller: 0.5A                                               |
|   +- Jetson (via barrel jack): 4A                                         |
|   +- GPS Module: 0.5A                                                      |
|   +- Camera: 0.5A                                                          |
|   +- Spare: 4.5A                                                           |
|                                                                             |
+-----------------------------------------------------------------------------+
```

### Power Budget

| Component | Voltage | Current (A) | Power (W) |
|-----------|---------|-------------|-----------|
| **Motors (max)** | 22.2V | 7 x 40A | 6,216W |
| **Motors (hover)** | 22.2V | 7 x 10A | 1,554W |
| **Jetson AGX Orin** | 5V | 4A | 20W |
| **Flight Controller** | 5V | 0.5A | 2.5W |
| **LIDAR** | 24V | 2A | 48W |
| **Camera** | 5V | 0.5A | 2.5W |
| **VTX** | 12V | 1A | 12W |
| **GPS/Compass** | 5V | 0.5A | 2.5W |
| **LED Strip** | 12V | 2A | 24W |
| **Misc** | 5V | 1A | 5W |
| **TOTAL (hover)** | - | - | **~1,670W** |
| **TOTAL (max)** | - | - | **~6,332W** |

---

## Telemetry and Communication

### Radio Telemetry (RFD900x / SiK Radio)

```
+--------------------------------------------------------------------+
|                  RADIO TELEMETRY CONNECTION                         |
+--------------------------------------------------------------------+
|                                                                     |
|   AIR UNIT                         ATHENA H743 PRO                 |
|   +---------------+                +---------------+               |
|   |               |                |               |               |
|   |  5V          |----------------| 5V           |               |
|   |  GND         |----------------| GND          |               |
|   |  TX          |----------------| RX3 (PD9)    |               |
|   |  RX          |----------------| TX3 (PD8)    |               |
|   |  CTS         |----------------| NC           |               |
|   |  RTS         |----------------| NC           |               |
|   |               |                |               |               |
|   +---------------+                +---------------+               |
|                                                                     |
|   GROUND UNIT                      Ground Station PC               |
|   +---------------+                +---------------+               |
|   |               |                |               |               |
|   |  USB          |----------------| USB Port     |               |
|   |               |                |               |               |
|   +---------------+                +---------------+               |
|                                                                     |
|   Configuration:                                                    |
|   - SERIAL3_PROTOCOL = 1 (MAVLink)                                 |
|   - SERIAL3_BAUD = 57 (57600)                                      |
|   - Air/Ground must have matching NETID                             |
|   - Default NETID = 25                                             |
|                                                                     |
+--------------------------------------------------------------------+
```

### 4G/5G Cellular (Optional)

```
+--------------------------------------------------------------------+
|                  CELLULAR MODEM CONNECTION                          |
+--------------------------------------------------------------------+
|                                                                     |
|   4G Modem (Quectel)               Jetson AGX Orin                 |
|   +---------------+                +---------------+               |
|   |               |                |               |               |
|   |  USB          |----------------| USB 3.0 Port |               |
|   |               |                |               |               |
|   |  SIM Card     |                |               |               |
|   |  Slot         |                |               |               |
|   |               |                |               |               |
|   +---------------+                +---------------+               |
|                                                                     |
|   Software:                                                         |
|   - ModemManager                                                    |
|   - QMI/WWAN driver                                                 |
|   - VPN tunnel (WireGuard/OpenVPN)                                  |
|                                                                     |
|   Connection:                                                       |
|   - Creates network interface (e.g., wwan0)                         |
|   - Internet access for cloud telemetry                             |
|   - Video streaming capability                                      |
|                                                                     |
+--------------------------------------------------------------------+
```

---

## Camera Integration

### Gimbal Connection (Optional)

```
+--------------------------------------------------------------------+
|                  GIMBAL SYSTEM WIRING                               |
+--------------------------------------------------------------------+
|                                                                     |
|   Gimbal Controller                ATHENA H743 PRO                 |
|   +---------------+                +---------------+               |
|   |               |                |               |               |
|   |  12V Power   |----------------| 12V Rail     |               |
|   |  GND         |----------------| GND          |               |
|   |               |                |               |               |
|   |  UART TX     |<---------------| RC7 (M8)     |               |
|   |  UART RX     |--------------->| RC7 (M8)     |               |
|   |  (Protocol)  |                | SERVO8_FUNC  |               |
|   |               |                |              |               |
|   +---------------+                +---------------+               |
|                                                                     |
|   Gimbal Control Channels:                                          |
|   - SERVO8_FUNCTION = 7 (MountTilt)                                |
|   - SERVO9_FUNCTION = 8 (MountRoll)                                |
|   - SERVO10_FUNCTION = 6 (MountPan)                                |
|                                                                     |
|   Common Gimbal Protocols:                                          |
|   - Storm32 (serial)                                                |
|   - AlexMos/SimpleBGC (serial)                                      |
|   - DJI (CAN bus)                                                   |
|                                                                     |
+--------------------------------------------------------------------+
```

---

## Wiring Checklist

### Before Power-On

- [ ] Battery polarity verified (red=positive, black=negative)
- [ ] All XT60/XT90 connectors secure
- [ ] No exposed wire strands
- [ ] No wires pinched in frame
- [ ] FC orientation arrow pointing forward
- [ ] GPS arrow pointing forward (same as FC)
- [ ] Compass orientation matches FC
- [ ] Motor wires secured to frame
- [ ] All JST connectors clicked in place
- [ ] Antennas properly mounted (not touching metal)

### Power-Up Sequence

1. **First Power-On** (without props!)
   - Connect battery
   - Check FC LED: Blue = good, Red = error
   - Listen for ESC initialization tones
   - Verify GPS lock (solid blue LED)

2. **Second Power-On** (with telemetry)
   - Connect ground station radio
   - Verify MAVLink connection
   - Check all sensor readings

3. **Final Check** (with props)
   - Perform motor test via Mission Planner
   - Verify correct motor rotation
   - Check motor order (M1-M6)

---

## Troubleshooting

### Common Wiring Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| FC won't power on | Reversed polarity | Check battery connection |
| ESCs not beeping | No signal from FC | Check PWM wires |
| GPS not detected | I2C wiring issue | Check SCL/SDA connections |
| Telemetry no link | Wrong baud rate | Match SERIAL3_BAUD |
| Motor spinning wrong | Phase wires swapped | Swap any two motor wires |
| Intermittent connection | Loose connector | Reseat all connectors |
| Video noise | Power ground loop | Star grounding topology |

---

## References

- [ATHENA H743 PRO Datasheet](https://example.com/athena-h743-pro)
- [ArduPilot Motor Connections](https://ardupilot.org/copter/docs/connect-escs-and-motors.html)
- [Jetson AGX Orin Pinout](https://developer.nvidia.com/embedded/jetson-agx-orin)
- [MAVLink Wiring Guide](https://ardupilot.org/copter/docs/common-telemetry-radio-wiring.html)
