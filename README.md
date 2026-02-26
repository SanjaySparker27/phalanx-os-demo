# ZEX ATHENA H743 PRO Custom Firmware

**Pure C++ Custom Firmware** for ZEX ATHENA H743 PRO (DAKEFPV H743 PRO) flight controller.

Developed by **Sanjay** for custom HEXA X + Pusher VTOL configuration.

---

## What This Firmware Does

This firmware runs **custom flight mode switching logic directly in compiled C++ code** — no Lua scripts needed!

### Key Features

| Feature | Description |
|---------|-------------|
| **HEXA X Frame** | 6-motor configuration |
| **Pusher Motor** | 7th motor on Output 7, controlled by RC8 (left slider) |
| **Smart Mode Switch** | RC8 slider auto-switches flight modes |
| **No SD Card Scripts** | Everything compiled into firmware |

---

## How Mode Switching Works

**RC8 (Left Slider) controls BOTH pusher motor AND flight mode:**

| Slider Position | Flight Mode | Pusher Motor |
|----------------|-------------|--------------|
| **0-16%** | Loiter (GPS hold) | Idle/Off |
| **>16%** | Stabilize (manual) | Active (throttle control) |

The firmware monitors RC8 in real-time and automatically switches modes at the 16% threshold.

---

## Complete RC Channel Mapping

| Channel | Function | Switch Type | Usage |
|---------|----------|-------------|-------|
| **CH1** | Roll | Stick (Right Horz) | Standard roll control |
| **CH2** | Pitch | Stick (Right Vert) | Standard pitch control |
| **CH3** | Throttle | Stick (Left Vert) | Main motor throttle |
| **CH4** | Yaw | Stick (Left Horz) | Standard yaw control |
| **CH5** | Flight Mode 2 | 2/3-position switch | Optional secondary mode switch |
| **CH6** | Tuning / Aux | Pot/Switch | In-flight tuning (optional) |
| **CH7** | Aux Function | Switch | Camera trigger, etc. (optional) |
| **CH8** | **Pusher + Mode Switch** | **Left Slider** | **Primary mode switch + pusher motor control** |
| **CH9-16** | Additional Aux | Switches/Pots | Additional functions as needed |

### Primary Flight Mode Switch: CH8 (Left Slider)

Your **left slider** is the main flight mode switch:

```
Slider Position    Mode
    0%          → Loiter
    5%          → Loiter
    10%         → Loiter
    15%         → Loiter
    16%         → Stabilize  ← Threshold
    50%         → Stabilize
    100%        → Stabilize
```

**Important**: When slider is above 16%, pusher motor is active and responds to throttle.

---

## Motor Layout (HEXA X + Pusher)

```
      Front
        ↑
   (5)     (6)
     \     /
      \   /
   (3)  ↑  (4)
        |
   (1)     (2)
        
   [Pusher] (7)  ← Controlled by RC8 (left slider)
```

- **M1-M6**: Main HEXA X motors (Outputs 1-6)
- **M7 (Output 7)**: Pusher motor controlled by RC8

---

## Quick Setup

### 1. Flash Firmware
- Connect flight controller via USB
- Open Mission Planner → "Install Firmware"
- Click "Load custom firmware"
- Select `Firmware/arducopter_dakefph743pro.apj`

### 2. Load Parameters
- Connect to Mission Planner
- Go to **Config/Tuning** → **Full Parameter List**
- Click **Load from file** → Select `athena_h743pro_params.param`
- Click **Write Params** and reboot

### 3. Calibrate RC
- Go to **Initial Setup** → **Mandatory Hardware** → **Radio Calibration**
- Calibrate all channels including RC8 (left slider)
- Verify RC8 shows 1000-2000 range

### 4. Test Setup
- **Motor Test**: Use Mission Planner to verify M1-M6 spin correctly
- **Pusher Test**: Move RC8 slider up, check M7 responds
- **Mode Switch Test**: 
  - Move slider to bottom (0-16%) → Check mode shows "Loiter"
  - Move slider up (>16%) → Check mode shows "Stabilize"
  - Check Messages tab for "Mode changed via slider" text

---

## Main Code Changes

### RC_Channel.cpp
Added `custom_slider_mode_switch()` function that:
- Reads RC8 percentage (0-100%)
- Switches to Loiter when ≤16%
- Switches to Stabilize when >16%
- Runs at 10Hz with debouncing

### Copter.cpp
Modified `rc_loop()` to call custom mode switch when `FLTMODE_CH = 8`

### RC_Channel.h
Added declaration for `custom_slider_mode_switch()`

---

## Key Parameters

```
FRAME_CLASS = 2          # Hexa
FRAME_TYPE = 13          # X configuration
FLTMODE_CH = 8           # RC8 is the flight mode channel
SERVO7_FUNCTION = 51     # RCIN8 passthrough (pusher motor)
RC8_OPTION = 0           # Do Nothing (handled by firmware)

RC8_MIN = 1000           # Slider low position
RC8_MAX = 2000           # Slider high position
RC8_TRIM = 1000          # Center/threshold position
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Mode not switching with slider | Check `FLTMODE_CH = 8`, verify RC8 calibrated (1000-2000 range) |
| Pusher motor not responding | Check `SERVO7_FUNCTION = 51`, verify RC8 moves full range |
| Wrong motor order | Use Mission Planner Motor Test to verify HEXA X order |
| Mode switches too fast | Normal - firmware runs at 10Hz with built-in debouncing |
| Shows wrong mode | Ensure slider is calibrated; 16% threshold is precise |

---

## Hardware Specifications

| Spec | Value |
|------|-------|
| MCU | STM32H743 @ 480 MHz |
| IMU | Dual ICM42688 |
| Barometer | SPL06 |
| OSD | AT7456E |
| Flash | 16MB |
| PWM Outputs | 13 (8 motors + 4 servos + LED) |
| UARTs | 8x |
| CAN | 1x |
| Battery | 4S-12S |
| BEC | 3.3V/0.5A, 5V/3A, 12V/3A |

---

## Files Included

| File | Purpose |
|------|---------|
| `arducopter_dakefph743pro.apj` | Main firmware (flash via Mission Planner) |
| `arducopter_dakefph743pro_bl.hex` | Firmware with bootloader (DFU mode) |
| `athena_h743pro_params.param` | Complete configuration parameters |
| `source_modifications/` | Modified C++ source files for reference |

---

## Developer

**[Sanjay](https://sanjaysparker27.github.io/sanjay-sparker-portfolio)**
- Custom firmware for ZEX ATHENA H743 PRO
- HEXA X + Pusher VTOL configuration
- Direct C++ modifications to ArduPilot

---

**No Lua Required** — All logic compiled into firmware for maximum performance.
