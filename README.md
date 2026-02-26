# ZEX ATHENA H743 PRO Custom Firmware

**Pure C++ Custom Firmware** for ZEX ATHENA H743 PRO (DAKEFPV H743 PRO) flight controller.

## What This Firmware Does

This firmware runs **custom flight mode switching logic directly in compiled C++ code** — no Lua scripts needed!

### Key Features

| Feature | Description |
|---------|-------------|
| **HEXA X Frame** | 6-motor configuration |
| **Pusher Motor** | 7th motor on Output 7, controlled by RC8 (left slider) |
| **Smart Mode Switch** | RC8 slider auto-switches flight modes |

## How Mode Switching Works

**RC8 (Left Slider) controls both pusher motor AND flight mode:**

| Slider Position | Flight Mode | Pusher Motor |
|----------------|-------------|--------------|
| **0-16%** | Loiter (GPS hold) | Idle/Off |
| **>16%** | Stabilize (manual) | Active (throttle control) |

The firmware monitors RC8 in real-time and automatically switches modes at the 16% threshold.

## Motor Layout

```
      Front
        ↑
   (5)     (6)
     \     /
      \   /
   (3)  ↑  (4)
        |
   (1)     (2)
        
   [Pusher] (7)  ← Controlled by RC8
```

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
- Calibrate RC8 (left slider) in Mission Planner
- Verify slider shows 1000-2000 range

### 4. Test
- Move RC8 slider:
  - Low position (0-16%) → Should show "Loiter" mode
  - High position (>16%) → Should show "Stabilize" mode
- Check Messages tab for "Mode changed" confirmation

## Main Code Changes

### 1. RC_Channel.cpp
Added `custom_slider_mode_switch()` function that:
- Reads RC8 percentage (0-100%)
- Switches to Loiter when ≤16%
- Switches to Stabilize when >16%
- Runs at 10Hz with debouncing

### 2. Copter.cpp
Modified `rc_loop()` to call custom mode switch when `FLTMODE_CH = 8`

### 3. RC_Channel.h
Added declaration for `custom_slider_mode_switch()`

## RC Channel Mapping

| Channel | Function |
|---------|----------|
| CH1 | Roll |
| CH2 | Pitch |
| CH3 | Throttle |
| CH4 | Yaw |
| CH8 | **Pusher + Mode Switch** (Slider) |

## Key Parameters

```
FRAME_CLASS = 2          # Hexa
FRAME_TYPE = 13          # X configuration
FLTMODE_CH = 8           # Use RC8 for mode switching
SERVO7_FUNCTION = 51     # RCIN8 passthrough (pusher)
RC8_OPTION = 0           # Do Nothing (handled by firmware)
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Mode not switching | Check `FLTMODE_CH = 8`, verify RC8 calibrated |
| Pusher not responding | Check `SERVO7_FUNCTION = 51`, RC8 range 1000-2000 |
| Wrong motor order | Use Mission Planner Motor Test to verify |

## Hardware Specs

- **MCU**: STM32H743 @ 480 MHz
- **IMU**: Dual ICM42688
- **PWM Outputs**: 13 (8 motors + 4 servos + LED)
- **Battery**: 4S-12S

## Files Included

| File | Purpose |
|------|---------|
| `arducopter_dakefph743pro.apj` | Main firmware |
| `arducopter_dakefph743pro_bl.hex` | Firmware + bootloader (DFU) |
| `athena_h743pro_params.param` | Configuration parameters |
| `source_modifications/` | Modified C++ source files |

---

**No Lua Required** — All logic is compiled into the firmware for maximum performance and reliability.
