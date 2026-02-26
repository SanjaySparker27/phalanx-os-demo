# ZEX ATHENA H743 PRO Custom Firmware Package

## Overview

This repository contains the **complete custom firmware package** for the **ZEX ATHENA H743 PRO** flight controller (compatible with DAKEFPV H743 PRO). 

### Key Features

- **HEXA X** frame configuration (6 motors)
- **Pusher motor** (7th motor) controlled via RCIN8 (left slider)
- **Flight mode switching** via RC8 slider:
  - **0-16%** slider position → **Loiter Mode**
  - **>16%** slider position → **Stabilize Mode**

## Hardware Specifications

| Feature | Specification |
|---------|--------------|
| MCU | STM32H743 @ 480 MHz |
| IMU | Dual ICM42688 |
| Barometer | SPL06 |
| OSD | AT7456E |
| Flash | 16MB |
| UARTs | 8x |
| CAN | 1x |
| PWM Outputs | 13 (8 motors + 4 servos + LED) |
| Battery | 4S-12S |
| BEC | 3.3V/0.5A, 5V/3A, 12V/3A (VTX) |

## Repository Structure

```
Athena-H743-Pro-Custom-Firmware/
├── README.md                           # This file
├── DAKEFPVH743Pro_Custom_Config.md     # Detailed configuration documentation
├── athena_h743pro_params.param         # Parameter configuration file
├── mode_switch.lua                     # Lua script for custom mode switching
│
├── Firmware/                           # Pre-built firmware files
│   ├── arducopter_dakefph743pro.apj         # Main firmware (1.3MB)
│   └── arducopter_dakefph743pro_bl.hex      # Firmware with bootloader (5MB)
│
├── source_modifications/               # Source code modifications
│   ├── ArduCopter/
│   │   ├── RC_Channel.cpp              # Modified for custom mode switching
│   │   ├── RC_Channel.h                # Header with new function declaration
│   │   └── Copter.cpp                  # Modified rc_loop for slider support
│   └── DAKEFPVH743Pro/
│       ├── hwdef.dat                   # Hardware definition
│       ├── hwdef-bl.dat                # Bootloader hardware definition
│       └── defaults.parm               # Default parameters
│
└── docs/
    └── ZEX_ATHENA_H743_PRO_User_Manual.pdf  # Flight controller manual
```

## Quick Start Guide

### 1. Flash Firmware

#### Option A: Using Mission Planner (Recommended)
1. Connect flight controller to PC via USB
2. Open Mission Planner → "Install Firmware" tab
3. Click "Load custom firmware"
4. Select `Firmware/arducopter_dakefph743pro.apj`

#### Option B: DFU Flashing (First Time / Recovery)
1. Hold **BOOT** button while connecting USB
2. Use STM32CubeProgrammer or dfu-util to flash `Firmware/arducopter_dakefph743pro_bl.hex`
3. Disconnect and reconnect without BOOT button

### 2. Configure Parameters

1. Connect to Mission Planner
2. Go to **Config/Tuning** → **Full Parameter List**
3. Click **Load from file** → Select `athena_h743pro_params.param`
4. Click **Write Params** and reboot

### 3. Install Lua Script (Required for Custom Mode Switching)

1. Insert SD card into computer
2. Create folder `/APM/scripts/` on SD card
3. Copy `mode_switch.lua` to `/APM/scripts/`
4. Insert SD card into flight controller
5. Enable scripting:
   - Set parameter `SCR_ENABLE = 1`
   - Set parameter `SCR_HEAP_SIZE = 102400`
6. Reboot

### 4. Verify Setup

1. **RC Calibration**: Calibrate RC8 (left slider) in Mission Planner
2. **Motor Test**: Use "Motor Test" to verify correct motor order
3. **Mode Switching**: Move slider to test:
   - 0-16% = Loiter Mode
   - >16% = Stabilize Mode

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
        
   [Pusher] (7)
```

- **M1-M6**: HEXA X configuration motors (Outputs 1-6)
- **M7 (Output 7)**: Pusher motor controlled by RCIN8

## RC Channel Mapping

| Channel | Function | Configuration |
|---------|----------|---------------|
| CH1 | Roll | Standard |
| CH2 | Pitch | Standard |
| CH3 | Throttle | Standard |
| CH4 | Yaw | Standard |
| CH5 | Flight Mode | (Optional) |
| CH6 | Tuning | (Optional) |
| CH7 | Aux | (Optional) |
| **CH8** | **Pusher + Mode Switch** | **Slider 0-16%=Loiter, >16%=Stabilize** |

## Key Parameters

```
FRAME_CLASS = 2          # Hexa
FRAME_TYPE = 13          # X configuration
SERVO7_FUNCTION = 51     # RCIN8 passthrough (pusher motor)
FLTMODE_CH = 8           # Use RC8 for mode switching
SCR_ENABLE = 1           # Enable Lua scripting
SCR_HEAP_SIZE = 102400   # Script memory allocation
```

## Source Code Modifications

This firmware includes custom modifications to the ArduPilot codebase:

### 1. RC_Channel.cpp
Added `custom_slider_mode_switch()` function that monitors RC8 and switches modes based on slider percentage (0-16% = Loiter, >16% = Stabilize).

### 2. RC_Channel.h  
Added function declaration for `custom_slider_mode_switch()`.

### 3. Copter.cpp
Modified `rc_loop()` to call custom mode switch when `FLTMODE_CH = 8`.

See `source_modifications/` directory for all modified files.

## Troubleshooting

### Pusher motor not responding to RC8
- Check `SERVO7_FUNCTION = 51`
- Verify RC8 is calibrated (1000-2000 range)
- Check `RC8_OPTION = 0`

### Mode not switching with slider
- Verify Lua script is running (check "Messages" tab in Mission Planner)
- Check `FLTMODE_CH = 8`
- Ensure `SCR_ENABLE = 1` and `SCR_HEAP_SIZE = 102400`
- Verify RC8 slider moves full range (1000-2000)

### Motors not spinning
- Check `ARMING_CHECK` = 0 or all checks pass
- Verify frame class and motor order
- Check safety switch is pressed (if equipped)

### Script not loading
- Verify SD card is formatted as FAT32
- Check `/APM/scripts/` folder exists
- Look for errors in "Messages" tab
- Try increasing `SCR_HEAP_SIZE` to 150000

## File Descriptions

| File | Description |
|------|-------------|
| `arducopter_dakefph743pro.apj` | ArduCopter firmware (flash via GCS) |
| `arducopter_dakefph743pro_bl.hex` | Firmware with bootloader (DFU flash) |
| `athena_h743pro_params.param` | Complete parameter file for HEXA X + pusher |
| `mode_switch.lua` | Lua script for slider-based mode switching |
| `DAKEFPVH743Pro_Custom_Config.md` | Detailed technical documentation |
| `ZEX_ATHENA_H743_PRO_User_Manual.pdf` | Official flight controller manual |

## Building from Source

To build the firmware from source:

```bash
# Clone ArduPilot repository
git clone --depth 1 --branch Copter-4.5 https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Configure for DAKEFPVH743Pro
./waf configure --board=DAKEFPVH743Pro

# Build firmware
./waf copter

# Output will be in build/DAKEFPVH743Pro/bin/
```

## Support & Resources

- **ArduPilot Documentation**: https://ardupilot.org/copter/
- **ArduPilot Forums**: https://discuss.ardupilot.org/
- **DAKEFPV H743 Pro Wiki**: https://ardupilot.org/copter/docs/common-dakefpvh743pro.html
- **ZEX/DAKEFPV Support**: Contact manufacturer

## License

ArduPilot is licensed under GPLv3. This custom configuration is provided as-is for educational and development purposes.

## Credits

- ArduPilot Development Team
- DAKEFPV for the H743 Pro hardware
- ZEX for the ATHENA H743 PRO flight controller
