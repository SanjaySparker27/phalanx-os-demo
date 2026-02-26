# ATHENA H743 PRO (DAKEFPV H743 PRO) Custom Firmware Configuration

## Overview

This configuration is for the **ATHENA H743 PRO** (compatible with DAKEFPV H743 PRO) flight controller, configured for:
- **HEXA X** frame with custom pusher motor
- **Pusher motor** (7th motor) controlled via RCIN8 (left slider)
- **Flight mode switching** via RC8 slider: 0-16% = Loiter, >16% = Stabilize

**IMPORTANT**: This is a **pure custom firmware** with all mode switching logic compiled directly into the C++ source code. **No Lua scripting is required!**

## Hardware Specifications

### ATHENA H743 PRO / DAKEFPV H743 PRO
- **MCU**: STM32H743 32-bit processor running at 480 MHz
- **IMU**: Dual ICM42688
- **Barometer**: SPL06
- **OSD**: AT7456E
- **Onboard Flash**: 16MByte
- **UARTs**: 8x
- **CAN**: 1x CAN port
- **PWM Outputs**: 13 (8 motors + 4 servos + LED)
- **Battery Input**: 4S-12S
- **BEC**: 3.3V 0.5A, 5V 3A, 12V 3A (VTX power)

## Parameter Configuration

### 1. Frame Configuration

```
FRAME_CLASS    2      # Hexa
FRAME_TYPE     13     # X configuration
```

### 2. Motor Outputs (HEXA X + Pusher)

Standard HEXA X motor layout on outputs 1-6:
```
SERVO1_FUNCTION  33   # Motor1
SERVO2_FUNCTION  34   # Motor2
SERVO3_FUNCTION  35   # Motor3
SERVO4_FUNCTION  36   # Motor4
SERVO5_FUNCTION  37   # Motor5
SERVO6_FUNCTION  38   # Motor6
```

**Pusher Motor Configuration** (RCIN8 passthrough):
```
SERVO7_FUNCTION  51   # RCIN8 passthrough (pusher motor)
RC8_OPTION       0    # Do Nothing (use for mode switching)
```

### 3. RC Configuration for Flight Mode Switching

**Using RC8 Slider for Mode Switching (Built into Firmware)**
```
FLTMODE_CH     8
RC8_OPTION     0      # Do Nothing
```

The firmware automatically monitors RC8 and switches modes:
- **0-16%** slider position → **Loiter Mode**
- **>16%** slider position → **Stabilize Mode**

This is handled by the `custom_slider_mode_switch()` function in the modified `RC_Channel.cpp` - no Lua required!

### 4. Complete Parameter File

Save as `athena_h743pro_params.param`:

```
# ATHENA H743 PRO Configuration
# HEXA X with Pusher Motor
# Pure Custom Firmware - No Lua Required!

# Frame
FRAME_CLASS,2
FRAME_TYPE,13

# RC8 for mode switching and pusher control
FLTMODE_CH,8
RC8_OPTION,0
SERVO7_FUNCTION,51

# Motor outputs
SERVO1_FUNCTION,33
SERVO2_FUNCTION,34
SERVO3_FUNCTION,35
SERVO4_FUNCTION,36
SERVO5_FUNCTION,37
SERVO6_FUNCTION,38

# RC8 configuration
RC8_MIN,1000
RC8_MAX,2000
RC8_TRIM,1000

# Pusher motor limits
SERVO7_MIN,1000
SERVO7_MAX,2000
SERVO7_TRIM,1000

# Safety
ARMING_CHECK,0
FS_THR_ENABLE,1
```

## Flashing Instructions

### Step 1: Flash Bootloader (First Time Only)
1. Download DAKEFPVH743Pro bootloader: `DAKEFPVH743Pro_bl.bin`
2. Hold BOOT button while connecting USB
3. Flash using STM32CubeProgrammer or dfu-util
4. Unplug and replug without BOOT button

### Step 2: Flash ArduCopter Firmware
1. Connect to Mission Planner / QGroundControl
2. Go to "Install Firmware" tab
3. Select "Load custom firmware" and choose `arducopter_dakefph743pro.apj`

### Step 3: Configure Parameters
1. Connect to Mission Planner
2. Go to "Config/Tuning" → "Full Parameter List"
3. Click "Load from file" and select `athena_h743pro_params.param`
4. Write parameters and reboot

**No Lua Script Required!** The custom mode switching is built directly into the firmware C++ code.

## Pinout Reference

### Motor Outputs (M1-M8)
| Output | Function | Pin |
|--------|----------|-----|
| M1 | Motor 1 | PA0 |
| M2 | Motor 2 | PA1 |
| M3 | Motor 3 | PA2 |
| M4 | Motor 4 | PA3 |
| M5 | Motor 5 | PE9 |
| M6 | Motor 6 | PE11 |
| M7 | Pusher Motor | PC8 |
| M8 | Spare | PC9 |

### UART Mapping
| Serial | Port | Function |
|--------|------|----------|
| SERIAL0 | USB | MAVLink |
| SERIAL1 | UART1 | GPS |
| SERIAL2 | UART2 | MAVLink2 |
| SERIAL3 | UART3 | ESC Telemetry |
| SERIAL4 | UART4 | DisplayPort |
| SERIAL5 | UART5 | RC Input |
| SERIAL6 | UART6 | User |
| SERIAL7 | UART7 | User |
| SERIAL8 | UART8 | User |

## Custom Firmware Source Code Changes

The following C++ code modifications enable the custom slider-based mode switching:

### RC_Channel.cpp - custom_slider_mode_switch()
```cpp
// Custom mode switch based on RC8 slider position
// For ATHENA H743 PRO with pusher motor
// 0-16% slider = Loiter Mode
// >16% slider = Stabilize Mode
void RC_Channel_Copter::custom_slider_mode_switch()
{
    RC_Channel *chan8 = rc().channel(7);
    if (chan8 == nullptr) return;
    
    uint8_t slider_percent = chan8->percent_input();
    static Mode::Number last_mode = Mode::Number::LOITER;
    static uint32_t last_check_ms = 0;
    
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_check_ms < 100) return;
    last_check_ms = now_ms;
    
    Mode::Number desired_mode = (slider_percent <= 16) 
        ? Mode::Number::LOITER 
        : Mode::Number::STABILIZE;
    
    if (desired_mode != last_mode) {
        if (copter.set_mode(desired_mode, ModeReason::RC_COMMAND)) {
            last_mode = desired_mode;
            gcs().send_text(MAV_SEVERITY_INFO, "Mode changed: %s (%u%%)", 
                          copter.flightmode->name(), (unsigned)slider_percent);
        }
    }
}
```

### Copter.cpp - rc_loop()
```cpp
void Copter::rc_loop()
{
    read_radio();
    rc().read_mode_switch();
    
    // Custom slider-based mode switching for ATHENA H743 PRO
    // Uses RC8 (slider) for flight mode: 0-16% = Loiter, >16% = Stabilize
    if (g.flight_mode_chan.get() == 8) {
        RC_Channel_Copter *chan8 = rc().channel(7);
        if (chan8 != nullptr) {
            chan8->custom_slider_mode_switch();
        }
    }
}
```

### RC_Channel.h
```cpp
class RC_Channel_Copter : public RC_Channel
{
    // ... existing code ...
    
    // Custom mode switch based on RC8 slider position (ATHENA H743 PRO)
    void custom_slider_mode_switch();
};
```

## Important Notes

1. **No Lua Required**: All mode switching is handled in compiled C++ code for maximum performance and reliability.

2. **Pusher Motor Safety**: The pusher motor (M7) is controlled directly by RC8. Ensure proper failsafe settings to prevent unintended throttle when switching modes.

3. **Mode Switching**: Modes switch at 16% threshold automatically by the firmware. You'll see "Mode changed via slider" messages in the Messages tab.

4. **RC Calibration**: Calibrate RC8 (left slider) in Mission Planner before flight.

5. **Motor Test**: Use Mission Planner's "Motor Test" to verify correct motor order and direction.

6. **Failsafe**: Set up radio failsafe properly for the pusher motor channel.

## Troubleshooting

### Pusher motor not responding to RC8
- Check `SERVO7_FUNCTION` is set correctly (51)
- Verify RC8 is calibrated and responding in RC calibration
- Check RC8_OPTION is set to 0

### Mode not switching with slider
- Verify `FLTMODE_CH` is set to 8
- Ensure RC8 range is 1000-2000
- Check Messages tab for "Mode changed via slider" text
- Verify you're using the custom firmware with C++ modifications

### Motors not spinning
- Check `ARMING_CHECK` is 0 or all checks pass
- Verify frame type and motor order
- Check safety switch is pressed (if equipped)

## Building from Source

```bash
# Clone ArduPilot repository
git clone --depth 1 --branch Copter-4.5 https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Copy custom source modifications
cp /path/to/source_modifications/ArduCopter/RC_Channel.cpp ArduCopter/RC_Channel.cpp
cp /path/to/source_modifications/ArduCopter/RC_Channel.h ArduCopter/RC_Channel.h
cp /path/to/source_modifications/ArduCopter/Copter.cpp ArduCopter/Copter.cpp

# Configure and build
./waf configure --board=DAKEFPVH743Pro
./waf copter

# Firmware will be in build/DAKEFPVH743Pro/bin/arducopter.apj
```

## Support

For issues with:
- **ArduPilot configuration**: https://discuss.ardupilot.org/
- **DAKEFPV H743 Pro hardware**: Contact DAKEFPV support
- **Custom firmware**: Refer to this document

## Firmware Files

Generated firmware files:
- `arducopter_dakefph743pro.apj` - Main firmware (flash via GCS)
- `arducopter_dakefph743pro_bl.hex` - With bootloader (DFU flash)
- `athena_h743pro_params.param` - Configuration parameters

## License

ArduPilot is licensed under GPLv3. This custom configuration is provided as-is for educational and development purposes.
