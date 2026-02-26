# Unified Sensor Library for Autonomous Vehicles

A comprehensive sensor abstraction layer supporting cameras, GNSS/RTK, IMUs, LiDARs, SATCOM, telemetry radios, and environmental sensors for autonomous vehicle platforms.

## Overview

This library provides a unified interface for integrating diverse sensor hardware commonly used in autonomous vehicles (UAVs, UGVs, USVs). It abstracts hardware-specific protocols behind a consistent API, enabling rapid development and easy sensor swapping.

## Supported Sensors

### Cameras
| Sensor | Resolution | Interface | Features |
|--------|------------|-----------|----------|
| Sony IMX477 | 4056x3040 | MIPI CSI-2 | 12MP, GStreamer pipelines |
| Sony IMX219 | 3280x2464 | MIPI CSI-2 | 8MP, Raspberry Pi compatible |

### GNSS/RTK
| Sensor | Constellations | RTK | Interface |
|--------|----------------|-----|-----------|
| u-blox ZED-F9P | GPS/GLONASS/Galileo/BeiDou | Yes | UART/USB |

### IMUs
| Sensor | Axes | Interface | Features |
|--------|------|-----------|----------|
| TDK ICM-20948 | 9-DOF | I2C/SPI | DMP, 9-axis fusion |
| Bosch BMI088 | 6-DOF | I2C/SPI | High-vibration rejection |

### LiDARs
| Sensor | Channels | Range | Rate |
|--------|----------|-------|------|
| Ouster OS1/OS2 | 16/32/64/128 | 120m | 10-20 Hz |
| Velodyne VLP-16 | 16 | 100m | 10 Hz |
| Velodyne VLP-32C | 32 | 200m | 10 Hz |

### SATCOM
| Module | Service | Data Rate |
|--------|---------|-----------|
| Iridium 9602/9603 | SBD | 340 B MO / 270 B MT |

### Telemetry Radios
| Radio | Frequency | Range | Protocol |
|-------|-----------|-------|----------|
| RFD900+ | 900 MHz | 40+ km | MAVLink |
| SiK (3DR/Holybro) | 433/915 MHz | 1-5 km | MAVLink |

### Environmental Sensors
| Sensor | Type | Interface |
|--------|------|-----------|
| MS5611 | Barometer | I2C/SPI |
| BMP388 | Barometer | I2C/SPI |
| LIS3MDL | Magnetometer | I2C/SPI |
| RM3100 | Magnetometer | I2C/SPI |
| MS4525DO | Airspeed | I2C |
| SDP3x | Airspeed | I2C |

## Architecture

```
sensors/
├── core/                    # Base abstraction layer
│   ├── sensor_types.h       # Common types and enums
│   ├── sensor_base.h/.c     # Base class implementation
│   └── sensor_manager.h/.c  # Multi-sensor coordination
├── cameras/                 # Camera drivers
├── gnss/                    # GPS/GNSS drivers
├── imu/                     # Inertial measurement units
├── lidar/                   # LiDAR drivers
├── satcom/                  # Satellite communication
├── telemetry/               # Radio telemetry
├── environment/             # Environmental sensors
├── utils/                   # Helper functions
└── tests/                   # Test suite
```

## Quick Start

### Building the Library

```bash
cd sensors/
make all              # Build shared and static libraries
make test             # Build and run tests
make install          # Install to /usr/local
```

### Basic Usage

```c
#include <sensors.h>

// Initialize library
sensors_init();

// Create sensor manager
sensor_manager_t *mgr = sensor_manager_create();
sensor_manager_init(mgr);

// Create and register sensors
sensor_base_t *imu = icm20948_create("primary_imu");
sensor_manager_register(mgr, imu, SENSOR_GROUP_IMU, true);

sensor_base_t *gnss = ublox_create("primary_gnss");
sensor_manager_register(mgr, gnss, SENSOR_GROUP_GNSS, true);

// Set up callbacks
sensor_base_set_callback(imu, on_imu_data, NULL);
sensor_base_set_callback(gnss, on_gnss_data, NULL);

// Start all sensors
sensor_manager_start_all(mgr);

// Main loop
while (running) {
    // Callbacks handle incoming data
    sleep(1);
}

// Cleanup
sensor_manager_destroy(mgr);
sensors_deinit();
```

### Camera Example (GStreamer)

```c
#include <cameras/gstreamer_pipeline.h>

// Create pipeline
gst_pipeline_t *pipe = gst_pipeline_create("front_camera");

// Configure IMX477
gst_pipeline_configure(pipe, "imx477", 4056, 3040, 30, GST_FORMAT_NV12);
gst_pipeline_start(pipe);

// Set frame callback
pipe->on_frame = process_frame;
pipe->userdata = my_context;

// ...

gst_pipeline_stop(pipe);
gst_pipeline_destroy(pipe);
```

## Configuration

Sensors can be configured via:
- **Runtime API**: Function calls to configure parameters
- **Configuration files**: JSON/XML files loaded at startup
- **Device tree**: Linux device tree overlays for embedded systems

## Hardware Interfaces

| Interface | Supported Sensors |
|-----------|-------------------|
| I2C | IMUs, barometers, magnetometers |
| SPI | IMUs, barometers (high-speed) |
| UART | GNSS, telemetry, SATCOM |
| USB | GNSS, cameras, LiDAR |
| Ethernet | LiDAR (Ouster, Velodyne) |
| MIPI CSI-2 | Cameras |

## Platform Support

- **NVIDIA Jetson**: Optimized GStreamer pipelines, hardware-accelerated encoding
- **Raspberry Pi**: Full camera and I2C support
- **ARM Cortex-A**: Generic embedded Linux support
- **x86_64**: Development and simulation

## API Reference

### Core API

| Function | Description |
|----------|-------------|
| `sensor_manager_create()` | Create sensor manager instance |
| `sensor_manager_register()` | Add sensor to manager |
| `sensor_manager_start_all()` | Start all registered sensors |
| `sensor_base_set_callback()` | Register data callback |

### Driver-Specific API

Each driver provides configuration and control functions:
- `icm20948_set_accel_range()`
- `ublox_enable_constellation()`
- `gst_pipeline_set_format()`
- `ouster_set_mode()`

## Testing

Run the test suite:

```bash
make test
./tests/sensor_test
```

Tests include:
- Sensor manager lifecycle
- Individual driver creation
- Pipeline generation
- Data callback verification

## License

MIT License - See LICENSE file for details.

## Contributing

Contributions welcome! Please follow the existing code style and add tests for new functionality.
