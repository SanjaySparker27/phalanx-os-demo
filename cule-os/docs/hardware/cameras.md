# Cameras

Camera integration for Cule OS including CSI, USB, and GMSL interfaces.

## Camera Overview

Cameras provide visual data for:
- First-person view (FPV) piloting
- Computer vision and AI inference
- Mapping and photogrammetry
- Object detection and tracking
- SLAM navigation

## Camera Types

### Interface Comparison

| Type | Bandwidth | Latency | Cable Length | Best For |
|------|-----------|---------|--------------|----------|
| MIPI CSI-2 | High | Low | <30cm | Onboard AI, close mounting |
| USB 2.0 | Medium | Medium | <5m | Webcams, simple vision |
| USB 3.0 | High | Medium | <5m | High-res vision |
| GMSL/FPD-Link | Very High | Very Low | <15m | Long cable, automotive |
| HDMI | Very High | Low | <5m | Display output |
| Ethernet | Medium | Variable | <100m | IP cameras, streaming |

### Resolution & Frame Rate

| Camera | Resolution | FPS | Interface | Platform |
|--------|-----------|-----|-----------|----------|
| Raspberry Pi Cam V2 | 8MP (3280x2464) | 30 | CSI | RPi, Jetson |
| Raspberry Pi Cam 3 | 12MP (4056x3040) | 30 | CSI | RPi, Jetson |
| Pi HQ Camera | 12MP | 30 | CSI | RPi, Jetson |
| IMX477 (Arducam) | 12MP | 60 | CSI | Jetson |
| Intel RealSense D435 | 1280x720 | 30 | USB 3.0 | All |
| ZED 2i | 1920x1080 | 30 | USB 3.0 | Jetson, PC |
| Leopard Imaging GMSL | 8MP | 60 | GMSL | Jetson |

## CSI Cameras

### Raspberry Pi Camera on Jetson

```
CSI Camera Connection (Jetson Orin)
══════════════════════════════════════════════════════════════════

     ┌─────────────────┐
     │   Jetson Orin   │
     │   CSI Port      │
     │                 │
     │   CAM0  CAM1    │
     │   ┌─┐   ┌─┐     │
     │   └─┘   └─┘     │
     └────┬─────┬──────┘
          │     │
          │     └──────────────┐
          │                    │
          ▼                    ▼
    ┌──────────┐         ┌──────────┐
    │   CSI    │         │   CSI    │
    │ Ribbon   │         │ Ribbon   │
    │ Cable    │         │ Cable    │
    └────┬─────┘         └────┬─────┘
         │                    │
         ▼                    ▼
    ┌──────────┐         ┌──────────┐
    │  IMX219  │         │  IMX219  │
    │ (Pi Cam  │         │ (Pi Cam  │
    │   V2)    │         │   V2)    │
    └──────────┘         └──────────┘

Cable Requirements:
- 15-pin, 1mm pitch FFC
- Length: 5-30cm (keep short for signal integrity)
- Direction: Contacts face PCB on camera
```

### CSI Camera Configuration (Jetson)

```bash
# 1. Enable CSI port
sudo /opt/nvidia/jetson-io/jetson-io.py
# Configure Jetson 40-pin header
# Enable CSI camera interface

# 2. Check camera detection
ls /dev/video*
# Should show /dev/video0

# 3. Test with GStreamer
gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM),\
    width=1920, height=1080, format=NV12, framerate=30/1' ! \
    nvvidconv ! xvimagesink

# 4. Cule OS configuration
sudo cule-config --camera csi --model imx219
```

### CSI Camera Configuration (Raspberry Pi)

```bash
# 1. Enable camera
sudo raspi-config
# Interface Options -> Camera -> Enable

# 2. Check detection
libcamera-hello --list-cameras

# 3. Test
camera-hello
camera-vid -t 10000 -o test.h264

# 4. Cule OS configuration
sudo cule-config --camera csi --model imx708
```

## USB Cameras

### USB Camera Connection

```
USB Camera Setup
══════════════════════════════════════════════════════════════════

     ┌─────────────────┐
     │ Companion       │
     │ Computer        │
     │                 │
     │  [USB 3.0] [USB 3.0]  [USB 2.0]  [USB 2.0] │
     └──────┬──────────┬─────────┬─────────┘
            │          │         │
            │          │         └──────────────┐
            │          │                        │
            ▼          ▼                        ▼
     ┌──────────┐ ┌──────────┐          ┌──────────┐
     │ RealSense│ │ ZED 2i   │          │ USB      │
     │ D435     │ │ Stereo   │          │ Webcam   │
     │ (Depth)  │ │ (Depth)  │          │ (Simple) │
     └──────────┘ └──────────┘          └──────────┘

Power Budget:
- USB 3.0 provides 900mA @ 5V
- RealSense D435: ~400mA
- ZED 2i: ~600mA (requires powered hub)
- Basic webcam: ~200mA
```

### Intel RealSense Setup

```bash
# 1. Install RealSense SDK
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake .. -DFORCE_RSUSB_BACKEND=true
make -j4
sudo make install

# 2. Test
realsense-viewer

# 3. Python wrapper
pip install pyrealsense2

# 4. Cule OS integration
sudo cule-config --camera usb --model realsense-d435
```

### ZED Camera Setup

```bash
# 1. Install ZED SDK
# Download from stereolabs.com
chmod +x ZED_SDK_Tegras_L4T36.x_v4.0.0.run
./ZED_SDK_Tegras_L4T36.x_v4.0.0.run

# 2. Test
/usr/local/zed/tools/ZED_Explorer

# 3. Python API
pip install pyzed

# 4. Cule OS integration
sudo cule-config --camera usb --model zed-2i
```

## GMSL/FPD-Link Cameras

### GMSL Overview

```
GMSL (Gigabit Multimedia Serial Link)
══════════════════════════════════════════════════════════════════

Advantages:
- Single coax cable (power + data)
- Up to 15m cable length
- High bandwidth (6Gbps)
- Low latency
- Automotive grade

Components:
- Serializer (camera side)
- Deserializer (host side)
- Coax cable with power over coax (PoC)

Supported Platforms:
- NVIDIA Jetson (via GMSL deserializer board)
- Custom FPGA solutions
```

### GMSL Camera Connection

```
GMSL Camera Setup (Jetson)
══════════════════════════════════════════════════════════════════

     ┌──────────────────────────────────┐
     │   GMSL Deserializer Board        │
     │   (e.g., Leopard Imaging)        │
     │                                  │
     │   GMSL0  GMSL1  GMSL2  GMSL3    │
     │    ●      ●      ●      ●       │
     └────┬──────┬──────┬──────┬───────┘
          │      │      │      │
          │      │      │      └─────────────┐
          │      │      │                    │
          │      │      └──────┐             │
          │      │             │             │
          ▼      ▼             ▼             ▼
    ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
    │  IMX390│ │  IMX390│ │  IMX490│ │ IMX624 │
    │ (Front)│ │  (Rear)│ │ (Side) │ │(Night) │
    │  2MP   │ │  2MP   │ │  5MP   │ │  8MP   │
    └────────┘ └────────┘ └────────┘ └────────┘
          │      │             │             │
          └──────┴─────────────┴─────────────┘
                         │
                         ▼
                ┌─────────────────┐
                │   Jetson Orin   │
                │   CSI (via GMSL │
                │   deserializer) │
                └─────────────────┘

Benefits for Robotics:
- Long cable runs for multi-camera
- Simplified wiring
- Synchronized multi-camera capture
```

## Camera Configuration for Cule OS

### Cule OS Camera API

```python
from cule import Camera

# Initialize camera
cam = Camera.open('/dev/video0')

# Configure
config = {
    'width': 1920,
    'height': 1080,
    'fps': 30,
    'format': 'NV12'
}
cam.configure(config)

# Start streaming
cam.start()

# Get frame
frame = cam.capture()

# Process with OpenCV
import cv2
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Stop
cam.stop()
cam.close()
```

### Multi-Camera Setup

```python
from cule import MultiCameraSystem

# Setup multi-camera system
system = MultiCameraSystem()

# Add cameras
system.add_camera('front', '/dev/video0', 'csi')
system.add_camera('down', '/dev/video1', 'csi')
system.add_camera('depth', '/dev/video2', 'realsense')

# Synchronized capture
frames = system.capture_sync()

# Access individual frames
front_frame = frames['front']
down_frame = frames['down']
depth_frame = frames['depth']
```

## Camera Calibration

### Intrinsic Calibration

```bash
# Using Cule OS calibration tool
sudo cule-calibrate camera --type intrinsic

# Process:
# 1. Print chessboard pattern
# 2. Show to camera from multiple angles
# 3. Capture 20+ images
# 4. Calculate camera matrix
# 5. Save calibration file

# Or use OpenCV directly
python3 calibrate_camera.py --width 9 --height 6 --square 0.025
```

### Stereo Calibration

```bash
# For stereo cameras (ZED, RealSense, dual CSI)
sudo cule-calibrate camera --type stereo

# Process:
# 1. Calibrate each camera individually
# 2. Stereo calibration for extrinsic parameters
# 3. Rectification maps
# 4. Disparity to depth calibration

# Output:
# - camera_matrix.yml
# - distortion_coeffs.yml
# - stereo_params.yml
```

## Vision Processing Pipeline

```
Cule OS Vision Pipeline
══════════════════════════════════════════════════════════════════

┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Camera    │───►│   Capture   │───►│  Preprocess │───►│    AI/ML    │
│  (CSI/USB)  │    │   (V4L2/    │    │ (Debayer,   │    │  (TensorRT/ │
│             │    │   GStreamer)│    │  Undistort) │    │   OpenCV)   │
└─────────────┘    └─────────────┘    └─────────────┘    └──────┬──────┘
                                                                 │
                                                                 ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Output    │◄───│  Postprocess│◄───│   Tracking  │◄───│   Detection │
│  (Display/  │    │  (Overlay,  │    │   (Kalman,  │    │  (YOLO/NMS) │
│   Stream)   │    │  Annotate)  │    │   Optical)  │    │             │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘

Hardware Acceleration:
- NVIDIA: VIC (Video Image Compositor) + GPU
- Jetson: NVMM buffers (zero-copy)
- Raspberry Pi: VideoCore GPU
```

## GStreamer Pipelines

### Basic Capture

```bash
# CSI camera on Jetson
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
    'video/x-raw(memory:NVMM), width=1920, height=1080, \
    format=NV12, framerate=30/1' ! nvvidconv ! \
    'video/x-raw, format=BGRx' ! videoconvert ! \
    'video/x-raw, format=BGR' ! appsink

# USB camera on any Linux
gst-launch-1.0 v4l2src device=/dev/video0 ! \
    'video/x-raw, width=1920, height=1080' ! \
    videoconvert ! appsink
```

### Hardware Encoding

```bash
# H.264 hardware encode (Jetson)
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM), width=1920, height=1080' ! \
    nvv4l2h264enc ! h264parse ! qtmux ! \
    filesink location=output.mp4

# H.265 hardware encode
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM), width=3840, height=2160' ! \
    nvv4l2h265enc ! h265parse ! qtmux ! \
    filesink location=output_4k.mp4
```

## Troubleshooting

### CSI Camera Issues

```bash
# Camera not detected
ls /dev/video*
# No output = camera not detected

# Check ribbon cable orientation
# Blue side up on Jetson, contacts face PCB on camera

# Check kernel messages
dmesg | grep -i imx

# Verify overlay loaded
ls /boot/overlays/ | grep cam

# Re-enable interface
sudo /opt/nvidia/jetson-io/jetson-io.py
```

### USB Camera Issues

```bash
# Check detection
lsusb
# Look for vendor ID

# Check device nodes
ls -la /dev/video*

# Test with v4l2
v4l2-ctl -d /dev/video0 --all

# Check formats
v4l2-ctl -d /dev/video0 --list-formats-ext

# Reset USB
sudo sh -c 'echo 0 > /sys/bus/usb/rescan'
```

## See Also

- [CSI Cameras](./csi-cameras.md) - Detailed CSI setup
- [USB Cameras](./usb-cameras.md) - USB camera guide
- [GMSL Cameras](./gmsl-cameras.md) - GMSL integration
- [Vision Processing](./vision.md) - AI/ML vision
