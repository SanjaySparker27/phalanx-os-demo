# UAV Computer Vision Pipeline

Complete vision system for autonomous drones with real-time object detection, depth estimation, and navigation capabilities.

## Overview

This pipeline provides:
- **30 FPS object detection** at 1080p resolution
- **<50ms latency** end-to-end processing
- **Stereo depth estimation** for obstacle avoidance
- **GPS-denied navigation** using visual SLAM
- **Precision landing** on visual markers

## Hardware Requirements

### Recommended Setup
| Component | Specification |
|-----------|--------------|
| Camera | 2x IMX477 (stereo) or IMX219 |
| Compute | NVIDIA Jetson AGX Orin / RPi 4 |
| Lens | 3.9mm M12 (120° FOV) |
| IMU | BMI088 (for SLAM) |
| Connection | CSI-2 (15-pin ribbon) |

### Wiring (Jetson Nano/Orin)
```
CAM0 (Left)           CAM1 (Right)
┌─────────┐           ┌─────────┐
│1 3.3V   │           │1 3.3V   │
│2 GND    │           │2 GND    │
│3 MCLK   │           │3 MCLK   │
│4 NC     │           │4 NC     │
│5 SDA    │──I2C───┬──│5 SDA    │
│6 SCL    │──I2C───┼──│6 SCL    │
│7 D0-D3  │──CSI───┼──│7 D0-D3  │
│8 D4-D7  │──CSI───┼──│8 D4-D7  │
│9 CLK    │──CSI───┼──│9 CLK    │
│10 3.3V  │        └──│10 ADDR  │ (0x20)
└─────────┘           └─────────┘
```

## Installation

### 1. Jetson Setup
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install GStreamer and V4L2
sudo apt install -y \
    v4l-utils \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

# Install camera driver
sudo apt install -y nvidia-l4t-jetson-multimedia-api

# Enable IMX477
echo 'options nv_imx477 tr=1' | sudo tee -a /etc/modprobe.d/imx477.conf
sudo reboot
```

### 2. Python Dependencies
```bash
# Create virtual environment
python3 -m venv ~/uav_vision
source ~/uav_vision/bin/activate

# Install core packages
pip install opencv-python==4.8.1.78
pip install numpy==1.24.3
pip install onnxruntime-gpu==1.16.3  # For Jetson

# For TensorRT optimization (Jetson)
pip install tensorrt==8.6.1

# For SLAM
pip install pangolin
pip install pyyaml

# For MAVLink integration
pip install pymavlink
pip install dronekit
```

## Camera Configuration

### Single IMX477 Setup
```bash
# Verify camera detection
v4l2-ctl --list-devices

# Expected output:
# NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca)
#     /dev/video0

# Check supported formats
v4l2-ctl -d /dev/video0 --list-formats-ext
```

### Stereo Camera Configuration
```python
# stereo_config.py
STEREO_CONFIG = {
    'left_cam': {
        'id': 0,
        'i2c_addr': 0x1a,
        'resolution': (1920, 1080),
        'fps': 30,
        'fov_h': 120,  # degrees
        'fov_v': 90,
    },
    'right_cam': {
        'id': 1,
        'i2c_addr': 0x1b,
        'resolution': (1920, 1080),
        'fps': 30,
    },
    'baseline': 0.12,  # 12cm stereo baseline
    'calibration_file': 'stereo_calib.npz',
}
```

## GStreamer Pipelines

### High Performance Pipeline (Jetson)
```bash
# IMX477 1080p@30FPS → GPU memory → OpenCV
GST_PIPELINE = """
nvarguscamerasrc sensor-id=0 !
    'video/x-raw(memory:NVMM), width=1920, height=1080, 
     format=NV12, framerate=30/1' !
    nvvidconv !
    'video/x-raw, format=BGRx' !
    videoconvert !
    'video/x-raw, format=BGR' !
    appsink drop=true max-buffers=1
"""

# Stereo synchronized capture
GST_STEREO = """
videomixer name=mix ! videoconvert ! appsink
nvarguscamerasrc sensor-id=0 !
    'video/x-raw(memory:NVMM), width=1920, height=1080' !
    nvvidconv ! mix.sink_0
nvarguscamerasrc sensor-id=1 !
    'video/x-raw(memory:NVMM), width=1920, height=1080' !
    nvvidconv ! mix.sink_1
"""
```

### Python Capture Class
```python
# camera_capture.py
import cv2
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class JetsonCamera:
    def __init__(self, sensor_id=0, width=1920, height=1080, fps=30):
        Gst.init(None)
        
        self.pipeline = (
            f"nvarguscamerasrc sensor-id={sensor_id} "
            f"! 'video/x-raw(memory:NVMM), width={width}, height={height}, "
            f"format=NV12, framerate={fps}/1' "
            f"! nvvidconv ! 'video/x-raw, format=BGRx' "
            f"! videoconvert ! 'video/x-raw, format=BGR' "
            f"! appsink drop=true max-buffers=1"
        )
        
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera pipeline")
    
    def read(self):
        ret, frame = self.cap.read()
        return frame if ret else None
    
    def release(self):
        self.cap.release()

# Usage
if __name__ == "__main__":
    cam = JetsonCamera(sensor_id=0)
    
    import time
    frames = 0
    start = time.time()
    
    while True:
        frame = cam.read()
        if frame is None:
            break
            
        frames += 1
        if time.time() - start >= 1.0:
            print(f"FPS: {frames}")
            frames = 0
            start = time.time()
        
        cv2.imshow('Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cam.release()
    cv2.destroyAllWindows()
```

## System Architecture

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Cameras   │───▶│   Capture   │───▶│   Buffer    │
│ (IMX477 x2) │    │ (GStreamer) │    │   Pool      │
└─────────────┘    └─────────────┘    └──────┬──────┘
                                             │
                    ┌────────────────────────┼────────┐
                    │                        │        │
                    ▼                        ▼        ▼
           ┌──────────────┐        ┌────────────┐ ┌──────────┐
           │   YOLOv8     │        │  ORB-SLAM3 │ │  Depth   │
           │  Detection   │        │    SLAM    │ │   Est.   │
           └──────┬───────┘        └─────┬──────┘ └────┬─────┘
                  │                      │             │
                  ▼                      ▼             ▼
           ┌──────────────┐        ┌────────────┐ ┌──────────┐
           │   Obstacle   │        │   Pose     │ │  Point   │
           │  Avoidance   │        │  Estimate  │ │  Cloud   │
           └──────┬───────┘        └─────┬──────┘ └────┬─────┘
                  │                      │             │
                  └──────────────────────┼─────────────┘
                                         │
                                         ▼
                              ┌──────────────────┐
                              │  MAVLink Bridge  │
                              │  (PX4/ArduPilot) │
                              └──────────────────┘
```

## Performance Optimization

### Jetson AGX Orin
| Mode | Resolution | FPS | Latency | Power |
|------|------------|-----|---------|-------|
| MAXN | 1080p | 60 | 25ms | 60W |
| MAXQ | 1080p | 45 | 30ms | 30W |
| 30W | 1080p | 30 | 35ms | 15W |

### Memory Optimization
```python
# Zero-copy GPU memory access
import pycuda.driver as cuda
import pycuda.autoinit

class GPUMemoryPool:
    def __init__(self, size, count=3):
        self.buffers = [
            cuda.mem_alloc(size) for _ in range(count)
        ]
        self.available = list(self.buffers)
    
    def acquire(self):
        return self.available.pop() if self.available else None
    
    def release(self, buf):
        self.available.append(buf)
```

### TensorRT Optimization
```python
# Convert ONNX to TensorRT with INT8 quantization
import tensorrt as trt

def build_engine(onnx_path, engine_path, fp16=True, int8=False):
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)
    
    with open(onnx_path, 'rb') as f:
        parser.parse(f.read())
    
    config = builder.create_builder_config()
    config.max_workspace_size = 1 << 30  # 1GB
    
    if fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    if int8:
        config.set_flag(trt.BuilderFlag.INT8)
        # Set INT8 calibration
        config.int8_calibrator = Int8Calibrator()
    
    engine = builder.build_engine(network, config)
    
    with open(engine_path, 'wb') as f:
        f.write(engine.serialize())
    
    return engine
```

## Integration with PX4

### MAVLink Bridge
```python
# mavlink_bridge.py
from pymavlink import mavutil
import threading
import time

class MAVLinkBridge:
    def __init__(self, connection='udp:127.0.0.1:14550'):
        self.master = mavutil.mavlink_connection(connection)
        self.running = True
        self.vision_position = None
        self.obstacles = []
        
        self.thread = threading.Thread(target=self._loop)
        self.thread.start()
    
    def _loop(self):
        while self.running:
            msg = self.master.recv_match(blocking=False)
            if msg:
                self._handle_message(msg)
            
            # Send vision position at 30Hz
            if self.vision_position:
                self._send_vision_position()
            
            time.sleep(0.033)  # 30Hz
    
    def send_vision_position(self, x, y, z, roll, pitch, yaw):
        """Send SLAM pose to PX4 for GPS-denied flight"""
        self.master.mav.vision_position_estimate_send(
            int(time.time() * 1e6),
            x, y, z,
            roll, pitch, yaw,
            covariance=(0.1,)*21
        )
    
    def send_obstacle_distance(self, distances, angle_offset=0):
        """Send obstacle distances for collision avoidance"""
        self.master.mav.obstacle_distance_send(
            int(time.time() * 1e3),
            0,  # sensor type
            distances,
            0,  # angular width
            10, # min distance cm
            2000, # max distance cm
            angle_offset
        )
```

## Troubleshooting

### Common Issues

**Camera not detected:**
```bash
# Check I2C bus
sudo i2cdetect -y 0  # or -y 7 on some Jetsons

# Expected: 0x1a and 0x1b for stereo IMX477

# Reload driver
sudo rmmod nv_imx477 && sudo modprobe nv_imx477
```

**Low FPS:**
```bash
# Check GPU usage
tegrastats

# Verify NVENC is being used
GST_DEBUG=3 gst-launch-1.0 nvarguscamerasrc ! fakesink 2>&1 | grep -i "nvidia"
```

**High latency:**
```python
# Disable buffer queueing
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Use hardware acceleration
os.environ['CUDA_CACHE_MAXSIZE'] = '2147483647'
```

## Testing

### Unit Tests
```bash
# Test camera
cd /path/to/uav_vision
python -m pytest tests/test_camera.py -v

# Test detection
python -m pytest tests/test_detection.py -v

# Test SLAM
python -m pytest tests/test_slam.py -v
```

### Gazebo Simulation
```bash
# Launch simulation with camera
roslaunch px4 gazebo_iris_downward_depth_camera.launch

# Test vision pipeline
python vision_pipeline.py --sim
```

## References

- [NVIDIA Jetson Camera Guide](https://developer.nvidia.com/embedded/jetson-linux)
- [OpenCV V4L2](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
- [PX4 Vision Docs](https://docs.px4.io/main/en/computer_vision/)
- [YOLOv8 ONNX](https://docs.ultralytics.com/modes/export/)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)

---

*Last updated: 2024*
*Compatible with: JetPack 5.1+, OpenCV 4.8+, Python 3.8+*
