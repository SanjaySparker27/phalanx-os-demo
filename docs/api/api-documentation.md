# API Documentation

Complete API reference for the ATHENA H743 PRO flight controller integration with Cule OS and companion computer systems.

## Table of Contents
- [MAVLink API](#mavlink-api)
- [Cule OS Agent API](#cule-os-agent-api)
- [Flight Controller Parameters](#flight-controller-parameters)
- [Jetson Integration API](#jetson-integration-api)
- [Custom Firmware Extensions](#custom-firmware-extensions)
- [Data Logging API](#data-logging-api)

---

## MAVLink API

### Connection Setup

```python
from pymavlink import mavutil

# Connect to flight controller via Jetson
master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)

# Wait for heartbeat
master.wait_heartbeat()
print(f"Heartbeat from system {master.target_system}")
```

### Core Messages

#### Heartbeat
```python
# Send heartbeat
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_QUADROTOR,
    mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
    mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
    0,
    mavutil.mavlink.MAV_STATE_ACTIVE
)
```

#### Command Long
```python
# Arm vehicle
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0  # param1=1 (arm)
)

# Change mode
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
    0,
    1,  # base mode
    mode,  # custom mode
    0, 0, 0, 0, 0
)
```

### Message Handlers

#### Attitude
```python
# Request attitude data
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    30,  # ATTITUDE message ID
    100000,  # interval in microseconds (10Hz)
    0, 0, 0, 0, 0, 0
)

# Handle attitude message
def handle_attitude(msg):
    roll = msg.roll
    pitch = msg.pitch
    yaw = msg.yaw
    rollspeed = msg.rollspeed
    pitchspeed = msg.pitchspeed
    yawspeed = msg.yawspeed
```

#### GPS
```python
# GPS_RAW_INT message
def handle_gps(msg):
    lat = msg.lat / 1e7  # degrees
    lon = msg.lon / 1e7  # degrees
    alt = msg.alt / 1000  # meters
    eph = msg.eph / 100   # HDOP
    satellites_visible = msg.satellites_visible
    fix_type = msg.fix_type  # 0-1: no fix, 2: 2D, 3: 3D
```

#### Battery
```python
# SYS_STATUS message
def handle_battery(msg):
    voltage = msg.voltage_battery / 1000.0  # volts
    current = msg.current_battery / 100.0   # amps
    battery_remaining = msg.battery_remaining  # percent
```

### RC Channel Interface

```python
# Send RC override (take control from transmitter)
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    1500,  # channel 1 (roll)
    1500,  # channel 2 (pitch)
    1000,  # channel 3 (throttle)
    1500,  # channel 4 (yaw)
    0,     # channel 5
    0,     # channel 6
    0,     # channel 7
    1200   # channel 8 (mode/pusher - Loiter)
)
```

### Mission Protocol

```python
# Upload mission
from pymavlink.mavutil import mavlink

# Create waypoint
wp = mavlink.MAVLink_mission_item_message(
    master.target_system,
    master.target_component,
    seq=0,           # waypoint sequence
    frame=mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    command=mavlink.MAV_CMD_NAV_WAYPOINT,
    current=1,
    autocontinue=1,
    param1=0,        # hold time
    param2=5,        # acceptance radius
    param3=0,
    param4=0,
    x=lat,           # latitude
    y=lon,           # longitude
    z=alt            # altitude
)

# Send waypoint count
master.waypoint_count_send(1)

# Wait for request and send waypoint
master.mav.send(wp)
```

---

## Cule OS Agent API

### Perception Agent

```python
from cule_os.agents import PerceptionAgent

# Initialize perception agent
perception = PerceptionAgent(
    camera_source='/dev/video0',
    model_path='/opt/cule-os/models/yolov8n.onnx',
    inference_rate=30
)

# Start perception
perception.start()

# Get detections
detections = perception.get_detections()
for det in detections:
    print(f"Class: {det.class_id}, Confidence: {det.confidence}")
    print(f"BBox: ({det.x1}, {det.y1}, {det.x2}, {det.y2})")
```

### Planning Agent

```python
from cule_os.agents import PlanningAgent

# Initialize planning agent
planning = PlanningAgent(
    mpc_horizon=20,
    mpc_dt=0.05,
    obstacle_avoidance=True
)

# Set target
target = {
    'position': [10.0, 5.0, -5.0],  # x, y, z (NED)
    'velocity': [2.0, 0.0, 0.0],
    'heading': 0.0
}

# Generate trajectory
trajectory = planning.plan_trajectory(
    current_state=current_state,
    target=target,
    obstacles=obstacles
)

# Get control commands
control = planning.get_control()
```

### Control Agent

```python
from cule_os.agents import ControlAgent

# Initialize control agent
control = ControlAgent(
    control_rate=1000,
    cbf_enabled=True
)

# Update setpoint
control.set_setpoint(
    position=[0, 0, -5],
    velocity=[1, 0, 0],
    attitude=[0, 0, 0],
    rates=[0, 0, 0]
)

# Get safe control commands
# CBF ensures physical constraints
safe_commands = control.compute_control(
    current_state=state,
    desired_commands=commands
)
```

### Communication Agent

```python
from cule_os.agents import CommunicationAgent

# Initialize communication
comm = CommunicationAgent(
    device='/dev/ttyTHS1',
    baudrate=921600,
    protocol='MAVLink2'
)

# Send telemetry
comm.send_telemetry({
    'position': [x, y, z],
    'velocity': [vx, vy, vz],
    'attitude': [roll, pitch, yaw]
})

# Receive commands
cmd = comm.receive_command()
if cmd:
    print(f"Received command: {cmd.type}")
```

---

## Flight Controller Parameters

### Parameter Read/Write

```python
# Read parameter
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'FRAME_CLASS',
    -1
)

# Handle parameter value
def handle_param_value(msg):
    name = msg.param_id
    value = msg.param_value
    print(f"{name} = {value}")

# Write parameter
master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'FRAME_TYPE',
    13.0,  # value
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)
```

### Critical Parameters Reference

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| FRAME_CLASS | Frame type | 2 | 0-15 |
| FRAME_TYPE | Frame configuration | 13 | 0-15 |
| FLTMODE_CH | Mode switch channel | 8 | 5-8 |
| SERVO7_FUNCTION | Pusher motor | 51 | 0-99 |
| RC8_OPTION | RC8 function | 0 | 0-300+ |
| ARMING_CHECK | Pre-arm checks | 0 | bitmask |
| FS_THR_ENABLE | Throttle failsafe | 1 | 0-2 |
| BATT_MONITOR | Battery monitor | 4 | 0-21 |

---

## Jetson Integration API

### GPIO Control

```python
import Jetson.GPIO as GPIO

# Setup GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

# Control output
GPIO.output(12, GPIO.HIGH)

# Cleanup
GPIO.cleanup()
```

### Camera Capture (GStreamer)

```python
import cv2

# Open camera with GStreamer pipeline
def gstreamer_pipeline(
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=30,
    flip_method=0
):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), "
        f"width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, "
        f"format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! appsink"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
```

### CUDA Operations

```python
import cupy as cp

# GPU array
x_gpu = cp.array([1, 2, 3, 4, 5])

# GPU computation
y_gpu = cp.sin(x_gpu)

# Transfer to CPU
y_cpu = cp.asnumpy(y_gpu)
```

---

## Custom Firmware Extensions

### Mode Switching Logic

The custom firmware implements mode switching in C++:

```cpp
// RC_Channel.cpp - custom_slider_mode_switch()
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

### Custom MAVLink Messages

```cpp
// Custom message for pusher motor status
PACKED(
struct mavlink_pusher_status_t {
    uint32_t timestamp;
    uint16_t pwm_output;
    uint8_t enabled;
    uint8_t mode;
});

// Send custom message
void send_pusher_status() {
    mavlink_pusher_status_t status;
    status.timestamp = AP_HAL::millis();
    status.pwm_output = SRV_Channels::get_output_pwm(SRV_Channel::k_motor7);
    status.enabled = (rc().channel(7)->get_control_in() > 0);
    status.mode = (uint8_t)copter.flightmode->mode_number();
    
    mavlink_msg_pusher_status_send_struct(
        MAVLINK_COMM_0, &status);
}
```

---

## Data Logging API

### Flight Log Analysis

```python
import pandas as pd
from pymavlink import mavutil

# Load log file
mavlog = mavutil.mavlink_connection('00000001.BIN')

# Extract messages
attitude_data = []
gps_data = []

while True:
    msg = mavlog.recv_match(blocking=False)
    if msg is None:
        break
    
    if msg.get_type() == 'ATT':
        attitude_data.append({
            'timestamp': msg.TimeUS,
            'roll': msg.Roll,
            'pitch': msg.Pitch,
            'yaw': msg.Yaw
        })
    elif msg.get_type() == 'GPS':
        gps_data.append({
            'timestamp': msg.TimeUS,
            'lat': msg.Lat,
            'lon': msg.Lng,
            'alt': msg.Alt,
            'hdop': msg.HDop
        })

# Convert to DataFrames
df_attitude = pd.DataFrame(attitude_data)
df_gps = pd.DataFrame(gps_data)
```

### Real-time Logging

```python
import logging

# Setup logging
logging.basicConfig(
    filename='/var/log/cule-os/flight.log',
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# Log events
logging.info(f"Mode changed to {new_mode}")
logging.warning(f"Battery low: {voltage}V")
logging.error(f"GPS fix lost")
```

### Telemetry Stream

```python
import asyncio
import websockets
import json

async def telemetry_server(websocket, path):
    while True:
        # Get latest telemetry
        telemetry = {
            'timestamp': time.time(),
            'position': [x, y, z],
            'attitude': [roll, pitch, yaw],
            'battery': voltage,
            'mode': current_mode
        }
        
        await websocket.send(json.dumps(telemetry))
        await asyncio.sleep(0.1)  # 10Hz

# Start server
start_server = websockets.serve(telemetry_server, "localhost", 8765)
asyncio.get_event_loop().run_until_complete(start_server)
```

---

## Example Integration

### Complete Control Loop

```python
import time
from pymavlink import mavutil
from cule_os.agents import PerceptionAgent, PlanningAgent, ControlAgent

class DroneController:
    def __init__(self):
        # Connect to flight controller
        self.master = mavutil.mavlink_connection('/dev/ttyTHS1', baud=921600)
        self.master.wait_heartbeat()
        
        # Initialize agents
        self.perception = PerceptionAgent()
        self.planning = PlanningAgent()
        self.control = ControlAgent()
        
    def run(self):
        while True:
            # 1. Get current state from MAVLink
            state = self.get_state()
            
            # 2. Perception - detect obstacles
            detections = self.perception.get_detections()
            obstacles = self.parse_detections(detections)
            
            # 3. Planning - compute trajectory
            trajectory = self.planning.plan_trajectory(
                current_state=state,
                target=self.target,
                obstacles=obstacles
            )
            
            # 4. Control - compute safe commands
            commands = self.control.compute_control(
                current_state=state,
                desired_trajectory=trajectory
            )
            
            # 5. Send commands to FC
            self.send_commands(commands)
            
            time.sleep(0.02)  # 50Hz
    
    def get_state(self):
        # Read attitude
        att_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
        # Read position
        pos_msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        # Combine into state vector
        return state
    
    def send_commands(self, commands):
        # Send RC override or setpoint
        pass

if __name__ == '__main__':
    controller = DroneController()
    controller.run()
```

---

## References

- [MAVLink Protocol](https://mavlink.io/)
- [ArduPilot MAVLink Interface](https://ardupilot.org/dev/docs/mavlink-basics.html)
- [DroneKit Python](https://dronekit-python.readthedocs.io/)
- [Cule OS Documentation](https://cule-os.io/api)
- [Jetson GPIO](https://github.com/NVIDIA/jetson-gpio)
