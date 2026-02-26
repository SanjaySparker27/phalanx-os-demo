# Mally Multi-Agent System

A Python-based multi-agent system for autonomous vehicles (UAV/USV/UGV) with specialized agents for perception, navigation, planning, communication, and swarm coordination.

## Architecture

```
mally/
├── core/                 # Base agent infrastructure
│   ├── base_agent.py     # Abstract base class, message bus, registry
├── agents/               # Specialized agents
│   ├── perception/       # YOLO/ONNX object detection
│   ├── navigation/       # ORB-SLAM3 GPS-denied navigation
│   ├── planning/         # Model Predictive Control
│   ├── communication/    # MAVLink 2.0 and SATCOM
│   └── swarm/            # Distributed consensus with Raft
├── utils/                # Utilities
│   └── config.py         # Configuration management
├── main.py               # System entry point
└── requirements.txt      # Dependencies
```

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Run with default configuration
python -m mally

# Run with custom configuration
python -m mally --config config.yaml
```

## Agents

### PerceptionAgent
Real-time object detection using YOLOv8 with ONNX Runtime.
- Camera-based detection (person, vehicle, obstacle, landing_zone)
- SORT tracking for persistent IDs
- CUDA/TensorRT acceleration
- Async inference pipeline

### NavigationAgent
GPS-denied navigation using ORB-SLAM3.
- Visual odometry with IMU integration
- Loop closure detection
- Map save/load for relocalization
- Trajectory estimation

### PlanningAgent
Model Predictive Control for trajectory generation.
- Real-time trajectory optimization
- Obstacle avoidance constraints
- Reference trajectory following
- Configurable constraints and costs

### CommunicationAgent
MAVLink 2.0 and SATCOM protocol support.
- MAVLink telemetry streaming
- Command and control interface
- SATCOM fallback (Iridium SBD)
- Multi-system coordination

### SwarmOrchestrator
Distributed multi-vehicle coordination.
- Raft consensus algorithm
- Leader election
- Formation control (line, grid, circle)
- Health monitoring

## Configuration

Create a `config.yaml`:

```yaml
perception:
  camera_source: "0"
  model_path: "models/yolov8n.onnx"
  detection_threshold: 0.5
  onnx_runtime: "cuda"

navigation:
  slam_config_path: "config/orb_slam3.yaml"
  use_gps_denied: true

planning:
  mpc_horizon: 20
  control_frequency: 50.0

communication:
  mavlink_port: "/dev/ttyUSB0"
  satcom_enabled: false

swarm:
  consensus_algorithm: "raft"
  formation_type: "leader_follower"
```

## Message Protocol

Agents communicate via async message bus:

```python
# Send detection request
await perception.send_message(
    target="navigation",
    payload={"frame_data": image},
    message_type="DETECT_REQUEST"
)

# Broadcast to all agents
await swarm.broadcast(
    payload={"formation": "grid"},
    message_type="FORMATION_REQUEST"
)
```

## Testing

```bash
# Run all tests
pytest mally/tests/

# Run specific agent tests
pytest mally/tests/test_perception.py
```

## License

MIT License - See LICENSE file
