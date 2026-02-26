"""
Configuration and Utilities for Mally Multi-Agent System
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Any
import json
import yaml
import logging
import asyncio
from datetime import datetime


@dataclass
class PerceptionConfig:
    """Perception Agent Configuration."""
    camera_source: str = "0"  # Device ID or RTSP URL
    model_path: str = "models/yolov8n.onnx"
    detection_threshold: float = 0.5
    onnx_runtime: str = "cuda"  # cpu, cuda, tensorrt
    input_size: tuple = (640, 640)
    classes_of_interest: List[str] = field(default_factory=lambda: [
        "person", "vehicle", "obstacle", "landing_zone"
    ])
    inference_interval: float = 0.033  # 30 FPS
    max_detections: int = 100
    tracking_enabled: bool = True
    track_history: int = 30


@dataclass
class NavigationConfig:
    """Navigation Agent Configuration."""
    slam_config_path: str = "config/orb_slam3.yaml"
    map_save_path: str = "maps/"
    use_gps_denied: bool = True
    visual_odometry: bool = True
    imu_topic: str = "/imu/data"
    camera_topic: str = "/camera/image_raw"
    gps_topic: str = "/gps/fix"
    voxel_size: float = 0.05
    localization_rate: float = 10.0  # Hz
    loop_closure_enabled: bool = True
    map_update_rate: float = 1.0  # Hz


@dataclass
class PlanningConfig:
    """Planning Agent Configuration."""
    mpc_horizon: int = 20
    control_frequency: float = 50.0  # Hz
    prediction_horizon: float = 2.0  # seconds
    constraints: Dict[str, Any] = field(default_factory=lambda: {
        "max_velocity": 15.0,  # m/s
        "max_acceleration": 5.0,  # m/s²
        "max_angular_rate": 2.0,  # rad/s
        "min_altitude": 2.0,  # m
        "max_altitude": 120.0,  # m
        "safety_margin": 1.0  # m
    })
    cost_weights: Dict[str, float] = field(default_factory=lambda: {
        "position_error": 1.0,
        "velocity_error": 0.5,
        "control_effort": 0.1,
        "obstacle_avoidance": 10.0
    })
    replan_threshold: float = 0.5  # m
    path_smoothing: bool = True


@dataclass
class CommunicationConfig:
    """Communication Agent Configuration."""
    mavlink_port: str = "/dev/ttyUSB0"
    mavlink_baudrate: int = 921600
    mavlink_sysid: int = 1
    mavlink_compid: int = 1
    satcom_enabled: bool = False
    satcom_endpoint: str = "iridium"
    satcom_imei: str = ""
    heartbeat_interval: float = 1.0  # seconds
    telemetry_rate: float = 10.0  # Hz
    command_ack_timeout: float = 5.0  # seconds
    max_reconnect_attempts: int = 5
    udp_listen_port: int = 14550
    udp_send_port: int = 14551


@dataclass
class SwarmConfig:
    """Swarm Orchestrator Configuration."""
    consensus_algorithm: str = "raft"  # raft, pbft, paxos
    neighbor_discovery: str = "broadcast"
    formation_config: Dict[str, Any] = field(default_factory=lambda: {
        "type": "leader_follower",  # leader_follower, virtual_structure, behavior_based
        "spacing": 5.0,  # m
        "alignment": "grid",  # grid, line, circle
        "max_neighbors": 10
    })
    election_timeout: float = 1.0  # seconds
    heartbeat_interval: float = 0.5  # seconds
    consensus_timeout: float = 5.0  # seconds
    fault_tolerance: int = 1  # max faulty nodes
    swarm_id: str = "swarm_001"
    leader_id: Optional[str] = None


@dataclass
class AgentConfig:
    """Master Configuration Container."""
    perception: PerceptionConfig = field(default_factory=PerceptionConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    planning: PlanningConfig = field(default_factory=PlanningConfig)
    communication: CommunicationConfig = field(default_factory=CommunicationConfig)
    swarm: SwarmConfig = field(default_factory=SwarmConfig)
    
    log_level: str = "INFO"
    log_path: str = "logs/"
    metrics_enabled: bool = True
    metrics_interval: float = 1.0
    
    @classmethod
    def from_yaml(cls, path: str) -> "AgentConfig":
        """Load configuration from YAML file."""
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return cls._from_dict(data)
    
    @classmethod
    def from_json(cls, path: str) -> "AgentConfig":
        """Load configuration from JSON file."""
        with open(path, 'r') as f:
            data = json.load(f)
        return cls._from_dict(data)
    
    @classmethod
    def _from_dict(cls, data: Dict[str, Any]) -> "AgentConfig":
        """Create config from dictionary."""
        config = cls()
        if 'perception' in data:
            config.perception = PerceptionConfig(**data['perception'])
        if 'navigation' in data:
            config.navigation = NavigationConfig(**data['navigation'])
        if 'planning' in data:
            config.planning = PlanningConfig(**data['planning'])
        if 'communication' in data:
            config.communication = CommunicationConfig(**data['communication'])
        if 'swarm' in data:
            config.swarm = SwarmConfig(**data['swarm'])
        return config
    
    def to_yaml(self, path: str):
        """Save configuration to YAML file."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w') as f:
            yaml.dump(self._to_dict(), f, default_flow_style=False)
    
    def to_json(self, path: str):
        """Save configuration to JSON file."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, 'w') as f:
            json.dump(self._to_dict(), f, indent=2)
    
    def _to_dict(self) -> Dict[str, Any]:
        """Convert config to dictionary."""
        return {
            'perception': self.perception.__dict__,
            'navigation': self.navigation.__dict__,
            'planning': self.planning.__dict__,
            'communication': self.communication.__dict__,
            'swarm': self.swarm.__dict__,
            'log_level': self.log_level,
            'log_path': self.log_path,
            'metrics_enabled': self.metrics_enabled,
            'metrics_interval': self.metrics_interval
        }


class AsyncLogger:
    """Asynchronous logger with file and console output."""
    
    def __init__(self, name: str, log_path: str = "logs/", level: str = "INFO"):
        self.name = name
        self.log_path = Path(log_path)
        self.log_path.mkdir(parents=True, exist_ok=True)
        
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.upper()))
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        console_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        console_handler.setFormatter(console_formatter)
        self.logger.addHandler(console_handler)
        
        # File handler
        log_file = self.log_path / f"{name}_{datetime.now().strftime('%Y%m%d')}.log"
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s'
        )
        file_handler.setFormatter(file_formatter)
        self.logger.addHandler(file_handler)
        
        self._lock = asyncio.Lock()
        
    async def debug(self, message: str):
        async with self._lock:
            self.logger.debug(message)
            
    async def info(self, message: str):
        async with self._lock:
            self.logger.info(message)
            
    async def warning(self, message: str):
        async with self._lock:
            self.logger.warning(message)
            
    async def error(self, message: str):
        async with self._lock:
            self.logger.error(message)
            
    async def critical(self, message: str):
        async with self._lock:
            self.logger.critical(message)


class MetricsCollector:
    """Collect and store agent metrics."""
    
    def __init__(self, interval: float = 1.0):
        self.interval = interval
        self._metrics: Dict[str, Dict[str, Any]] = {}
        self._history: Dict[str, List[Dict[str, Any]]] = {}
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()
        
    async def start(self):
        """Start metrics collection."""
        self._running = True
        self._task = asyncio.create_task(self._collect_loop())
        
    async def stop(self):
        """Stop metrics collection."""
        self._running = False
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
                
    async def record(self, agent_id: str, metric_name: str, value: Any):
        """Record a metric value."""
        async with self._lock:
            if agent_id not in self._metrics:
                self._metrics[agent_id] = {}
                self._history[agent_id] = []
            self._metrics[agent_id][metric_name] = {
                'value': value,
                'timestamp': datetime.utcnow().isoformat()
            }
            
    async def get_metrics(self, agent_id: Optional[str] = None) -> Dict[str, Any]:
        """Get current metrics."""
        async with self._lock:
            if agent_id:
                return self._metrics.get(agent_id, {})
            return self._metrics.copy()
            
    async def get_history(self, agent_id: str, metric_name: Optional[str] = None,
                         limit: int = 100) -> List[Dict[str, Any]]:
        """Get metric history."""
        async with self._lock:
            history = self._history.get(agent_id, [])
            if metric_name:
                history = [h for h in history if h.get('name') == metric_name]
            return history[-limit:]
            
    async def _collect_loop(self):
        """Background collection loop."""
        while self._running:
            try:
                await asyncio.sleep(self.interval)
                async with self._lock:
                    for agent_id, metrics in self._metrics.items():
                        self._history[agent_id].append({
                            'timestamp': datetime.utcnow().isoformat(),
                            'metrics': metrics.copy()
                        })
                        # Trim history
                        if len(self._history[agent_id]) > 1000:
                            self._history[agent_id] = self._history[agent_id][-1000:]
            except Exception as e:
                logging.error(f"Metrics collection error: {e}")


# Default configuration instance
default_config = AgentConfig()
