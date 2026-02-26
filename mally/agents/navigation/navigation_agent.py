"""
Navigation Agent - ORB-SLAM3 Integration for GPS-Denied Navigation
Provides visual odometry, SLAM, and relocalization capabilities.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
import asyncio
import numpy as np
import time
from pathlib import Path

from ...core.base_agent import BaseAgent, AgentMessage, MessagePriority, AgentState
from ...utils.config import NavigationConfig, AsyncLogger


@dataclass
class Pose:
    """3D Pose with covariance."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z
    orientation: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 1]))  # quaternion x,y,z,w
    covariance: np.ndarray = field(default_factory=lambda: np.eye(6) * 0.1)
    timestamp: float = field(default_factory=time.time)
    frame_id: str = ""
    
    @property
    def rotation_matrix(self) -> np.ndarray:
        """Convert quaternion to rotation matrix."""
        x, y, z, w = self.orientation
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
        ])
    
    @property
    def transformation_matrix(self) -> np.ndarray:
        """Get 4x4 transformation matrix (T_wc)."""
        T = np.eye(4)
        T[:3, :3] = self.rotation_matrix
        T[:3, 3] = self.position
        return T
    
    def transform_point(self, point: np.ndarray) -> np.ndarray:
        """Transform point from camera to world frame."""
        p_homo = np.append(point, 1)
        return (self.transformation_matrix @ p_homo)[:3]


@dataclass
class MapPoint:
    """3D map point from SLAM."""
    id: int
    position: np.ndarray
    observations: int = 0
    descriptor: Optional[np.ndarray] = None
    
    
@dataclass
class KeyFrame:
    """SLAM keyframe."""
    id: int
    pose: Pose
    timestamp: float
    is_looped: bool = False


class ORBSLAM3Interface:
    """Interface to ORB-SLAM3 system."""
    
    def __init__(self, vocab_path: str, settings_path: str, 
                 sensor_type: str = "MONOCULAR"):
        self.vocab_path = vocab_path
        self.settings_path = settings_path
        self.sensor_type = sensor_type
        self.slam_system = None
        self.current_pose: Optional[Pose] = None
        self.trajectory: List[Pose] = []
        self.keyframes: List[KeyFrame] = []
        self.map_points: List[MapPoint] = []
        
    def initialize(self) -> bool:
        """Initialize ORB-SLAM3 system."""
        try:
            # Import ORB-SLAM3 Python bindings
            # Note: This requires ORB-SLAM3 Python wrapper to be installed
            import orbslam3
            
            self.slam_system = orbslam3.System(
                self.vocab_path,
                self.settings_path,
                orbslam3.Sensor.MONOCULAR
            )
            
            return True
        except ImportError:
            # Fallback to mock implementation for development
            return self._initialize_mock()
        except Exception as e:
            print(f"ORB-SLAM3 initialization error: {e}")
            return False
    
    def _initialize_mock(self) -> bool:
        """Mock initialization for development."""
        self.current_pose = Pose(
            position=np.zeros(3),
            orientation=np.array([0, 0, 0, 1])
        )
        return True
    
    def process_frame(self, image: np.ndarray, timestamp: float) -> Optional[Pose]:
        """Process a new frame."""
        if self.slam_system is None:
            return self._process_frame_mock(image, timestamp)
        
        try:
            pose_matrix = self.slam_system.process_frame(image, timestamp)
            
            if pose_matrix is not None:
                # Extract pose from 4x4 matrix
                self.current_pose = self._matrix_to_pose(pose_matrix, timestamp)
                self.trajectory.append(self.current_pose)
                
                return self.current_pose
            
            return None
            
        except Exception as e:
            print(f"Frame processing error: {e}")
            return None
    
    def _process_frame_mock(self, image: np.ndarray, timestamp: float) -> Optional[Pose]:
        """Mock frame processing."""
        # Simulate visual odometry drift
        drift = np.array([
            np.sin(timestamp) * 0.1,
            np.cos(timestamp) * 0.1,
            0.0
        ])
        
        self.current_pose = Pose(
            position=self.current_pose.position + drift if self.current_pose else drift,
            orientation=np.array([0, 0, 0, 1]),
            timestamp=timestamp
        )
        
        self.trajectory.append(self.current_pose)
        return self.current_pose
    
    def _matrix_to_pose(self, T: np.ndarray, timestamp: float) -> Pose:
        """Convert 4x4 matrix to Pose."""
        position = T[:3, 3]
        
        # Convert rotation matrix to quaternion
        R = T[:3, :3]
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            # Handle other cases
            w, x, y, z = 1, 0, 0, 0
        
        return Pose(
            position=position,
            orientation=np.array([x, y, z, w]),
            timestamp=timestamp
        )
    
    def save_map(self, path: str) -> bool:
        """Save current map to file."""
        if self.slam_system:
            self.slam_system.save_map(path)
            return True
        return False
    
    def load_map(self, path: str) -> bool:
        """Load map from file."""
        if self.slam_system:
            self.slam_system.load_map(path)
            return True
        return False
    
    def get_map_points(self) -> List[MapPoint]:
        """Get all map points."""
        if self.slam_system:
            points = self.slam_system.get_map_points()
            return [
                MapPoint(
                    id=p.id,
                    position=np.array(p.position),
                    observations=p.observations
                )
                for p in points
            ]
        return []
    
    def shutdown(self):
        """Shutdown SLAM system."""
        if self.slam_system:
            self.slam_system.shutdown()


class NavigationAgent(BaseAgent):
    """Navigation Agent for GPS-denied navigation using ORB-SLAM3."""
    
    def __init__(self, agent_id: str, config: NavigationConfig, message_bus):
        super().__init__(agent_id, "navigation", message_bus)
        self.config = config
        self.logger = AsyncLogger(f"navigation_{agent_id}")
        
        # SLAM components
        self.slam: Optional[ORBSLAM3Interface] = None
        self.vocab_path = "config/ORBvoc.txt"
        self.settings_path = config.slam_config_path
        
        # State
        self.current_pose: Optional[Pose] = None
        self.trajectory: List[Pose] = []
        self.is_localized = False
        self.localization_quality = 0.0
        
        # Map management
        self.map_save_path = Path(config.map_save_path)
        self.map_save_path.mkdir(parents=True, exist_ok=True)
        
        # Loop closure
        self.loop_closures: List[Tuple[int, int]] = []
        
        # Processing
        self._processing_task: Optional[asyncio.Task] = None
        self._running = False
        
    async def initialize(self) -> bool:
        """Initialize the navigation agent."""
        await self.set_state(AgentState.INITIALIZING)
        await self.logger.info("Initializing Navigation Agent...")
        
        try:
            # Initialize SLAM
            self.slam = ORBSLAM3Interface(
                self.vocab_path,
                self.settings_path
            )
            
            if not self.slam.initialize():
                await self.logger.error("Failed to initialize SLAM system")
                await self.set_state(AgentState.ERROR)
                return False
            
            await self.logger.info("ORB-SLAM3 initialized successfully")
            
            # Subscribe to message bus
            await self._message_bus.subscribe(self.agent_id, self._message_handler)
            
            await self.set_state(AgentState.IDLE)
            return True
            
        except Exception as e:
            await self.logger.error(f"Initialization error: {e}")
            await self.set_state(AgentState.ERROR)
            return False
    
    async def process(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process incoming messages."""
        msg_type = message.message_type
        payload = message.payload
        
        if msg_type == "LOCALIZATION_REQUEST":
            pose = await self.get_current_pose()
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="LOCALIZATION_RESPONSE",
                payload={
                    "pose": self._pose_to_dict(pose) if pose else None,
                    "is_localized": self.is_localized,
                    "quality": self.localization_quality
                }
            )
        
        elif msg_type == "TRAJECTORY_REQUEST":
            trajectory = await self.get_trajectory()
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="TRAJECTORY_RESPONSE",
                payload={
                    "trajectory": [self._pose_to_dict(p) for p in trajectory]
                }
            )
        
        elif msg_type == "LOAD_MAP":
            map_path = payload.get("map_path")
            success = await self.set_relocalization_map(map_path)
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="MAP_LOADED" if success else "MAP_LOAD_FAILED",
                payload={"map_path": map_path, "success": success}
            )
        
        elif msg_type == "SAVE_MAP":
            map_name = payload.get("map_name", f"map_{int(time.time())}")
            map_path = self.map_save_path / f"{map_name}.map"
            success = self.slam.save_map(str(map_path)) if self.slam else False
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="MAP_SAVED" if success else "MAP_SAVE_FAILED",
                payload={"map_path": str(map_path), "success": success}
            )
        
        elif msg_type == "RESET_SLAM":
            success = await self.reset_slam()
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="SLAM_RESET",
                payload={"success": success}
            )
        
        elif msg_type == "FRAME":
            # Process incoming frame for SLAM
            frame_data = payload.get("frame_data")
            timestamp = payload.get("timestamp", time.time())
            if frame_data is not None:
                pose = await self._process_frame(frame_data, timestamp)
                if pose:
                    # Broadcast pose update
                    await self.broadcast(
                        payload={
                            "type": "pose_update",
                            "pose": self._pose_to_dict(pose),
                            "quality": self.localization_quality
                        },
                        message_type="NAVIGATION_STATE",
                        priority=MessagePriority.NORMAL
                    )
        
        elif msg_type == "GET_MAP_POINTS":
            map_points = self.slam.get_map_points() if self.slam else []
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="MAP_POINTS",
                payload={
                    "points": [
                        {
                            "id": p.id,
                            "position": p.position.tolist(),
                            "observations": p.observations
                        }
                        for p in map_points
                    ]
                }
            )
        
        return None
    
    async def get_current_pose(self) -> Optional[Pose]:
        """Get current camera pose in world frame."""
        return self.current_pose
    
    async def get_trajectory(self) -> List[Pose]:
        """Get estimated trajectory."""
        return self.trajectory.copy()
    
    async def set_relocalization_map(self, map_path: str) -> bool:
        """Load existing map for relocalization."""
        if self.slam:
            return self.slam.load_map(map_path)
        return False
    
    async def reset_slam(self) -> bool:
        """Reset SLAM system."""
        self.trajectory = []
        self.current_pose = None
        self.is_localized = False
        
        if self.slam:
            self.slam.shutdown()
            return self.slam.initialize()
        
        return True
    
    async def _process_frame(self, frame_data: np.ndarray, timestamp: float) -> Optional[Pose]:
        """Process a frame through SLAM."""
        if self.slam is None:
            return None
        
        pose = self.slam.process_frame(frame_data, timestamp)
        
        if pose:
            self.current_pose = pose
            self.is_localized = True
            
            # Calculate localization quality
            self.localization_quality = self._calculate_quality(pose)
            
            # Update metrics
            await self.update_metric("pose_x", pose.position[0])
            await self.update_metric("pose_y", pose.position[1])
            await self.update_metric("pose_z", pose.position[2])
            await self.update_metric("localization_quality", self.localization_quality)
        
        return pose
    
    def _calculate_quality(self, pose: Pose) -> float:
        """Calculate localization quality score."""
        # Based on covariance trace
        cov_trace = np.trace(pose.covariance[:3, :3])
        quality = max(0.0, 1.0 - cov_trace / 10.0)
        return min(1.0, quality)
    
    def _pose_to_dict(self, pose: Pose) -> Dict[str, Any]:
        """Convert pose to dictionary."""
        return {
            "position": pose.position.tolist(),
            "orientation": pose.orientation.tolist(),
            "covariance": pose.covariance.tolist(),
            "timestamp": pose.timestamp,
            "frame_id": pose.frame_id
        }
    
    async def shutdown(self) -> bool:
        """Shutdown the navigation agent."""
        await self.set_state(AgentState.SHUTTING_DOWN)
        await self.logger.info("Shutting down Navigation Agent...")
        
        self._running = False
        
        if self._processing_task:
            self._processing_task.cancel()
            try:
                await self._processing_task
            except asyncio.CancelledError:
                pass
        
        # Save trajectory
        if self.trajectory:
            traj_path = self.map_save_path / f"trajectory_{int(time.time())}.npy"
            np.save(traj_path, [self._pose_to_dict(p) for p in self.trajectory])
            await self.logger.info(f"Trajectory saved to {traj_path}")
        
        if self.slam:
            self.slam.shutdown()
            self.slam = None
        
        await self.set_state(AgentState.SHUTDOWN)
        await self.logger.info("Navigation Agent shutdown complete")
        return True
