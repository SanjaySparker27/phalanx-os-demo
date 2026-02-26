"""
Planning Agent - Model Predictive Control (MPC) for Autonomous Vehicles
Provides real-time trajectory planning and control with obstacle avoidance.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
import asyncio
import numpy as np
import time

from ...core.base_agent import BaseAgent, AgentMessage, MessagePriority, AgentState
from ...utils.config import PlanningConfig, AsyncLogger


@dataclass
class State:
    """Vehicle state vector."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # x, y, z
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # vx, vy, vz
    orientation: np.ndarray = field(default_factory=lambda: np.array([0, 0, 0, 1]))  # quaternion
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # wx, wy, wz
    timestamp: float = field(default_factory=time.time)
    
    def to_vector(self) -> np.ndarray:
        """Convert to state vector [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]."""
        return np.concatenate([
            self.position,
            self.velocity,
            self.orientation,
            self.angular_velocity
        ])
    
    @classmethod
    def from_vector(cls, vector: np.ndarray) -> "State":
        """Create State from vector."""
        return cls(
            position=vector[0:3],
            velocity=vector[3:6],
            orientation=vector[6:10],
            angular_velocity=vector[10:13]
        )


@dataclass
class Control:
    """Control inputs."""
    thrust: float = 0.0  # N or throttle 0-1
    roll_rate: float = 0.0  # rad/s
    pitch_rate: float = 0.0  # rad/s
    yaw_rate: float = 0.0  # rad/s
    timestamp: float = field(default_factory=time.time)
    
    def to_vector(self) -> np.ndarray:
        """Convert to control vector."""
        return np.array([self.thrust, self.roll_rate, self.pitch_rate, self.yaw_rate])
    
    @classmethod
    def from_vector(cls, vector: np.ndarray) -> "Control":
        """Create Control from vector."""
        return cls(
            thrust=vector[0],
            roll_rate=vector[1],
            pitch_rate=vector[2],
            yaw_rate=vector[3]
        )


@dataclass
class Trajectory:
    """Planned trajectory."""
    states: List[State] = field(default_factory=list)
    controls: List[Control] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)
    cost: float = 0.0
    is_valid: bool = True
    
    @property
    def duration(self) -> float:
        """Get trajectory duration."""
        if len(self.timestamps) < 2:
            return 0.0
        return self.timestamps[-1] - self.timestamps[0]
    
    @property
    def length(self) -> float:
        """Get trajectory length."""
        if len(self.states) < 2:
            return 0.0
        
        length = 0.0
        for i in range(1, len(self.states)):
            diff = self.states[i].position - self.states[i-1].position
            length += np.linalg.norm(diff)
        return length


@dataclass
class Obstacle:
    """Obstacle representation."""
    position: np.ndarray
    radius: float
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    confidence: float = 1.0
    
    def distance_to_point(self, point: np.ndarray) -> float:
        """Get distance from point to obstacle surface."""
        return max(0, np.linalg.norm(point - self.position) - self.radius)


class MPController:
    """Model Predictive Controller implementation."""
    
    def __init__(self, config: PlanningConfig):
        self.config = config
        self.horizon = config.mpc_horizon
        self.dt = 1.0 / config.control_frequency
        
        # Constraints
        self.max_velocity = config.constraints["max_velocity"]
        self.max_acceleration = config.constraints["max_acceleration"]
        self.max_angular_rate = config.constraints["max_angular_rate"]
        self.min_altitude = config.constraints["min_altitude"]
        self.max_altitude = config.constraints["max_altitude"]
        self.safety_margin = config.constraints["safety_margin"]
        
        # Cost weights
        self.Q_pos = config.cost_weights["position_error"]
        self.Q_vel = config.cost_weights["velocity_error"]
        self.R_ctrl = config.cost_weights["control_effort"]
        self.Q_obs = config.cost_weights["obstacle_avoidance"]
        
        # Vehicle model (simplified quadrotor)
        self.mass = 1.5  # kg
        self.gravity = 9.81
        
    def dynamics(self, state: State, control: Control) -> State:
        """Compute state derivatives using vehicle dynamics."""
        # Simplified quadrotor dynamics
        # State: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
        # Control: [thrust, roll_rate, pitch_rate, yaw_rate]
        
        # Convert quaternion to rotation matrix
        q = state.orientation
        R = self._quat_to_rotation_matrix(q)
        
        # Acceleration in world frame
        thrust_body = np.array([0, 0, control.thrust * self.mass * self.gravity])
        thrust_world = R @ thrust_body
        
        acceleration = thrust_world / self.mass - np.array([0, 0, self.gravity])
        
        # Limit acceleration
        acc_norm = np.linalg.norm(acceleration)
        if acc_norm > self.max_acceleration:
            acceleration = acceleration / acc_norm * self.max_acceleration
        
        # Quaternion derivative
        omega = np.array([control.roll_rate, control.pitch_rate, control.yaw_rate])
        q_dot = 0.5 * self._quat_multiply(q, np.concatenate([[0], omega]))
        
        # New state
        new_state = State()
        new_state.position = state.position + state.velocity * self.dt
        new_state.velocity = state.velocity + acceleration * self.dt
        new_state.orientation = state.orientation + q_dot * self.dt
        new_state.orientation = new_state.orientation / np.linalg.norm(new_state.orientation)
        new_state.angular_velocity = omega
        
        return new_state
    
    def _quat_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert quaternion to rotation matrix."""
        w, x, y, z = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
        ])
    
    def _quat_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    def cost_function(self, trajectory: Trajectory, reference: Trajectory,
                      obstacles: List[Obstacle]) -> float:
        """Calculate trajectory cost."""
        cost = 0.0
        
        for i, (state, control) in enumerate(zip(trajectory.states, trajectory.controls)):
            # Position error
            pos_error = state.position - reference.states[i].position
            cost += self.Q_pos * np.dot(pos_error, pos_error)
            
            # Velocity error
            vel_error = state.velocity - reference.states[i].velocity
            cost += self.Q_vel * np.dot(vel_error, vel_error)
            
            # Control effort
            cost += self.R_ctrl * np.dot(control.to_vector(), control.to_vector())
            
            # Obstacle avoidance
            for obs in obstacles:
                dist = obs.distance_to_point(state.position)
                if dist < self.safety_margin:
                    cost += self.Q_obs * (self.safety_margin - dist) ** 2
        
        return cost
    
    def solve(self, current_state: State, reference: Trajectory,
              obstacles: List[Obstacle]) -> Tuple[Trajectory, Control]:
        """Solve MPC optimization."""
        # Initialize trajectory with reference
        trajectory = Trajectory()
        trajectory.states = [current_state] + reference.states[1:self.horizon+1]
        trajectory.controls = [Control() for _ in range(self.horizon)]
        trajectory.timestamps = [time.time() + i * self.dt for i in range(self.horizon)]
        
        # Simple gradient-free optimization (CMA-ES or similar could be used)
        # For now, use iterative refinement
        best_cost = float('inf')
        best_trajectory = trajectory
        
        for _ in range(10):  # Iterations
            # Simulate forward
            for i in range(self.horizon - 1):
                trajectory.states[i+1] = self.dynamics(
                    trajectory.states[i],
                    trajectory.controls[i]
                )
            
            # Calculate cost
            cost = self.cost_function(trajectory, reference, obstacles)
            
            if cost < best_cost:
                best_cost = cost
                best_trajectory = trajectory
            
            # Simple control update (gradient descent)
            for i in range(self.horizon):
                trajectory.controls[i].thrust += np.random.normal(0, 0.01)
                trajectory.controls[i].roll_rate += np.random.normal(0, 0.1)
                trajectory.controls[i].pitch_rate += np.random.normal(0, 0.1)
                trajectory.controls[i].yaw_rate += np.random.normal(0, 0.1)
                
                # Apply limits
                trajectory.controls[i].thrust = np.clip(
                    trajectory.controls[i].thrust, 0.1, 0.9
                )
                trajectory.controls[i].roll_rate = np.clip(
                    trajectory.controls[i].roll_rate, -self.max_angular_rate, self.max_angular_rate
                )
                trajectory.controls[i].pitch_rate = np.clip(
                    trajectory.controls[i].pitch_rate, -self.max_angular_rate, self.max_angular_rate
                )
                trajectory.controls[i].yaw_rate = np.clip(
                    trajectory.controls[i].yaw_rate, -self.max_angular_rate, self.max_angular_rate
                )
        
        best_trajectory.cost = best_cost
        
        # Return first control input
        return best_trajectory, best_trajectory.controls[0]


class PlanningAgent(BaseAgent):
    """Planning Agent for MPC-based trajectory planning."""
    
    def __init__(self, agent_id: str, config: PlanningConfig, message_bus):
        super().__init__(agent_id, "planning", message_bus)
        self.config = config
        self.logger = AsyncLogger(f"planning_{agent_id}")
        
        # MPC
        self.mpc = MPController(config)
        
        # State
        self.current_state: Optional[State] = None
        self.current_trajectory: Optional[Trajectory] = None
        self.obstacles: List[Obstacle] = []
        
        # Planning thread
        self._planning_task: Optional[asyncio.Task] = None
        self._running = False
        
    async def initialize(self) -> bool:
        """Initialize the planning agent."""
        await self.set_state(AgentState.INITIALIZING)
        await self.logger.info("Initializing Planning Agent...")
        
        try:
            # Subscribe to message bus
            await self._message_bus.subscribe(self.agent_id, self._message_handler)
            
            await self.set_state(AgentState.IDLE)
            await self.logger.info("Planning Agent initialized")
            return True
            
        except Exception as e:
            await self.logger.error(f"Initialization error: {e}")
            await self.set_state(AgentState.ERROR)
            return False
    
    async def process(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process incoming messages."""
        msg_type = message.message_type
        payload = message.payload
        
        if msg_type == "PLANNING_REQUEST":
            start = np.array(payload.get("start", [0, 0, 0]))
            goal = np.array(payload.get("goal", [0, 0, 0]))
            obstacles_data = payload.get("obstacles", [])
            
            obstacles = [
                Obstacle(
                    position=np.array(o["position"]),
                    radius=o.get("radius", 1.0),
                    velocity=np.array(o.get("velocity", [0, 0, 0]))
                )
                for o in obstacles_data
            ]
            
            trajectory = await self.plan_path(start, goal, obstacles)
            
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="TRAJECTORY",
                payload={
                    "trajectory": self._trajectory_to_dict(trajectory),
                    "request_id": payload.get("request_id")
                }
            )
        
        elif msg_type == "CONTROL_REQUEST":
            current_state_data = payload.get("current_state")
            reference_data = payload.get("reference")
            
            if current_state_data:
                current_state = State.from_vector(np.array(current_state_data))
                reference = self._dict_to_trajectory(reference_data) if reference_data else None
                
                trajectory, control = await self.compute_control(current_state, reference)
                
                return AgentMessage(
                    source=self.agent_id,
                    target=message.source,
                    message_type="CONTROL_OUTPUT",
                    payload={
                        "control": {
                            "thrust": control.thrust,
                            "roll_rate": control.roll_rate,
                            "pitch_rate": control.pitch_rate,
                            "yaw_rate": control.yaw_rate
                        },
                        "trajectory": self._trajectory_to_dict(trajectory)
                    }
                )
        
        elif msg_type == "STATE_UPDATE":
            state_data = payload.get("state")
            if state_data:
                self.current_state = State.from_vector(np.array(state_data))
        
        elif msg_type == "OBSTACLES":
            obstacles_data = payload.get("obstacles", [])
            self.obstacles = [
                Obstacle(
                    position=np.array(o["position"]),
                    radius=o.get("radius", 1.0),
                    velocity=np.array(o.get("velocity", [0, 0, 0]))
                )
                for o in obstacles_data
            ]
        
        elif msg_type == "UPDATE_CONSTRAINTS":
            new_constraints = payload.get("constraints", {})
            self.mpc.config.constraints.update(new_constraints)
        
        return None
    
    async def plan_path(self, start: np.ndarray, goal: np.ndarray,
                       obstacles: List[Obstacle]) -> Trajectory:
        """Generate collision-free trajectory."""
        # Create initial state
        initial_state = State(position=start)
        
        # Create reference trajectory (straight line with waypoints)
        reference = self._create_reference(start, goal)
        
        # Solve MPC
        trajectory, _ = self.mpc.solve(initial_state, reference, obstacles)
        
        self.current_trajectory = trajectory
        
        # Update metrics
        await self.update_metric("trajectory_cost", trajectory.cost)
        await self.update_metric("trajectory_length", trajectory.length)
        
        return trajectory
    
    def _create_reference(self, start: np.ndarray, goal: np.ndarray) -> Trajectory:
        """Create reference trajectory from start to goal."""
        reference = Trajectory()
        
        # Interpolate waypoints
        num_points = self.config.mpc_horizon + 1
        for i in range(num_points):
            t = i / (num_points - 1)
            position = start + t * (goal - start)
            
            state = State(position=position)
            # Desired velocity direction
            direction = goal - start
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction)
                state.velocity = direction * self.mpc.max_velocity * 0.5
            
            reference.states.append(state)
            reference.timestamps.append(time.time() + i * self.mpc.dt)
        
        reference.controls = [Control() for _ in range(num_points - 1)]
        
        return reference
    
    async def compute_control(self, current_state: State,
                             reference: Optional[Trajectory]) -> Tuple[Trajectory, Control]:
        """Compute optimal control using MPC."""
        if reference is None:
            # Create simple reference
            reference = Trajectory()
            reference.states = [current_state] * (self.config.mpc_horizon + 1)
            reference.controls = [Control()] * self.config.mpc_horizon
        
        trajectory, control = self.mpc.solve(current_state, reference, self.obstacles)
        
        # Update metrics
        await self.update_metric("control_thrust", control.thrust)
        await self.update_metric("control_roll_rate", control.roll_rate)
        await self.update_metric("control_pitch_rate", control.pitch_rate)
        await self.update_metric("control_yaw_rate", control.yaw_rate)
        
        return trajectory, control
    
    async def update_constraints(self, constraints: Dict[str, Any]):
        """Update dynamic constraints."""
        self.mpc.config.constraints.update(constraints)
        await self.logger.info(f"Updated constraints: {constraints}")
    
    async def replan_if_needed(self, threshold: float = None) -> bool:
        """Trigger replanning if deviation exceeds threshold."""
        if self.current_trajectory is None or self.current_state is None:
            return False
        
        threshold = threshold or self.config.replan_threshold
        
        # Check deviation from planned trajectory
        current_pos = self.current_state.position
        closest_dist = float('inf')
        
        for state in self.current_trajectory.states:
            dist = np.linalg.norm(state.position - current_pos)
            closest_dist = min(closest_dist, dist)
        
        if closest_dist > threshold:
            # Trigger replanning
            await self.logger.info(f"Replanning triggered: deviation {closest_dist:.2f}m")
            return True
        
        return False
    
    def _trajectory_to_dict(self, trajectory: Trajectory) -> Dict[str, Any]:
        """Convert trajectory to dictionary."""
        return {
            "states": [
                {
                    "position": s.position.tolist(),
                    "velocity": s.velocity.tolist(),
                    "orientation": s.orientation.tolist(),
                    "timestamp": s.timestamp
                }
                for s in trajectory.states
            ],
            "controls": [
                {
                    "thrust": c.thrust,
                    "roll_rate": c.roll_rate,
                    "pitch_rate": c.pitch_rate,
                    "yaw_rate": c.yaw_rate
                }
                for c in trajectory.controls
            ],
            "cost": trajectory.cost,
            "duration": trajectory.duration,
            "length": trajectory.length,
            "is_valid": trajectory.is_valid
        }
    
    def _dict_to_trajectory(self, data: Dict[str, Any]) -> Trajectory:
        """Convert dictionary to trajectory."""
        trajectory = Trajectory()
        
        for s in data.get("states", []):
            state = State()
            state.position = np.array(s["position"])
            state.velocity = np.array(s["velocity"])
            state.orientation = np.array(s["orientation"])
            state.timestamp = s.get("timestamp", time.time())
            trajectory.states.append(state)
        
        for c in data.get("controls", []):
            control = Control(
                thrust=c["thrust"],
                roll_rate=c["roll_rate"],
                pitch_rate=c["pitch_rate"],
                yaw_rate=c["yaw_rate"]
            )
            trajectory.controls.append(control)
        
        trajectory.cost = data.get("cost", 0.0)
        trajectory.is_valid = data.get("is_valid", True)
        
        return trajectory
    
    async def shutdown(self) -> bool:
        """Shutdown the planning agent."""
        await self.set_state(AgentState.SHUTTING_DOWN)
        await self.logger.info("Shutting down Planning Agent...")
        
        self._running = False
        
        if self._planning_task:
            self._planning_task.cancel()
            try:
                await self._planning_task
            except asyncio.CancelledError:
                pass
        
        await self.set_state(AgentState.SHUTDOWN)
        await self.logger.info("Planning Agent shutdown complete")
        return True
