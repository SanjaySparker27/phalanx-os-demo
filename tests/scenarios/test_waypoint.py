#!/usr/bin/env python3
"""
Waypoint Navigation Test Scenario
Validates autonomous waypoint following for UAV/USV/UGV.
"""

import unittest
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple, Literal
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix


VehicleType = Literal['uav', 'usv', 'ugv']


@dataclass
class Waypoint:
    """Navigation waypoint."""
    x: float
    y: float
    z: float = 0.0
    heading: float = 0.0
    tolerance: float = 1.0
    loiter_time: float = 0.0


@dataclass
class WaypointMetrics:
    """Metrics for waypoint navigation."""
    total_time: float
    total_distance: float
    avg_speed: float
    max_cross_track_error: float
    waypoints_reached: int
    success_rate: float
    path_efficiency: float


class WaypointNavigator(Node):
    """ROS2 node for waypoint navigation testing."""
    
    def __init__(self, vehicle_type: VehicleType = 'uav'):
        super().__init__(f'waypoint_nav_{vehicle_type}')
        
        self.vehicle_type = vehicle_type
        
        # Control parameters
        self.position_gain = 1.0
        self.velocity_limit = 5.0 if vehicle_type == 'uav' else 2.0
        self.heading_gain = 0.5
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{vehicle_type}/pose', self.pose_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, f'/{vehicle_type}/gps/fix', self.gps_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{vehicle_type}/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, f'/{vehicle_type}/planned_path', 10)
        
        # State
        self.current_pose: Pose = None
        self.current_gps: NavSatFix = None
        self.position_history: List[Tuple[float, float, float]] = []
        
    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
        self.position_history.append((
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ))
    
    def gps_callback(self, msg: NavSatFix):
        self.current_gps = msg
    
    def distance_to_waypoint(self, waypoint: Waypoint) -> float:
        """Calculate distance to waypoint."""
        if self.current_pose is None:
            return float('inf')
        
        dx = waypoint.x - self.current_pose.position.x
        dy = waypoint.y - self.current_pose.position.y
        dz = waypoint.z - self.current_pose.position.z
        return np.sqrt(dx**2 + dy**2 + dz**2)
    
    def calculate_cross_track_error(self, waypoint: Waypoint, prev_waypoint: Waypoint) -> float:
        """Calculate perpendicular distance from path line."""
        if self.current_pose is None:
            return 0.0
        
        # Vector from prev to current waypoint
        path_vec = np.array([waypoint.x - prev_waypoint.x, waypoint.y - prev_waypoint.y])
        path_len = np.linalg.norm(path_vec)
        
        if path_len < 0.001:
            return 0.0
        
        # Unit vector along path
        path_unit = path_vec / path_len
        
        # Vector from prev waypoint to current position
        pos_vec = np.array([
            self.current_pose.position.x - prev_waypoint.x,
            self.current_pose.position.y - prev_waypoint.y
        ])
        
        # Project onto path
        projection = np.dot(pos_vec, path_unit)
        projection = np.clip(projection, 0, path_len)
        
        # Closest point on path
        closest = np.array([prev_waypoint.x, prev_waypoint.y]) + projection * path_unit
        
        # Cross-track error
        error = np.linalg.norm([
            self.current_pose.position.x - closest[0],
            self.current_pose.position.y - closest[1]
        ])
        
        return error
    
    def navigate_to_waypoint(self, waypoint: Waypoint) -> bool:
        """Navigate to a single waypoint. Returns True if reached."""
        start_time = time.time()
        max_time = 60.0  # Maximum time per waypoint
        
        while time.time() - start_time < max_time:
            if self.current_pose is None:
                time.sleep(0.01)
                continue
            
            distance = self.distance_to_waypoint(waypoint)
            
            # Check if reached
            if distance < waypoint.tolerance:
                # Loiter if specified
                if waypoint.loiter_time > 0:
                    self.get_logger().info(f'Loitering for {waypoint.loiter_time}s')
                    time.sleep(waypoint.loiter_time)
                return True
            
            # Calculate velocity command
            dx = waypoint.x - self.current_pose.position.x
            dy = waypoint.y - self.current_pose.position.y
            dz = waypoint.z - self.current_pose.position.z
            
            # Normalize and scale
            dist_xy = np.sqrt(dx**2 + dy**2)
            if dist_xy > 0.001:
                cmd = Twist()
                cmd.linear.x = min(self.velocity_limit, self.position_gain * dist_xy) * (dx / dist_xy)
                cmd.linear.y = min(self.velocity_limit, self.position_gain * dist_xy) * (dy / dist_xy)
                cmd.linear.z = self.position_gain * dz
                
                # Heading control
                target_heading = np.arctan2(dy, dx)
                cmd.angular.z = self.heading_gain * (target_heading - waypoint.heading)
                
                self.cmd_pub.publish(cmd)
            
            time.sleep(0.02)
        
        return False
    
    def execute_mission(self, waypoints: List[Waypoint]) -> WaypointMetrics:
        """Execute full waypoint mission."""
        self.get_logger().info(f'Starting waypoint mission with {len(waypoints)} waypoints')
        
        start_time = time.time()
        start_pos = (self.current_pose.position.x, self.current_pose.position.y) if self.current_pose else (0, 0)
        
        waypoints_reached = 0
        max_cross_track = 0.0
        total_distance = 0.0
        prev_waypoint = None
        
        for i, waypoint in enumerate(waypoints):
            self.get_logger().info(f'Navigating to waypoint {i+1}/{len(waypoints)}')
            
            reached = self.navigate_to_waypoint(waypoint)
            
            if reached:
                waypoints_reached += 1
                self.get_logger().info(f'Waypoint {i+1} reached')
            else:
                self.get_logger().warn(f'Failed to reach waypoint {i+1}')
            
            # Track cross-track error
            if prev_waypoint is not None:
                cte = self.calculate_cross_track_error(waypoint, prev_waypoint)
                max_cross_track = max(max_cross_track, cte)
                
                # Calculate path distance
                total_distance += np.sqrt(
                    (waypoint.x - prev_waypoint.x)**2 +
                    (waypoint.y - prev_waypoint.y)**2
                )
            
            prev_waypoint = waypoint
        
        total_time = time.time() - start_time
        
        # Calculate metrics
        if len(self.position_history) > 1:
            actual_distance = sum(
                np.sqrt(
                    (self.position_history[i][0] - self.position_history[i-1][0])**2 +
                    (self.position_history[i][1] - self.position_history[i-1][1])**2
                )
                for i in range(1, len(self.position_history))
            )
        else:
            actual_distance = total_distance
        
        path_efficiency = total_distance / actual_distance if actual_distance > 0 else 1.0
        avg_speed = actual_distance / total_time if total_time > 0 else 0.0
        
        return WaypointMetrics(
            total_time=total_time,
            total_distance=actual_distance,
            avg_speed=avg_speed,
            max_cross_track_error=max_cross_track,
            waypoints_reached=waypoints_reached,
            success_rate=waypoints_reached / len(waypoints) if waypoints else 0.0,
            path_efficiency=path_efficiency
        )


class TestWaypointNavigation(unittest.TestCase):
    """Unit tests for waypoint navigation."""
    
    def setUp(self):
        rclpy.init()
        self.node = WaypointNavigator('uav')
        self.waypoints = [
            Waypoint(10, 0, 10, tolerance=1.0),
            Waypoint(10, 10, 10, tolerance=1.0),
            Waypoint(0, 10, 10, tolerance=1.0),
            Waypoint(0, 0, 10, tolerance=1.0),
        ]
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_waypoint_mission(self):
        """Test complete waypoint mission."""
        metrics = self.node.execute_mission(self.waypoints)
        self.assertEqual(
            metrics.waypoints_reached, len(self.waypoints),
            f"Only reached {metrics.waypoints_reached}/{len(self.waypoints)} waypoints"
        )
    
    def test_path_efficiency(self):
        """Test path following efficiency."""
        metrics = self.node.execute_mission(self.waypoints)
        self.assertGreater(
            metrics.path_efficiency, 0.8,
            f"Path efficiency too low: {metrics.path_efficiency:.2f}"
        )


def run_waypoint_test(vehicle_type: VehicleType = 'uav'):
    """Run waypoint navigation test."""
    rclpy.init()
    node = WaypointNavigator(vehicle_type)
    
    # Define mission waypoints
    waypoints = {
        'uav': [
            Waypoint(10, 0, 10, tolerance=1.0),
            Waypoint(10, 10, 10, tolerance=1.0),
            Waypoint(0, 10, 10, tolerance=1.0),
            Waypoint(0, 0, 10, tolerance=1.0),
        ],
        'usv': [
            Waypoint(20, 0, 0, tolerance=2.0),
            Waypoint(20, 20, 0, tolerance=2.0),
            Waypoint(0, 20, 0, tolerance=2.0),
            Waypoint(0, 0, 0, tolerance=2.0),
        ],
        'ugv': [
            Waypoint(5, 0, 0, tolerance=0.5),
            Waypoint(5, 5, 0, tolerance=0.5),
            Waypoint(0, 5, 0, tolerance=0.5),
            Waypoint(0, 0, 0, tolerance=0.5),
        ],
    }
    
    try:
        metrics = node.execute_mission(waypoints[vehicle_type])
        
        print("\n" + "="*50)
        print(f"WAYPOINT NAVIGATION TEST - {vehicle_type.upper()}")
        print("="*50)
        print(f"Waypoints reached: {metrics.waypoints_reached}/{len(waypoints[vehicle_type])}")
        print(f"Success rate: {metrics.success_rate*100:.1f}%")
        print(f"Total time: {metrics.total_time:.2f}s")
        print(f"Total distance: {metrics.total_distance:.2f}m")
        print(f"Average speed: {metrics.avg_speed:.2f} m/s")
        print(f"Max cross-track error: {metrics.max_cross_track_error:.2f}m")
        print(f"Path efficiency: {metrics.path_efficiency:.2%}")
        print("="*50)
        
        return metrics
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_waypoint_test('uav')
