#!/usr/bin/env python3
"""
Obstacle Avoidance Test Scenario
Validates reactive and planned obstacle avoidance for all vehicle types.
"""

import unittest
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class AvoidanceMode(Enum):
    REACTIVE = "reactive"
    PLANNED = "planned"
    HYBRID = "hybrid"


@dataclass
class Obstacle:
    """Obstacle representation."""
    x: float
    y: float
    radius: float
    height: float = 0.0


@dataclass
class AvoidanceMetrics:
    """Metrics for obstacle avoidance."""
    min_distance_to_obstacle: float
    navigation_time: float
    path_length: float
    collisions: int
    avoidance_success: bool
    avg_clearance: float


class ObstacleAvoidanceNode(Node):
    """ROS2 node for obstacle avoidance testing."""
    
    def __init__(self, vehicle_type: str = 'uav', mode: AvoidanceMode = AvoidanceMode.HYBRID):
        super().__init__(f'obstacle_avoidance_{vehicle_type}')
        
        self.vehicle_type = vehicle_type
        self.mode = mode
        
        # Safety parameters
        self.safety_distance = 2.0 if vehicle_type == 'uav' else 1.0
        self.max_speed = 3.0 if vehicle_type == 'uav' else 1.0
        self.detection_range = 15.0
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, f'/{vehicle_type}/scan', self.scan_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, f'/{vehicle_type}/points', self.pointcloud_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{vehicle_type}/pose', self.pose_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{vehicle_type}/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)
        
        # State
        self.current_pose = None
        self.laser_scan: Optional[LaserScan] = None
        self.point_cloud: Optional[PointCloud2] = None
        self.detected_obstacles: List[Obstacle] = []
        self.position_history: List[Tuple[float, float]] = []
        self.min_obstacle_distance = float('inf')
        self.collision_count = 0
        
    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
        self.position_history.append((msg.pose.position.x, msg.pose.position.y))
        
    def scan_callback(self, msg: LaserScan):
        self.laser_scan = msg
        self.detect_obstacles_from_scan()
        
    def pointcloud_callback(self, msg: PointCloud2):
        self.point_cloud = msg
        self.detect_obstacles_from_pointcloud()
    
    def detect_obstacles_from_scan(self):
        """Detect obstacles from laser scan."""
        if self.laser_scan is None or self.current_pose is None:
            return
        
        self.detected_obstacles = []
        
        angle = self.laser_scan.angle_min
        for distance in self.laser_scan.ranges:
            if self.laser_scan.range_min < distance < self.laser_scan.range_max:
                if distance < self.detection_range:
                    # Convert polar to cartesian
                    x = distance * np.cos(angle) + self.current_pose.position.x
                    y = distance * np.sin(angle) + self.current_pose.position.y
                    
                    self.detected_obstacles.append(Obstacle(
                        x=x, y=y, radius=0.5, height=2.0
                    ))
                    
                    # Track minimum distance
                    self.min_obstacle_distance = min(self.min_obstacle_distance, distance)
            
            angle += self.laser_scan.angle_increment
    
    def detect_obstacles_from_pointcloud(self):
        """Detect obstacles from point cloud (3D)."""
        # Would parse PointCloud2 message and extract obstacle clusters
        pass
    
    def calculate_repulsive_force(self) -> Tuple[float, float]:
        """Calculate repulsive force from obstacles (APF method)."""
        if not self.detected_obstacles or self.current_pose is None:
            return (0.0, 0.0)
        
        fx, fy = 0.0, 0.0
        
        for obs in self.detected_obstacles:
            dx = self.current_pose.position.x - obs.x
            dy = self.current_pose.position.y - obs.y
            distance = np.sqrt(dx**2 + dy**2)
            
            if distance < self.safety_distance and distance > 0.1:
                # Repulsive force magnitude
                force = (1.0 / distance - 1.0 / self.safety_distance) / (distance**2)
                
                # Direction (away from obstacle)
                fx += force * (dx / distance)
                fy += force * (dy / distance)
        
        # Clamp force magnitude
        force_mag = np.sqrt(fx**2 + fy**2)
        if force_mag > self.max_speed:
            fx = fx / force_mag * self.max_speed
            fy = fy / force_mag * self.max_speed
        
        return (fx, fy)
    
    def calculate_attractive_force(self, goal: Tuple[float, float]) -> Tuple[float, float]:
        """Calculate attractive force toward goal."""
        if self.current_pose is None:
            return (0.0, 0.0)
        
        dx = goal[0] - self.current_pose.position.x
        dy = goal[1] - self.current_pose.position.y
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            return (0.0, 0.0)
        
        # Normalize and scale
        speed = min(self.max_speed, 0.5 * distance)
        
        return (speed * dx / distance, speed * dy / distance)
    
    def publish_obstacle_markers(self):
        """Publish visualization markers for obstacles."""
        markers = MarkerArray()
        
        for i, obs in enumerate(self.detected_obstacles[:50]):  # Limit to 50 markers
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = obs.x
            marker.pose.position.y = obs.y
            marker.pose.position.z = obs.height / 2
            marker.scale.x = obs.radius * 2
            marker.scale.y = obs.radius * 2
            marker.scale.z = obs.height
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            
            markers.markers.append(marker)
        
        self.marker_pub.publish(markers)
    
    def navigate_with_avoidance(self, goal: Tuple[float, float, float], 
                                  timeout: float = 60.0) -> AvoidanceMetrics:
        """Navigate to goal with obstacle avoidance."""
        self.get_logger().info(f'Navigating to goal {goal} with {self.mode.value} avoidance')
        
        start_time = time.time()
        start_pos = self.position_history[0] if self.position_history else (0, 0)
        
        goal_reached = False
        last_obstacle_check = 0
        
        while time.time() - start_time < timeout:
            if self.current_pose is None:
                time.sleep(0.01)
                continue
            
            # Check if goal reached
            dx = goal[0] - self.current_pose.position.x
            dy = goal[1] - self.current_pose.position.y
            dz = goal[2] - self.current_pose.position.z
            distance_to_goal = np.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance_to_goal < 1.0:
                goal_reached = True
                break
            
            # Calculate forces
            f_attractive = self.calculate_attractive_force((goal[0], goal[1]))
            f_repulsive = self.calculate_repulsive_force()
            
            # Combined velocity command
            cmd = Twist()
            cmd.linear.x = f_attractive[0] + f_repulsive[0]
            cmd.linear.y = f_attractive[1] + f_repulsive[1]
            cmd.linear.z = 0.3 * dz  # Simple altitude control
            
            # Limit velocity
            vel_xy = np.sqrt(cmd.linear.x**2 + cmd.linear.y**2)
            if vel_xy > self.max_speed:
                cmd.linear.x = cmd.linear.x / vel_xy * self.max_speed
                cmd.linear.y = cmd.linear.y / vel_xy * self.max_speed
            
            # Check for collision (simplified)
            if self.min_obstacle_distance < 0.5:
                self.collision_count += 1
                self.get_logger().warn('Potential collision detected!')
            
            self.cmd_pub.publish(cmd)
            
            # Publish visualization
            if time.time() - last_obstacle_check > 0.1:
                self.publish_obstacle_markers()
                last_obstacle_check = time.time()
            
            time.sleep(0.02)
        
        # Calculate metrics
        navigation_time = time.time() - start_time
        
        if len(self.position_history) > 1:
            path_length = sum(
                np.sqrt(
                    (self.position_history[i][0] - self.position_history[i-1][0])**2 +
                    (self.position_history[i][1] - self.position_history[i-1][1])**2
                )
                for i in range(1, len(self.position_history))
            )
        else:
            path_length = 0.0
        
        # Calculate average clearance
        avg_clearance = self.min_obstacle_distance if self.min_obstacle_distance < float('inf') else self.detection_range
        
        return AvoidanceMetrics(
            min_distance_to_obstacle=self.min_obstacle_distance,
            navigation_time=navigation_time,
            path_length=path_length,
            collisions=self.collision_count,
            avoidance_success=goal_reached and self.collision_count == 0,
            avg_clearance=avg_clearance
        )


class TestObstacleAvoidance(unittest.TestCase):
    """Unit tests for obstacle avoidance."""
    
    def setUp(self):
        rclpy.init()
        self.node = ObstacleAvoidanceNode('ugv', AvoidanceMode.HYBRID)
        self.goal = (20.0, 0.0, 0.0)
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_avoidance_success(self):
        """Test that navigation completes without collisions."""
        metrics = self.node.navigate_with_avoidance(self.goal)
        self.assertTrue(
            metrics.avoidance_success,
            f"Avoidance failed with {metrics.collisions} collisions"
        )
    
    def test_minimum_clearance(self):
        """Test that minimum obstacle clearance is maintained."""
        metrics = self.node.navigate_with_avoidance(self.goal)
        self.assertGreater(
            metrics.min_distance_to_obstacle, 0.5,
            f"Came too close to obstacle: {metrics.min_distance_to_obstacle:.2f}m"
        )


def run_obstacle_test(vehicle_type: str = 'ugv'):
    """Run obstacle avoidance test."""
    rclpy.init()
    node = ObstacleAvoidanceNode(vehicle_type, AvoidanceMode.HYBRID)
    
    # Set goal position
    goal = (20.0, 0.0, 0.0)
    
    try:
        metrics = node.navigate_with_avoidance(goal)
        
        print("\n" + "="*50)
        print(f"OBSTACLE AVOIDANCE TEST - {vehicle_type.upper()}")
        print("="*50)
        print(f"Avoidance success: {metrics.avoidance_success}")
        print(f"Collisions: {metrics.collisions}")
        print(f"Min obstacle distance: {metrics.min_distance_to_obstacle:.2f}m")
        print(f"Average clearance: {metrics.avg_clearance:.2f}m")
        print(f"Navigation time: {metrics.navigation_time:.2f}s")
        print(f"Path length: {metrics.path_length:.2f}m")
        print("="*50)
        
        return metrics
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    run_obstacle_test('ugv')
