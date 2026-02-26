#!/usr/bin/env python3
"""
UAV Takeoff Test Scenario
Validates vertical takeoff performance and stability.
"""

import unittest
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


@dataclass
class TakeoffMetrics:
    """Metrics for takeoff performance."""
    time_to_altitude: float
    max_overshoot: float
    settling_time: float
    stability_score: float
    success: bool


class TakeoffTestNode(Node):
    """ROS2 node for takeoff testing."""
    
    def __init__(self):
        super().__init__('takeoff_test')
        
        # Parameters
        self.target_altitude = 10.0  # meters
        self.tolerance = 0.5  # meters
        self.timeout = 30.0  # seconds
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/uav/pose', self.pose_callback, 10)
        self.altitude_sub = self.create_subscription(
            Float64, '/uav/altitude', self.altitude_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)
        
        # State
        self.current_altitude = 0.0
        self.current_pose = None
        self.altitude_history: List[Tuple[float, float]] = []
        self.start_time = None
        self.takeoff_complete = False
        
    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg
        
    def altitude_callback(self, msg: Float64):
        self.current_altitude = msg.data
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.altitude_history.append((current_time, msg.data))
        
    def execute_takeoff(self) -> TakeoffMetrics:
        """Execute takeoff maneuver and return metrics."""
        self.get_logger().info('Starting takeoff test...')
        self.start_time = time.time()
        
        # Command vertical velocity
        cmd = Twist()
        cmd.linear.z = 2.0  # m/s ascent rate
        
        start_altitude = self.current_altitude
        reached_target = False
        target_reach_time = None
        max_altitude = start_altitude
        
        while time.time() - self.start_time < self.timeout:
            # Update command based on current altitude
            error = self.target_altitude - self.current_altitude
            
            if error > 2.0:
                cmd.linear.z = 2.0
            elif error > 0.5:
                cmd.linear.z = 1.0
            else:
                cmd.linear.z = 0.3
            
            if self.current_altitude >= self.target_altitude and not reached_target:
                reached_target = True
                target_reach_time = time.time() - self.start_time
                self.get_logger().info(f'Target reached in {target_reach_time:.2f}s')
            
            max_altitude = max(max_altitude, self.current_altitude)
            
            # Check settling
            if reached_target:
                if abs(self.current_altitude - self.target_altitude) < self.tolerance:
                    settling_time = time.time() - self.start_time
                    overshoot = max_altitude - self.target_altitude
                    
                    # Calculate stability score
                    recent_altitudes = [a for t, a in self.altitude_history[-50:]]
                    if len(recent_altitudes) > 1:
                        std_dev = np.std(recent_altitudes)
                        stability_score = max(0, 100 - std_dev * 100)
                    else:
                        stability_score = 0
                    
                    self.takeoff_complete = True
                    return TakeoffMetrics(
                        time_to_altitude=target_reach_time,
                        max_overshoot=overshoot,
                        settling_time=settling_time,
                        stability_score=stability_score,
                        success=True
                    )
            
            self.cmd_pub.publish(cmd)
            time.sleep(0.01)
        
        # Timeout
        return TakeoffMetrics(
            time_to_altitude=time.time() - self.start_time,
            max_overshoot=max_altitude - self.target_altitude,
            settling_time=self.timeout,
            stability_score=0.0,
            success=False
        )


class TestTakeoff(unittest.TestCase):
    """Unit tests for takeoff scenario."""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TakeoffTestNode()
        
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
    
    def test_takeoff_success(self):
        """Test that takeoff completes successfully."""
        metrics = self.node.execute_takeoff()
        self.assertTrue(metrics.success, "Takeoff failed to complete")
    
    def test_takeoff_time(self):
        """Test that takeoff completes within acceptable time."""
        metrics = self.node.execute_takeoff()
        self.assertLess(
            metrics.time_to_altitude, 15.0,
            f"Takeoff took too long: {metrics.time_to_altitude:.2f}s"
        )
    
    def test_takeoff_overshoot(self):
        """Test that overshoot is within limits."""
        metrics = self.node.execute_takeoff()
        self.assertLess(
            metrics.max_overshoot, 2.0,
            f"Overshoot too large: {metrics.max_overshoot:.2f}m"
        )
    
    def test_takeoff_stability(self):
        """Test hover stability after takeoff."""
        metrics = self.node.execute_takeoff()
        self.assertGreater(
            metrics.stability_score, 80.0,
            f"Stability score too low: {metrics.stability_score:.1f}"
        )


def run_takeoff_test():
    """Run takeoff test scenario."""
    rclpy.init()
    node = TakeoffTestNode()
    
    try:
        metrics = node.execute_takeoff()
        
        print("\n" + "="*50)
        print("TAKEOFF TEST RESULTS")
        print("="*50)
        print(f"Success: {metrics.success}")
        print(f"Time to altitude: {metrics.time_to_altitude:.2f}s")
        print(f"Max overshoot: {metrics.max_overshoot:.2f}m")
        print(f"Settling time: {metrics.settling_time:.2f}s")
        print(f"Stability score: {metrics.stability_score:.1f}/100")
        print("="*50)
        
        return metrics
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # Run as standalone test
    run_takeoff_test()
