#!/usr/bin/env python3
"""
ROS2-Gazebo Bridge Node
Handles communication between ROS2 and Gazebo/Ignition simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import Imu, NavSatFix, Image, CameraInfo, LaserScan
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import numpy as np
import threading
import queue


class GazeboBridgeNode(Node):
    """Bridge between ROS2 and Gazebo simulation."""
    
    def __init__(self):
        super().__init__('gazebo_bridge')
        
        # Parameters
        self.declare_parameter('vehicle_type', 'uav')
        self.declare_parameter('model_name', 'vehicle')
        self.declare_parameter('publish_rate', 50.0)
        
        self.vehicle_type = self.get_parameter('vehicle_type').value
        self.model_name = self.get_parameter('model_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data', self.sensor_qos)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', self.sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', self.sensor_qos)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', self.sensor_qos)
        
        # Vehicle-specific publishers
        if self.vehicle_type == 'uav':
            self.altitude_pub = self.create_publisher(Float64, 'altitude', self.sensor_qos)
            self.battery_pub = self.create_publisher(Float64, 'battery/voltage', self.sensor_qos)
        elif self.vehicle_type == 'usv':
            self.water_depth_pub = self.create_publisher(Float64, 'water_depth', self.sensor_qos)
        elif self.vehicle_type == 'ugv':
            self.lidar_pub = self.create_publisher(LaserScan, 'scan', self.sensor_qos)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, self.cmd_qos)
        self.cmd_pose_sub = self.create_subscription(
            PoseStamped, 'cmd_pose', self.cmd_pose_callback, self.cmd_qos)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Command queue
        self.cmd_queue = queue.Queue(maxsize=10)
        
        # State
        self.current_pose = None
        self.current_twist = None
        
        # Timers
        self.create_timer(1.0 / self.publish_rate, self.publish_sensors)
        self.create_timer(0.01, self.process_commands)
        
        self.get_logger().info(f'Gazebo bridge initialized for {self.vehicle_type}')
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands."""
        try:
            self.cmd_queue.put(('vel', msg), block=False)
        except queue.Full:
            self.get_logger().warn('Command queue full')
    
    def cmd_pose_callback(self, msg: PoseStamped):
        """Handle pose commands."""
        try:
            self.cmd_queue.put(('pose', msg), block=False)
        except queue.Full:
            self.get_logger().warn('Command queue full')
    
    def process_commands(self):
        """Process queued commands."""
        try:
            while True:
                cmd_type, cmd_msg = self.cmd_queue.get_nowait()
                if cmd_type == 'vel':
                    self.send_velocity_command(cmd_msg)
                elif cmd_type == 'pose':
                    self.send_pose_command(cmd_msg)
        except queue.Empty:
            pass
    
    def send_velocity_command(self, twist: Twist):
        """Send velocity command to Gazebo."""
        self.get_logger().debug(
            f'Vel cmd: linear=({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f})'
        )
    
    def send_pose_command(self, pose: PoseStamped):
        """Send pose command to Gazebo."""
        self.get_logger().debug(
            f'Pose cmd: position=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})'
        )
    
    def publish_sensors(self):
        """Publish sensor data."""
        now = self.get_clock().now().to_msg()
        
        # IMU
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'base_link'
        self.imu_pub.publish(imu_msg)
        
        # GPS
        gps_msg = NavSatFix()
        gps_msg.header.stamp = now
        gps_msg.header.frame_id = 'gps_link'
        gps_msg.status.status = 0
        gps_msg.status.service = 1
        self.gps_pub.publish(gps_msg)
        
        # Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        self.odom_pub.publish(odom_msg)
        
        # TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
