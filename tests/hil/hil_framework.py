#!/usr/bin/env python3
"""
Hardware-in-Loop (HIL) Testing Framework
Tests real hardware with simulated sensors/actuators.
"""

import serial
import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional, Callable, Dict, List, Any
from enum import Enum
import numpy as np


class HILMode(Enum):
    """HIL simulation modes."""
    SIL = "software_in_loop"  # Fully simulated
    HIL = "hardware_in_loop"   # Real hardware + simulated environment
    PIL = "processor_in_loop"  # Real processor code + simulated hardware


class MAVLinkMessage(Enum):
    """MAVLink message IDs."""
    HEARTBEAT = 0
    ATTITUDE = 30
    GLOBAL_POSITION = 33
    RC_CHANNELS = 65
    HIL_STATE = 90
    HIL_CONTROLS = 91
    HIL_SENSOR = 107


@dataclass
class SensorData:
    """Simulated sensor data for HIL."""
    timestamp: float
    accel: tuple  # x, y, z in m/s^2
    gyro: tuple   # x, y, z in rad/s
    mag: tuple    # x, y, z in gauss
    pressure: float  # hPa
    gps_lat: float   # degrees
    gps_lon: float   # degrees
    gps_alt: float   # meters


@dataclass
class ActuatorOutputs:
    """Actuator outputs from flight controller."""
    timestamp: float
    throttle: float  # 0-1
    roll: float      # -1 to 1
    pitch: float     # -1 to 1
    yaw: float       # -1 to 1
    aux1: float = 0.0
    aux2: float = 0.0


class HILSimulator:
    """Base HIL simulator class."""
    
    def __init__(self, mode: HILMode = HILMode.HIL):
        self.mode = mode
        self.running = False
        self.simulation_rate = 1000  # Hz
        
        # Vehicle state
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.angular_rate = np.array([0.0, 0.0, 0.0])
        
        # Control inputs
        self.actuator_outputs: Optional[ActuatorOutputs] = None
        
        # Callbacks
        self.sensor_callbacks: List[Callable[[SensorData], None]] = []
        self.control_callbacks: List[Callable[[ActuatorOutputs], None]] = []
        
    def register_sensor_callback(self, callback: Callable[[SensorData], None]):
        """Register callback for sensor data."""
        self.sensor_callbacks.append(callback)
    
    def register_control_callback(self, callback: Callable[[ActuatorOutputs], None]):
        """Register callback for control outputs."""
        self.control_callbacks.append(callback)
    
    def update_dynamics(self, dt: float):
        """Update vehicle dynamics based on actuator outputs."""
        if self.actuator_outputs is None:
            return
        
        # Simple quadcopter dynamics model
        thrust = self.actuator_outputs.throttle * 20.0  # Max thrust N
        
        # Calculate forces
        roll = self.actuator_outputs.roll * 0.5  # max 0.5 rad
        pitch = self.actuator_outputs.pitch * 0.5
        yaw_rate = self.actuator_outputs.yaw * 2.0
        
        # Update attitude
        self.attitude[0] += roll * dt * 2.0
        self.attitude[1] += pitch * dt * 2.0
        self.attitude[2] += yaw_rate * dt
        
        # Limit attitude
        self.attitude[0] = np.clip(self.attitude[0], -1.0, 1.0)
        self.attitude[1] = np.clip(self.attitude[1], -1.0, 1.0)
        
        # Calculate acceleration
        gravity = np.array([0, 0, -9.81])
        thrust_vec = np.array([
            thrust * np.sin(self.attitude[1]),
            -thrust * np.sin(self.attitude[0]),
            thrust * np.cos(self.attitude[0]) * np.cos(self.attitude[1])
        ])
        
        mass = 1.5  # kg
        acceleration = gravity + thrust_vec / mass
        
        # Drag
        acceleration -= 0.1 * self.velocity
        
        # Update velocity and position
        self.velocity += acceleration * dt
        self.position += self.velocity * dt
        
        # Ground collision
        if self.position[2] < 0:
            self.position[2] = 0
            self.velocity[2] = max(0, self.velocity[2])
    
    def generate_sensor_data(self) -> SensorData:
        """Generate simulated sensor data."""
        now = time.time()
        
        # Add noise
        accel_noise = np.random.normal(0, 0.1, 3)
        gyro_noise = np.random.normal(0, 0.01, 3)
        mag_noise = np.random.normal(0, 0.001, 3)
        
        # Calculate accelerometer reading (including gravity)
        gravity_body = np.array([
            -9.81 * np.sin(self.attitude[1]),
            9.81 * np.sin(self.attitude[0]),
            -9.81 * np.cos(self.attitude[0]) * np.cos(self.attitude[1])
        ])
        
        # Add linear acceleration
        accel = gravity_body + accel_noise
        
        return SensorData(
            timestamp=now,
            accel=tuple(accel),
            gyro=tuple(self.angular_rate + gyro_noise),
            mag=tuple(np.array([0.2, 0.0, 0.4]) + mag_noise),
            pressure=1013.25 - self.position[2] * 0.12,
            gps_lat=37.7749 + self.position[0] / 111320,
            gps_lon=-122.4194 + self.position[1] / (111320 * np.cos(np.radians(37.7749))),
            gps_alt=self.position[2]
        )
    
    def simulation_loop(self):
        """Main simulation loop."""
        dt = 1.0 / self.simulation_rate
        
        while self.running:
            start = time.time()
            
            # Update physics
            self.update_dynamics(dt)
            
            # Generate sensor data
            sensor_data = self.generate_sensor_data()
            
            # Notify callbacks
            for callback in self.sensor_callbacks:
                callback(sensor_data)
            
            # Maintain timing
            elapsed = time.time() - start
            if elapsed < dt:
                time.sleep(dt - elapsed)
    
    def start(self):
        """Start HIL simulation."""
        self.running = True
        self.sim_thread = threading.Thread(target=self.simulation_loop)
        self.sim_thread.start()
    
    def stop(self):
        """Stop HIL simulation."""
        self.running = False
        if hasattr(self, 'sim_thread'):
            self.sim_thread.join()


class MAVLinkHILInterface:
    """MAVLink HIL interface for flight controllers."""
    
    def __init__(self, connection_string: str = "udp:127.0.0.1:14550"):
        self.connection_string = connection_string
        self.socket: Optional[socket.socket] = None
        self.serial_port: Optional[serial.Serial] = None
        
        self.hil_simulator = HILSimulator(HILMode.HIL)
        self.hil_simulator.register_sensor_callback(self.send_hil_sensor)
        
    def connect(self) -> bool:
        """Connect to flight controller."""
        try:
            if self.connection_string.startswith("udp:"):
                parts = self.connection_string[4:].split(':')
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.socket.bind((parts[0], int(parts[1])))
                print(f"Connected via UDP to {self.connection_string}")
            elif self.connection_string.startswith("serial:"):
                port = self.connection_string[7:]
                self.serial_port = serial.Serial(port, 921600, timeout=1)
                print(f"Connected via serial to {port}")
            
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def send_hil_sensor(self, sensor_data: SensorData):
        """Send HIL sensor data to flight controller."""
        # Pack MAVLink HIL_SENSOR message
        # Simplified - full implementation would use proper MAVLink library
        msg_id = MAVLinkMessage.HIL_SENSOR.value
        
        # Pack data (simplified structure)
        data = struct.pack('<QfffffffffffH',
            int(sensor_data.timestamp * 1e6),  # time_usec
            sensor_data.accel[0],
            sensor_data.accel[1],
            sensor_data.accel[2],
            sensor_data.gyro[0],
            sensor_data.gyro[1],
            sensor_data.gyro[2],
            sensor_data.mag[0],
            sensor_data.mag[1],
            sensor_data.mag[2],
            sensor_data.pressure * 100,  # Pascal
            0,  # diff_pressure
            0xFFF  # fields_updated
        )
        
        # Send message
        if self.socket:
            # Would need proper MAVLink framing
            pass
    
    def receive_hil_actuator_controls(self) -> Optional[ActuatorOutputs]:
        """Receive actuator controls from flight controller."""
        if not self.socket:
            return None
        
        try:
            self.socket.settimeout(0.001)
            data, addr = self.socket.recvfrom(1024)
            
            # Parse MAVLink message (simplified)
            if len(data) > 10:
                # Extract HIL_ACTUATOR_CONTROLS
                return ActuatorOutputs(
                    timestamp=time.time(),
                    throttle=0.5,  # Would parse from message
                    roll=0.0,
                    pitch=0.0,
                    yaw=0.0
                )
        except socket.timeout:
            pass
        
        return None
    
    def start(self):
        """Start HIL interface."""
        if self.connect():
            self.hil_simulator.start()
            
            # Start receive thread
            self.running = True
            self.recv_thread = threading.Thread(target=self.receive_loop)
            self.recv_thread.start()
    
    def receive_loop(self):
        """Receive loop for control commands."""
        while self.running:
            controls = self.receive_hil_actuator_controls()
            if controls:
                self.hil_simulator.actuator_outputs = controls
            time.sleep(0.001)
    
    def stop(self):
        """Stop HIL interface."""
        self.running = False
        self.hil_simulator.stop()
        if hasattr(self, 'recv_thread'):
            self.recv_thread.join()


class HILTestRunner:
    """Run HIL tests and collect results."""
    
    def __init__(self):
        self.tests: List[Dict[str, Any]] = []
        self.results: List[Dict[str, Any]] = []
        
    def add_test(self, name: str, duration: float, 
                 expected_behavior: Callable[[HILSimulator], bool]):
        """Add a HIL test."""
        self.tests.append({
            'name': name,
            'duration': duration,
            'expected_behavior': expected_behavior
        })
    
    def run_tests(self) -> Dict[str, Any]:
        """Run all HIL tests."""
        print("\n" + "="*60)
        print("HARDWARE-IN-LOOP TEST SUITE")
        print("="*60)
        
        hil = MAVLinkHILInterface()
        hil.start()
        
        try:
            for test in self.tests:
                print(f"\nRunning: {test['name']}")
                
                # Reset simulator
                hil.hil_simulator.position = np.array([0.0, 0.0, 0.0])
                hil.hil_simulator.velocity = np.array([0.0, 0.0, 0.0])
                
                # Run test
                start_time = time.time()
                success = True
                
                while time.time() - start_time < test['duration']:
                    if not test['expected_behavior'](hil.hil_simulator):
                        success = False
                        break
                    time.sleep(0.01)
                
                result = {
                    'name': test['name'],
                    'success': success,
                    'duration': time.time() - start_time
                }
                self.results.append(result)
                
                status = "✓ PASSED" if success else "✗ FAILED"
                print(f"  {status}")
        
        finally:
            hil.stop()
        
        # Summary
        passed = sum(1 for r in self.results if r['success'])
        total = len(self.results)
        
        print("\n" + "="*60)
        print(f"HIL TEST SUMMARY: {passed}/{total} passed")
        print("="*60)
        
        return {
            'total': total,
            'passed': passed,
            'failed': total - passed,
            'results': self.results
        }


def main():
    """Run HIL test suite."""
    runner = HILTestRunner()
    
    # Add tests
    runner.add_test(
        "Hover Stability",
        duration=10.0,
        expected_behavior=lambda sim: abs(sim.position[2] - 5.0) < 1.0
    )
    
    runner.add_test(
        "Altitude Climb",
        duration=15.0,
        expected_behavior=lambda sim: sim.position[2] > 8.0
    )
    
    runner.add_test(
        "Position Hold",
        duration=20.0,
        expected_behavior=lambda sim: np.linalg.norm(sim.position[:2]) < 2.0
    )
    
    # Run tests
    results = runner.run_tests()
    
    return results


if __name__ == '__main__':
    main()
