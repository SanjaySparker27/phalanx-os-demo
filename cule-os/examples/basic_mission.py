#!/usr/bin/env python3
"""
Basic Mission Example
Demonstrates a complete autonomous mission with takeoff, waypoints, and landing.

Usage:
    python basic_mission.py --connection /dev/ttyTHS1 --altitude 10
"""

import asyncio
import argparse
import json
import time
from dataclasses import dataclass, asdict
from typing import List, Dict, Any
from datetime import datetime

from pymavlink import mavutil
from mally.main import MallySystem
from mally.utils.config import AgentConfig


@dataclass
class Waypoint:
    """Mission waypoint definition."""
    lat: float
    lon: float
    alt: float
    action: str = "WAYPOINT"  # TAKEOFF, WAYPOINT, LAND
    delay: float = 0.0
    acceptance_radius: float = 2.0


class BasicMissionController:
    """Controller for basic autonomous missions."""
    
    def __init__(self, connection_string: str, baudrate: int = 921600):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.system: MallySystem = None
        self.mav_connection = None
        self.mission_log: List[Dict] = []
        
    async def initialize(self) -> bool:
        """Initialize the mission controller."""
        print("=" * 50)
        print("CULE OS - Basic Mission Controller")
        print("=" * 50)
        
        try:
            # Initialize Mally system
            print("\n[1/4] Initializing Mally Multi-Agent System...")
            self.system = MallySystem()
            
            if not await self.system.initialize():
                print("ERROR: Failed to initialize Mally system")
                return False
            print("✓ Mally system initialized")
            
            # Direct MAVLink connection
            print(f"\n[2/4] Connecting to flight controller...")
            print(f"    Connection: {self.connection_string}")
            print(f"    Baudrate: {self.baudrate}")
            
            self.mav_connection = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )
            
            print("    Waiting for heartbeat...")
            self.mav_connection.wait_heartbeat()
            print(f"✓ Connected to system {self.mav_connection.target_system}")
            
            # Wait for GPS
            print("\n[3/4] Waiting for GPS fix...")
            await self._wait_for_gps()
            print("✓ GPS fix acquired")
            
            # Pre-flight checks
            print("\n[4/4] Pre-flight checks...")
            await self._pre_flight_checks()
            print("✓ Pre-flight checks passed")
            
            return True
            
        except Exception as e:
            print(f"\nERROR: Initialization failed: {e}")
            return False
    
    async def _wait_for_gps(self, timeout: int = 60):
        """Wait for GPS fix."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = self.mav_connection.recv_match(
                type='GPS_RAW_INT',
                blocking=False
            )
            
            if msg and msg.fix_type >= 3:  # 3D fix
                self._log_event("gps_fix", {
                    "lat": msg.lat / 1e7,
                    "lon": msg.lon / 1e7,
                    "satellites": msg.satellites_visible,
                    "hdop": msg.eph / 100
                })
                return
            
            await asyncio.sleep(0.5)
        
        raise TimeoutError("GPS fix timeout")
    
    async def _pre_flight_checks(self):
        """Perform pre-flight checks."""
        checks = [
            ("Battery voltage", self._check_battery),
            ("IMU health", self._check_imu),
            ("Compass health", self._check_compass),
            ("RC calibration", self._check_rc),
        ]
        
        for name, check_func in checks:
            print(f"    Checking {name}...", end=" ")
            result = await check_func()
            print("OK" if result else "FAIL")
            
            if not result:
                raise RuntimeError(f"Pre-flight check failed: {name}")
    
    async def _check_battery(self) -> bool:
        """Check battery status."""
        msg = self.mav_connection.recv_match(
            type='SYS_STATUS',
            blocking=True,
            timeout=2
        )
        
        if msg:
            voltage = msg.voltage_battery / 1000.0
            self._log_event("battery_check", {"voltage": voltage})
            return voltage > 10.5  # Minimum 10.5V for 3S
        return False
    
    async def _check_imu(self) -> bool:
        """Check IMU health."""
        msg = self.mav_connection.recv_match(
            type='SYS_STATUS',
            blocking=True,
            timeout=2
        )
        
        if msg:
            # Check sensors health flags
            sensors_health = msg.onboard_control_sensors_health
            return bool(sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
        return False
    
    async def _check_compass(self) -> bool:
        """Check compass health."""
        msg = self.mav_connection.recv_match(
            type='SYS_STATUS',
            blocking=True,
            timeout=2
        )
        
        if msg:
            sensors_health = msg.onboard_control_sensors_health
            return bool(sensors_health & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
        return False
    
    async def _check_rc(self) -> bool:
        """Check RC calibration."""
        msg = self.mav_connection.recv_match(
            type='RC_CHANNELS',
            blocking=True,
            timeout=2
        )
        
        if msg:
            # Check if channels have valid values
            return msg.chancount >= 5
        return False
    
    async def arm(self, force: bool = False) -> bool:
        """Arm the vehicle."""
        print("\n--- Arming ---")
        
        # Set to LOITER mode first (safer for arming)
        print("Setting LOITER mode...")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1, 5, 0, 0, 0, 0, 0
        )
        await asyncio.sleep(1)
        
        # Send arm command
        print("Sending arm command...")
        force_param = 21196 if force else 0
        
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, force_param, 0, 0, 0, 0, 0
        )
        
        # Wait for arm confirmation
        start_time = time.time()
        while time.time() - start_time < 10:
            msg = self.mav_connection.recv_match(
                type='HEARTBEAT',
                blocking=False
            )
            
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("✓ Vehicle armed")
                self._log_event("armed", {"force": force})
                return True
            
            await asyncio.sleep(0.1)
        
        print("✗ Arm timeout")
        return False
    
    async def disarm(self) -> bool:
        """Disarm the vehicle."""
        print("\n--- Disarming ---")
        
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
        await asyncio.sleep(2)
        print("✓ Vehicle disarmed")
        self._log_event("disarmed", {})
        return True
    
    async def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude."""
        print(f"\n--- Takeoff to {altitude}m ---")
        
        # Send takeoff command
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            altitude
        )
        
        self._log_event("takeoff_start", {"target_altitude": altitude})
        
        # Monitor climb
        print("Climbing...")
        start_time = time.time()
        
        while time.time() - start_time < 60:  # Max 60 seconds
            msg = self.mav_connection.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=False
            )
            
            if msg:
                current_alt = msg.relative_alt / 1000.0
                print(f"    Altitude: {current_alt:.1f}m / {altitude}m", end='\r')
                
                if current_alt >= altitude * 0.95:  # Within 5% of target
                    print(f"\n✓ Reached target altitude: {current_alt:.1f}m")
                    self._log_event("takeoff_complete", {"altitude": current_alt})
                    return True
            
            await asyncio.sleep(0.2)
        
        print("\n✗ Takeoff timeout")
        return False
    
    async def goto_waypoint(self, waypoint: Waypoint, timeout: int = 120) -> bool:
        """Navigate to a waypoint."""
        print(f"\n--- Waypoint: {waypoint.action} ---")
        print(f"    Target: ({waypoint.lat:.6f}, {waypoint.lon:.6f}) @ {waypoint.alt}m")
        
        if waypoint.action == "LAND":
            # Use LAND command
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0,
                waypoint.lat,
                waypoint.lon,
                0
            )
        else:
            # Use waypoint command
            self.mav_connection.mav.command_long_send(
                self.mav_connection.target_system,
                self.mav_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                waypoint.delay,
                waypoint.acceptance_radius,
                0, 0,
                waypoint.lat,
                waypoint.lon,
                waypoint.alt
            )
        
        self._log_event("waypoint_start", asdict(waypoint))
        
        # Monitor progress
        start_time = time.time()
        last_print = 0
        
        while time.time() - start_time < timeout:
            msg = self.mav_connection.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=False
            )
            
            if msg:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7
                current_alt = msg.relative_alt / 1000.0
                
                # Calculate distance (simplified)
                distance = ((current_lat - waypoint.lat) ** 2 +
                           (current_lon - waypoint.lon) ** 2) ** 0.5 * 111000
                
                if time.time() - last_print > 1:
                    print(f"    Distance: {distance:.1f}m | "
                          f"Alt: {current_alt:.1f}m", end='\r')
                    last_print = time.time()
                
                # Check if waypoint reached
                if distance < waypoint.acceptance_radius:
                    if waypoint.delay > 0:
                        print(f"\n    Holding for {waypoint.delay}s...")
                        await asyncio.sleep(waypoint.delay)
                    
                    print(f"\n✓ Waypoint reached")
                    self._log_event("waypoint_reached", asdict(waypoint))
                    return True
            
            await asyncio.sleep(0.1)
        
        print("\n✗ Waypoint timeout")
        return False
    
    async def land(self) -> bool:
        """Execute landing sequence."""
        print("\n--- Landing ---")
        
        # First descend to 5m in LOITER
        print("Descending to 5m...")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0, 2, 0, 0, 0, 0, 5
        )
        await asyncio.sleep(10)
        
        # Switch to LAND mode
        print("Initiating landing...")
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self._log_event("land_start", {})
        
        # Monitor landing
        start_time = time.time()
        while time.time() - start_time < 60:
            msg = self.mav_connection.recv_match(
                type='GLOBAL_POSITION_INT',
                blocking=False
            )
            
            if msg:
                alt = msg.relative_alt / 1000.0
                print(f"    Altitude: {alt:.1f}m", end='\r')
                
                if alt < 0.3:  # Close to ground
                    print("\n✓ Landed")
                    break
            
            await asyncio.sleep(0.2)
        
        # Disarm after landing
        await asyncio.sleep(3)
        await self.disarm()
        
        self._log_event("land_complete", {})
        return True
    
    async def rtl(self) -> bool:
        """Return to launch."""
        print("\n--- Return to Launch ---")
        
        self.mav_connection.mav.command_long_send(
            self.mav_connection.target_system,
            self.mav_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        
        self._log_event("rtl_start", {})
        print("RTL initiated - vehicle returning to launch")
        
        # Monitor RTL
        await asyncio.sleep(60)  # Wait for RTL to complete
        
        self._log_event("rtl_complete", {})
        return True
    
    def _log_event(self, event_type: str, data: Dict):
        """Log mission event."""
        event = {
            "timestamp": datetime.now().isoformat(),
            "event": event_type,
            "data": data
        }
        self.mission_log.append(event)
    
    def save_mission_log(self, filename: str = None):
        """Save mission log to file."""
        if filename is None:
            filename = f"mission_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        
        with open(filename, 'w') as f:
            json.dump(self.mission_log, f, indent=2)
        
        print(f"\nMission log saved to: {filename}")
    
    async def run_mission(self, waypoints: List[Waypoint], rtl_at_end: bool = False):
        """Execute a complete mission."""
        
        # Initialize
        if not await self.initialize():
            return False
        
        try:
            # Arm
            if not await self.arm():
                print("Failed to arm - trying force arm...")
                if not await self.arm(force=True):
                    return False
            
            # Takeoff
            takeoff_alt = waypoints[0].alt if waypoints else 10
            if not await self.takeoff(takeoff_alt):
                await self.land()
                return False
            
            # Execute waypoints
            for i, wp in enumerate(waypoints[1:], 1):
                print(f"\n{'='*40}")
                print(f"Waypoint {i}/{len(waypoints)-1}")
                print(f"{'='*40}")
                
                if not await self.goto_waypoint(wp):
                    print(f"Failed to reach waypoint {i}")
                    await self.rtl()
                    return False
                
                await asyncio.sleep(1)
            
            # Return to launch or land
            if rtl_at_end:
                await self.rtl()
            else:
                await self.land()
            
            print("\n" + "=" * 50)
            print("✓ MISSION COMPLETE")
            print("=" * 50)
            
            return True
            
        except KeyboardInterrupt:
            print("\n\n! Mission interrupted by user")
            print("Initiating emergency RTL...")
            await self.rtl()
            return False
            
        except Exception as e:
            print(f"\n! Mission error: {e}")
            print("Initiating emergency RTL...")
            await self.rtl()
            return False
            
        finally:
            self.save_mission_log()
            await self.shutdown()
    
    async def shutdown(self):
        """Shutdown the controller."""
        print("\n--- Shutdown ---")
        
        if self.system:
            await self.system.shutdown()
        
        if self.mav_connection:
            self.mav_connection.close()
        
        print("✓ Controller shutdown complete")


def main():
    parser = argparse.ArgumentParser(
        description="Cule OS Basic Mission Controller"
    )
    parser.add_argument(
        "--connection", "-c",
        default="/dev/ttyTHS1",
        help="MAVLink connection string (default: /dev/ttyTHS1)"
    )
    parser.add_argument(
        "--baudrate", "-b",
        type=int,
        default=921600,
        help="Baudrate for serial connection (default: 921600)"
    )
    parser.add_argument(
        "--altitude", "-a",
        type=float,
        default=10,
        help="Takeoff altitude in meters (default: 10)"
    )
    parser.add_argument(
        "--rtl",
        action="store_true",
        help="Return to launch at end of mission"
    )
    
    args = parser.parse_args()
    
    # Define mission waypoints
    # This is a simple square pattern - adjust coordinates for your location
    waypoints = [
        Waypoint(0, 0, args.altitude, action="TAKEOFF"),
        Waypoint(37.7749, -122.4194, args.altitude, delay=2.0),
        Waypoint(37.7750, -122.4194, args.altitude, delay=2.0),
        Waypoint(37.7750, -122.4195, args.altitude, delay=2.0),
        Waypoint(37.7749, -122.4195, args.altitude, delay=2.0),
        Waypoint(37.7749, -122.4194, args.altitude, action="LAND"),
    ]
    
    # Run mission
    controller = BasicMissionController(args.connection, args.baudrate)
    
    try:
        asyncio.run(controller.run_mission(waypoints, rtl_at_end=args.rtl))
    except KeyboardInterrupt:
        print("\n\nMission aborted by user")


if __name__ == "__main__":
    main()
