#!/usr/bin/env python3
"""
Takeoff and Land Example
Demonstrates safe takeoff, hover, and landing sequences.

Usage:
    python takeoff_land.py --connection /dev/ttyTHS1 --altitude 10 --hover 10
"""

import asyncio
import argparse
import time
from datetime import datetime
from typing import Dict, Any, Optional

from pymavlink import mavutil
from mally.main import MallySystem


class TakeoffLandController:
    """Controller for takeoff, hover, and landing operations."""
    
    def __init__(self, connection_string: str, baudrate: int = 921600):
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.system: Optional[MallySystem] = None
        self.mav = None
        self.telemetry: Dict[str, Any] = {}
        
    async def initialize(self) -> bool:
        """Initialize connections and systems."""
        print("=" * 60)
        print("  CULE OS - Takeoff and Landing Controller")
        print("=" * 60)
        
        try:
            # Initialize Mally system
            print("\n[INIT] Starting Mally system...")
            self.system = MallySystem()
            
            if not await self.system.initialize():
                print("[ERROR] Failed to initialize Mally system")
                return False
            print("[OK] Mally system ready")
            
            # Connect to flight controller
            print(f"\n[INIT] Connecting to FC: {self.connection_string} @ {self.baudrate}")
            self.mav = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )
            
            print("[INIT] Waiting for heartbeat...")
            self.mav.wait_heartbeat()
            print(f"[OK] Connected to system {self.mav.target_system}")
            
            # Enable data streams
            self._request_data_streams()
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Initialization failed: {e}")
            return False
    
    def _request_data_streams(self):
        """Request telemetry data streams."""
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2),
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA3, 2),
        ]
        
        for stream_id, rate in streams:
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,
                self.mav.target_component,
                stream_id,
                rate,
                1
            )
    
    async def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data."""
        telemetry = {}
        
        # Get position
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            telemetry['position'] = {
                'lat': msg.lat / 1e7,
                'lon': msg.lon / 1e7,
                'alt': msg.alt / 1000.0,
                'relative_alt': msg.relative_alt / 1000.0,
                'vx': msg.vx / 100.0,
                'vy': msg.vy / 100.0,
                'vz': msg.vz / 100.0
            }
        
        # Get attitude
        msg = self.mav.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            telemetry['attitude'] = {
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'rollspeed': msg.rollspeed,
                'pitchspeed': msg.pitchspeed,
                'yawspeed': msg.yawspeed
            }
        
        # Get battery
        msg = self.mav.recv_match(type='SYS_STATUS', blocking=False)
        if msg:
            telemetry['battery'] = {
                'voltage': msg.voltage_battery / 1000.0,
                'current': msg.current_battery / 100.0,
                'remaining': msg.battery_remaining
            }
        
        # Get GPS
        msg = self.mav.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            telemetry['gps'] = {
                'fix_type': msg.fix_type,
                'satellites': msg.satellites_visible,
                'hdop': msg.eph / 100
            }
        
        # Get flight mode
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            telemetry['status'] = {
                'mode': mode,
                'armed': armed
            }
        
        self.telemetry.update(telemetry)
        return telemetry
    
    def print_telemetry(self):
        """Print current telemetry."""
        if not self.telemetry:
            return
        
        print("\n" + "-" * 60)
        print("TELEMETRY:")
        print("-" * 60)
        
        if 'position' in self.telemetry:
            pos = self.telemetry['position']
            print(f"  Position: ({pos['lat']:.6f}, {pos['lon']:.6f})")
            print(f"  Altitude: {pos['relative_alt']:.1f}m (rel) / {pos['alt']:.1f}m (abs)")
            print(f"  Velocity: ({pos['vx']:.1f}, {pos['vy']:.1f}, {pos['vz']:.1f}) m/s")
        
        if 'attitude' in self.telemetry:
            att = self.telemetry['attitude']
            print(f"  Attitude: Roll={att['roll']:.2f}, Pitch={att['pitch']:.2f}, Yaw={att['yaw']:.2f}")
        
        if 'battery' in self.telemetry:
            bat = self.telemetry['battery']
            print(f"  Battery: {bat['voltage']:.1f}V, {bat['current']:.1f}A, {bat['remaining']}%")
        
        if 'gps' in self.telemetry:
            gps = self.telemetry['gps']
            fix_types = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS", 5: "RTK"}
            print(f"  GPS: {fix_types.get(gps['fix_type'], 'Unknown')}, "
                  f"Sats: {gps['satellites']}, HDOP: {gps['hdop']}")
        
        if 'status' in self.telemetry:
            status = self.telemetry['status']
            armed_str = "ARMED" if status['armed'] else "DISARMED"
            print(f"  Status: {armed_str}, Mode: {status['mode']}")
        
        print("-" * 60)
    
    async def wait_for_condition(self, condition_func, timeout: float = 30) -> bool:
        """Wait for a condition to be met."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            await self.get_telemetry()
            
            if condition_func(self.telemetry):
                return True
            
            await asyncio.sleep(0.1)
        
        return False
    
    async def set_mode(self, mode_name: str) -> bool:
        """Set flight mode."""
        mode_map = {
            'STABILIZE': 0,
            'ACRO': 1,
            'ALT_HOLD': 2,
            'AUTO': 3,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'CIRCLE': 7,
            'LAND': 9,
            'DRIFT': 11,
            'SPORT': 13,
            'FLIP': 14,
            'AUTOTUNE': 15,
            'POSHOLD': 16,
            'BRAKE': 17,
            'THROW': 18,
            'AVOID_ADSB': 19,
            'GUIDED_NOGPS': 20,
            'SMART_RTL': 21
        }
        
        mode_id = mode_map.get(mode_name.upper())
        if mode_id is None:
            print(f"[ERROR] Unknown mode: {mode_name}")
            return False
        
        print(f"\n[MODE] Setting mode to {mode_name}...")
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1, mode_id, 0, 0, 0, 0, 0
        )
        
        # Wait for mode change
        success = await self.wait_for_condition(
            lambda t: t.get('status', {}).get('mode') == mode_name,
            timeout=5
        )
        
        if success:
            print(f"[OK] Mode set to {mode_name}")
        else:
            print(f"[WARNING] Mode change timeout")
        
        return success
    
    async def arm(self, force: bool = False) -> bool:
        """Arm the vehicle."""
        print("\n[ARM] Arming vehicle...")
        
        # First ensure we're in LOITER mode
        await self.set_mode('LOITER')
        
        force_param = 21196 if force else 0
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, force_param, 0, 0, 0, 0, 0
        )
        
        # Wait for arm
        success = await self.wait_for_condition(
            lambda t: t.get('status', {}).get('armed', False),
            timeout=10
        )
        
        if success:
            print("[OK] Vehicle armed")
        else:
            print("[ERROR] Arming failed")
        
        return success
    
    async def disarm(self) -> bool:
        """Disarm the vehicle."""
        print("\n[DISARM] Disarming vehicle...")
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        
        await asyncio.sleep(2)
        
        # Wait for disarm
        success = await self.wait_for_condition(
            lambda t: not t.get('status', {}).get('armed', True),
            timeout=5
        )
        
        if success:
            print("[OK] Vehicle disarmed")
        else:
            print("[WARNING] Disarm timeout")
        
        return success
    
    async def takeoff(self, target_altitude: float) -> bool:
        """Takeoff to target altitude."""
        print(f"\n[TAKEOFF] Taking off to {target_altitude}m...")
        
        # Ensure we're armed
        await self.get_telemetry()
        if not self.telemetry.get('status', {}).get('armed', False):
            if not await self.arm():
                return False
        
        # Send takeoff command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0, 0, 0,
            target_altitude
        )
        
        # Monitor climb
        print("[TAKEOFF] Climbing...")
        start_time = time.time()
        
        while time.time() - start_time < 60:
            await self.get_telemetry()
            
            if 'position' in self.telemetry:
                alt = self.telemetry['position']['relative_alt']
                print(f"  Altitude: {alt:.1f}m / {target_altitude}m", end='\r')
                
                if alt >= target_altitude * 0.95:
                    print(f"\n[OK] Reached target altitude: {alt:.1f}m")
                    return True
            
            await asyncio.sleep(0.2)
        
        print("\n[ERROR] Takeoff timeout")
        return False
    
    async def hover(self, duration: float):
        """Hover in place for specified duration."""
        print(f"\n[HOVER] Hovering for {duration} seconds...")
        print("-" * 60)
        
        start_time = time.time()
        last_telemetry = 0
        
        while time.time() - start_time < duration:
            await self.get_telemetry()
            
            # Print telemetry every 2 seconds
            if time.time() - last_telemetry > 2:
                elapsed = time.time() - start_time
                remaining = duration - elapsed
                
                if 'position' in self.telemetry:
                    alt = self.telemetry['position']['relative_alt']
                    print(f"  Time: {elapsed:.1f}s / {duration}s | Alt: {alt:.1f}m | Remaining: {remaining:.1f}s")
                
                last_telemetry = time.time()
            
            await asyncio.sleep(0.1)
        
        print("-" * 60)
        print(f"[OK] Hover complete")
    
    async def land(self) -> bool:
        """Execute safe landing."""
        print("\n[LAND] Initiating landing sequence...")
        
        # Step 1: Descend to 5m in LOITER
        print("[LAND] Step 1: Descending to 5m...")
        await self.set_mode('LOITER')
        
        # Send waypoint at 5m
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0, 2, 0, 0, 0, 0, 5
        )
        
        # Wait for descent
        await asyncio.sleep(8)
        
        # Step 2: Switch to LAND mode
        print("[LAND] Step 2: Switching to LAND mode...")
        await self.set_mode('LAND')
        
        # Monitor landing
        print("[LAND] Descending...")
        start_time = time.time()
        
        while time.time() - start_time < 60:
            await self.get_telemetry()
            
            if 'position' in self.telemetry:
                alt = self.telemetry['position']['relative_alt']
                print(f"  Altitude: {alt:.1f}m", end='\r')
                
                if alt < 0.3:
                    print(f"\n[OK] Touchdown detected")
                    break
            
            await asyncio.sleep(0.2)
        
        # Step 3: Wait and disarm
        print("[LAND] Step 3: Stabilizing...")
        await asyncio.sleep(3)
        
        await self.disarm()
        
        print("[OK] Landing complete")
        return True
    
    async def run_sequence(self, altitude: float, hover_time: float):
        """Run complete takeoff-hover-land sequence."""
        
        if not await self.initialize():
            return False
        
        try:
            # Show initial telemetry
            await self.get_telemetry()
            self.print_telemetry()
            
            # Wait for user confirmation
            print("\n" + "=" * 60)
            print("READY FOR FLIGHT")
            print(f"  Target altitude: {altitude}m")
            print(f"  Hover time: {hover_time}s")
            print("=" * 60)
            
            input("\nPress ENTER to start sequence (Ctrl+C to abort)...")
            
            # Execute sequence
            print("\n" + "=" * 60)
            print("SEQUENCE STARTED")
            print("=" * 60)
            
            # 1. Takeoff
            if not await self.takeoff(altitude):
                print("\n[ERROR] Takeoff failed - aborting")
                await self.land()
                return False
            
            # 2. Hover
            await self.hover(hover_time)
            
            # 3. Land
            await self.land()
            
            # Show final telemetry
            await self.get_telemetry()
            self.print_telemetry()
            
            print("\n" + "=" * 60)
            print("SEQUENCE COMPLETE - SUCCESS")
            print("=" * 60)
            
            return True
            
        except KeyboardInterrupt:
            print("\n\n[!] Interrupted by user")
            print("Initiating emergency landing...")
            await self.land()
            return False
            
        except Exception as e:
            print(f"\n[!] Error: {e}")
            print("Initiating emergency landing...")
            await self.land()
            return False
            
        finally:
            await self.shutdown()
    
    async def shutdown(self):
        """Shutdown the controller."""
        print("\n[SHUTDOWN] Cleaning up...")
        
        if self.system:
            await self.system.shutdown()
        
        if self.mav:
            self.mav.close()
        
        print("[OK] Shutdown complete")


def main():
    parser = argparse.ArgumentParser(
        description="Cule OS Takeoff and Landing Controller"
    )
    parser.add_argument(
        "--connection", "-c",
        default="/dev/ttyTHS1",
        help="MAVLink connection string"
    )
    parser.add_argument(
        "--baudrate", "-b",
        type=int,
        default=921600,
        help="Serial baudrate"
    )
    parser.add_argument(
        "--altitude", "-a",
        type=float,
        default=10.0,
        help="Takeoff altitude (meters)"
    )
    parser.add_argument(
        "--hover", "-t",
        type=float,
        default=10.0,
        help="Hover time (seconds)"
    )
    
    args = parser.parse_args()
    
    controller = TakeoffLandController(args.connection, args.baudrate)
    
    try:
        asyncio.run(controller.run_sequence(args.altitude, args.hover))
    except KeyboardInterrupt:
        print("\n\nAborted by user")


if __name__ == "__main__":
    main()
