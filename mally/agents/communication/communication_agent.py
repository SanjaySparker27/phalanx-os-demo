"""
Communication Agent - MAVLink 2.0 and SATCOM Protocol Support
Provides telemetry, command, and control communications for autonomous vehicles.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Callable, Tuple
import asyncio
import time
import serial
import socket
from enum import Enum, auto

from ...core.base_agent import BaseAgent, AgentMessage, MessagePriority, AgentState
from ...utils.config import CommunicationConfig, AsyncLogger


class MAVLinkConnectionType(Enum):
    """MAVLink connection types."""
    SERIAL = auto()
    UDP = auto()
    TCP = auto()


@dataclass
class MAVLinkMessage:
    """MAVLink message wrapper."""
    msg_id: int
    system_id: int
    component_id: int
    message_type: str
    payload: Dict[str, Any]
    timestamp: float = field(default_factory=time.time)
    

@dataclass
class TelemetryData:
    """Vehicle telemetry data."""
    position: Dict[str, float] = field(default_factory=dict)  # lat, lon, alt
    attitude: Dict[str, float] = field(default_factory=dict)  # roll, pitch, yaw
    velocity: Dict[str, float] = field(default_factory=dict)  # vx, vy, vz
    battery: Dict[str, float] = field(default_factory=dict)  # voltage, current, remaining
    gps: Dict[str, Any] = field(default_factory=dict)  # fix_type, satellites, hdop
    timestamp: float = field(default_factory=time.time)


class MAVLinkConnection:
    """MAVLink connection handler."""
    
    def __init__(self, conn_type: MAVLinkConnectionType, 
                 port: str = None, baudrate: int = 57600,
                 udp_listen_port: int = 14550, udp_send_port: int = 14551,
                 sysid: int = 1, compid: int = 1):
        self.conn_type = conn_type
        self.port = port
        self.baudrate = baudrate
        self.udp_listen_port = udp_listen_port
        self.udp_send_port = udp_send_port
        self.sysid = sysid
        self.compid = compid
        
        self.connection = None
        self.mav = None
        self.is_connected = False
        self._receive_task = None
        self._callbacks: List[Callable] = []
        
    async def connect(self) -> bool:
        """Establish MAVLink connection."""
        try:
            from pymavlink import mavutil
            
            if self.conn_type == MAVLinkConnectionType.SERIAL:
                self.connection = mavutil.mavlink_connection(
                    self.port,
                    baud=self.baudrate,
                    source_system=self.sysid,
                    source_component=self.compid
                )
            elif self.conn_type == MAVLinkConnectionType.UDP:
                self.connection = mavutil.mavlink_connection(
                    f"udp:{self.udp_listen_port}",
                    source_system=self.sysid,
                    source_component=self.compid
                )
            
            # Wait for heartbeat
            self.connection.wait_heartbeat()
            self.mav = self.connection.mav
            self.is_connected = True
            
            # Start receive loop
            self._receive_task = asyncio.create_task(self._receive_loop())
            
            return True
            
        except Exception as e:
            print(f"MAVLink connection error: {e}")
            return False
    
    async def disconnect(self):
        """Close MAVLink connection."""
        self.is_connected = False
        
        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass
        
        if self.connection:
            self.connection.close()
            self.connection = None
    
    def send_message(self, msg_type: str, **kwargs):
        """Send MAVLink message."""
        if not self.is_connected or not self.mav:
            return
        
        try:
            msg = self.mav.command_long_encode(
                self.sysid,
                self.compid,
                **kwargs
            )
            self.connection.mav.send(msg)
        except Exception as e:
            print(f"Send error: {e}")
    
    def send_heartbeat(self, mav_type: int = 2, mav_autopilot: int = 12):
        """Send heartbeat message."""
        if not self.is_connected or not self.mav:
            return
        
        try:
            self.mav.heartbeat_send(
                mav_type,  # MAV_TYPE_QUADROTOR
                mav_autopilot,  # MAV_AUTOPILOT_PX4
                0,  # base mode
                0,  # custom mode
                3   # system status
            )
        except Exception as e:
            print(f"Heartbeat error: {e}")
    
    def send_telemetry(self, telemetry: TelemetryData):
        """Send telemetry data."""
        if not self.is_connected or not self.mav:
            return
        
        try:
            # Global position
            if telemetry.position:
                self.mav.global_position_int_send(
                    int(telemetry.timestamp * 1000),
                    int(telemetry.position.get("lat", 0) * 1e7),
                    int(telemetry.position.get("lon", 0) * 1e7),
                    int(telemetry.position.get("alt", 0) * 1000),
                    int(telemetry.position.get("relative_alt", 0) * 1000),
                    int(telemetry.velocity.get("vx", 0) * 100),
                    int(telemetry.velocity.get("vy", 0) * 100),
                    int(telemetry.velocity.get("vz", 0) * 100),
                    0  # heading
                )
            
            # Attitude
            if telemetry.attitude:
                self.mav.attitude_send(
                    int(telemetry.timestamp * 1000),
                    telemetry.attitude.get("roll", 0),
                    telemetry.attitude.get("pitch", 0),
                    telemetry.attitude.get("yaw", 0),
                    0, 0, 0  # rollspeed, pitchspeed, yawspeed
                )
            
            # Battery status
            if telemetry.battery:
                self.mav.battery_status_send(
                    0,  # battery ID
                    0,  # battery function
                    0,  # type
                    int(telemetry.battery.get("temperature", 0)),
                    [int(telemetry.battery.get("voltage", 0) * 1000), 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    int(telemetry.battery.get("current", 0) * 100),
                    int(telemetry.battery.get("current", 0) * 100),
                    int(telemetry.battery.get("remaining", 0)),
                    0, 0, 0, 0, 0, 0
                )
                
        except Exception as e:
            print(f"Telemetry send error: {e}")
    
    def add_callback(self, callback: Callable[[Any], None]):
        """Add message receive callback."""
        self._callbacks.append(callback)
    
    async def _receive_loop(self):
        """Background receive loop."""
        while self.is_connected:
            try:
                msg = self.connection.recv_match(blocking=False)
                if msg:
                    for callback in self._callbacks:
                        try:
                            if asyncio.iscoroutinefunction(callback):
                                asyncio.create_task(callback(msg))
                            else:
                                callback(msg)
                        except Exception as e:
                            print(f"Callback error: {e}")
                
                await asyncio.sleep(0.001)  # 1ms
                
            except Exception as e:
                print(f"Receive error: {e}")
                await asyncio.sleep(0.1)


class SatcomConnection:
    """Satellite communication connection (Iridium SBD)."""
    
    def __init__(self, imei: str = "", endpoint: str = "iridium"):
        self.imei = imei
        self.endpoint = endpoint
        self.is_connected = False
        self._receive_task = None
        self._callbacks: List[Callable] = []
        
    async def connect(self) -> bool:
        """Connect to SATCOM service."""
        # Implementation depends on specific SATCOM provider
        # This is a stub for Iridium SBD
        self.is_connected = True
        self._receive_task = asyncio.create_task(self._poll_loop())
        return True
    
    async def disconnect(self):
        """Disconnect from SATCOM."""
        self.is_connected = False
        if self._receive_task:
            self._receive_task.cancel()
            try:
                await self._receive_task
            except asyncio.CancelledError:
                pass
    
    async def send(self, data: bytes) -> bool:
        """Send data via SATCOM."""
        if not self.is_connected:
            return False
        
        # Implement Iridium SBD send
        # This would use the Iridium SBD API
        print(f"SATCOM send: {len(data)} bytes")
        return True
    
    async def receive(self) -> Optional[bytes]:
        """Receive data from SATCOM."""
        # Implement Iridium SBD receive
        return None
    
    def add_callback(self, callback: Callable[[bytes], None]):
        """Add receive callback."""
        self._callbacks.append(callback)
    
    async def _poll_loop(self):
        """Poll for incoming messages."""
        while self.is_connected:
            try:
                data = await self.receive()
                if data:
                    for callback in self._callbacks:
                        try:
                            if asyncio.iscoroutinefunction(callback):
                                asyncio.create_task(callback(data))
                            else:
                                callback(data)
                        except Exception as e:
                            print(f"SATCOM callback error: {e}")
                
                await asyncio.sleep(10)  # Poll every 10 seconds
                
            except Exception as e:
                print(f"SATCOM poll error: {e}")
                await asyncio.sleep(30)


class CommunicationAgent(BaseAgent):
    """Communication Agent for MAVLink and SATCOM."""
    
    def __init__(self, agent_id: str, config: CommunicationConfig, message_bus):
        super().__init__(agent_id, "communication", message_bus)
        self.config = config
        self.logger = AsyncLogger(f"communication_{agent_id}")
        
        # Connections
        self.mavlink: Optional[MAVLinkConnection] = None
        self.satcom: Optional[SatcomConnection] = None
        
        # State
        self.telemetry = TelemetryData()
        self.connected_systems: Dict[int, Dict] = {}
        self._heartbeat_task: Optional[asyncio.Task] = None
        self._telemetry_task: Optional[asyncio.Task] = None
        
    async def initialize(self) -> bool:
        """Initialize communication agent."""
        await self.set_state(AgentState.INITIALIZING)
        await self.logger.info("Initializing Communication Agent...")
        
        try:
            # Initialize MAVLink connection
            self.mavlink = MAVLinkConnection(
                MAVLinkConnectionType.SERIAL,
                port=self.config.mavlink_port,
                baudrate=self.config.mavlink_baudrate,
                udp_listen_port=self.config.udp_listen_port,
                udp_send_port=self.config.udp_send_port,
                sysid=self.config.mavlink_sysid,
                compid=self.config.mavlink_compid
            )
            
            if not await self.mavlink.connect():
                await self.logger.error("MAVLink connection failed")
                # Continue anyway - we can retry
            else:
                await self.logger.info("MAVLink connected")
                self.mavlink.add_callback(self._on_mavlink_message)
            
            # Initialize SATCOM if enabled
            if self.config.satcom_enabled:
                self.satcom = SatcomConnection(
                    imei=self.config.satcom_imei,
                    endpoint=self.config.satcom_endpoint
                )
                if await self.satcom.connect():
                    await self.logger.info("SATCOM connected")
                    self.satcom.add_callback(self._on_satcom_message)
                else:
                    await self.logger.warning("SATCOM connection failed")
            
            # Subscribe to message bus
            await self._message_bus.subscribe(self.agent_id, self._message_handler)
            
            # Start background tasks
            self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            self._telemetry_task = asyncio.create_task(self._telemetry_loop())
            
            await self.set_state(AgentState.RUNNING)
            return True
            
        except Exception as e:
            await self.logger.error(f"Initialization error: {e}")
            await self.set_state(AgentState.ERROR)
            return False
    
    async def process(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process incoming messages."""
        msg_type = message.message_type
        payload = message.payload
        
        if msg_type == "TELEMETRY_UPDATE":
            # Update internal telemetry
            self.telemetry = TelemetryData(
                position=payload.get("position", {}),
                attitude=payload.get("attitude", {}),
                velocity=payload.get("velocity", {}),
                battery=payload.get("battery", {}),
                gps=payload.get("gps", {})
            )
        
        elif msg_type == "SEND_MAVLINK_COMMAND":
            command = payload.get("command")
            params = payload.get("params", {})
            await self.send_mavlink_command(command, **params)
        
        elif msg_type == "SEND_SATCOM":
            data = payload.get("data", b"")
            if isinstance(data, str):
                data = data.encode()
            success = await self.send_satcom(data)
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="SATCOM_SENT" if success else "SATCOM_FAILED",
                payload={"success": success}
            )
        
        elif msg_type == "TELEMETRY_REQUEST":
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="TELEMETRY_RESPONSE",
                payload={
                    "telemetry": {
                        "position": self.telemetry.position,
                        "attitude": self.telemetry.attitude,
                        "velocity": self.telemetry.velocity,
                        "battery": self.telemetry.battery,
                        "gps": self.telemetry.gps
                    }
                }
            )
        
        return None
    
    async def send_mavlink_command(self, command: str, **kwargs):
        """Send MAVLink command."""
        if self.mavlink and self.mavlink.is_connected:
            self.mavlink.send_message(command, **kwargs)
    
    async def send_satcom(self, data: bytes) -> bool:
        """Send data via SATCOM."""
        if self.satcom and self.satcom.is_connected:
            return await self.satcom.send(data)
        return False
    
    async def broadcast_heartbeat(self):
        """Broadcast MAVLink heartbeat."""
        if self.mavlink and self.mavlink.is_connected:
            self.mavlink.send_heartbeat()
    
    async def _on_mavlink_message(self, msg):
        """Handle incoming MAVLink message."""
        msg_type = msg.get_type()
        
        if msg_type == "HEARTBEAT":
            system_id = msg.get_srcSystem()
            self.connected_systems[system_id] = {
                "type": msg.type,
                "autopilot": msg.autopilot,
                "last_seen": time.time()
            }
        
        elif msg_type == "COMMAND_ACK":
            # Forward command acknowledgment
            await self.send_message(
                "orchestrator",
                {
                    "type": "command_ack",
                    "command": msg.command,
                    "result": msg.result
                },
                message_type="COMMAND_ACK"
            )
        
        # Broadcast message to other agents
        await self.broadcast(
            payload={
                "type": "mavlink",
                "msg_type": msg_type,
                "data": msg.to_dict()
            },
            message_type="MAVLINK_MESSAGE"
        )
    
    async def _on_satcom_message(self, data: bytes):
        """Handle incoming SATCOM message."""
        await self.broadcast(
            payload={
                "type": "satcom",
                "data": data.hex()
            },
            message_type="SATCOM_MESSAGE",
            priority=MessagePriority.HIGH
        )
    
    async def _heartbeat_loop(self):
        """Send periodic heartbeats."""
        while self.state in [AgentState.RUNNING, AgentState.IDLE]:
            try:
                await self.broadcast_heartbeat()
                await asyncio.sleep(self.config.heartbeat_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                await self.logger.error(f"Heartbeat error: {e}")
                await asyncio.sleep(1)
    
    async def _telemetry_loop(self):
        """Send periodic telemetry."""
        while self.state in [AgentState.RUNNING]:
            try:
                if self.mavlink and self.mavlink.is_connected:
                    self.mavlink.send_telemetry(self.telemetry)
                
                await asyncio.sleep(1.0 / self.config.telemetry_rate)
            except asyncio.CancelledError:
                break
            except Exception as e:
                await self.logger.error(f"Telemetry error: {e}")
                await asyncio.sleep(1)
    
    async def shutdown(self) -> bool:
        """Shutdown communication agent."""
        await self.set_state(AgentState.SHUTTING_DOWN)
        await self.logger.info("Shutting down Communication Agent...")
        
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass
        
        if self._telemetry_task:
            self._telemetry_task.cancel()
            try:
                await self._telemetry_task
            except asyncio.CancelledError:
                pass
        
        if self.mavlink:
            await self.mavlink.disconnect()
        
        if self.satcom:
            await self.satcom.disconnect()
        
        await self.set_state(AgentState.SHUTDOWN)
        await self.logger.info("Communication Agent shutdown complete")
        return True
