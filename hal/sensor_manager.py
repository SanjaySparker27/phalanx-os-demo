#!/usr/bin/env python3
"""
Unified Sensor Manager for Cule OS
High-level Python interface for all vehicle sensors with hotplug detection,
automatic discovery, and health monitoring.
"""

import os
import sys
import time
import json
import yaml
import queue
import signal
import logging
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, List, Callable, Optional, Any, Tuple
from enum import Enum, auto
from pathlib import Path
import ctypes
from ctypes import CDLL, Structure, c_float, c_double, c_int, c_uint, c_uint64, c_char, c_bool

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('SensorManager')


class SensorState(Enum):
    """Sensor operational states"""
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ACTIVE = auto()
    ERROR = auto()
    DEGRADED = auto()


class SensorCategory(Enum):
    """Sensor categories"""
    CAMERA = "camera"
    GNSS = "gnss"
    IMU = "imu"
    LIDAR = "lidar"
    RADAR = "radar"
    BAROMETER = "barometer"
    MAGNETOMETER = "magnetometer"
    SATCOM = "satcom"
    TELEMETRY = "telemetry"
    AIRSPEED = "airspeed"
    TEMPERATURE = "temperature"
    CUSTOM = "custom"


@dataclass
class SensorInfo:
    """Sensor metadata"""
    name: str
    category: SensorCategory
    driver_type: str
    device_path: str
    bus_type: str  # i2c, spi, uart, usb, ethernet
    bus_address: Optional[str] = None
    serial_number: Optional[str] = None
    firmware_version: Optional[str] = None
    capabilities: List[str] = field(default_factory=list)


@dataclass
class SensorHealth:
    """Sensor health status"""
    state: SensorState
    is_healthy: bool
    last_update: float
    packet_loss_rate: float = 0.0
    error_count: int = 0
    reconnect_count: int = 0
    latency_ms: float = 0.0
    temperature: Optional[float] = None
    voltage: Optional[float] = None
    message: str = ""


@dataclass
class SensorConfig:
    """Sensor configuration"""
    enabled: bool = True
    auto_start: bool = True
    hotplug_enabled: bool = True
    health_timeout_ms: int = 2000
    sample_rate_hz: float = 10.0
    priority: int = 5
    parameters: Dict[str, Any] = field(default_factory=dict)


class BaseSensor(ABC):
    """Abstract base class for all sensors"""
    
    def __init__(self, info: SensorInfo, config: SensorConfig):
        self.info = info
        self.config = config
        self.health = SensorHealth(
            state=SensorState.DISCONNECTED,
            is_healthy=False,
            last_update=0.0
        )
        self._callbacks: List[Callable] = []
        self._health_callbacks: List[Callable] = []
        self._data_queue: queue.Queue = queue.Queue(maxsize=100)
        self._lock = threading.RLock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._health_thread: Optional[threading.Thread] = None
        self._last_data_time = 0.0
        
    @abstractmethod
    def connect(self) -> bool:
        """Connect to the sensor hardware"""
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from sensor hardware"""
        pass
    
    @abstractmethod
    def read(self) -> Optional[Dict[str, Any]]:
        """Read a single sample from the sensor"""
        pass
    
    def start(self) -> bool:
        """Start the sensor data acquisition"""
        with self._lock:
            if self._running:
                return True
            
            if not self.connect():
                return False
            
            self._running = True
            self._thread = threading.Thread(target=self._acquisition_loop, daemon=True)
            self._thread.start()
            
            if self.config.hotplug_enabled:
                self._health_thread = threading.Thread(target=self._health_monitor, daemon=True)
                self._health_thread.start()
            
            self.health.state = SensorState.ACTIVE
            logger.info(f"Started sensor: {self.info.name}")
            return True
    
    def stop(self) -> None:
        """Stop the sensor data acquisition"""
        with self._lock:
            self._running = False
            
            if self._thread:
                self._thread.join(timeout=2.0)
                self._thread = None
            
            if self._health_thread:
                self._health_thread.join(timeout=1.0)
                self._health_thread = None
            
            self.disconnect()
            self.health.state = SensorState.DISCONNECTED
            logger.info(f"Stopped sensor: {self.info.name}")
    
    def register_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Register a data callback"""
        self._callbacks.append(callback)
    
    def unregister_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """Unregister a data callback"""
        if callback in self._callbacks:
            self._callbacks.remove(callback)
    
    def register_health_callback(self, callback: Callable[[SensorHealth], None]) -> None:
        """Register a health status callback"""
        self._health_callbacks.append(callback)
    
    def get_data(self, timeout: float = 0.0) -> Optional[Dict[str, Any]]:
        """Get data from queue (non-blocking if timeout=0)"""
        try:
            return self._data_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_health(self) -> SensorHealth:
        """Get current health status"""
        with self._lock:
            return self.health
    
    def _acquisition_loop(self) -> None:
        """Main data acquisition loop"""
        while self._running:
            try:
                data = self.read()
                if data:
                    data['_timestamp'] = time.time()
                    data['_sensor_name'] = self.info.name
                    data['_sensor_category'] = self.info.category.value
                    
                    self._last_data_time = time.time()
                    
                    # Add to queue
                    try:
                        self._data_queue.put_nowait(data)
                    except queue.Full:
                        pass
                    
                    # Notify callbacks
                    for callback in self._callbacks:
                        try:
                            callback(data)
                        except Exception as e:
                            logger.error(f"Callback error in {self.info.name}: {e}")
                
                # Rate limiting
                time.sleep(1.0 / self.config.sample_rate_hz)
                
            except Exception as e:
                logger.error(f"Acquisition error in {self.info.name}: {e}")
                self.health.error_count += 1
                time.sleep(0.1)
    
    def _health_monitor(self) -> None:
        """Health monitoring thread"""
        was_healthy = False
        
        while self._running:
            time.sleep(1.0)
            
            with self._lock:
                elapsed_ms = (time.time() - self._last_data_time) * 1000
                is_healthy = elapsed_ms < self.config.health_timeout_ms
                
                self.health.is_healthy = is_healthy
                self.health.last_update = time.time()
                
                if is_healthy and self.health.state != SensorState.ACTIVE:
                    self.health.state = SensorState.ACTIVE
                elif not is_healthy and self.health.state == SensorState.ACTIVE:
                    self.health.state = SensorState.DEGRADED
                    logger.warning(f"Sensor {self.info.name} health degraded")
                
                if is_healthy != was_healthy:
                    for callback in self._health_callbacks:
                        try:
                            callback(self.health)
                        except Exception as e:
                            logger.error(f"Health callback error: {e}")
                    was_healthy = is_healthy
    
    def _update_health(self, **kwargs) -> None:
        """Update health metrics"""
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self.health, key):
                    setattr(self.health, key, value)


class I2CSensor(BaseSensor):
    """Base class for I2C sensors"""
    
    def __init__(self, info: SensorInfo, config: SensorConfig):
        super().__init__(info, config)
        self._i2c_bus = None
        self._i2c_address = int(info.bus_address, 16) if info.bus_address else 0
    
    def connect(self) -> bool:
        try:
            import smbus2
            bus_num = int(self.info.device_path.replace('/dev/i2c-', ''))
            self._i2c_bus = smbus2.SMBus(bus_num)
            
            # Test connection
            try:
                self._i2c_bus.read_byte(self._i2c_address)
            except:
                pass
            
            return True
        except Exception as e:
            logger.error(f"Failed to connect to I2C sensor {self.info.name}: {e}")
            return False
    
    def disconnect(self) -> None:
        if self._i2c_bus:
            self._i2c_bus.close()
            self._i2c_bus = None


class SPISensor(BaseSensor):
    """Base class for SPI sensors"""
    
    def __init__(self, info: SensorInfo, config: SensorConfig):
        super().__init__(info, config)
        self._spi = None
    
    def connect(self) -> bool:
        try:
            import spidev
            self._spi = spidev.SpiDev()
            bus, device = 0, 0  # Parse from device_path
            self._spi.open(bus, device)
            self._spi.max_speed_hz = self.config.parameters.get('speed_hz', 1000000)
            self._spi.mode = self.config.parameters.get('mode', 0)
            return True
        except Exception as e:
            logger.error(f"Failed to connect to SPI sensor {self.info.name}: {e}")
            return False
    
    def disconnect(self) -> None:
        if self._spi:
            self._spi.close()
            self._spi = None


class UARTSensor(BaseSensor):
    """Base class for UART sensors"""
    
    def __init__(self, info: SensorInfo, config: SensorConfig):
        super().__init__(info, config)
        self._serial = None
    
    def connect(self) -> bool:
        try:
            import serial
            self._serial = serial.Serial(
                port=self.info.device_path,
                baudrate=self.config.parameters.get('baudrate', 115200),
                timeout=0.1
            )
            return True
        except Exception as e:
            logger.error(f"Failed to connect to UART sensor {self.info.name}: {e}")
            return False
    
    def disconnect(self) -> None:
        if self._serial:
            self._serial.close()
            self._serial = None


class EthernetSensor(BaseSensor):
    """Base class for Ethernet/UDP sensors (LiDAR, etc.)"""
    
    def __init__(self, info: SensorInfo, config: SensorConfig):
        super().__init__(info, config)
        self._socket = None
        self._target_ip = config.parameters.get('target_ip', '192.168.1.100')
        self._target_port = config.parameters.get('target_port', 7502)
    
    def connect(self) -> bool:
        import socket
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.settimeout(1.0)
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Ethernet sensor {self.info.name}: {e}")
            return False
    
    def disconnect(self) -> None:
        if self._socket:
            self._socket.close()
            self._socket = None


class SensorManager:
    """Central manager for all vehicle sensors"""
    
    def __init__(self, config_path: Optional[str] = None):
        self.sensors: Dict[str, BaseSensor] = {}
        self._sensor_infos: Dict[str, SensorInfo] = {}
        self._configs: Dict[str, SensorConfig] = {}
        self._callbacks: Dict[str, List[Callable]] = {}
        self._lock = threading.RLock()
        self._discovery_thread: Optional[threading.Thread] = None
        self._running = False
        self._config_path = config_path
        
        # Statistics
        self._stats = {
            'total_sensors': 0,
            'active_sensors': 0,
            'healthy_sensors': 0,
            'total_samples': 0,
            'errors': 0
        }
        
        # Load configuration
        if config_path and os.path.exists(config_path):
            self._load_config(config_path)
    
    def _load_config(self, path: str) -> None:
        """Load sensor configuration from file"""
        try:
            with open(path, 'r') as f:
                if path.endswith('.yaml') or path.endswith('.yml'):
                    config = yaml.safe_load(f)
                else:
                    config = json.load(f)
            
            for name, cfg in config.get('sensors', {}).items():
                self._configs[name] = SensorConfig(
                    enabled=cfg.get('enabled', True),
                    auto_start=cfg.get('auto_start', True),
                    hotplug_enabled=cfg.get('hotplug_enabled', True),
                    health_timeout_ms=cfg.get('health_timeout_ms', 2000),
                    sample_rate_hz=cfg.get('sample_rate_hz', 10.0),
                    priority=cfg.get('priority', 5),
                    parameters=cfg.get('parameters', {})
                )
            
            logger.info(f"Loaded configuration for {len(self._configs)} sensors")
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
    
    def register_sensor(self, sensor: BaseSensor, auto_start: bool = False) -> bool:
        """Register a sensor with the manager"""
        with self._lock:
            name = sensor.info.name
            
            if name in self.sensors:
                logger.warning(f"Sensor {name} already registered")
                return False
            
            self.sensors[name] = sensor
            self._sensor_infos[name] = sensor.info
            
            # Apply configuration if exists
            if name in self._configs:
                sensor.config = self._configs[name]
            
            # Register health callback
            sensor.register_health_callback(self._on_sensor_health_change)
            
            self._stats['total_sensors'] += 1
            logger.info(f"Registered sensor: {name} ({sensor.info.category.value})")
            
            if auto_start and sensor.config.auto_start:
                sensor.start()
            
            return True
    
    def unregister_sensor(self, name: str) -> bool:
        """Unregister and stop a sensor"""
        with self._lock:
            if name not in self.sensors:
                return False
            
            sensor = self.sensors[name]
            sensor.stop()
            del self.sensors[name]
            del self._sensor_infos[name]
            
            self._stats['total_sensors'] -= 1
            logger.info(f"Unregistered sensor: {name}")
            return True
    
    def get_sensor(self, name: str) -> Optional[BaseSensor]:
        """Get a sensor by name"""
        return self.sensors.get(name)
    
    def get_sensors_by_category(self, category: SensorCategory) -> List[BaseSensor]:
        """Get all sensors of a specific category"""
        return [s for s in self.sensors.values() if s.info.category == category]
    
    def start_all(self) -> None:
        """Start all registered sensors"""
        for sensor in self.sensors.values():
            if sensor.config.enabled:
                sensor.start()
    
    def stop_all(self) -> None:
        """Stop all registered sensors"""
        for sensor in self.sensors.values():
            sensor.stop()
    
    def start_discovery(self, interval_sec: float = 5.0) -> None:
        """Start automatic sensor discovery"""
        self._running = True
        self._discovery_thread = threading.Thread(
            target=self._discovery_loop,
            args=(interval_sec,),
            daemon=True
        )
        self._discovery_thread.start()
        logger.info("Started sensor discovery")
    
    def stop_discovery(self) -> None:
        """Stop automatic sensor discovery"""
        self._running = False
        if self._discovery_thread:
            self._discovery_thread.join(timeout=2.0)
    
    def _discovery_loop(self, interval_sec: float) -> None:
        """Sensor discovery loop"""
        while self._running:
            self._discover_i2c_sensors()
            self._discover_spi_sensors()
            self._discover_uart_sensors()
            time.sleep(interval_sec)
    
    def _discover_i2c_sensors(self) -> None:
        """Discover I2C sensors on all buses"""
        try:
            import smbus2
            
            # Known I2C device signatures
            signatures = {
                0x1C: ('LIS3MDL', SensorCategory.MAGNETOMETER),
                0x1E: ('LIS3MDL_ALT', SensorCategory.MAGNETOMETER),
                0x20: ('RM3100', SensorCategory.MAGNETOMETER),
                0x21: ('RM3100_ALT', SensorCategory.MAGNETOMETER),
                0x76: ('MS5611', SensorCategory.BAROMETER),
                0x77: ('MS5611_ALT', SensorCategory.BAROMETER),
                0x68: ('ICM20948', SensorCategory.IMU),
                0x69: ('ICM20948_ALT', SensorCategory.IMU),
            }
            
            # Scan common I2C buses
            for bus_num in range(0, 10):
                bus_path = f'/dev/i2c-{bus_num}'
                if not os.path.exists(bus_path):
                    continue
                
                try:
                    bus = smbus2.SMBus(bus_num)
                    for addr in range(0x03, 0x77):
                        try:
                            bus.read_byte(addr)
                            if addr in signatures:
                                name, category = signatures[addr]
                                unique_name = f"{name}_{bus_num}_{addr:02X}"
                                
                                if unique_name not in self.sensors:
                                    info = SensorInfo(
                                        name=unique_name,
                                        category=category,
                                        driver_type=name.lower(),
                                        device_path=bus_path,
                                        bus_type='i2c',
                                        bus_address=f'0x{addr:02X}'
                                    )
                                    logger.info(f"Discovered I2C sensor: {unique_name}")
                        except:
                            pass
                    bus.close()
                except Exception as e:
                    pass
        except ImportError:
            pass
    
    def _discover_spi_sensors(self) -> None:
        """Discover SPI sensors"""
        # SPI sensors typically require chip select handling
        # Implementation would scan /dev/spidev* devices
        pass
    
    def _discover_uart_sensors(self) -> None:
        """Discover UART sensors"""
        import glob
        
        uart_patterns = ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyS*']
        for pattern in uart_patterns:
            for device in glob.glob(pattern):
                # Try to identify sensor by querying it
                pass
    
    def _on_sensor_health_change(self, health: SensorHealth) -> None:
        """Handle sensor health changes"""
        with self._lock:
            healthy_count = sum(1 for s in self.sensors.values() if s.health.is_healthy)
            self._stats['healthy_sensors'] = healthy_count
            
            active_count = sum(1 for s in self.sensors.values() 
                             if s.health.state in [SensorState.ACTIVE, SensorState.CONNECTED])
            self._stats['active_sensors'] = active_count
    
    def get_stats(self) -> Dict[str, Any]:
        """Get manager statistics"""
        with self._lock:
            return self._stats.copy()
    
    def get_all_health(self) -> Dict[str, SensorHealth]:
        """Get health status for all sensors"""
        return {name: sensor.get_health() for name, sensor in self.sensors.items()}
    
    def calibrate_all(self) -> None:
        """Trigger calibration for all applicable sensors"""
        for sensor in self.sensors.values():
            if hasattr(sensor, 'calibrate'):
                try:
                    sensor.calibrate()
                except Exception as e:
                    logger.error(f"Calibration failed for {sensor.info.name}: {e}")
    
    def save_config(self, path: str) -> None:
        """Save current configuration to file"""
        config = {
            'sensors': {}
        }
        
        for name, sensor in self.sensors.items():
            config['sensors'][name] = {
                'enabled': sensor.config.enabled,
                'auto_start': sensor.config.auto_start,
                'hotplug_enabled': sensor.config.hotplug_enabled,
                'health_timeout_ms': sensor.config.health_timeout_ms,
                'sample_rate_hz': sensor.config.sample_rate_hz,
                'priority': sensor.config.priority,
                'parameters': sensor.config.parameters
            }
        
        with open(path, 'w') as f:
            if path.endswith('.yaml') or path.endswith('.yml'):
                yaml.dump(config, f)
            else:
                json.dump(config, f, indent=2)
        
        logger.info(f"Saved configuration to {path}")
    
    def shutdown(self) -> None:
        """Shutdown the sensor manager"""
        logger.info("Shutting down sensor manager...")
        self.stop_discovery()
        self.stop_all()
        
        # Save configuration if path was provided
        if self._config_path:
            try:
                self.save_config(self._config_path)
            except Exception as e:
                logger.error(f"Failed to save config: {e}")


# Global sensor manager instance
_global_manager: Optional[SensorManager] = None


def get_sensor_manager(config_path: Optional[str] = None) -> SensorManager:
    """Get or create the global sensor manager"""
    global _global_manager
    if _global_manager is None:
        _global_manager = SensorManager(config_path)
    return _global_manager


# Example usage and test
if __name__ == '__main__':
    # Create sensor manager
    manager = SensorManager()
    
    # Example: Create and register a mock sensor
    class MockSensor(BaseSensor):
        def connect(self) -> bool:
            return True
        
        def disconnect(self) -> None:
            pass
        
        def read(self) -> Optional[Dict[str, Any]]:
            import random
            return {
                'value': random.random() * 100,
                'unit': 'mock_units'
            }
    
    mock_info = SensorInfo(
        name='mock_sensor_1',
        category=SensorCategory.CUSTOM,
        driver_type='mock',
        device_path='/dev/null',
        bus_type='virtual'
    )
    
    mock_sensor = MockSensor(mock_info, SensorConfig(sample_rate_hz=5.0))
    manager.register_sensor(mock_sensor, auto_start=True)
    
    # Run for a while
    try:
        print("Sensor manager running. Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
            stats = manager.get_stats()
            print(f"Stats: {stats}")
    except KeyboardInterrupt:
        pass
    finally:
        manager.shutdown()
