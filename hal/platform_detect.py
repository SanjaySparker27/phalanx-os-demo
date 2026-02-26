#!/usr/bin/env python3
"""
Platform Detection for Cule OS
Automatically detects hardware platform: NVIDIA Jetson, Raspberry Pi, x86
Provides platform-specific configuration and capabilities
"""

import os
import re
import json
import logging
import subprocess
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from pathlib import Path

logger = logging.getLogger('PlatformDetect')


class PlatformType(Enum):
    """Supported hardware platforms"""
    UNKNOWN = auto()
    NVIDIA_JETSON_AGX_ORIN = auto()
    NVIDIA_JETSON_XAVIER = auto()
    NVIDIA_JETSON_NANO = auto()
    NVIDIA_JETSON_TX2 = auto()
    RASPBERRY_PI_4 = auto()
    RASPBERRY_PI_5 = auto()
    RASPBERRY_PI_3 = auto()
    X86_64 = auto()
    ARM_GENERIC = auto()


class BusType(Enum):
    """Hardware bus types"""
    I2C = "i2c"
    SPI = "spi"
    UART = "uart"
    CAN = "can"
    USB = "usb"
    PCIE = "pcie"
    ETHERNET = "ethernet"


@dataclass
class HardwareInfo:
    """Hardware platform information"""
    platform: PlatformType
    cpu_model: str
    cpu_cores: int
    ram_mb: int
    gpu_model: Optional[str] = None
    gpu_compute: Optional[str] = None
    has_cuda: bool = False
    has_tensorrt: bool = False
    has_vpi: bool = False
    kernel_version: str = ""
    board_revision: Optional[str] = None


@dataclass
class BusInfo:
    """Bus interface information"""
    bus_type: BusType
    device_path: str
    bus_number: int
    is_available: bool = True
    description: str = ""


@dataclass
class PlatformCapabilities:
    """Platform-specific capabilities"""
    max_cameras: int = 0
    camera_interfaces: List[str] = field(default_factory=list)
    i2c_buses: List[int] = field(default_factory=list)
    spi_buses: List[int] = field(default_factory=list)
    uart_ports: List[str] = field(default_factory=list)
    can_interfaces: List[str] = field(default_factory=list)
    ethernet_ports: List[str] = field(default_factory=list)
    gpu_acceleration: bool = False
    ai_acceleration: bool = False
    recommended_video_enc: str = "software"
    recommended_inference_engine: str = "cpu"


class PlatformDetector:
    """Hardware platform detector"""
    
    def __init__(self):
        self.info: Optional[HardwareInfo] = None
        self.capabilities: Optional[PlatformCapabilities] = None
        self.buses: List[BusInfo] = []
        self._detected = False
    
    def detect(self) -> HardwareInfo:
        """Detect hardware platform"""
        if self._detected and self.info:
            return self.info
        
        logger.info("Detecting hardware platform...")
        
        # Check for Jetson
        jetson_model = self._detect_jetson()
        if jetson_model:
            self.info = self._get_jetson_info(jetson_model)
        # Check for Raspberry Pi
        elif self._detect_raspberry_pi():
            self.info = self._get_rpi_info()
        # Check for x86
        elif self._detect_x86():
            self.info = self._get_x86_info()
        else:
            self.info = HardwareInfo(
                platform=PlatformType.UNKNOWN,
                cpu_model="Unknown",
                cpu_cores=os.cpu_count() or 1,
                ram_mb=self._get_ram_mb()
            )
        
        # Detect buses
        self.buses = self._detect_buses()
        
        # Determine capabilities
        self.capabilities = self._determine_capabilities()
        
        self._detected = True
        
        logger.info(f"Detected platform: {self.info.platform.name}")
        logger.info(f"CPU: {self.info.cpu_model} ({self.info.cpu_cores} cores)")
        logger.info(f"RAM: {self.info.ram_mb} MB")
        if self.info.gpu_model:
            logger.info(f"GPU: {self.info.gpu_model}")
        
        return self.info
    
    def _detect_jetson(self) -> Optional[str]:
        """Detect NVIDIA Jetson platform"""
        try:
            # Check for Jetson-specific files
            if os.path.exists('/etc/nv_tegra_release'):
                # Read model from device tree
                model_path = Path('/proc/device-tree/model')
                if model_path.exists():
                    model = model_path.read_text().strip().strip('\x00')
                    return model
                
                # Alternative: check via nvgetty or tegrastats
                result = subprocess.run(['tegrastats', '--version'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    return "NVIDIA Jetson (tegrastats)"
        except Exception as e:
            pass
        
        return None
    
    def _get_jetson_info(self, model: str) -> HardwareInfo:
        """Get Jetson-specific hardware info"""
        # Parse model string
        if 'ORIN' in model.upper() or 'P3701' in model:
            platform = PlatformType.NVIDIA_JETSON_AGX_ORIN
            gpu = "Ampere GA10B"
            gpu_compute = "8.7"
            has_tensorrt = True
            has_vpi = True
        elif 'XAVIER' in model.upper():
            platform = PlatformType.NVIDIA_JETSON_XAVIER
            gpu = "Volta"
            gpu_compute = "7.2"
            has_tensorrt = True
            has_vpi = True
        elif 'NANO' in model.upper():
            platform = PlatformType.NVIDIA_JETSON_NANO
            gpu = "Maxwell"
            gpu_compute = "5.3"
            has_tensorrt = True
            has_vpi = False
        elif 'TX2' in model.upper():
            platform = PlatformType.NVIDIA_JETSON_TX2
            gpu = "Pascal"
            gpu_compute = "6.2"
            has_tensorrt = True
            has_vpi = False
        else:
            platform = PlatformType.NVIDIA_JETSON_AGX_ORIN
            gpu = "Unknown"
            gpu_compute = None
            has_tensorrt = False
            has_vpi = False
        
        # Get CPU info
        cpu_model = self._get_cpu_model()
        cpu_cores = os.cpu_count() or 8
        ram_mb = self._get_ram_mb()
        
        return HardwareInfo(
            platform=platform,
            cpu_model=cpu_model,
            cpu_cores=cpu_cores,
            ram_mb=ram_mb,
            gpu_model=gpu,
            gpu_compute=gpu_compute,
            has_cuda=True,
            has_tensorrt=has_tensorrt,
            has_vpi=has_vpi,
            kernel_version=self._get_kernel_version(),
            board_revision=self._get_jetson_board_revision()
        )
    
    def _detect_raspberry_pi(self) -> bool:
        """Detect Raspberry Pi"""
        try:
            model_path = Path('/proc/device-tree/model')
            if model_path.exists():
                model = model_path.read_text().strip().strip('\x00')
                return 'Raspberry Pi' in model
            
            # Alternative: check cpuinfo
            with open('/proc/cpuinfo', 'r') as f:
                content = f.read()
                return 'Raspberry Pi' in content or 'BCM' in content
        except:
            pass
        return False
    
    def _get_rpi_info(self) -> HardwareInfo:
        """Get Raspberry Pi hardware info"""
        model = "Raspberry Pi"
        
        try:
            model_path = Path('/proc/device-tree/model')
            if model_path.exists():
                model = model_path.read_text().strip().strip('\x00')
        except:
            pass
        
        # Determine specific model
        if '5' in model:
            platform = PlatformType.RASPBERRY_PI_5
            gpu = "VideoCore VII"
        elif '4' in model:
            platform = PlatformType.RASPBERRY_PI_4
            gpu = "VideoCore VI"
        elif '3' in model:
            platform = PlatformType.RASPBERRY_PI_3
            gpu = "VideoCore IV"
        else:
            platform = PlatformType.RASPBERRY_PI_4
            gpu = "VideoCore"
        
        # Get revision
        revision = None
        try:
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if 'Revision' in line:
                        revision = line.split(':')[1].strip()
                        break
        except:
            pass
        
        return HardwareInfo(
            platform=platform,
            cpu_model=self._get_cpu_model(),
            cpu_cores=os.cpu_count() or 4,
            ram_mb=self._get_ram_mb(),
            gpu_model=gpu,
            kernel_version=self._get_kernel_version(),
            board_revision=revision
        )
    
    def _detect_x86(self) -> bool:
        """Detect x86 platform"""
        try:
            with open('/proc/cpuinfo', 'r') as f:
                content = f.read()
                return 'Intel' in content or 'AMD' in content or 'x86' in content
        except:
            pass
        return False
    
    def _get_x86_info(self) -> HardwareInfo:
        """Get x86 hardware info"""
        cpu_model = self._get_cpu_model()
        
        # Check for NVIDIA GPU
        gpu_model = None
        has_cuda = False
        try:
            result = subprocess.run(['nvidia-smi', '-L'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                if lines:
                    gpu_model = lines[0].split(':')[1].split('(')[0].strip()
                    has_cuda = True
        except:
            pass
        
        # Check for Intel GPU
        if not gpu_model:
            try:
                result = subprocess.run(['lspci'], capture_output=True, text=True, timeout=5)
                if 'VGA compatible controller: Intel' in result.stdout:
                    gpu_model = "Intel Integrated"
            except:
                pass
        
        return HardwareInfo(
            platform=PlatformType.X86_64,
            cpu_model=cpu_model,
            cpu_cores=os.cpu_count() or 4,
            ram_mb=self._get_ram_mb(),
            gpu_model=gpu_model,
            has_cuda=has_cuda,
            has_tensorrt=has_cuda,  # Assume TensorRT if CUDA
            kernel_version=self._get_kernel_version()
        )
    
    def _get_cpu_model(self) -> str:
        """Get CPU model name"""
        try:
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if 'model name' in line or 'Model' in line:
                        return line.split(':')[1].strip()
        except:
            pass
        return "Unknown"
    
    def _get_ram_mb(self) -> int:
        """Get total RAM in MB"""
        try:
            with open('/proc/meminfo', 'r') as f:
                for line in f:
                    if 'MemTotal' in line:
                        kb = int(line.split()[1])
                        return kb // 1024
        except:
            pass
        return 0
    
    def _get_kernel_version(self) -> str:
        """Get kernel version"""
        try:
            result = subprocess.run(['uname', '-r'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                return result.stdout.strip()
        except:
            pass
        return ""
    
    def _get_jetson_board_revision(self) -> Optional[str]:
        """Get Jetson board revision"""
        try:
            # Check ODMDATA or EEPROM
            if os.path.exists('/sys/module/tegra_fuse/parameters/tegra_chip_id'):
                with open('/sys/module/tegra_fuse/parameters/tegra_chip_id', 'r') as f:
                    return f.read().strip()
        except:
            pass
        return None
    
    def _detect_buses(self) -> List[BusInfo]:
        """Detect available hardware buses"""
        buses = []
        
        # Detect I2C buses
        for i in range(0, 20):
            path = f'/dev/i2c-{i}'
            if os.path.exists(path):
                buses.append(BusInfo(
                    bus_type=BusType.I2C,
                    device_path=path,
                    bus_number=i,
                    description=f"I2C Bus {i}"
                ))
        
        # Detect SPI buses
        for i in range(0, 10):
            for j in range(0, 2):
                path = f'/dev/spidev{i}.{j}'
                if os.path.exists(path):
                    buses.append(BusInfo(
                        bus_type=BusType.SPI,
                        device_path=path,
                        bus_number=i,
                        description=f"SPI Bus {i}.{j}"
                    ))
        
        # Detect UART ports
        import glob
        for pattern in ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyTHS*', '/dev/ttyS*']:
            for path in glob.glob(pattern):
                buses.append(BusInfo(
                    bus_type=BusType.UART,
                    device_path=path,
                    bus_number=0,
                    description=f"UART {os.path.basename(path)}"
                ))
        
        # Detect CAN interfaces
        try:
            import glob
            for path in glob.glob('/sys/class/net/can*'):
                iface = os.path.basename(path)
                buses.append(BusInfo(
                    bus_type=BusType.CAN,
                    device_path=f'/dev/{iface}',
                    bus_number=0,
                    description=f"CAN Interface {iface}"
                ))
        except:
            pass
        
        # Detect Ethernet
        try:
            import glob
            for path in glob.glob('/sys/class/net/eth*'):
                iface = os.path.basename(path)
                buses.append(BusInfo(
                    bus_type=BusType.ETHERNET,
                    device_path=f'/dev/{iface}',
                    bus_number=0,
                    description=f"Ethernet {iface}"
                ))
        except:
            pass
        
        return buses
    
    def _determine_capabilities(self) -> PlatformCapabilities:
        """Determine platform capabilities"""
        caps = PlatformCapabilities()
        
        if not self.info:
            return caps
        
        platform = self.info.platform
        
        # Camera interfaces
        if 'JETSON' in platform.name:
            caps.max_cameras = 6
            caps.camera_interfaces = ['csi', 'usb', 'gmsl']
            caps.gpu_acceleration = True
            caps.ai_acceleration = True
            caps.recommended_video_enc = 'nvenc'
            caps.recommended_inference_engine = 'tensorrt'
        elif 'RASPBERRY_PI' in platform.name:
            caps.max_cameras = 1
            caps.camera_interfaces = ['csi', 'usb']
            caps.gpu_acceleration = True
            caps.recommended_video_enc = 'v4l2m2m'
            caps.recommended_inference_engine = 'onnx'
        elif platform == PlatformType.X86_64:
            caps.max_cameras = 4
            caps.camera_interfaces = ['usb', 'gige']
            caps.gpu_acceleration = self.info.has_cuda
            caps.ai_acceleration = self.info.has_cuda
            caps.recommended_video_enc = 'nvenc' if self.info.has_cuda else 'x264'
            caps.recommended_inference_engine = 'tensorrt' if self.info.has_tensorrt else 'onnx'
        
        # I2C buses
        caps.i2c_buses = [b.bus_number for b in self.buses if b.bus_type == BusType.I2C]
        
        # SPI buses
        caps.spi_buses = list(set([b.bus_number for b in self.buses if b.bus_type == BusType.SPI]))
        
        # UART ports
        caps.uart_ports = [b.device_path for b in self.buses if b.bus_type == BusType.UART]
        
        # CAN interfaces
        caps.can_interfaces = [b.device_path for b in self.buses if b.bus_type == BusType.CAN]
        
        # Ethernet
        caps.ethernet_ports = [b.device_path for b in self.buses if b.bus_type == BusType.ETHERNET]
        
        return caps
    
    def get_platform_type(self) -> PlatformType:
        """Get detected platform type"""
        if not self._detected:
            self.detect()
        return self.info.platform if self.info else PlatformType.UNKNOWN
    
    def is_jetson(self) -> bool:
        """Check if running on NVIDIA Jetson"""
        platform = self.get_platform_type()
        return 'JETSON' in platform.name
    
    def is_raspberry_pi(self) -> bool:
        """Check if running on Raspberry Pi"""
        platform = self.get_platform_type()
        return 'RASPBERRY_PI' in platform.name
    
    def is_x86(self) -> bool:
        """Check if running on x86"""
        return self.get_platform_type() == PlatformType.X86_64
    
    def get_optimal_thread_count(self) -> int:
        """Get optimal thread count for this platform"""
        if not self.info:
            return 4
        
        # Leave some cores for other tasks
        return max(2, self.info.cpu_cores - 2)
    
    def get_i2c_bus(self, preferred: Optional[int] = None) -> str:
        """Get preferred I2C bus path"""
        if not self.capabilities:
            self.detect()
        
        if preferred is not None and preferred in self.capabilities.i2c_buses:
            return f'/dev/i2c-{preferred}'
        
        if self.capabilities.i2c_buses:
            return f'/dev/i2c-{self.capabilities.i2c_buses[0]}'
        
        return '/dev/i2c-1'  # Default fallback
    
    def get_spi_bus(self) -> str:
        """Get preferred SPI bus path"""
        if not self.capabilities:
            self.detect()
        
        if self.capabilities.spi_buses:
            return f'/dev/spidev{self.capabilities.spi_buses[0]}.0'
        
        return '/dev/spidev0.0'
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        if not self._detected:
            self.detect()
        
        return {
            'platform': self.info.platform.name if self.info else 'UNKNOWN',
            'hardware': {
                'cpu_model': self.info.cpu_model if self.info else 'Unknown',
                'cpu_cores': self.info.cpu_cores if self.info else 0,
                'ram_mb': self.info.ram_mb if self.info else 0,
                'gpu_model': self.info.gpu_model if self.info else None,
                'gpu_compute': self.info.gpu_compute if self.info else None,
                'has_cuda': self.info.has_cuda if self.info else False,
                'has_tensorrt': self.info.has_tensorrt if self.info else False,
                'kernel_version': self.info.kernel_version if self.info else '',
            },
            'capabilities': {
                'max_cameras': self.capabilities.max_cameras if self.capabilities else 0,
                'camera_interfaces': self.capabilities.camera_interfaces if self.capabilities else [],
                'i2c_buses': self.capabilities.i2c_buses if self.capabilities else [],
                'spi_buses': self.capabilities.spi_buses if self.capabilities else [],
                'uart_ports': self.capabilities.uart_ports if self.capabilities else [],
                'can_interfaces': self.capabilities.can_interfaces if self.capabilities else [],
                'gpu_acceleration': self.capabilities.gpu_acceleration if self.capabilities else False,
                'ai_acceleration': self.capabilities.ai_acceleration if self.capabilities else False,
            },
            'buses': [
                {
                    'type': b.bus_type.value,
                    'path': b.device_path,
                    'description': b.description
                }
                for b in self.buses
            ]
        }
    
    def print_summary(self) -> None:
        """Print platform summary"""
        if not self._detected:
            self.detect()
        
        print("=" * 60)
        print("PLATFORM DETECTION SUMMARY")
        print("=" * 60)
        print(f"Platform: {self.info.platform.name}")
        print(f"CPU: {self.info.cpu_model}")
        print(f"Cores: {self.info.cpu_cores}")
        print(f"RAM: {self.info.ram_mb} MB")
        
        if self.info.gpu_model:
            print(f"GPU: {self.info.gpu_model}")
        if self.info.gpu_compute:
            print(f"GPU Compute: {self.info.gpu_compute}")
        
        print(f"\nKernel: {self.info.kernel_version}")
        
        print(f"\nCapabilities:")
        print(f"  Max Cameras: {self.capabilities.max_cameras}")
        print(f"  Camera Interfaces: {', '.join(self.capabilities.camera_interfaces)}")
        print(f"  GPU Acceleration: {'Yes' if self.capabilities.gpu_acceleration else 'No'}")
        print(f"  AI Acceleration: {'Yes' if self.capabilities.ai_acceleration else 'No'}")
        print(f"  Video Encoder: {self.capabilities.recommended_video_enc}")
        print(f"  Inference Engine: {self.capabilities.recommended_inference_engine}")
        
        print(f"\nAvailable Buses:")
        print(f"  I2C: {self.capabilities.i2c_buses}")
        print(f"  SPI: {self.capabilities.spi_buses}")
        print(f"  UART: {len(self.capabilities.uart_ports)} ports")
        print(f"  CAN: {self.capabilities.can_interfaces}")
        print(f"  Ethernet: {self.capabilities.ethernet_ports}")
        print("=" * 60)


# Global detector instance
_global_detector: Optional[PlatformDetector] = None


def get_platform_detector() -> PlatformDetector:
    """Get or create the global platform detector"""
    global _global_detector
    if _global_detector is None:
        _global_detector = PlatformDetector()
    return _global_detector


def get_platform_type() -> PlatformType:
    """Quick function to get platform type"""
    return get_platform_detector().get_platform_type()


# Example usage
if __name__ == '__main__':
    detector = PlatformDetector()
    detector.detect()
    detector.print_summary()
    
    # Output JSON
    print("\nJSON Output:")
    print(json.dumps(detector.to_dict(), indent=2))
