# Jetson Installation Guide

Complete step-by-step installation guide for setting up NVIDIA Jetson as a companion computer for the ZEX ATHENA H743 PRO flight controller.

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Jetson OS Installation](#jetson-os-installation)
- [System Configuration](#system-configuration)
- [Companion Computer Setup](#companion-computer-setup)
- [MAVLink Communication Setup](#mavlink-communication-setup)
- [Cule OS Installation](#cule-os-installation)
- [Verification](#verification)

---

## Hardware Requirements

### Required Components

| Component | Specification | Purpose |
|-----------|--------------|---------|
| **Jetson Device** | AGX Orin / Xavier NX / Nano | Companion computer |
| **MicroSD Card** | 128GB+ (U3/A2 rated) | OS storage |
| **Power Supply** | 5V 4A (barrel jack) | Jetson power |
| **USB-C Cable** | High-quality data cable | Flashing/debug |
| **UART Cable** | FTDI USB-to-TTL 3.3V | MAVLink telemetry |
| **Ethernet Cable** | Cat5e/Cat6 | Network setup |
| **WiFi Module** | Intel AX200 or compatible | Wireless connection |

### Optional Components

| Component | Purpose |
|-----------|---------|
| **M.2 NVMe SSD** | Fast storage expansion |
| **Cooling Fan** | Active cooling for sustained loads |
| **GPS Module** | u-blox ZED-F9P for RTK |
| **4G/5G Modem** | Cellular telemetry backup |

---

## Jetson OS Installation

### Step 1: Download JetPack SDK

1. Visit [NVIDIA JetPack](https://developer.nvidia.com/embedded/jetpack)
2. Download JetPack 6.0 (or latest stable) for your Jetson model
3. Download SDK Manager for your host PC

### Step 2: Flash Jetson OS

#### Method A: SDK Manager (Recommended)

```bash
# On host Ubuntu PC
sudo apt install ./sdkmanager_[version]-[build]_amd64.deb
sdkmanager
```

1. Log in with NVIDIA developer account
2. Select your Jetson hardware
3. Select JetPack version (6.0 recommended)
4. Connect Jetson via USB-C in recovery mode:
   - Power off Jetson
   - Connect USB-C to host PC
   - Hold RECOVERY button, press POWER, release RECOVERY
5. Follow SDK Manager prompts to flash OS

#### Method B: SD Card Image (Jetson Nano/NX)

```bash
# Download image from https://developer.nvidia.com/jetson-nano-sd-card-image
# Flash using balenaEtcher or dd
sudo dd if=jetson-image.img of=/dev/sdX bs=1M status=progress
```

### Step 3: Initial Boot Configuration

1. Insert SD card (if using) or power on flashed Jetson
2. Complete initial setup wizard:
   - Accept license
   - Select language/locale
   - Create user account (recommended: `cule`)
   - Connect to WiFi or ethernet

### Step 4: System Update

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git wget curl nano htop
```

---

## System Configuration

### Step 1: Configure Power Mode

```bash
# Check available power modes
sudo nvpmodel -q

# Set maximum performance mode (Jetson AGX Orin)
sudo nvpmodel -m 0

# Apply jetson_clocks for maximum performance
sudo jetson_clocks

# Make persistent
sudo systemctl enable nvpmodel
```

### Step 2: Configure Swap (if needed)

```bash
# Check current swap
free -h

# Create 8GB swap file
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make persistent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Step 3: Enable UART/Serial Ports

```bash
# Check available UARTs
ls /dev/ttyTHS*

# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Configure UART permissions
sudo chmod 666 /dev/ttyTHS0
sudo chmod 666 /dev/ttyTHS1
```

### Step 4: Enable I2C and SPI (for sensors)

```bash
# Enable I2C
sudo apt install -y i2c-tools
sudo usermod -a -G i2c $USER

# Test I2C
sudo i2cdetect -y -r 0
sudo i2cdetect -y -r 1

# Enable SPI (if needed)
sudo /opt/nvidia/jetson-io/jetson-io.py
```

### Step 5: Configure Network

#### Static IP (Recommended for drone)

```bash
# Edit netplan configuration
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

```bash
sudo netplan apply
```

#### WiFi Configuration

```bash
# Using nmcli
nmcli device wifi list
nmcli device wifi connect "SSID" password "password"

# Set static IP for WiFi
nmcli connection modify "SSID" ipv4.addresses 192.168.1.101/24
nmcli connection modify "SSID" ipv4.gateway 192.168.1.1
nmcli connection modify "SSID" ipv4.dns "8.8.8.8,8.8.4.4"
nmcli connection modify "SSID" ipv4.method manual
nmcli connection up "SSID"
```

---

## Companion Computer Setup

### Step 1: Install Required Packages

```bash
# Install dependencies
sudo apt install -y \
    python3-pip python3-venv python3-dev \
    libopencv-dev python3-opencv \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    cmake build-essential pkg-config \
    libjpeg-dev libtiff-dev libpng-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libgtk-3-dev libatlas-base-dev gfortran \
    can-utils v4l-utils

# Upgrade pip
pip3 install --upgrade pip
```

### Step 2: Install MAVProxy/MAVLink Tools

```bash
# Install MAVProxy
sudo pip3 install MAVProxy

# Create MAVProxy startup script
cat > ~/start_mavproxy.sh << 'EOF'
#!/bin/bash
# MAVProxy connection to flight controller
mavproxy.py --master=/dev/ttyTHS1 --baudrate 921600 \
    --out=tcpin:0.0.0.0:14550 \
    --out=tcpin:0.0.0.0:14551 \
    --out=udp:192.168.1.100:14550 \
    --aircraft=MyDrone
EOF
chmod +x ~/start_mavproxy.sh
```

### Step 3: Install DroneKit

```bash
pip3 install dronekit dronekit-sitl
```

### Step 4: Install ROS2 (Optional but Recommended)

```bash
# Add ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install colcon
sudo apt install -y python3-colcon-common-extensions
```

---

## MAVLink Communication Setup

### Step 1: Hardware Connection

Connect Jetson to ATHENA H743 PRO flight controller:

```
Jetson                    ATHENA H743 PRO
------                    ---------------
GND    ---------------->  GND (Telem port)
TXD    ---------------->  RX (Telem2)
RXD    ---------------->  TX (Telem2)
5V     ---------------->  5V (optional, if powering FC)
```

**UART Mapping:**
| Jetson UART | Device | Use |
|-------------|--------|-----|
| UART0 | /dev/ttyTHS0 | Debug console |
| UART1 | /dev/ttyTHS1 | MAVLink to FC |
| UART2 | /dev/ttyTHS2 | GPS module |

### Step 2: Configure UART Parameters

```bash
# Check which UART is connected
ls -la /dev/ttyTHS*

# Set baud rate for MAVLink (921600 recommended)
stty -F /dev/ttyTHS1 921600 raw -echo

# Test connection
mavlink_shell.py /dev/ttyTHS1 --baudrate 921600
```

### Step 3: Configure Flight Controller Serial Port

In Mission Planner or QGroundControl:

```
SERIAL2_PROTOCOL = 2    # MAVLink2
SERIAL2_BAUD = 921      # 921600 baud
```

### Step 4: Test MAVLink Connection

```bash
# Terminal 1: Start MAVProxy
mavproxy.py --master=/dev/ttyTHS1 --baudrate 921600

# Terminal 2: Test with MAVLink messages
# Should see heartbeat messages
rostopic echo /mavros/state
```

---

## Cule OS Installation

### Step 1: Download Cule OS

```bash
# Create installation directory
mkdir -p ~/cule-os && cd ~/cule-os

# Download Cule OS for Jetson
wget https://cule-os.io/releases/v1.0/cule-os-jetson.run
chmod +x cule-os-jetson.run
```

### Step 2: Install Cule OS

```bash
# Run installer
sudo ./cule-os-jetson.run --install

# Follow interactive prompts:
# 1. Accept license
# 2. Select installation path (/opt/cule-os)
# 3. Configure MAVLink endpoint
# 4. Enable auto-start services
```

### Step 3: Configure Cule OS

```bash
# Edit configuration
sudo nano /opt/cule-os/config/cule-os.conf
```

```ini
# Cule OS Configuration for Jetson
[system]
platform = jetson-agx-orin
log_level = info
data_dir = /var/lib/cule-os

[mavlink]
device = /dev/ttyTHS1
baudrate = 921600
protocol = 2.0
heartbeat_rate = 1.0

[agents]
enabled = perception,planning,control,communication
swarm_enabled = false

[perception]
camera_enabled = true
camera_source = /dev/video0
inference_rate = 30
model_path = /opt/cule-os/models/yolov8n.onnx

[planning]
mpc_horizon = 20
mpc_dt = 0.05
obstacle_avoidance = true

[control]
control_rate = 1000
cbf_enabled = true

[communication]
telemetry_rate = 10
video_stream_enabled = true
```

### Step 4: Start Cule OS Services

```bash
# Start all services
sudo systemctl start cule-kernel
sudo systemctl start cule-agent
sudo systemctl start cule-perception
sudo systemctl start cule-planning
sudo systemctl start cule-control
sudo systemctl start cule-communication
sudo systemctl start cule-dashboard

# Enable auto-start
sudo systemctl enable cule-kernel cule-agent cule-perception cule-planning cule-control cule-communication cule-dashboard
```

---

## Verification

### Step 1: Verify Jetson Health

```bash
# Check system resources
htop

# Check GPU status
nvidia-smi

# Check temperature
cat /sys/class/thermal/thermal_zone*/temp

# Jetson-specific monitoring
tegrastats
```

### Step 2: Verify MAVLink Connection

```bash
# Check serial connection
ls -la /dev/ttyTHS1

# Test with MAVProxy
mavproxy.py --master=/dev/ttyTHS1 --baudrate 921600 --console

# In MAVProxy console, type:
# - mode      (should show current flight mode)
# - arm check (should show pre-arm checks)
# - param show FRAME_CLASS
```

### Step 3: Verify Cule OS Services

```bash
# Check service status
cule-status

# Or check individual services
sudo systemctl status cule-agent
sudo systemctl status cule-perception

# View logs
sudo journalctl -u cule-agent -f
```

### Step 4: Test Dashboard

1. Open browser on Jetson: `http://localhost:8080`
2. Or from another computer: `http://<jetson-ip>:8080`
3. Verify:
   - System status shows "Online"
   - All agents show "Active"
   - Telemetry data is streaming
   - Video feed is visible (if camera connected)

---

## Troubleshooting

### Jetson Won't Boot
- Check power supply (minimum 5V 4A)
- Verify SD card is properly inserted
- Try recovery mode and re-flash

### MAVLink Not Connecting
- Verify baud rate matches on both ends
- Check TX/RX are not reversed
- Verify flight controller serial port is enabled
- Test with oscilloscope/logic analyzer

### Cule OS Services Failing
- Check logs: `sudo journalctl -u cule-agent -n 100`
- Verify configuration file syntax
- Ensure MAVLink endpoint is accessible

### High CPU Temperature
- Verify cooling fan is working
- Reduce power mode: `sudo nvpmodel -m 1`
- Check for runaway processes

---

## Next Steps

1. **Hardware Wiring**: Follow the [Hardware Wiring Guide](../hardware/wiring-diagrams.md)
2. **Configuration**: Set up [ArduPilot Parameters](../configuration/ardupilot-config.md)
3. **Calibration**: Complete all [Calibration Procedures](../calibration/calibration-procedures.md)
4. **First Flight**: Follow the [First Flight Checklist](../operations/first-flight-checklist.md)

---

## References

- [NVIDIA Jetson Documentation](https://developer.nvidia.com/embedded/documentation)
- [ArduPilot Companion Computers](https://ardupilot.org/dev/docs/companion-computers.html)
- [MAVLink Developer Guide](https://mavlink.io/)
- [Cule OS Documentation](https://cule-os.io/docs)