# STM32 Data Reader

A C++20 application that runs on a Raspberry Pi to communicate with an STM32 microcontroller over SPI. It reads sensor data (IMU, analog inputs, digital inputs) and controls motors and servos, publishing all data over LCM (Lightweight Communications and Marshalling) multicast messaging.

## Features

- **IMU Data**: Gyroscope, accelerometer, magnetometer, and orientation quaternion with accuracy indicators
- **Motor Control**: 4 motor ports with speed/direction control and back-EMF sensing
- **Servo Control**: 4 servo ports with position control
- **Analog Inputs**: 6 analog sensor ports
- **Digital I/O**: 16-bit digital input reading
- **System Monitoring**: Battery voltage, IMU temperature, CPU temperature
- **LCM Messaging**: Real-time publish/subscribe communication with change detection

## Prerequisites

### For Cross-Compilation (Recommended)

- Docker with BuildX support
- QEMU user-mode emulation (for ARM64 builds on x86)

### For Local Development

- CMake 3.16+
- C++20 compatible compiler (GCC 10+ or Clang 12+)
- Ninja or Make

### On Target Raspberry Pi

- Raspberry Pi OS (64-bit recommended)
- SPI enabled (`sudo raspi-config` → Interface Options → SPI)
- Network connectivity for LCM multicast

## Building

### Cross-Compile for Raspberry Pi (ARM64)

The recommended way to build for deployment:

```bash
# Standard release build
./build.sh

# Force CMake reconfiguration
FORCE_RECONFIGURE=1 ./build.sh

# Debug build
CMAKE_BUILD_TYPE=Debug ./build.sh

# Rebuild Docker image
REBUILD_IMAGE=1 ./build.sh
```

The binary is output to `build/stm32_data_reader`.

### Local Development Build

For testing on your development machine using the mock SPI implementation:

```bash
mkdir -p cmake-build-debug
cd cmake-build-debug
cmake .. -DUSE_SPI_MOCK=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j$(nproc)
```

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `USE_SPI_MOCK` | `ON` | Use mock SPI for testing without hardware |
| `CMAKE_BUILD_TYPE` | `Release` | Build type (Debug, Release, RelWithDebInfo) |

## Deployment

### Automatic Deployment

Deploy to a Raspberry Pi over SSH:

```bash
# Uses default settings (pi@10.101.156.14)
./deploy.sh

# Override target
RPI_USER=myuser RPI_HOST=192.168.1.100 ./deploy.sh

# Custom install directory
RPI_DIR=/opt/stm32_data_reader ./deploy.sh
```

The deploy script will:
1. Cross-compile the binary
2. Stop any running service
3. Copy the binary and systemd service files
4. Set up the multicast loopback service

### Manual Deployment

```bash
# Build
./build.sh

# Copy binary
scp build/stm32_data_reader pi@<raspberry-pi>:/home/pi/stm32_data_reader/

# Copy and enable services
scp systemd/*.service pi@<raspberry-pi>:/tmp/
ssh pi@<raspberry-pi> 'sudo mv /tmp/*.service /etc/systemd/system/ && sudo systemctl daemon-reload'
ssh pi@<raspberry-pi> 'sudo systemctl enable --now lcm-loopback-multicast stm32_data_reader'
```

### Running as a Service

```bash
# Start the service
sudo systemctl start stm32_data_reader

# Enable on boot
sudo systemctl enable stm32_data_reader

# View logs
journalctl -u stm32_data_reader -f
```

### Running Manually

```bash
cd /home/pi/stm32_data_reader
./stm32_data_reader
```

## LCM Channels

### Sensor Data (Published)

| Channel | Type | Description |
|---------|------|-------------|
| `libstp/gyro/value` | `vector3f_t` | Gyroscope readings (rad/s) |
| `libstp/accel/value` | `vector3f_t` | Accelerometer readings (m/s²) |
| `libstp/mag/value` | `vector3f_t` | Magnetometer readings |
| `libstp/imu/quaternion` | `quaternion_t` | Orientation quaternion |
| `libstp/imu/temp/value` | `scalar_f_t` | IMU temperature (°C) |
| `libstp/cpu/temp/value` | `scalar_f_t` | Raspberry Pi CPU temperature (°C) |
| `libstp/battery/voltage` | `scalar_f_t` | Battery voltage |
| `libstp/analog/{0-5}/value` | `scalar_i32_t` | Analog sensor readings |
| `libstp/digital/{0-15}/value` | `scalar_i32_t` | Digital input states |
| `libstp/bemf/{0-3}/value` | `scalar_i32_t` | Motor back-EMF readings |
| `libstp/gyro/accuracy` | `scalar_i8_t` | Gyro calibration status (0-3) |
| `libstp/accel/accuracy` | `scalar_i8_t` | Accelerometer calibration status |
| `libstp/mag/accuracy` | `scalar_i8_t` | Magnetometer calibration status |
| `libstp/imu/quaternion_accuracy` | `scalar_i8_t` | Quaternion accuracy |

### Commands (Subscribed)

| Channel | Type | Description |
|---------|------|-------------|
| `libstp/motor/{0-3}/power_cmd` | `scalar_i32_t` | Set motor power (-100 to 100) |
| `libstp/motor/{0-3}/stop_cmd` | `scalar_i32_t` | Emergency stop motor |
| `libstp/servo/{0-3}/position_cmd` | `scalar_i32_t` | Set servo position |
| `libstp/bemf/{0-3}/reset_cmd` | `scalar_i32_t` | Reset BEMF accumulator |
| `libstp/bemf/{0-3}/scale_cmd` | `scalar_f_t` | Set BEMF scale factor |
| `libstp/bemf/{0-3}/offset_cmd` | `scalar_f_t` | Set BEMF offset |
| `libstp/bemf/nominal_voltage_cmd` | `scalar_i32_t` | Set nominal battery voltage |
| `libstp/system/dump_request` | `scalar_i32_t` | Request full data dump |

## Project Structure

```
stm32-data-reader/
├── src/
│   ├── main.cpp                 # Entry point
│   └── wombat/
│       ├── Application.cpp      # Main application orchestrator
│       ├── core/                # Logger, Result, utilities
│       ├── hardware/            # SPI implementation (real + mock)
│       ├── messaging/           # LCM broker wrapper
│       └── services/            # DeviceController, DataPublisher, CommandSubscriber
├── include/
│   ├── wombat/                  # Header files matching src/
│   └── spi/
│       └── pi_buffer.h          # SPI protocol structures (shared with STM32)
├── lcm-messages/
│   └── types/                   # LCM message definitions (.lcm files)
├── systemd/                     # Systemd service files
├── build.sh                     # Cross-compilation script
├── deploy.sh                    # Deployment script
├── Dockerfile                   # ARM64 build environment
└── CMakeLists.txt
```

## Network Setup for LCM

LCM uses UDP multicast. For local testing or when not using a network:

```bash
# Add multicast route to loopback (required for local-only operation)
sudo ip route add 239.255.76.67/32 dev lo

# Or use the provided systemd service
sudo systemctl enable --now lcm-loopback-multicast
```

For network operation, ensure multicast is enabled on your interface and firewall allows UDP port 7667.

## Troubleshooting

### SPI Not Working

1. Ensure SPI is enabled: `ls /dev/spidev*` should show devices
2. Check permissions: user must be in `spi` group or run as root
3. Verify wiring and STM32 firmware

### LCM Messages Not Received

1. Check multicast route: `ip route | grep 239.255`
2. Verify firewall allows UDP 7667
3. Use `lcm-spy` to monitor messages: `lcm-spy`

### Service Won't Start

```bash
# Check service status
sudo systemctl status stm32_data_reader

# Check dependencies
sudo systemctl status lcm-loopback-multicast

# View detailed logs
journalctl -u stm32_data_reader -n 100
```

## Contributing

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Build and test locally with `USE_SPI_MOCK=ON`
4. Ensure code compiles without warnings (`-Wall -Wextra -Wpedantic`)
5. Submit a pull request

### Code Style

- C++20 standard
- Use the `wombat` namespace
- Follow existing patterns for new services
- Use `Result<T>` for error handling instead of exceptions
- Prefer `std::shared_ptr` for shared ownership

## License

Copyright (C) 2026 Tobias Madlberger  
Licensed under the GNU General Public License v3.0 — see [COPYING](COPYING) for details.