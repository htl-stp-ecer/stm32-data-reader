# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32 Data Reader is a C++20 application that runs on a Raspberry Pi, communicating with an STM32 microcontroller over SPI to read sensor data (IMU, analog, digital) and control motors/servos. Data is published and commands received via LCM (Lightweight Communications and Marshalling) multicast messaging.

## Build Commands

### Cross-compile for ARM64 (production build):
```bash
./build.sh                              # Uses Docker to cross-compile for ARM64
FORCE_RECONFIGURE=1 ./build.sh          # Force CMake reconfiguration
CMAKE_BUILD_TYPE=Debug ./build.sh       # Debug build
```

### Local development (mock SPI):
```bash
mkdir -p cmake-build-debug && cd cmake-build-debug
cmake .. -DUSE_SPI_MOCK=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j$(nproc)
```

### Deploy to Raspberry Pi:
```bash
./deploy.sh                             # Build and deploy to Pi
RPI_HOST=192.168.1.100 ./deploy.sh      # Override target host
```

## Architecture

### Core Namespace: `wombat`

**Application** (`src/wombat/Application.cpp`)
- Main orchestrator that owns and coordinates all services
- Manages lifecycle: initialize -> run main loop -> shutdown
- Signal handling for graceful shutdown

**Services Layer:**
- `DeviceController` - Coordinates SPI communication and maintains device state (motors, servos, sensors)
- `DataPublisher` - Publishes sensor data to LCM channels with change detection to reduce traffic
- `CommandSubscriber` - Subscribes to LCM command channels and routes to DeviceController

**Infrastructure:**
- `LcmBroker` - Typed publish/subscribe wrapper around LCM with PIMPL pattern
- `Spi` / `SpiMock` - Hardware abstraction for SPI communication (toggled via `USE_SPI_MOCK`)
- `Logger` - spdlog-based logging wrapper
- `Result<T>` - Error handling monad (success/failure pattern)

### Data Flow

```
STM32 <--SPI--> Spi --> DeviceController --> DataPublisher --> LCM (multicast)
                              ^
LCM (multicast) --> CommandSubscriber ----|
```

### LCM Message Types

Defined in `lcm-messages/types/*.lcm`, auto-generated to `exlcm::*` C++ headers:
- `vector3f_t` - 3D vectors (gyro, accel, mag)
- `quaternion_t` - Orientation quaternion
- `scalar_f_t`, `scalar_i32_t`, `scalar_i8_t` - Scalar values
- `string_t` - Error messages

### Channel Naming Convention

All channels follow pattern `libstp/<device>/<property>` defined in `wombat::Channels` namespace:
- Sensor data: `libstp/gyro/value`, `libstp/accel/value`, `libstp/imu/quaternion`
- Motor control: `libstp/motor/{port}/power_cmd`, `libstp/bemf/{port}/value`
- System: `libstp/system/dump_request`, `libstp/errors`

### SPI Protocol

Binary protocol defined in `include/spi/pi_buffer.h`:
- `TxBuffer` - Data from STM32 (sensors, BEMF readings)
- `RxBuffer` - Commands to STM32 (motor/servo control, calibration)
- Protocol version handshake ensures compatibility

## Key Configuration

Located in `wombat::Configuration` struct:
- SPI: device path, speed (20MHz default), mode, retry attempts
- Logging: level, pattern
- Main loop delay

## Dependencies

Fetched via CMake FetchContent:
- **LCM** (v1.5.0) - Multicast messaging
- **spdlog** (v1.15.3) - Logging

## Systemd Services

- `stm32_data_reader.service` - Main application service
- `lcm-loopback-multicast.service` - Required for LCM multicast on loopback
