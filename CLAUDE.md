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

Defined in `lcm-messages/types/*.lcm`, auto-generated to `raccoon::*` C++ headers:
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

Binary protocol defined in `shared/spi/pi_buffer.h` (single source of truth, used by both reader and firmware):
- `TxBuffer` - Data from STM32 (sensors, BEMF readings)
- `RxBuffer` - Commands to STM32 (motor/servo control, calibration)
- Protocol version handshake ensures compatibility
- Has `extern "C"` guards for C/C++ compatibility

## Key Configuration

Located in `wombat::Configuration` struct:
- SPI: device path, speed (20MHz default), mode, retry attempts
- Logging: level, pattern
- Main loop delay

## Dependencies

Fetched via CMake FetchContent:
- **LCM** (v1.5.0) - Multicast messaging
- **spdlog** (v1.15.3) - Logging

## Firmware (STM32)

The STM32 firmware lives in `firmware/` (merged from Firmware-Stp). It shares the SPI protocol header at
`shared/spi/pi_buffer.h`.

### Firmware build (requires Docker or gcc-arm-none-eabi):

```bash
cd firmware && bash build.sh              # Builds wombat.bin via Docker
```

### Firmware build (native, no Docker):

```bash
cd firmware && mkdir -p build && cd build
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake ..
cmake --build . -- -j$(nproc)
```

### Combined build (reader + firmware):

```bash
./build.sh                                # Builds both; firmware artifacts in build/
SKIP_FIRMWARE=1 ./build.sh                # Skip firmware, reader only
```

### Deploy both to Pi:

```bash
./deploy.sh                               # Builds both, uploads, flashes firmware, starts reader
```

## Systemd Services

- `stm32_data_reader.service` - Main application service
- `lcm-loopback-multicast.service` - Required for LCM multicast on loopback

## Testing on Hardware (Autonomous)

When making firmware or reader changes that affect motor/sensor behavior, deploy and test on the real hardware. If the
user doesn't provide the Pi's IP address, ask for it.

### Deploy & test workflow:

```bash
RPI_HOST=<ip> bash deploy.sh              # Build + deploy reader + firmware
```

### Running Python test scripts on the Pi:

```bash
scp test_script.py pi@<ip>:/home/pi/      # Copy test script
ssh pi@<ip> "python3 /home/pi/test_script.py"  # Run it
```

### Raccoon transport Python library (on the Pi):

The raccoon-transport Python package is installed system-wide on the Pi. Use it to send LCM commands and read sensor
data. Key imports:

```python
from raccoon_transport import Transport
from raccoon_transport.channels import Channels
from raccoon_transport.types.raccoon import vector3f_t, scalar_i32_t, scalar_f_t
```

### Motor command examples (Python, runs on Pi):

- **Position command** (`vector3f_t`): `x` = speed limit, `y` = goal position. Publish to
  `Channels.motor_position_command(port)` with `reliable=True`.
- **Position reset** (`scalar_i32_t`): `value` must be non-zero (e.g. `1`) to trigger. Publish to
  `Channels.motor_position_reset_command(port)` with `reliable=True`.
- **Stop motor** (`scalar_i32_t`): Publish to `Channels.motor_stop_command(port)` with `reliable=True`.
- **Read position**: Subscribe to `Channels.motor_position(port)`, decode as `scalar_i32_t`, field is `.value`.
- **Read done flag**: Subscribe to `Channels.motor_done(port)`, decode as `scalar_i32_t`, field is `.value`.
- **Read BEMF**: Subscribe to `Channels.back_emf(port)`, decode as `scalar_i32_t`, field is `.value`.

### Important notes:

- Motor commands that change state (position, stop, reset, mode) use `reliable=True` delivery.
- Velocity/power commands (continuous) do NOT use reliable delivery.
- `scalar_i8_t` has field `.dir`, not `.value`. Used for servo mode only.
- Position reset resets the counter on the STM32 directly (not just a Pi-side offset).
- Use `time.sleep(0.3-0.5)` after reset before sending new commands.
- Use timeouts when waiting for done — motors may not finish if blocked or misconfigured.
- The `test_mtp.py` script in the repo root is a reference for MTP testing.
