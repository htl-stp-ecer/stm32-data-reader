# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32 Data Reader is a C++20 application that runs on a Raspberry Pi, communicating with an STM32 microcontroller over SPI to read sensor data (IMU, analog, digital) and control motors/servos. Data is published and commands received through `raccoon::Transport` (shared memory under `/dev/shm`, no network).

## Build Commands

### Build for the Pi (cross compiled in Docker, no emulation):
```bash
python3 build.py                        # reader + firmware -> builds/stm32-data-reader_0.0_arm64.deb
python3 build.py --version 60           # release build (CI does this)
python3 build.py --test                 # also run the unit tests (qemu-user in cross mode)
python3 build.py --mode native          # default on aarch64 (CI ARM runner); on x86 the default is cross
python3 build.py --skip-firmware        # reuse the firmware of the last build
```

### Local development (mock SPI):
```bash
mkdir -p cmake-build-debug && cd cmake-build-debug
cmake .. -DUSE_SPI_MOCK=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j$(nproc)
```

### Deploy to Raspberry Pi:
```bash
python3 deploy.py --host 192.168.1.100  # installs builds/*.deb, flashes the firmware, sets the reader to "dev" in the bundle file
```
Only `build.py` and `deploy.py` build and install; CI runs `build.py` too. `deploy.py` is for local builds; releases are installed by `raccoon update`.

## Architecture

### Core Namespace: `wombat`

**Application** (`src/wombat/Application.cpp`)
- Main orchestrator that owns and coordinates all services
- Manages lifecycle: initialize -> run main loop -> shutdown
- Signal handling for graceful shutdown

**Services Layer:**
- `DeviceController` - Coordinates SPI communication and maintains device state (motors, servos, sensors)
- `DataPublisher` - Publishes sensor data to the transport channels with change detection to reduce traffic
- `CommandSubscriber` - Subscribes to the transport command channels and routes to DeviceController

**Infrastructure:**
- `LcmBroker` - Typed publish/subscribe wrapper around `raccoon::Transport` with PIMPL pattern
- `Spi` / `SpiMock` - Hardware abstraction for SPI communication (toggled via `USE_SPI_MOCK`)
- `Logger` - spdlog-based logging wrapper
- `Result<T>` - Error handling monad (success/failure pattern)

### Data Flow

```
STM32 <--SPI--> Spi --> DeviceController --> DataPublisher --> raccoon::Transport (SHM)
                              ^
raccoon::Transport (SHM) --> CommandSubscriber ----|
```

### Message Types

Defined in raccoon-transport (`messages/`) as `raccoon::*` C++ structs:
- `vector3f_t` - 3D vectors (gyro, accel, mag)
- `quaternion_t` - Orientation quaternion
- `scalar_f_t`, `scalar_i32_t`, `scalar_i8_t` - Scalar values
- `message_t` - Messages for the user: a title and a text (`messages/error`, `messages/warn`)

### Channel Naming Convention

Channels are listed in the channel table of raccoon-transport (`channels.yml`) and follow the pattern `<root>/<device>/<kind>/<property>`; `include/wombat/core/Channels.h` gives the reader short names for them:
- Sensor data: `hardware/imu/state/gyro`, `hardware/imu/state/quaternion`
- Motor control: `hardware/motor/{port}/cmd/power`, `hardware/motor/{port}/state/bemf`
- Messages: `messages/error`, `messages/warn`

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
- **spdlog** (v1.15.3) - Logging

## Firmware (STM32)

The STM32 firmware lives in `firmware/` (merged from Firmware-Stp). It shares the SPI protocol header at
`shared/spi/pi_buffer.h`.

### Firmware build:

`python3 build.py` builds the firmware in Docker together with the reader.

### Firmware build (native, no Docker):

```bash
cd firmware && mkdir -p build && cd build
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake ..
cmake --build . -- -j$(nproc)
```

### Combined build and deploy (reader + firmware):

```bash
python3 build.py && python3 deploy.py --host <ip>   # the package flashes the firmware when installed
```

## Systemd Services

- `stm32_data_reader.service` - Main application service

## Testing on Hardware (Autonomous)

When making firmware or reader changes that affect motor/sensor behavior, deploy and test on the real hardware. If the
user doesn't provide the Pi's IP address, ask for it.

### Deploy & test workflow:

```bash
python3 build.py && python3 deploy.py --host <ip>   # Build + deploy reader + firmware
```

### Running Python test scripts on the Pi:

```bash
scp test_script.py pi@<ip>:/home/pi/      # Copy test script
ssh pi@<ip> "python3 /home/pi/test_script.py"  # Run it
```

### Raccoon transport Python library (on the Pi):

The raccoon-transport Python package is installed system-wide on the Pi. Use it to send commands and read sensor
data. Key imports:

```python
import raccoon_transport as rt
from raccoon_transport import channels
from raccoon_transport.types.raccoon import vector3f_t, scalar_i32_t, scalar_f_t

transport = rt.get_transport()
```

### Motor command examples (Python, runs on Pi):

- **Position command** (`vector3f_t`): `x` = speed limit, `y` = goal position. Publish to
  `channels.hardware.motor.cmd.position(port)`.
- **Position reset** (`scalar_i32_t`): `value` must be non-zero (e.g. `1`) to trigger. Publish to
  `channels.hardware.motor.cmd.reset_position(port)`.
- **Stop motor** (`scalar_i32_t`): Publish to `channels.hardware.motor.cmd.stop(port)`.
- **Read position**: Subscribe to `channels.hardware.motor.state.position(port)`, decode as `scalar_i32_t`, field is `.value`.
- **Read BEMF**: Subscribe to `channels.hardware.motor.state.bemf(port)`, decode as `scalar_i32_t`, field is `.value`.

### Important notes:

- All per-port motor and servo commands are reliable channels, including velocity and power: they are also sent as
  one-shot commands, and a lost one-shot command is never repeated.
- The chassis velocity command (`hardware/chassis/cmd/velocity`) is a stream; the stop is its own reliable channel
  `hardware/chassis/cmd/stop`.
- `scalar_i8_t` has field `.dir`, not `.value`. Used for servo mode only.
- Position reset resets the counter on the STM32 directly (not just a Pi-side offset).
- Use `time.sleep(0.3-0.5)` after reset before sending new commands.
- There is no done flag. A client detects "reached" by comparing `channels.hardware.motor.state.position(port)` with the goal
  (with a tolerance) and should use a timeout — motors may not arrive if blocked or misconfigured.
- Position mode (MTP) is sticky: after reaching the goal the firmware keeps holding it (and drives back if pushed
  away) until another mode is commanded (off, brake, velocity, power).
- Only absolute position commands exist; relative moves are resolved by the client (`goal = position + delta`).
- The `test_mtp.py` script in the repo root is a reference for MTP testing.
