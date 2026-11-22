# Contributing to stm32-data-reader

---

## Dev setup

Local builds use a mock SPI implementation so you don't need hardware:

```bash
mkdir -p cmake-build-debug && cd cmake-build-debug
cmake .. -DUSE_SPI_MOCK=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j$(nproc)
```

To build for the Pi (ARM64):

```bash
python3 build.py     # requires only Docker; cross compiled, add --test to run the tests with qemu-user
```

---

## Code style

- C++20 standard, `wombat` namespace throughout
- Use `Result<T>` for error handling — no exceptions
- Prefer `std::shared_ptr` for shared ownership
- Compile-clean at `-Wall -Wextra -Wpedantic`
- Channel name constants live in `wombat::Channels` — add new ones there, never hardcode strings

---

## Adding a new channel

### 1. Add the channel to the channel table

A channel is an entry in `channels.yml` of raccoon-transport (name, message type and the ring properties). A new message type is added to raccoon-transport as well. See its CONTRIBUTING.md; after the change run `python3 scripts/channels.py --write` and `python3 scripts/contract.py --write` there.

### 2. Give the channel a name in the reader

```cpp
// include/wombat/core/Channels.h
inline const std::string MY_SENSOR{table::hardware::my_device::state::MY_SENSOR};
```

### 3. Publish in DataPublisher

```cpp
// src/wombat/services/DataPublisher.cpp
auto msg = raccoon::scalar_f_t{};
msg.value = device.getMyValue();
broker_.publish(Channels::my_sensor, msg);
```

### 4. Subscribe in CommandSubscriber (if it's a command channel)

```cpp
broker_.subscribe<raccoon::scalar_i32_t>(
    Channels::my_command,
    [this](const raccoon::scalar_i32_t& msg) {
        device_.handleMyCommand(msg.value);
    }
);
```

---

## Adding a new service

Services follow the same lifecycle pattern as existing ones:

```cpp
// include/wombat/services/MyService.hpp
namespace wombat {

class MyService {
public:
    explicit MyService(LcmBroker& broker, DeviceController& device);
    Result<void> init();
    void update();   // called each main loop tick
};

}
```

Register it in `Application.cpp` alongside the existing services.

---

## SPI protocol

The SPI protocol is defined in `shared/spi/pi_buffer.h` — this file is the **single source of truth** shared between the reader and the STM32 firmware. Any change here must be reflected in the firmware and vice versa. The structs use `extern "C"` guards for C/C++ compatibility.

Do not change the protocol without also updating and rebuilding the firmware.

---

## Firmware

STM32 firmware lives in `firmware/`. `python3 build.py` builds it in Docker (`Dockerfile.firmware`, `gcc-arm-none-eabi`) together with the reader.

Natively with `gcc-arm-none-eabi` installed:

```bash
cd firmware/build
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake ..
cmake --build . -- -j$(nproc)
```

`python3 deploy.py --host <pi>` installs the package, which flashes the firmware to the STM32.

---

## Testing on hardware

```bash
python3 build.py && python3 deploy.py --host <your-pi-ip>   # build + deploy reader + firmware

# Show the rings on the Pi
ls -l /dev/shm/raccoon-rings/
```

Use the `raccoon-transport` Python package on the Pi to send test commands:

```python
import raccoon_transport as rt
from raccoon_transport import channels
from raccoon_transport.types.raccoon import scalar_i32_t

transport = rt.get_transport()
transport.publish(channels.hardware.motor.cmd.power(0), scalar_i32_t(timestamp=0, value=50))
```

The commands of the motors and servos are reliable channels: the sender repeats a frame until the reader acknowledges it.
