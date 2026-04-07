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
./build.sh           # requires Docker with BuildX + QEMU user-mode emulation
```

---

## Code style

- C++20 standard, `wombat` namespace throughout
- Use `Result<T>` for error handling — no exceptions
- Prefer `std::shared_ptr` for shared ownership
- Compile-clean at `-Wall -Wextra -Wpedantic`
- Channel name constants live in `wombat::Channels` — add new ones there, never hardcode strings

---

## Adding a new LCM channel

### 1. Define the message type (if needed)

Add a `.lcm` file in `lcm-messages/types/`:

```
package raccoon;
struct my_value_t {
    float value;
}
```

Re-run the LCM generator to produce the C++ headers.

### 2. Add the channel constant

```cpp
// include/wombat/Channels.hpp
namespace wombat::Channels {
    constexpr auto my_sensor = "libstp/my_sensor/value";
}
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

STM32 firmware lives in `firmware/`. Build it with Docker:

```bash
cd firmware && bash build.sh
```

Or natively with `gcc-arm-none-eabi`:

```bash
cd firmware/build
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake ..
cmake --build . -- -j$(nproc)
```

`./deploy.sh` from the repo root builds and flashes the firmware to the STM32 after deploying the reader.

---

## Testing on hardware

```bash
RPI_HOST=<your-pi-ip> bash deploy.sh   # build + deploy reader + firmware

# Monitor LCM traffic on the Pi
lcm-spy
```

Use the `raccoon-transport` Python package on the Pi to send test commands:

```python
from raccoon_transport import Transport
from raccoon_transport.channels import Channels
from raccoon_transport.types.raccoon import scalar_i32_t

t = Transport()
msg = scalar_i32_t()
msg.value = 50
t.publish(Channels.motor_power_command(0), msg.encode())
```

Motor commands that change state (position, stop, reset) use `reliable=True`. Continuous commands (power) do not.
