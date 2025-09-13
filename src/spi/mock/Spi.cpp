#include <thread>
#include <chrono>

#include "spi/mock/Spi.h"

#include "spi/pi_buffer.h"

using namespace platform::wombat::core;

bool Spi::init(uint32_t /*speed_hz*/)
{
    m_fd = 1;
    return true;
}

bool Spi::update()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    m_rx[RX_TRANSFER_VERSION] = TRANSFER_VERSION;
    return true;
}

bool Spi::forceUpdate()
{
    return update();
}
