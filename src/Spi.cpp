//
// Created by tobias on 6/9/25.
//
#include "spi/Spi.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <thread>

#include <spdlog/spdlog.h>
using namespace platform::wombat::core;

#define SPI_MINIMUM_UPDATE_DELAY 20
namespace util
{
    void reset_STM()
    {
        if (const int result = system("bash ~/flashFiles/wallaby_reset_coproc"); result != 0)//sucessful execution of the script
            throw std::runtime_error("STM reset faild - restart the Wombat");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // delay 1s to ensure the STM is running
    }
}

bool Spi::init(uint32_t speed_hz)
{
    if (m_fd >= 0) return true; // already open

    m_fd = open(SPI_DEVICE, O_RDWR | O_CLOEXEC);
    if (m_fd < 0)
    {
        std::perror("SPI open");
        return false;
    }

    const uint8_t mode = SPI_MODE_0, bits = 8;
    if (ioctl(m_fd, SPI_IOC_WR_MODE, &mode) ||
        ioctl(m_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) ||
        ioctl(m_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz))
    {
        std::perror("SPI cfg");
        close(m_fd);
        m_fd = -1;
        return false;
    }

    /* prime TX buffer with protocol version */
    m_tx[TX_TRANSFER_VERSION] = TRANSFER_VERSION;

    m_tr = {
        .tx_buf = reinterpret_cast<unsigned long>(m_tx),
        .rx_buf = reinterpret_cast<unsigned long>(m_rx),
        .len = kBufSize,
        .speed_hz = speed_hz,
        .bits_per_word = bits
    };
    return true;
}

bool Spi::update()
{
    if (m_fd < 0) return false;

    using namespace std::chrono;
    static steady_clock::time_point last_call = steady_clock::now() - milliseconds(SPI_MINIMUM_UPDATE_DELAY);

    auto now = steady_clock::now();
    //ensure that the STM32 hase enough time to process data before the next transfer starts
    if (duration_cast<milliseconds>(now - last_call).count() >= SPI_MINIMUM_UPDATE_DELAY)
    {
        last_call = now;

        constexpr int maximumTrys = 3;//try three times to start the transmiton otherwise
        for (int trys = 0; trys < maximumTrys; trys++)
        {
            //do the spi transfer
            if (ioctl(m_fd, SPI_IOC_MESSAGE(1), &m_tr) < 0)
            {
                //reopen the spi prepheral on the Pi
                SPDLOG_ERROR("SPI communication on the Raspberry Pi failed - reopening the prepheral");
                close(m_fd);
                init();
                if (trys < maximumTrys - 1)
                    continue;

                throw std::runtime_error("SPI communication failed on the Raspberry Pi - restart the Wombat");
            }

            if (m_rx[RX_TRANSFER_VERSION] == TRANSFER_VERSION) //transmition sucessfull
                return true;

            SPDLOG_ERROR("SPI communication between Raspberry Pi and STM32 - reopening the prepheral");
            /*
             * -> transmission version out dated or transmission while transmission
             *
             * method to try to correct the failure:
             * - will retry the transmission at the first loop, if only the single transmission failed
             * - the following trys will reset the STM before starting the transmission again
             * - the last try will not reset the STM
             * - if the failure couldn't be solved the update methode will throw an error
             */

            //restart the STM32 bevor the last try
            if (trys > 0 && (trys != (maximumTrys - 1)))
            {
                SPDLOG_ERROR("Restarting STM32 - Transfer will be retryed");
                util::reset_STM();
            }
            if (trys == maximumTrys - 1)
            {
                throw std::runtime_error("SPI communication failed - "
                                         "Firmware transmition version does not equal the Libary Version");
            }
        }
        throw std::runtime_error("SPI communication failed - no specific cause");

    }
    return m_rx[RX_TRANSFER_VERSION] == TRANSFER_VERSION;
}

bool Spi::forceUpdate()
{
    //make sure the STM is ready to recive the transfer
    std::this_thread::sleep_for(std::chrono::milliseconds(SPI_MINIMUM_UPDATE_DELAY));

    return update();
}

Spi::~Spi()
{
    if (m_fd >= 0) close(m_fd);
}