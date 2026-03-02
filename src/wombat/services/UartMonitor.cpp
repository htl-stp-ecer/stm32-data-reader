#include "wombat/services/UartMonitor.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <algorithm>

namespace wombat
{
    static constexpr size_t MAX_LINE_BUFFER = 4096;
    static constexpr size_t READ_BUFFER_SIZE = 256;

    UartMonitor::UartMonitor(std::shared_ptr<Logger> logger, const Configuration::Uart& config)
        : logger_{std::move(logger)}, config_{config}
    {
    }

    Result<void> UartMonitor::initialize()
    {
        fd_ = ::open(config_.devicePath.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            logger_->warn("Failed to open UART device " + config_.devicePath + ": " + std::strerror(errno));
            return Result<void>::success();
        }

        struct termios tty{};
        if (::tcgetattr(fd_, &tty) != 0)
        {
            logger_->warn("Failed to get termios for " + config_.devicePath + ": " + std::strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return Result<void>::success();
        }

        // Raw mode — no echo, no canonical processing, no signals
        cfmakeraw(&tty);

        // 8N1
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;

        // Enable receiver, ignore modem control lines
        tty.c_cflag |= CREAD | CLOCAL;

        // Non-blocking: return immediately if no data
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        // Set baud rate
        speed_t baud;
        switch (config_.baudRate)
        {
        case 9600: baud = B9600;
            break;
        case 19200: baud = B19200;
            break;
        case 38400: baud = B38400;
            break;
        case 57600: baud = B57600;
            break;
        case 115200: baud = B115200;
            break;
        case 230400: baud = B230400;
            break;
        case 460800: baud = B460800;
            break;
        default:
            logger_->warn("Unsupported baud rate " + std::to_string(config_.baudRate) + ", defaulting to 115200");
            baud = B115200;
            break;
        }
        cfsetispeed(&tty, baud);
        cfsetospeed(&tty, baud);

        if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            logger_->warn("Failed to set termios for " + config_.devicePath + ": " + std::strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return Result<void>::success();
        }

        // Flush any stale data
        ::tcflush(fd_, TCIFLUSH);

        isOpen_ = true;
        logger_->info(
            "UART monitor opened " + config_.devicePath + " at " + std::to_string(config_.baudRate) + " baud");
        return Result<void>::success();
    }

    Result<void> UartMonitor::processUpdate()
    {
        if (!isOpen_)
        {
            return Result<void>::success();
        }

        char buf[READ_BUFFER_SIZE];
        ssize_t bytesRead = ::read(fd_, buf, sizeof(buf));

        if (bytesRead < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return Result<void>::success();
            }
            return Result<void>::failure("UART read error: " + std::string(std::strerror(errno)));
        }

        if (bytesRead == 0)
        {
            return Result<void>::success();
        }

        lineBuffer_.append(buf, static_cast<size_t>(bytesRead));

        // Extract and log complete lines
        size_t pos;
        while ((pos = lineBuffer_.find('\n')) != std::string::npos)
        {
            std::string line = lineBuffer_.substr(0, pos);
            lineBuffer_.erase(0, pos + 1);

            // Strip trailing \r
            if (!line.empty() && line.back() == '\r')
            {
                line.pop_back();
            }

            if (!line.empty())
            {
                std::string tagged = "[STM32] " + line;

                // Route error/warning lines through appropriate log levels
                // so they get published to the LCM error channel
                if (line.find("[ERROR]") != std::string::npos
                    || line.find("Error") != std::string::npos
                    || line.find("FAULT") != std::string::npos)
                {
                    logger_->error(tagged);
                }
                else if (line.find("[WARN]") != std::string::npos)
                {
                    logger_->warn(tagged);
                }
                else
                {
                    logger_->info(tagged);
                }
            }
        }

        // Prevent unbounded buffer growth
        if (lineBuffer_.size() > MAX_LINE_BUFFER)
        {
            logger_->warn("[STM32] Line buffer overflow, discarding partial data");
            lineBuffer_.clear();
        }

        return Result<void>::success();
    }

    Result<void> UartMonitor::shutdown()
    {
        if (isOpen_&& fd_ 
        >=
        0
        )
        {
            ::close(fd_);
            fd_ = -1;
            isOpen_ = false;
            logger_->info("UART monitor closed");
        }
        return Result<void>::success();
    }
}