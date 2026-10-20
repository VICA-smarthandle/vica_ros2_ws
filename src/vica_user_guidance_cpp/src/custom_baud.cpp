#include <asm/termbits.h>
#include <sys/ioctl.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace vica_guidance
{

void set_custom_baudrate(int fd, int64_t baudrate)
{
  struct termios2 t {};
  if (::ioctl(fd, TCGETS2, &t) < 0) {
    throw std::runtime_error("Failed to set custom baud rate (" + std::to_string(baudrate) +
            "): " + std::strerror(errno));
  }
  t.c_cflag &= ~CBAUD;
  t.c_cflag |= BOTHER;
  t.c_ispeed = static_cast<speed_t>(baudrate);
  t.c_ospeed = static_cast<speed_t>(baudrate);
  if (::ioctl(fd, TCSETS2, &t) < 0) {
    throw std::runtime_error("Failed to set custom baud rate (" + std::to_string(baudrate) +
            "): " + std::strerror(errno));
  }
}

}
