// pyserial 3.5 serialposix.Serial 을 이 노드가 쓰는 만큼만 옮겼다(기본 인자: 8N1, 흐름제어 없음,
// exclusive 없음, inter_byte_timeout 없음). 대조 기준은 /usr/lib/python3/dist-packages/serial/serialposix.py.
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

#include "vica_user_guidance_cpp/serial_link.hpp"

namespace vica_guidance
{

// 표준 표에 없는 속도(BOTHER). termios.h 와 asm/termbits.h 가 충돌해 별도 파일에 둔다.
void set_custom_baudrate(int fd, int64_t baudrate);

namespace
{

std::string errno_text(int e)
{
  return "[Errno " + std::to_string(e) + "] " + std::strerror(e);
}

bool standard_speed(int64_t baud, speed_t & out)
{
  switch (baud) {
    case 0: out = B0; return true;
    case 50: out = B50; return true;
    case 75: out = B75; return true;
    case 110: out = B110; return true;
    case 134: out = B134; return true;
    case 150: out = B150; return true;
    case 200: out = B200; return true;
    case 300: out = B300; return true;
    case 600: out = B600; return true;
    case 1200: out = B1200; return true;
    case 1800: out = B1800; return true;
    case 2400: out = B2400; return true;
    case 4800: out = B4800; return true;
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 500000: out = B500000; return true;
    case 576000: out = B576000; return true;
    case 921600: out = B921600; return true;
    case 1000000: out = B1000000; return true;
    case 1152000: out = B1152000; return true;
    case 1500000: out = B1500000; return true;
    case 2000000: out = B2000000; return true;
    case 2500000: out = B2500000; return true;
    case 3000000: out = B3000000; return true;
    case 3500000: out = B3500000; return true;
    case 4000000: out = B4000000; return true;
    default: return false;
  }
}

// pyserial Timeout: None = 무한, 0 = 논블로킹, 그 밖 = 남은 시간.
class Deadline
{
public:
  explicit Deadline(double seconds)
  : non_blocking_(seconds == 0.0),
    end_(std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(seconds > 0 ? seconds : 0.0))) {}
  bool non_blocking() const {return non_blocking_;}
  bool expired() const {return std::chrono::steady_clock::now() >= end_;}
  int left_ms() const
  {
    const auto left = end_ - std::chrono::steady_clock::now();
    if (left <= std::chrono::steady_clock::duration::zero()) {return 0;}
    return static_cast<int>(
      std::ceil(std::chrono::duration<double, std::milli>(left).count()));
  }

private:
  bool non_blocking_;
  std::chrono::steady_clock::time_point end_;
};

bool transient(int e)
{
  return e == EAGAIN || e == EALREADY || e == EWOULDBLOCK || e == EINPROGRESS || e == EINTR;
}

class PosixSerial : public SerialPort
{
public:
  PosixSerial(const std::string & port, int64_t baudrate, double timeout, double write_timeout)
  : timeout_(timeout), write_timeout_(write_timeout)
  {
    // pyserial 의 설정 검사: 음수 속도·음수 시한은 ValueError.
    if (baudrate < 0) {throw std::invalid_argument("Not a valid baudrate: " + std::to_string(baudrate));}
    if (write_timeout < 0) {throw std::invalid_argument("Not a valid timeout");}
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      const int e = errno;
      throw std::runtime_error(errno_text(e) + ": could not open port " + port);
    }
    try {
      configure(baudrate);
      // DTR·RTS 켬. 아닌 장치의 EINVAL·ENOTTY 는 무시한다.
      int bits = TIOCM_DTR;
      if (::ioctl(fd_, TIOCMBIS, &bits) < 0 && errno != EINVAL && errno != ENOTTY) {
        throw std::runtime_error(errno_text(errno));
      }
      bits = TIOCM_RTS;
      if (::ioctl(fd_, TIOCMBIS, &bits) < 0 && errno != EINVAL && errno != ENOTTY) {
        throw std::runtime_error(errno_text(errno));
      }
      ::tcflush(fd_, TCIFLUSH);
    } catch (...) {
      ::close(fd_);
      fd_ = -1;
      throw;
    }
  }

  ~PosixSerial() override {close();}

  void write(uint8_t byte) override
  {
    require_open();
    Deadline deadline(write_timeout_);
    std::size_t left = 1;
    while (left > 0) {
      const ssize_t n = ::write(fd_, &byte, 1);
      if (n < 0) {
        const int e = errno;
        if (!transient(e)) {throw std::runtime_error("write failed: " + errno_text(e));}
      } else {
        if (deadline.non_blocking()) {return;}
        if (deadline.expired()) {throw std::runtime_error("Write timeout");}
        // pyserial: 시한이 있으면 쓴 뒤에도 남은 시간 동안 쓰기 가능해질 때까지 기다린다.
        pollfd p{fd_, POLLOUT, 0};
        const int r = ::poll(&p, 1, deadline.left_ms());
        if (r == 0) {throw std::runtime_error("Write timeout");}
        if (r < 0 && !transient(errno)) {throw std::runtime_error("write failed: " + errno_text(errno));}
        left -= static_cast<std::size_t>(n);
      }
      // pyserial 은 다 쓴 뒤에도 시한이 지났으면 실패로 본다.
      if (!deadline.non_blocking() && deadline.expired()) {
        throw std::runtime_error("Write timeout");
      }
    }
  }

  void flush() override
  {
    require_open();
    // C++ 판은 전송이 신호를 받는 주 스레드에서 돈다. rclcpp 신호 처리기는 SA_RESTART 가 없어
    // Ctrl+C 가 이 몇 ms 에 걸리면 EINTR 이 온다 — 실패로 보면 포트를 닫아 종료 때 중립을 못 보낸다.
    // (파이썬 판은 전송이 실행기 작업 스레드에서 돌아 이 자리에 신호가 거의 걸리지 않는다.)
    while (::tcdrain(fd_) < 0) {
      if (errno != EINTR) {throw std::runtime_error(errno_text(errno));}
    }
  }

  std::size_t in_waiting() override
  {
    require_open();
    int n = 0;
    if (::ioctl(fd_, TIOCINQ, &n) < 0) {throw std::runtime_error(errno_text(errno));}
    return static_cast<std::size_t>(n);
  }

  Bytes read(std::size_t size) override
  {
    require_open();
    Bytes out;
    Deadline deadline(timeout_);
    while (out.size() < size) {
      pollfd p{fd_, POLLIN, 0};
      const int r = ::poll(&p, 1, deadline.left_ms());
      if (r < 0) {
        if (!transient(errno)) {throw std::runtime_error("read failed: " + errno_text(errno));}
      } else if (r == 0) {
        break;   // 시한
      } else {
        uint8_t buf[512];
        const std::size_t want = std::min(sizeof(buf), size - out.size());
        const ssize_t n = ::read(fd_, buf, want);
        if (n < 0) {
          if (!transient(errno)) {throw std::runtime_error("read failed: " + errno_text(errno));}
        } else if (n == 0) {
          // 빠진 장치는 '읽을 게 있다'고 하고 아무것도 주지 않는다.
          throw std::runtime_error(
                  "device reports readiness to read but returned no data "
                  "(device disconnected or multiple access on port?)");
        } else {
          out.insert(out.end(), buf, buf + n);
        }
      }
      if (deadline.expired()) {break;}
    }
    return out;
  }

  void close() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  void require_open() const
  {
    if (fd_ < 0) {throw std::runtime_error("Attempting to use a port that is not open");}
  }

  void configure(int64_t baudrate)
  {
    termios t{};
    if (::tcgetattr(fd_, &t) < 0) {
      throw std::runtime_error("Could not configure port: " + errno_text(errno));
    }
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN | ECHOCTL | ECHOKE);
    t.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    t.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK | IUCLC | PARMRK);
    speed_t speed = B38400;
    const bool custom = !standard_speed(baudrate, speed);
    if (custom) {speed = B38400;}   // BOTHER 는 아래 set_custom_baudrate 가 따로 건다
    ::cfsetispeed(&t, speed);
    ::cfsetospeed(&t, speed);
    t.c_cflag &= ~CSIZE;
    t.c_cflag |= CS8;
    t.c_cflag &= ~CSTOPB;
    t.c_iflag &= ~(INPCK | ISTRIP);
    t.c_cflag &= ~(PARENB | PARODD | CMSPAR);
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_cflag &= ~CRTSCTS;
    t.c_cc[VMIN] = 0;   // 시한은 poll 이 한다
    t.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &t) < 0) {
      throw std::runtime_error("Could not configure port: " + errno_text(errno));
    }
    if (custom) {set_custom_baudrate(fd_, baudrate);}
  }

  int fd_ = -1;
  double timeout_;
  double write_timeout_;
};

}  // namespace

std::unique_ptr<SerialPort> open_posix_serial(
  const std::string & port, int64_t baudrate, double timeout, double write_timeout)
{
  return std::make_unique<PosixSerial>(port, baudrate, timeout, write_timeout);
}

}  // namespace vica_guidance
