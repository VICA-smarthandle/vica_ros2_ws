#include "mdrobot_can_control_cpp/socketcan.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

namespace mdrobot
{
namespace
{

// can.CanOperationError(f"{prefix}: {error.strerror}", error.errno) 의 str()
std::string operation_error(const char * prefix, int err)
{
  return std::string(prefix) + ": " + std::strerror(err) + " [Error Code " + std::to_string(err) +
         "]";
}

// str(OSError(errno, strerror)) == "[Errno N] strerror"
std::string os_error(int err)
{
  return "[Errno " + std::to_string(err) + "] " + std::strerror(err);
}

// 닫힌 소켓을 건드렸을 때 python-can 이 내는 ValueError 의 글자.
constexpr const char * kClosedSocket = "file descriptor cannot be a negative integer (-1)";
// select.select 가 FD_SETSIZE 이상의 fd 에 내는 ValueError 의 글자(FD_SET 의 UB 를 막는다).
constexpr const char * kFdOutOfRange = "filedescriptor out of range in select()";

// 파이썬 repr(str) — 따옴표 고르기와 흔한 이스케이프만(인터페이스 이름·flags 글자용).
std::string py_repr(const std::string & s)
{
  const bool has_single = s.find('\'') != std::string::npos;
  const bool has_double = s.find('"') != std::string::npos;
  const char quote = (has_single && !has_double) ? '"' : '\'';
  std::string out(1, quote);
  for (const unsigned char c : s) {
    if (c == '\\') {
      out += "\\\\";
    } else if (c == static_cast<unsigned char>(quote)) {
      out += '\\';
      out += static_cast<char>(c);
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else if (c < 0x20 || c == 0x7F) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\x%02x", c);
      out += buf;
    } else {
      out += static_cast<char>(c);
    }
  }
  out += quote;
  return out;
}

// str.strip() 의 ASCII 공백.
std::string py_strip(const std::string & s)
{
  const char * ws = " \t\n\r\x0b\x0c\x1c\x1d\x1e\x1f";
  const auto b = s.find_first_not_of(ws);
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

int digit_value(char c)
{
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  if (c >= 'a' && c <= 'z') {
    return c - 'a' + 10;
  }
  if (c >= 'A' && c <= 'Z') {
    return c - 'A' + 10;
  }
  return 99;
}

// int(raw, 0) 의 ASCII 문법: 부호, 0x/0o/0b 접두사, 숫자 사이 밑줄 하나, 0 이 아닌 십진수의
// 앞자리 0 금지. 맞으면 true.
bool parse_py_int_base0(const std::string & raw, int64_t & out)
{
  size_t i = 0;
  bool negative = false;
  if (i < raw.size() && (raw[i] == '+' || raw[i] == '-')) {
    negative = raw[i] == '-';
    ++i;
  }
  int base = 10;
  if (i + 1 < raw.size() && raw[i] == '0') {
    const char p = raw[i + 1];
    if (p == 'x' || p == 'X') {
      base = 16;
    } else if (p == 'o' || p == 'O') {
      base = 8;
    } else if (p == 'b' || p == 'B') {
      base = 2;
    }
    if (base != 10) {
      i += 2;
    }
  }
  const std::string body = raw.substr(i);
  if (body.empty()) {
    return false;
  }
  // 접두사 바로 뒤의 밑줄 하나는 허용된다(0x_1). 그 밖의 밑줄은 숫자 사이에만.
  size_t j = 0;
  if (base != 10 && body[0] == '_') {
    j = 1;
  }
  if (j >= body.size()) {
    return false;
  }
  uint64_t value = 0;
  constexpr uint64_t kLimit = static_cast<uint64_t>(INT64_MAX) + 1;  // 2^63
  bool prev_underscore = true;  // 맨 앞(또는 접두사 밑줄 뒤)의 밑줄은 금지
  bool all_zero = true;
  bool first_digit_zero = false;
  size_t ndigits = 0;
  for (; j < body.size(); ++j) {
    const char c = body[j];
    if (c == '_') {
      if (prev_underscore) {
        return false;
      }
      prev_underscore = true;
      continue;
    }
    const int d = digit_value(c);
    if (d >= base) {
      return false;
    }
    if (ndigits == 0) {
      first_digit_zero = d == 0;
    }
    all_zero = all_zero && d == 0;
    ++ndigits;
    prev_underscore = false;
    if (value > (kLimit - static_cast<uint64_t>(d)) / static_cast<uint64_t>(base)) {
      return false;  // flags 로는 나올 수 없는 크기
    }
    value = value * static_cast<uint64_t>(base) + static_cast<uint64_t>(d);
  }
  if (prev_underscore) {
    return false;  // 끝의 밑줄
  }
  if (base == 10 && first_digit_zero && !all_zero) {
    return false;  // int('010', 0) 은 ValueError
  }
  if (!negative && value == kLimit) {
    return false;
  }
  out = negative ? static_cast<int64_t>(~value + 1) : static_cast<int64_t>(value);
  return true;
}

}  // namespace

SocketCan::SocketCan(const std::string & channel)
{
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw CanFailure(os_error(errno));
  }
  // python-can 도 실패를 로그만 남기고 넘어가는 옵션들이다(이 커널에선 실패하지 않는다).
  int one = 1;
  int zero = 0;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &one, sizeof(one));
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &zero, sizeof(zero));
  can_err_mask_t err_mask = 0x1FFFFFFF;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
  ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one));

  // sock.bind((channel,)) — 파이썬 소켓 모듈도 이름을 ifindex 로 바꿔 bind 한다.
  int err = 0;
  const unsigned idx = ::if_nametoindex(channel.c_str());
  if (idx == 0) {
    err = ENODEV;
  } else {
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = static_cast<int>(idx);
    if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      err = errno;
    }
  }
  if (err != 0) {
    ::close(fd_);
    fd_ = -1;
    const std::string text = os_error(err);
    // python-can: log.error("Could not access SocketCAN device %s (%s)", channel, error)
    // 핸들러가 없어 파이썬 lastResort 가 이 한 줄을 stderr 에 그대로 찍는다.
    std::fprintf(stderr, "Could not access SocketCAN device %s (%s)\n", channel.c_str(), text.c_str());
    std::fflush(stderr);
    throw CanFailure(text);
  }
}

SocketCan::~SocketCan() {shutdown();}

void SocketCan::shutdown()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::optional<Frame> SocketCan::recv_nowait()
{
  if (fd_ < 0) {
    throw CanFailure(kClosedSocket);
  }
  if (fd_ >= FD_SETSIZE) {
    throw CanFailure(kFdOutOfRange);
  }
  int ready = 0;
  fd_set rset;
  do {  // 파이썬(PEP 475)처럼 EINTR 은 다시 시도한다
    FD_ZERO(&rset);
    FD_SET(fd_, &rset);
    timeval tv{0, 0};
    ready = ::select(fd_ + 1, &rset, nullptr, nullptr, &tv);
  } while (ready < 0 && errno == EINTR);
  if (ready < 0) {
    throw CanFailure(operation_error("Failed to receive", errno));
  }
  if (ready == 0 || !FD_ISSET(fd_, &rset)) {
    return std::nullopt;
  }

  // sock.recvmsg(CANFD_MTU, 타임스탬프 하나)
  canfd_frame cf{};
  iovec iov{&cf, sizeof(cf)};
  alignas(cmsghdr) char ctrl[CMSG_SPACE(sizeof(timespec))];
  msghdr mh{};
  mh.msg_iov = &iov;
  mh.msg_iovlen = 1;
  mh.msg_control = ctrl;
  mh.msg_controllen = sizeof(ctrl);
  ssize_t n = 0;
  do {
    n = ::recvmsg(fd_, &mh, 0);
  } while (n < 0 && errno == EINTR);
  if (n < 0) {
    throw CanFailure(operation_error("Error receiving", errno));
  }

  // dissect_can_frame: 고전 프레임의 data 는 frame[8 : 8 + len]
  Frame f;
  f.id = (cf.can_id & CAN_EFF_FLAG) ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
  if (cf.can_id & CAN_RTR_FLAG) {
    // python-can Message(is_remote_frame=True) 는 data 를 비운다 → len(d) == 0
    f.dlc = 0;
    return f;
  }
  f.dlc = cf.len <= CAN_MAX_DLEN ? cf.len : CAN_MAX_DLEN;
  std::memcpy(f.data.data(), cf.data, CAN_MAX_DLEN);
  return f;
}

void SocketCan::send(const Frame & f)
{
  if (fd_ < 0) {
    throw CanFailure(kClosedSocket);
  }
  if (fd_ >= FD_SETSIZE) {
    throw CanFailure(kFdOutOfRange);
  }
  const std::array<uint8_t, 16> frame = pack_classic_frame(f);

  int ready = 0;
  fd_set wset;
  do {
    FD_ZERO(&wset);
    FD_SET(fd_, &wset);
    timeval tv{0, 0};
    ready = ::select(fd_ + 1, nullptr, &wset, nullptr, &tv);
  } while (ready < 0 && errno == EINTR);
  if (ready < 0) {
    // 파이썬은 여기서 OSError 를 감싸지 않는다(CAN_FAILURES 에 OSError 가 들어 있다).
    throw CanFailure(os_error(errno));
  }
  if (ready == 0) {
    throw CanFailure("Transmit buffer full");
  }
  ssize_t sent = 0;
  do {
    sent = ::send(fd_, frame.data(), frame.size(), 0);
  } while (sent < 0 && errno == EINTR);
  if (sent < 0) {
    throw CanFailure(operation_error("Failed to transmit", errno));
  }
  if (sent != static_cast<ssize_t>(frame.size())) {
    // 파이썬은 나머지를 다시 보내려다 timeout(0) 이 지나 이 글자로 끝난다.
    throw CanFailure("Transmit buffer full");
  }
}

std::array<uint8_t, 16> pack_classic_frame(const Frame & f)
{
  static_assert(sizeof(can_frame) == 16, "classic CAN frame is 16 bytes");
  can_frame cf{};
  cf.can_id = f.id;
  cf.len = f.dlc;
  cf.len8_dlc = f.dlc;
  std::memcpy(cf.data, f.data.data(), CAN_MAX_DLEN);
  std::array<uint8_t, 16> out{};
  std::memcpy(out.data(), &cf, out.size());
  return out;
}

std::string format_hex_upper(int64_t value, int width)
{
  const bool negative = value < 0;
  const uint64_t mag = negative ? (~static_cast<uint64_t>(value) + 1) : static_cast<uint64_t>(value);
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%llX", static_cast<unsigned long long>(mag));
  std::string digits(buf);
  // 파이썬 '{:0Nx}' 는 부호를 앞에 두고 남는 폭을 0 으로 채운다.
  const int pad = width - static_cast<int>(digits.size()) - (negative ? 1 : 0);
  if (pad > 0) {
    digits.insert(0, static_cast<size_t>(pad), '0');
  }
  return negative ? "-" + digits : digits;
}

int64_t require_can_interface_up(const std::string & interface, const std::string & sys_class_net)
{
  // Path(interface).name != interface: 비었거나 '/' 가 있거나 '.' 이면 이름이 아니다.
  if (interface.empty() || interface.find('/') != std::string::npos || interface == ".") {
    throw std::runtime_error(
            "[MOTOR START BLOCKED] invalid CAN interface name: " + py_repr(interface));
  }
  const std::string flags_path = sys_class_net + "/" + interface + "/flags";
  int fd = -1;
  do {
    fd = ::open(flags_path.c_str(), O_RDONLY | O_CLOEXEC);
  } while (fd < 0 && errno == EINTR);
  int open_err = fd < 0 ? errno : 0;
  struct stat st{};
  if (fd >= 0 && ::fstat(fd, &st) == 0 && S_ISDIR(st.st_mode)) {
    // 파이썬 open() 은 디렉터리를 여는 순간 IsADirectoryError(경로 포함)를 던진다.
    ::close(fd);
    fd = -1;
    open_err = EISDIR;
  }
  if (fd < 0) {
    const int err = open_err;
    if (err == ENOENT) {  // FileNotFoundError
      throw std::runtime_error(
              "[MOTOR START BLOCKED] CAN interface " + interface + " does not exist");
    }
    // 여는 데 실패한 OSError 의 str() 에는 파일 경로가 repr 로 붙는다.
    throw std::runtime_error(
            "[MOTOR START BLOCKED] cannot read CAN interface " + interface + ": " + os_error(err) +
            ": " + py_repr(flags_path));
  }
  std::string raw;
  int read_err = 0;
  char buf[256];
  while (true) {
    const ssize_t n = ::read(fd, buf, sizeof(buf));
    if (n > 0) {
      raw.append(buf, static_cast<size_t>(n));
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0) {
      read_err = errno;
    }
    break;
  }
  ::close(fd);
  if (read_err != 0) {  // 읽기 중 OSError 에는 경로가 붙지 않는다
    throw std::runtime_error(
            "[MOTOR START BLOCKED] cannot read CAN interface " + interface + ": " +
            os_error(read_err));
  }
  const std::string stripped = py_strip(raw);
  int64_t flags = 0;
  if (!parse_py_int_base0(stripped, flags)) {
    throw std::runtime_error(
            "[MOTOR START BLOCKED] invalid flags for CAN interface " + interface + ": " +
            py_repr(stripped));
  }
  constexpr int64_t kIffUp = 0x1;
  if (!(flags & kIffUp)) {
    throw std::runtime_error(
            "[MOTOR START BLOCKED] CAN interface " + interface +
            " is DOWN; configure and bring it UP before starting the motor");
  }
  return flags;
}

}  // namespace mdrobot
