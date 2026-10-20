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

std::string operation_error(const char * prefix, int err)
{
  return std::string(prefix) + ": " + std::strerror(err) + " [Error Code " + std::to_string(err) +
         "]";
}

std::string os_error(int err)
{
  return "[Errno " + std::to_string(err) + "] " + std::strerror(err);
}

constexpr const char * kClosedSocket = "file descriptor cannot be a negative integer (-1)";
constexpr const char * kFdOutOfRange = "filedescriptor out of range in select()";

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
  size_t j = 0;
  if (base != 10 && body[0] == '_') {
    j = 1;
  }
  if (j >= body.size()) {
    return false;
  }
  uint64_t value = 0;
  constexpr uint64_t kLimit = static_cast<uint64_t>(INT64_MAX) + 1;
  bool prev_underscore = true;
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
      return false;
    }
    value = value * static_cast<uint64_t>(base) + static_cast<uint64_t>(d);
  }
  if (prev_underscore) {
    return false;
  }
  if (base == 10 && first_digit_zero && !all_zero) {
    return false;
  }
  if (!negative && value == kLimit) {
    return false;
  }
  out = negative ? static_cast<int64_t>(~value + 1) : static_cast<int64_t>(value);
  return true;
}

}

SocketCan::SocketCan(const std::string & channel)
{
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw CanFailure(os_error(errno));
  }
  int one = 1;
  int zero = 0;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &one, sizeof(one));
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &zero, sizeof(zero));
  can_err_mask_t err_mask = 0x1FFFFFFF;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
  ::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one));

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
  do {
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

  Frame f;
  f.id = (cf.can_id & CAN_EFF_FLAG) ? (cf.can_id & CAN_EFF_MASK) : (cf.can_id & CAN_SFF_MASK);
  if (cf.can_id & CAN_RTR_FLAG) {
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
  const int pad = width - static_cast<int>(digits.size()) - (negative ? 1 : 0);
  if (pad > 0) {
    digits.insert(0, static_cast<size_t>(pad), '0');
  }
  return negative ? "-" + digits : digits;
}

int64_t require_can_interface_up(const std::string & interface, const std::string & sys_class_net)
{
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
    ::close(fd);
    fd = -1;
    open_err = EISDIR;
  }
  if (fd < 0) {
    const int err = open_err;
    if (err == ENOENT) {
      throw std::runtime_error(
              "[MOTOR START BLOCKED] CAN interface " + interface + " does not exist");
    }
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
  if (read_err != 0) {
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

}
