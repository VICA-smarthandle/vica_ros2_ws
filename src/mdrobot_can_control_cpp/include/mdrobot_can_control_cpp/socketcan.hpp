#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "mdrobot_can_control_cpp/motor_core.hpp"

namespace mdrobot
{

class SocketCan
{
public:
  explicit SocketCan(const std::string & channel);
  ~SocketCan();
  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  void shutdown();
  std::optional<Frame> recv_nowait();
  void send(const Frame & f);

private:
  int fd_ = -1;
};

std::array<uint8_t, 16> pack_classic_frame(const Frame & f);

int64_t require_can_interface_up(
  const std::string & interface, const std::string & sys_class_net = "/sys/class/net");

std::string format_hex_upper(int64_t value, int width = 0);

}
