// python-can SocketcanBus(interface='socketcan', 기본 인자) 와 같게 움직이는 raw CAN 소켓,
// 그리고 can_preflight.require_can_interface_up.
//
// 맞춘 것(python-can 4.6.1 socketcan.py):
//   - 소켓 옵션: LOOPBACK 1, RECV_OWN_MSGS 0, ERR_FILTER 0x1FFFFFFF(오류 프레임도 받음), SO_TIMESTAMPNS 1
//   - recv(timeout=0.0): select 0초 → 읽을 게 있으면 recvmsg 하나
//   - send(msg): select(쓰기, 0초) 가 아니면 "Transmit buffer full", send 실패는 "Failed to transmit: …"
//   - 오류 글자는 str(exc) 그대로("… [Error Code N]", "[Errno N] …")
//   - bind 실패 때 python-can 이 stderr 에 찍는 "Could not access SocketCAN device …" 한 줄
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
  // 실패하면 CanFailure. 그때 열린 소켓은 남기지 않는다.
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

// python-can build_can_frame 과 같은 16 바이트: 헤더 "=IBB1xB"(can_id, len, flags=0, len8_dlc=dlc)
// + data 8 바이트. SocketCan::send 가 이것을 그대로 보내고, 대조 시험도 이것을 찍는다.
std::array<uint8_t, 16> pack_classic_frame(const Frame & f);

// /sys/class/net/<iface>/flags 를 읽어 IFF_UP 이 아니면 기동을 막는다(std::runtime_error).
// 글자는 파이썬 판 RuntimeError 와 같다. 돌려주는 값은 flags.
int64_t require_can_interface_up(
  const std::string & interface, const std::string & sys_class_net = "/sys/class/net");

// 파이썬 format(value, '0{width}X') 와 같은 글자. 음수는 '-' 를 앞에 두고 남는 폭을 0 으로 채운다.
std::string format_hex_upper(int64_t value, int width = 0);

}  // namespace mdrobot
