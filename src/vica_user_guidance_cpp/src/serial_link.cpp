#include "vica_user_guidance_cpp/serial_link.hpp"

#include <exception>
#include <utility>

#include "vica_user_guidance_cpp/pyfmt.hpp"

namespace vica_guidance
{

SerialLink::SerialLink(
  std::string port, int64_t baudrate, bool enabled, double write_timeout_sec,
  int64_t reconnect_interval_ns, SerialFactory factory)
: port_name_(std::move(port)), baudrate_(baudrate), enabled_(enabled),
  write_timeout_sec_(write_timeout_sec), reconnect_interval_ns_(reconnect_interval_ns),
  factory_(std::move(factory))
{
  if (enabled_) {open();}
}

bool SerialLink::write_byte(uint8_t byte, int64_t now_ns)
{
  try {
    port_->write(byte);
    port_->flush();
  } catch (const std::exception &) {
    // 어떤 예외든 밖으로 던지지 않는다. 안내 실패가 노드를 죽이면 안 된다.
    ++write_error_count_;
    fault_code_ = FAULT_WRITE_FAIL;
    close_port();
    last_attempt_ns_ = now_ns;
    return false;
  }
  return true;
}

bool SerialLink::send(int state_code, int64_t now_ns)
{
  if (!protocol::is_sendable(state_code)) {
    throw PyError(
            "ValueError", "전송할 수 없는 상태코드입니다: " + std::to_string(state_code) + " (" +
            protocol::state_name(state_code) +
            "). 코드 4는 펌웨어 워치독 전용이고 6·7은 펌웨어 미구현입니다.");
  }
  if (!port_) {return false;}
  if (!write_byte(static_cast<uint8_t>(state_code), now_ns)) {return false;}
  last_state_code_ = state_code;
  fault_code_ = FAULT_NONE;
  return true;
}

bool SerialLink::send_raw(int byte, int64_t now_ns)
{
  if (0 <= byte && byte <= protocol::STATE_CHARGED) {
    throw PyError(
            "ValueError", "상태코드 범위(" + std::to_string(byte) +
            ")는 send_raw 로 보내지 않는다. send() 를 쓸 것.");
  }
  if (!port_) {return false;}
  if (!write_byte(static_cast<uint8_t>(byte), now_ns)) {return false;}
  fault_code_ = FAULT_NONE;
  return true;
}

Bytes SerialLink::read_available(int64_t now_ns)
{
  if (!port_) {return {};}
  try {
    const std::size_t waiting = port_->in_waiting();
    if (!waiting) {return {};}
    return port_->read(waiting);
  } catch (const std::exception &) {
    // 포트를 닫아야 maybe_reconnect 가 살아난다.
    fault_code_ = FAULT_WRITE_FAIL;
    close_port();
    last_attempt_ns_ = now_ns;
    return {};
  }
}

void SerialLink::maybe_reconnect(int64_t now_ns)
{
  if (!enabled_ || port_) {return;}
  if (!last_attempt_ns_) {
    last_attempt_ns_ = now_ns;
    return;
  }
  const int64_t elapsed = now_ns - *last_attempt_ns_;
  if (0 <= elapsed && elapsed < reconnect_interval_ns_) {return;}
  last_attempt_ns_ = now_ns;
  open();
}

void SerialLink::open()
{
  try {
    port_ = factory_(port_name_, baudrate_, 1.0, write_timeout_sec_);
  } catch (const std::exception &) {
    port_.reset();
    fault_code_ = FAULT_PORT_OPEN;
    return;
  }
  fault_code_ = FAULT_NONE;
}

void SerialLink::close_port()
{
  if (port_) {
    try {
      port_->close();
    } catch (const std::exception &) {
    }
    port_.reset();
  }
}

}  // namespace vica_guidance
