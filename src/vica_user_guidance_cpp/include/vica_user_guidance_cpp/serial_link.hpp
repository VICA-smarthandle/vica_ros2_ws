#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "vica_user_guidance_cpp/frames.hpp"

namespace vica_guidance
{

constexpr int FAULT_NONE = 0;
constexpr int FAULT_PORT_OPEN = 1;
constexpr int FAULT_WRITE_FAIL = 2;
constexpr int FAULT_NOT_CONFIGURED = 3;

class SerialPort
{
public:
  virtual ~SerialPort() = default;
  virtual void write(uint8_t byte) = 0;
  virtual void flush() = 0;
  virtual std::size_t in_waiting() = 0;
  virtual Bytes read(std::size_t n) = 0;
  virtual void close() = 0;
};

using SerialFactory = std::function<std::unique_ptr<SerialPort>(
      const std::string & port, int64_t baudrate, double timeout, double write_timeout)>;

class SerialLink
{
public:
  SerialLink(
    std::string port, int64_t baudrate, bool enabled, double write_timeout_sec,
    int64_t reconnect_interval_ns, SerialFactory factory);

  bool connected() const {return port_ != nullptr;}
  int fault_code() const {return fault_code_;}
  int last_state_code() const {return last_state_code_;}
  int64_t write_error_count() const {return write_error_count_;}

  bool send(int state_code, int64_t now_ns);
  bool send_raw(int byte, int64_t now_ns);
  Bytes read_available(int64_t now_ns);
  void maybe_reconnect(int64_t now_ns);
  void close() {close_port();}

private:
  void open();
  void close_port();
  bool write_byte(uint8_t byte, int64_t now_ns);

  std::string port_name_;
  int64_t baudrate_;
  bool enabled_;
  double write_timeout_sec_;
  int64_t reconnect_interval_ns_;
  SerialFactory factory_;
  std::unique_ptr<SerialPort> port_;
  int fault_code_ = FAULT_NOT_CONFIGURED;
  int last_state_code_ = protocol::STATE_NORMAL;
  int64_t write_error_count_ = 0;
  std::optional<int64_t> last_attempt_ns_;
};

std::unique_ptr<SerialPort> open_posix_serial(
  const std::string & port, int64_t baudrate, double timeout, double write_timeout);

}
