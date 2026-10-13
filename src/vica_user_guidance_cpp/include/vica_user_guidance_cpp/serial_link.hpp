// Smart Handle 시리얼 래퍼 — 파이썬 판 serial_link.py 와 같은 동작.
//
// 포트가 없어도 예외를 밖으로 던지지 않는다. 전송·수신 실패는 반환값과 fault_code 로 보고하고
// 포트를 닫아, send 주기의 maybe_reconnect 가 2 초마다 다시 연다(케이블 재삽입 뒤 재기동 불필요).
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "vica_user_guidance_cpp/frames.hpp"

namespace vica_guidance
{

// SmartHandleState.msg 의 fault_code 상수.
constexpr int FAULT_NONE = 0;
constexpr int FAULT_PORT_OPEN = 1;
constexpr int FAULT_WRITE_FAIL = 2;
constexpr int FAULT_NOT_CONFIGURED = 3;

// 열린 포트 하나. 실패는 예외(어떤 std::exception 이든)로 알린다 — pyserial 과 같은 계약.
class SerialPort
{
public:
  virtual ~SerialPort() = default;
  virtual void write(uint8_t byte) = 0;          // pyserial write(bytes([b])) — write_timeout 적용
  virtual void flush() = 0;                      // pyserial flush() = tcdrain
  virtual std::size_t in_waiting() = 0;          // pyserial in_waiting = TIOCINQ
  virtual Bytes read(std::size_t n) = 0;         // pyserial read(n)
  virtual void close() = 0;
};

// pyserial serial.Serial(port=, baudrate=, timeout=, write_timeout=) 와 같은 인자. 실패는 예외.
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

  // 상태코드 1바이트. 보내면 안 되는 코드(4·6·7 등)는 프로그래밍 오류라 PyError(ValueError).
  bool send(int state_code, int64_t now_ns);
  // 상태코드가 아닌 바이트(햅틱). 0~7 은 PyError(ValueError) — 워치독 계약 우회 방지.
  bool send_raw(int byte, int64_t now_ns);
  // 쌓인 바이트 전부. 없거나 미연결이면 빈 값. 수신 중 단절은 전송 실패와 같은 경로.
  Bytes read_available(int64_t now_ns);
  // backoff 간격이 지났으면 재연결 1회. 첫 호출은 기준 시각만 잡는다.
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

// 진짜 tty — pyserial 3.5 serialposix 의 open·설정·write·flush·in_waiting·read·close 를 그대로 옮겼다.
//   열기: O_RDWR|O_NOCTTY|O_NONBLOCK, raw 8N1·흐름제어 없음·VMIN 0·VTIME 0, DTR·RTS 켬,
//         입력 버퍼 비움(TCIFLUSH). 아두이노 나노는 이 열기에서 리셋된다(파이썬 판과 같다).
//   쓰기: write_timeout 안에 끝나지 않으면 예외("Write timeout"). flush 는 tcdrain.
std::unique_ptr<SerialPort> open_posix_serial(
  const std::string & port, int64_t baudrate, double timeout, double write_timeout);

}  // namespace vica_guidance
