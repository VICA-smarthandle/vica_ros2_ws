// 상향 프레임 누적기 — 파이썬 판 ultrasonic_frame.py · touch_frame.py · ultrasonic_stats.py 와 같은 동작.
//
// 시리얼은 바이트 경계를 보장하지 않는다. 한 번 읽은 덩어리에 프레임이 반쪽만 오거나 여러 개가
// 붙어 오고, 재연결 직후에는 쓰레기 바이트로 시작한다. 그래서 스트림을 쌓아 두고 헤더를 찾으며,
// 체크섬이 깨진 헤더 후보는 프레임 길이만큼이 아니라 1바이트만 버린다(진짜 프레임 중간에
// 우연히 헤더 모양이 나와도 뒤따르는 진짜 프레임을 잃지 않게).
//
// 네 누적기는 같은 스트림을 각자 훑는다(파이썬 판과 같다). 자기 헤더가 아닌 바이트는 1개씩
// 지나치므로 서로를 삼키지 않는다.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "vica_user_guidance_cpp/protocol.hpp"

namespace vica_guidance
{

using Bytes = std::vector<uint8_t>;

struct UltrasonicFrame
{
  int seq = 0;
  // 펌웨어 원시값: 0 = 무효, 1~3000 = mm, 3001 = 에코 없음(clear)
  std::array<int, protocol::US_CHANNELS> distances_mm{};
};

struct TouchFrame
{
  int seq = 0;
  bool touched = false;
};

struct UltrasonicStatFrame
{
  int seq = 0;
  std::array<std::array<int, protocol::US_STAT_KINDS>, protocol::US_CHANNELS> counts{};
};

struct UltrasonicConfigFrame
{
  int seq = 0;
  std::array<std::array<int, 2>, protocol::US_CHANNELS> levels{};   // (지향각, 잡음 저감), 0xFF = 되읽기 실패
};

namespace detail
{

// 헤더·길이·최대 버퍼가 다른 네 누적기의 공통 몸통.
class StreamScanner
{
public:
  StreamScanner(const uint8_t header[2], std::size_t frame_len, std::size_t max_buffer)
  : h0_(header[0]), h1_(header[1]), len_(frame_len), max_(max_buffer) {}

  // 완성된 프레임마다 parse(chunk 시작) 를 불러 성공한 것만 모은다.
  template<typename Frame, typename Parse>
  std::vector<Frame> feed(const Bytes & data, Parse parse)
  {
    buf_.insert(buf_.end(), data.begin(), data.end());
    std::vector<Frame> frames;
    std::size_t i = 0;
    while (buf_.size() - i >= len_) {
      if (buf_[i] != h0_ || buf_[i + 1] != h1_) {++i; continue;}
      const uint8_t * chunk = buf_.data() + i;
      uint8_t x = 0;
      for (std::size_t k = 2; k + 1 < len_; ++k) {x ^= chunk[k];}   // seq ~ 본문 끝
      if (x != chunk[len_ - 1]) {++i; continue;}
      frames.push_back(parse(chunk));
      i += len_;
    }
    buf_.erase(buf_.begin(), buf_.begin() + static_cast<std::ptrdiff_t>(i));
    if (buf_.size() > max_) {
      buf_.erase(buf_.begin(), buf_.end() - static_cast<std::ptrdiff_t>(max_));
    }
    return frames;
  }

  std::size_t buffered() const {return buf_.size();}

private:
  uint8_t h0_, h1_;
  std::size_t len_, max_;
  Bytes buf_;
};

}  // namespace detail

// AA 57 seq d0L d0H … d7L d7H xor (20 B). 버퍼 상한 = 32 × 20.
class FrameAccumulator
{
public:
  FrameAccumulator()
  : scan_(protocol::US_FRAME_HEADER, protocol::US_FRAME_LEN, 32 * protocol::US_FRAME_LEN) {}
  std::vector<UltrasonicFrame> feed(const Bytes & data);
  std::size_t buffered() const {return scan_.buffered();}

private:
  detail::StreamScanner scan_;
};

// AA 56 seq flags xor (5 B). 버퍼 상한 256.
class TouchFrameAccumulator
{
public:
  TouchFrameAccumulator()
  : scan_(protocol::TOUCH_FRAME_HEADER, protocol::TOUCH_FRAME_LEN, 256) {}
  std::vector<TouchFrame> feed(const Bytes & data);
  std::size_t buffered() const {return scan_.buffered();}

private:
  detail::StreamScanner scan_;
};

// AA 58 seq [ch0: ok clr ffff fffe oth i2c] … xor (52 B). 버퍼 상한 8 × 52.
class StatFrameAccumulator
{
public:
  StatFrameAccumulator()
  : scan_(protocol::US_STAT_FRAME_HEADER, protocol::US_STAT_FRAME_LEN,
      8 * protocol::US_STAT_FRAME_LEN) {}
  std::vector<UltrasonicStatFrame> feed(const Bytes & data);
  std::size_t buffered() const {return scan_.buffered();}

private:
  detail::StreamScanner scan_;
};

// AA 59 seq [ch0: angle noise] … xor (20 B). 버퍼 상한은 통계와 같은 8 × 52(파이썬 판이 같은 상수를 쓴다).
class ConfigFrameAccumulator
{
public:
  ConfigFrameAccumulator()
  : scan_(protocol::US_CFG_FRAME_HEADER, protocol::US_CFG_FRAME_LEN,
      8 * protocol::US_STAT_FRAME_LEN) {}
  std::vector<UltrasonicConfigFrame> feed(const Bytes & data);
  std::size_t buffered() const {return scan_.buffered();}

private:
  detail::StreamScanner scan_;
};

// "[US 통계 #3] (ok/clr/ffff/fffe/oth/i2c) left_wheel 12/0/0/0/0/0 · …  fffe 합 1"
std::string format_stat_line(
  const UltrasonicStatFrame & frame, const std::vector<std::string> & channel_names);

// "[US 설정 #1] (센서 되읽기) left_wheel 각2/잡음1 · …"
std::string format_config_line(
  const UltrasonicConfigFrame & frame, const std::vector<std::string> & channel_names);

// touch_frame.resolve_contact — 상향이 끊겼으면 마지막 값을 붙들지 않고 false.
inline bool resolve_contact(bool touched, bool uplink_fresh) {return touched && uplink_fresh;}

}  // namespace vica_guidance
