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
  std::array<std::array<int, 2>, protocol::US_CHANNELS> levels{};
};

namespace detail
{

class StreamScanner
{
public:
  StreamScanner(const uint8_t header[2], std::size_t frame_len, std::size_t max_buffer)
  : h0_(header[0]), h1_(header[1]), len_(frame_len), max_(max_buffer) {}

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
      for (std::size_t k = 2; k + 1 < len_; ++k) {x ^= chunk[k];}
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

}

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

std::string format_stat_line(
  const UltrasonicStatFrame & frame, const std::vector<std::string> & channel_names);

std::string format_config_line(
  const UltrasonicConfigFrame & frame, const std::vector<std::string> & channel_names);

inline bool resolve_contact(bool touched, bool uplink_fresh) {return touched && uplink_fresh;}

}
