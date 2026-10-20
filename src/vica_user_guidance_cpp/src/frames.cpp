#include "vica_user_guidance_cpp/frames.hpp"

namespace vica_guidance
{

std::vector<UltrasonicFrame> FrameAccumulator::feed(const Bytes & data)
{
  return scan_.feed<UltrasonicFrame>(data, [](const uint8_t * c) {
             UltrasonicFrame f;
             f.seq = c[2];
             for (int ch = 0; ch < protocol::US_CHANNELS; ++ch) {
               f.distances_mm[ch] = c[3 + 2 * ch] | (c[4 + 2 * ch] << 8);
             }
             return f;
           });
}

std::vector<TouchFrame> TouchFrameAccumulator::feed(const Bytes & data)
{
  return scan_.feed<TouchFrame>(data, [](const uint8_t * c) {
             TouchFrame f;
             f.seq = c[2];
             f.touched = (c[3] & protocol::TOUCH_FLAG_CONTACT) != 0;
             return f;
           });
}

std::vector<UltrasonicStatFrame> StatFrameAccumulator::feed(const Bytes & data)
{
  return scan_.feed<UltrasonicStatFrame>(data, [](const uint8_t * c) {
             UltrasonicStatFrame f;
             f.seq = c[2];
             for (int ch = 0; ch < protocol::US_CHANNELS; ++ch) {
               for (int k = 0; k < protocol::US_STAT_KINDS; ++k) {
                 f.counts[ch][k] = c[3 + ch * protocol::US_STAT_KINDS + k];
               }
             }
             return f;
           });
}

std::vector<UltrasonicConfigFrame> ConfigFrameAccumulator::feed(const Bytes & data)
{
  return scan_.feed<UltrasonicConfigFrame>(data, [](const uint8_t * c) {
             UltrasonicConfigFrame f;
             f.seq = c[2];
             for (int ch = 0; ch < protocol::US_CHANNELS; ++ch) {
               f.levels[ch] = {c[3 + 2 * ch], c[4 + 2 * ch]};
             }
             return f;
           });
}

namespace
{
std::string channel_name(const std::vector<std::string> & names, int ch)
{
  return ch < static_cast<int>(names.size()) ? names[ch] : "ch" + std::to_string(ch);
}
}

std::string format_stat_line(
  const UltrasonicStatFrame & frame, const std::vector<std::string> & channel_names)
{
  std::string parts;
  int total_fffe = 0;
  for (int ch = 0; ch < protocol::US_CHANNELS; ++ch) {
    if (ch) {parts += " · ";}
    parts += channel_name(channel_names, ch) + " ";
    for (int k = 0; k < protocol::US_STAT_KINDS; ++k) {
      if (k) {parts += "/";}
      parts += std::to_string(frame.counts[ch][k]);
    }
    total_fffe += frame.counts[ch][protocol::US_STAT_FFFE_INDEX];
  }
  return "[US 통계 #" + std::to_string(frame.seq) + "] (ok/clr/ffff/fffe/oth/i2c) " + parts +
         "  fffe 합 " + std::to_string(total_fffe);
}

std::string format_config_line(
  const UltrasonicConfigFrame & frame, const std::vector<std::string> & channel_names)
{
  std::string parts;
  for (int ch = 0; ch < protocol::US_CHANNELS; ++ch) {
    if (ch) {parts += " · ";}
    const int a = frame.levels[ch][0];
    const int nz = frame.levels[ch][1];
    parts += channel_name(channel_names, ch) + " 각" + (a == 0xFF ? "?" : std::to_string(a)) +
      "/잡음" + (nz == 0xFF ? "?" : std::to_string(nz));
  }
  return "[US 설정 #" + std::to_string(frame.seq) + "] (센서 되읽기) " + parts;
}

}
