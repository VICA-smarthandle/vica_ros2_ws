#include <cstdio>
#include <stdexcept>

#include "mdrobot_can_control_cpp/socketcan.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    return 2;
  }
  try {
    const int64_t flags = mdrobot::require_can_interface_up(argv[2], argv[1]);
    std::printf("OK 0x%s\n", mdrobot::format_hex_upper(flags).c_str());
  } catch (const std::runtime_error & e) {
    std::printf("ERR %s\n", e.what());
  }
  return 0;
}
