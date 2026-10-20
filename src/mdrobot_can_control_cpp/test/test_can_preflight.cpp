#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdlib>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

#include <sys/stat.h>

#include "mdrobot_can_control_cpp/socketcan.hpp"

namespace
{

class Preflight : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char tmpl[] = "/tmp/motor_preflight_XXXXXX";
    ASSERT_NE(::mkdtemp(tmpl), nullptr);
    root_ = tmpl;
  }

  void TearDown() override
  {
    const std::string cmd = "rm -rf '" + root_ + "'";
    ASSERT_EQ(std::system(cmd.c_str()), 0);
  }

  void write_flags(const std::string & iface, const std::string & value)
  {
    ASSERT_EQ(::mkdir((root_ + "/" + iface).c_str(), 0755), 0);
    std::ofstream(root_ + "/" + iface + "/flags") << value;
  }

  std::string error_of(const std::string & iface)
  {
    try {
      mdrobot::require_can_interface_up(iface, root_);
    } catch (const std::runtime_error & e) {
      return e.what();
    }
    return "";
  }

  std::string root_;
};

}

TEST_F(Preflight, accepts_interface_with_iff_up)
{
  write_flags("can1", "0x1\n");
  EXPECT_EQ(mdrobot::require_can_interface_up("can1", root_), 0x1);
}

TEST_F(Preflight, blocks_down_interface)
{
  write_flags("can1", "0x0\n");
  EXPECT_TRUE(std::regex_search(error_of("can1"), std::regex(R"(\[MOTOR START BLOCKED\].*can1.*DOWN)")));
}

TEST_F(Preflight, blocks_missing_interface)
{
  EXPECT_TRUE(
    std::regex_search(
      error_of("can1"), std::regex(R"(\[MOTOR START BLOCKED\].*can1.*does not exist)")));
}

TEST_F(Preflight, reads_real_sysfs_format)
{
  write_flags("can1", "0x40c1\n");
  EXPECT_EQ(mdrobot::require_can_interface_up("can1", root_), 0x40c1);
}

TEST(FormatHex, matches_python_format)
{
  EXPECT_EQ(mdrobot::format_hex_upper(0x40c1), "40C1");
  EXPECT_EQ(mdrobot::format_hex_upper(1, 3), "001");
  EXPECT_EQ(mdrobot::format_hex_upper(0x1abc, 3), "1ABC");
  EXPECT_EQ(mdrobot::format_hex_upper(-1, 3), "-01");
}
