// 부품 시험. 판정 전체의 정답 대조는 test/parity(파이썬 원본과 줄 단위 비교)가 맡고, 여기는
// 파이썬 판 시험(vica_user_guidance/test)의 핵심 사례와 진짜 tty(pty) 동작을 본다.
#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "vica_user_guidance_cpp/frames.hpp"
#include "vica_user_guidance_cpp/guidance.hpp"
#include "vica_user_guidance_cpp/pyfmt.hpp"
#include "vica_user_guidance_cpp/pyjson.hpp"
#include "vica_user_guidance_cpp/serial_link.hpp"

using namespace vica_guidance;   // NOLINT

namespace
{

Bytes frame(uint8_t h1, const Bytes & body)
{
  Bytes f{0xAA, h1};
  uint8_t x = 0;
  for (uint8_t b : body) {f.push_back(b); x ^= b;}
  f.push_back(x);
  return f;
}

Bytes us_frame(uint8_t seq, const std::vector<int> & mm)
{
  Bytes body{seq};
  for (int d : mm) {body.push_back(static_cast<uint8_t>(d & 0xFF)); body.push_back(static_cast<uint8_t>(d >> 8));}
  return frame(0x57, body);
}

}  // namespace

// ── 표기(파이썬 repr) ──

TEST(PyFmt, FloatReprMatchesPython)
{
  EXPECT_EQ(py_float(0.4), "0.4");
  EXPECT_EQ(py_float(1.5), "1.5");
  EXPECT_EQ(py_float(3.0), "3.0");
  EXPECT_EQ(py_float(0.349), "0.349");
  EXPECT_EQ(py_float(1e16), "1e+16");
  EXPECT_EQ(py_float(1e-5), "1e-05");
  EXPECT_EQ(py_float(0.0001), "0.0001");
  EXPECT_EQ(py_float(1234567890123456.0), "1234567890123456.0");
  EXPECT_EQ(py_float(-0.0), "-0.0");
  EXPECT_EQ(py_float(1e22), "1e+22");
  EXPECT_EQ(py_float(std::nan("")), "nan");
  EXPECT_EQ(py_float(-std::numeric_limits<double>::infinity()), "-inf");
}

TEST(PyFmt, StrReprMatchesPython)
{
  EXPECT_EQ(py_str_repr("a'b"), "\"a'b\"");
  EXPECT_EQ(py_str_repr("a\"b"), "'a\"b'");
  EXPECT_EQ(py_str_repr("a'\"b"), "'a\\'\"b'");
  // repr('\x7f\x85\xa0\u200b\u3000가😀\n') == '\x7f\x85\xa0\u200b\u3000가😀\n'
  EXPECT_EQ(
    py_str_repr("\x7f\xc2\x85\xc2\xa0\xe2\x80\x8b\xe3\x80\x80가😀\n"),
    "'\\x7f\\x85\\xa0\\u200b\\u3000가😀\\n'");
  EXPECT_EQ(py_list_repr(std::vector<std::string>{"/u/a", "b"}), "['/u/a', 'b']");
}

TEST(PyFmt, IntConversionsMatchPython)
{
  EXPECT_EQ(sec_to_ns(1.0), 1000000000);
  EXPECT_EQ(sec_to_ns(0.05), 50000000);
  EXPECT_EQ(sec_to_ns(-1.5), -1500000000);
  EXPECT_EQ(sec_to_ns(1e10), std::numeric_limits<int64_t>::max());
  EXPECT_THROW(sec_to_ns(std::nan("")), PyError);
  EXPECT_EQ(py_round_decimal(2.5), "2");    // 짝수 쪽
  EXPECT_EQ(py_round_decimal(3.5), "4");
  EXPECT_EQ(py_round_decimal(-0.4), "0");
  EXPECT_EQ(py_round_decimal(1e20), "100000000000000000000");
  EXPECT_EQ(py_strip("\u3000 Long\t\n"), "Long");
  EXPECT_EQ(py_lower_for_lookup("TIC\u212A"), "tick");
  EXPECT_EQ(py_slice("가나다라", 2), "가나");
}

// ── JSON(파이썬 json.loads 문법) ──

TEST(PyJson, AcceptsWhatPythonAccepts)
{
  EXPECT_EQ(parse_goal_event("{\"event\": \"goal_succeeded\", \"x\": NaN}").value(), "goal_succeeded");
  EXPECT_EQ(parse_goal_event("{\"event\":\"a\",\"event\":\"goal_succeeded\"}").value(), "goal_succeeded");
  EXPECT_EQ(parse_goal_event(" {\"event\":\"goal_\\u0073ucceeded\"} \n").value(), "goal_succeeded");
  EXPECT_EQ(parse_goal_event("{\"event\":\"x\",\"n\":-Infinity,\"m\":1e400}").value(), "x");
}

TEST(PyJson, RejectsWhatPythonRejects)
{
  for (const char * bad : {"", "not json", "{\"event\":\"a\"} x", "\xEF\xBB\xBF{\"event\":\"a\"}",
      "{\"event\":\"a\",}", "{\"event\":\"a\tb\"}", "{'event':'a'}", "{\"x\":01}", "{\"x\":.5}",
      "{\"x\":5.}", "{\"x\":-}", "{\"x\":infinity}", "{\"k\":\"\\x\"}", "{\"event\":\"a\""})
  {
    EXPECT_FALSE(py_json_loads(bad).has_value()) << bad;
  }
  EXPECT_FALSE(parse_goal_event("[1, 2]").has_value());
  EXPECT_FALSE(parse_goal_event("{\"event\": 5}").has_value());
  EXPECT_FALSE(parse_goal_event("{\"event\":\"goal_succeeded\",\"event\":1}").has_value());
}

// ── 상향 프레임 ──

TEST(Frames, SplitGarbageAndCorruptFramesLikePython)
{
  FrameAccumulator acc;
  const Bytes good = us_frame(7, {100, 3001, 0, 2999, 1, 400, 401, 65535});
  Bytes stream{0x00, 0xAA, 0x01};                  // 쓰레기
  Bytes bad = us_frame(8, {1, 2, 3, 4, 5, 6, 7, 8});
  bad[5] ^= 0x10;                                  // 체크섬 깨짐
  stream.insert(stream.end(), bad.begin(), bad.end());
  stream.insert(stream.end(), good.begin(), good.begin() + 9);   // 반쪽
  EXPECT_TRUE(acc.feed(stream).empty());
  const auto frames = acc.feed(Bytes(good.begin() + 9, good.end()));
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].seq, 7);
  EXPECT_EQ(frames[0].distances_mm[1], 3001);
  EXPECT_EQ(frames[0].distances_mm[7], 65535);
}

TEST(Frames, BufferIsCappedLikePython)
{
  FrameAccumulator acc;
  acc.feed(Bytes(5000, 0xAA));
  EXPECT_LE(acc.buffered(), 32 * protocol::US_FRAME_LEN);
}

TEST(Frames, TouchStatAndConfigFrames)
{
  TouchFrameAccumulator touch;
  const auto t = touch.feed(frame(0x56, {3, 0x01}));
  ASSERT_EQ(t.size(), 1u);
  EXPECT_TRUE(t[0].touched);

  StatFrameAccumulator stat;
  Bytes body{9};
  for (int k = 0; k < 48; ++k) {body.push_back(static_cast<uint8_t>(k % 6 == 3 ? 2 : 0));}
  const auto s = stat.feed(frame(0x58, body));
  ASSERT_EQ(s.size(), 1u);
  const std::vector<std::string> names{"a", "b"};
  EXPECT_EQ(
    format_stat_line(s[0], names),
    "[US 통계 #9] (ok/clr/ffff/fffe/oth/i2c) a 0/0/0/2/0/0 · b 0/0/0/2/0/0 · ch2 0/0/0/2/0/0 · "
    "ch3 0/0/0/2/0/0 · ch4 0/0/0/2/0/0 · ch5 0/0/0/2/0/0 · ch6 0/0/0/2/0/0 · ch7 0/0/0/2/0/0  "
    "fffe 합 16");
}

// ── 우선순위·게이트 ──

TEST(Priority, EstopArrivalTurnOrder)
{
  GuidanceInputs in;
  in.estop_last_ns = 1000;
  in.turn_direction = DIRECTION_LEFT;
  in.turn_last_ns = 1000;
  // 왼쪽이면 왼쪽(여기서 뒤집지 않는다)
  EXPECT_STREQ(resolve_state_code(in, 1500, 1000, 1000, 4000, true).reason, "turn_left");
  in.arrival_started_ns = 1200;
  EXPECT_EQ(resolve_state_code(in, 1500, 1000, 1000, 4000, true).state_code, protocol::STATE_ARRIVED);
  EXPECT_EQ(resolve_state_code(in, 3000, 1000, 10000, 4000, true).state_code, protocol::STATE_ESTOP);
  EXPECT_STREQ(resolve_state_code(in, 3000, 1000, 10000, 4000, true).reason, "estop_stale");
  EXPECT_EQ(resolve_state_code(in, 3000, 1000, 10000, 4000, false).state_code, protocol::STATE_ARRIVED);
  in.estop_active = true;
  EXPECT_STREQ(resolve_state_code(in, 1500, 1000, 1000, 4000, true).reason, "estop_active");
  // 시간 역전은 stale
  EXPECT_FALSE(is_fresh_ns(2000, 1999, 1000));
}

// ── 순찰 표시(2026-11-14, 경비 모드 명세 4.D) ──

TEST(Patrol, DialogStateToCode)
{
  EXPECT_EQ(patrol_state_code("patrolling"), protocol::STATE_PATROL);
  EXPECT_EQ(patrol_state_code("patrol_dwell"), protocol::STATE_PATROL);
  EXPECT_EQ(patrol_state_code("patrol_paused"), protocol::STATE_PATROL);
  EXPECT_EQ(patrol_state_code("patrol_light"), protocol::STATE_PATROL_LIGHT);
  EXPECT_EQ(patrol_state_code("patrol_alarm"), protocol::STATE_PATROL_ALARM);
  EXPECT_EQ(protocol::STATE_PATROL, 8);
  EXPECT_EQ(protocol::STATE_PATROL_LIGHT, 9);
  EXPECT_EQ(protocol::STATE_PATROL_ALARM, 10);
  // 순찰이 아닌 값·닮은 값
  for (const char * s : {"", "idle", "navigating", "paused", "estopped", "waiting", "Patrolling",
      " patrol_alarm", "patro"})
  {
    EXPECT_EQ(patrol_state_code(s), PATROL_NONE) << s;
  }
  // "patrol" 로 시작하는 모르는 값은 순찰(8)로 본다(명세 2절)
  EXPECT_EQ(patrol_state_code("patrol_alarm2"), protocol::STATE_PATROL);
}

TEST(Patrol, SitsBetweenEstopAndArrival)
{
  const int64_t sec = 1000000000;
  const int64_t now = 100 * sec;
  GuidanceInputs in;
  in.estop_last_ns = now;
  in.turn_direction = DIRECTION_LEFT;
  in.turn_last_ns = now;
  in.arrival_started_ns = now;
  auto run = [&](bool estop_required = true) {
      return resolve_state_code(in, now, sec, sec, 4 * sec, estop_required);
    };
  EXPECT_EQ(run().state_code, protocol::STATE_ARRIVED);   // 순찰 값이 없으면 기존 판단
  in.patrol_last_ns = now;
  for (int code : {protocol::STATE_PATROL, protocol::STATE_PATROL_LIGHT, protocol::STATE_DOCKED,
      protocol::STATE_PATROL_ALARM})
  {
    in.patrol_code = code;
    in.estop_active = false;
    EXPECT_EQ(run().state_code, code);              // 도착·회전보다 위
    EXPECT_TRUE(protocol::is_sendable(run().state_code));
    in.estop_active = true;
    EXPECT_EQ(run().state_code, protocol::STATE_ESTOP);   // 비상정지가 위
  }
  in.estop_active = false;
  in.estop_last_ns = std::nullopt;
  EXPECT_STREQ(run().reason, "estop_stale");        // /estop_state 끊김도 순찰보다 위
  EXPECT_STREQ(run(false).reason, "patrol_alarm");
  // 순찰 코드가 아닌 값(0·4·6·7 …)은 버린다
  in.estop_last_ns = now;
  for (int code : {0, 1, 3, 4, 5, 7, 11, -1}) {   // 6 은 2026-11-18 도킹(docked)
    in.patrol_code = code;
    EXPECT_EQ(run().state_code, protocol::STATE_ARRIVED) << code;
  }
}

TEST(Patrol, DroppedWhenRobotStateIsSilentForThreeSeconds)
{
  const int64_t sec = 1000000000;
  const int64_t now = 100 * sec;
  GuidanceInputs in;
  in.estop_last_ns = now;
  in.turn_direction = DIRECTION_RIGHT;
  in.turn_last_ns = now;
  in.patrol_code = protocol::STATE_PATROL;
  auto code = [&]() {return resolve_state_code(in, now, sec, sec, 4 * sec, true).state_code;};
  EXPECT_EQ(PATROL_STATE_TIMEOUT_NS, 3 * sec);
  in.patrol_last_ns = now - 3 * sec;               // 딱 3 s 까지는 유지
  EXPECT_EQ(code(), protocol::STATE_PATROL);
  in.patrol_last_ns = now - 3 * sec - 1;           // 넘으면 기존 판단(회전)
  EXPECT_EQ(code(), protocol::STATE_RIGHT);
  in.patrol_last_ns = std::nullopt;                // 받은 적 없음
  EXPECT_EQ(code(), protocol::STATE_RIGHT);
  in.patrol_last_ns = now + 1;                     // 시간 역전은 stale
  EXPECT_EQ(code(), protocol::STATE_RIGHT);
  EXPECT_EQ(active_patrol_code(protocol::STATE_PATROL_ALARM, now, now), protocol::STATE_PATROL_ALARM);
  EXPECT_EQ(active_patrol_code(protocol::STATE_PATROL_ALARM, now - 4 * sec, now), PATROL_NONE);
}

TEST(Gate, ReportsTransitionOnce)
{
  RangeTfGate gate(2);
  EXPECT_FALSE(gate.take_transition(0).has_value());
  EXPECT_FALSE(gate.allow(0, false));
  EXPECT_EQ(gate.take_transition(0), std::optional<bool>(false));
  EXPECT_FALSE(gate.take_transition(0).has_value());
  gate.allow(0, false);
  EXPECT_TRUE(gate.allow(0, true));
  EXPECT_EQ(gate.take_transition(0), std::optional<bool>(true));
  EXPECT_EQ(gate.blocked_count(0), 2);
}

// ── 진짜 tty: pty 로 손잡이 보드 흉내 ──

class PtyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char name[128];
    ASSERT_EQ(openpty(&master_, &slave_, name, nullptr, nullptr), 0);
    path_ = name;
    ::close(slave_);   // 슬레이브는 시험 대상이 연다
  }
  void TearDown() override
  {
    if (master_ >= 0) {::close(master_);}
  }
  int master_ = -1, slave_ = -1;
  std::string path_;
};

TEST_F(PtyTest, WriteReadAndHangupLikePyserial)
{
  auto port = open_posix_serial(path_, 115200, 1.0, 0.05);
  port->write(0x03);
  port->flush();
  uint8_t got = 0;
  ASSERT_EQ(::read(master_, &got, 1), 1);
  EXPECT_EQ(got, 0x03);

  const uint8_t up[5] = {0xAA, 0x56, 0x01, 0x01, 0x00};
  ASSERT_EQ(::write(master_, up, 5), 5);
  for (int k = 0; k < 100 && port->in_waiting() < 5; ++k) {usleep(1000);}
  ASSERT_EQ(port->in_waiting(), 5u);
  const Bytes data = port->read(5);
  EXPECT_EQ(data, Bytes(up, up + 5));

  // 선을 뽑은 것처럼 상대를 닫는다 → 읽기 쪽이 예외로 알린다(SerialLink 가 포트를 닫고 재연결).
  ::close(master_);
  master_ = -1;
  EXPECT_ANY_THROW({port->in_waiting(); port->read(1); port->write(0x00); port->flush();});
}

TEST_F(PtyTest, LinkRecoversAfterHangup)
{
  int opens = 0;
  SerialLink link(path_, 115200, true, 0.05, 2000000000,
    [&](const std::string & p, int64_t b, double t, double w) {++opens; return open_posix_serial(p, b, t, w);});
  ASSERT_TRUE(link.connected());
  EXPECT_TRUE(link.send(protocol::STATE_LEFT, 0));
  EXPECT_EQ(link.last_state_code(), protocol::STATE_LEFT);
  ::close(master_);
  master_ = -1;
  for (int k = 0; k < 5 && link.connected(); ++k) {
    link.read_available(10);
    link.send(protocol::STATE_NORMAL, 10);
  }
  EXPECT_FALSE(link.connected());
  EXPECT_EQ(link.fault_code(), FAULT_WRITE_FAIL);
  link.maybe_reconnect(1000000000);   // 2 초 안 — 아직 안 연다
  EXPECT_EQ(opens, 1);
}

TEST(SerialLinkRules, RefusesWatchdogAndStateCodesOnRawPath)
{
  SerialLink link("/nonexistent", 115200, false, 0.05, 2000000000, open_posix_serial);
  EXPECT_EQ(link.fault_code(), FAULT_NOT_CONFIGURED);
  EXPECT_THROW(link.send(protocol::STATE_LINK_LOST, 0), PyError);
  EXPECT_THROW(link.send_raw(protocol::STATE_ESTOP, 0), PyError);
  EXPECT_FALSE(link.send(protocol::STATE_NORMAL, 0));
  // 순찰 코드 8·9·10 은 send() 로만 나간다. 6·7 은 여전히 못 보낸다(2026-11-14).
  for (int code : {protocol::STATE_PATROL, protocol::STATE_PATROL_LIGHT,
      protocol::STATE_PATROL_ALARM})
  {
    EXPECT_NO_THROW(link.send(code, 0));
    EXPECT_THROW(link.send_raw(code, 0), PyError);
  }
  EXPECT_THROW(link.send(protocol::STATE_CHARGED, 0), PyError);
  EXPECT_THROW(link.send(11, 0), PyError);
}

TEST_F(PtyTest, PatrolCodesGoOutAsOneByte)
{
  SerialLink link(path_, 115200, true, 0.05, 2000000000, open_posix_serial);
  ASSERT_TRUE(link.connected());
  for (int code : {protocol::STATE_PATROL, protocol::STATE_PATROL_LIGHT,
      protocol::STATE_PATROL_ALARM})
  {
    EXPECT_TRUE(link.send(code, 0));
    uint8_t got = 0xFF;
    ASSERT_EQ(::read(master_, &got, 1), 1);
    EXPECT_EQ(got, code);   // 10 = 0x0A — tty 가 줄바꿈으로 바꾸거나 삼키지 않아야 한다
    EXPECT_EQ(link.last_state_code(), code);
  }
}

TEST(SerialLinkRules, MissingPortIsPortOpenFault)
{
  SerialLink link("/dev/vica_no_such_handle", 115200, true, 0.05, 2000000000, open_posix_serial);
  EXPECT_FALSE(link.connected());
  EXPECT_EQ(link.fault_code(), FAULT_PORT_OPEN);
}

// 2026-11-18 도킹: dialog_state "docked" → 6(연두 숨쉬기), 파이썬 판과 같다.
TEST(Docking, DockedDialogStateMapsToCode6)
{
  EXPECT_EQ(patrol_state_code("docked"), protocol::STATE_DOCKED);
  EXPECT_EQ(protocol::STATE_DOCKED, 6);
  EXPECT_TRUE(protocol::is_sendable(protocol::STATE_DOCKED));
  EXPECT_EQ(patrol_state_code("docking"), PATROL_NONE);
  EXPECT_EQ(patrol_state_code("undocking"), PATROL_NONE);
  EXPECT_STREQ(patrol_reason(protocol::STATE_DOCKED), "docked");
}
