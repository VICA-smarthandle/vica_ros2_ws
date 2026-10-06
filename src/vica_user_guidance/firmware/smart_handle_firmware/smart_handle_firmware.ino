// VICA Smart Handle — 서보·LED 사용자 안내 펌웨어
//
// 젯슨(ROS 2)이 1바이트 상태코드를 10Hz로 보내면 서보와 LED로 사용자에게 안내한다.
// 원본 초안(led_servoMotor.txt)에서 확장한 항목:
//   - 상태코드 4(LINK_LOST), 5(ARRIVED) 추가
//   - everConnected: 첫 수신 전까지 워치독 보류 (부팅 중 오탐 방지)
//   - 통신두절을 E-stop과 분리 (원본은 둘 다 STATE_ESTOP)
//   - 보드레이트 115200
//
// 설계 계획서: devlog/2026-07-28-smart-handle-guidance-plan.md
//
// ── 2026-09-22 보드 교체: Arduino Nano → ESP32U (ESP32-WROOM-32U, CP2102 USB) ──
// 시나리오·상태코드·프레임·타이밍은 **한 줄도 바꾸지 않았다.** 바뀐 것은 보드에
// 종속된 네 가지뿐이다. 이식 기록: devlog/2026-09-22-esp32-스마트핸들-이식.md
//   1. 핀 번호 (아래 #define). 5V 부품은 레벨시프터를 거친다 — OE 핀이 새로 생겼다.
//   2. 서보 라이브러리: AVR 전용 <Servo.h> → <ESP32Servo.h> (같은 API·같은 기본 펄스폭)
//   3. I2C: Wire.begin(SDA, SCL) 로 핀 명시, setWireTimeout(AVR 전용) → setTimeOut(ms)
//   4. LED 개수 30 → 31 (실물 확인: 한쪽 끝 1개가 안 켜졌다 — 사용자, 2026-09-22)
// 빌드: arduino-cli compile --fqbn esp32:esp32:esp32 firmware/smart_handle_firmware
//
// ── 2026-09-23 초음파 8채널 (사용자 승인) ──
// 상향에 8채널 프레임(AA 57, 20B)을 추가했다. 옛 2채널 프레임(AA 55)은 호환용으로 함께
// 보낸다. 젯슨 파서·설정·URDF·costmap 레이어가 같은 커밋에서 8채널로 바뀐다.
//
// ── 2026-10-05 보드 복귀: ESP32U → Arduino Nano (ATmega328P, FT232RL USB) ──
// ESP32 보드가 레벨시프터·GPIO 손상으로 고장 나 다시 나노로 돌아왔다(09-22 이전과 같은
// 나노, serial B003UMKG). 5V 보드라 레벨시프터가 통째로 빠진다. 핀맵 정본은 하드웨어
// 인수인계 문서 "로봇 제어보드 인수인계: 핀맵과 제어"(2026-10-05) 2장이다:
//   D2 터치 · D7 진동(L9110 A-IA) · D9 서보 · D10 LED B · D12 LED A · A4/A5 I2C
// 시나리오·상태코드·프레임은 그대로다. 바뀐 것은 핀, 서보 라이브러리(<Servo.h>),
// I2C 호출(Wire.begin()·setWireTimeout) 세 가지다.
// 빌드: arduino-cli compile --fqbn arduino:avr:nano firmware/smart_handle_firmware
//
// [주의] 이 장치는 안내 전용이다. 서보는 로봇을 조향하지 않고,
//        LED 표시는 모터 정지를 보장하지 않는다. 정지 권한은 Safety 계층에 있다.

// 서보는 Adafruit_TiCoServo(2026-10-05). 표준 <Servo.h> 는 펄스 끝을 Timer1 인터럽트로
// 내는데, 시계(millis)·시리얼·I2C 인터럽트가 그 순간과 겹치면 펄스가 몇 us 길어져
// 대기 중에도 가끔 '틱' 했다(10-05 실물: LED 시간 창을 넣은 뒤에도 작게 남음).
// TiCoServo 는 Timer1 하드웨어 PWM(OC1A = D9)이 펄스를 직접 만들어 인터럽트와 무관하다.
// 나노에서는 D9·D10 만 쓸 수 있다(라이브러리 known_16bit_timers.h). attach/write 는
// Servo.h 와 같고, write 값이 544 미만이면 각도로 읽는다.
// 이것으로도 대기 중 틱이 남으면 원인은 전원·서보 자체 쪽이다 — 메모리 handle-servo-signal-policy.
#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// ══════════════════════════════════════════
#define NUM_LEDS_A  21   // 2026-10-05 나노 복귀: 줄당 21개(사용자 실측). 20 으로 두면 끝 1개가 안 바뀐다
#define NUM_LEDS_B  21
// ══════════════════════════════════════════

// ── 나노 핀 (2026-10-05 하드웨어 인수인계 문서 2장) ─────────────────────────
// 5V 보드라 서보·LED·터치·I2C 를 시프터 없이 직접 잇는다.
// D10 은 Servo 라이브러리(Timer1) 때문에 analogWrite 가 꺼지지만, 네오픽셀은 디지털
// 비트 신호라 상관없다(인수인계 문서). D0/D1 은 USB 시리얼, D13 은 보드 LED 라 비워 둔다.
#define SERVO_PIN         9   // 서보 SG90 신호(주황선)
// 좌우: 코드의 뜻은 A=왼쪽·B=오른쪽이다. 2026-10-05 실물 확인 — "왼쪽"(코드 1)에서 서보는
// 왼쪽인데 물결이 D12 줄(오른쪽)로 흘렀다. 그래서 핀만 맞바꿨다: D10 = 왼쪽 줄, D12 = 오른쪽 줄.
// (인수인계 문서의 "A=D12·B=D10" 은 작업자 시험 펌웨어의 이름이고 좌우와 무관하다.)
#define LED_A_PIN        10   // 네오픽셀 왼쪽 줄 데이터 (330~470Ω 직렬)
#define LED_B_PIN        12   // 네오픽셀 오른쪽 줄 데이터
// I2C 는 나노 고정 핀 A4(SDA)·A5(SCL). Wire.begin() 이 알아서 잡는다. 풀업 3.3kΩ 은 버스에 하나.
// 서보 펄스폭. 나노 Servo.h 기본값과 같은 값을 명시해 각도 해석이 바뀌지 않게 한다.
#define SERVO_US_MIN    544
#define SERVO_US_MAX   2400
#define SERVO_HZ         50   // 참고값: Servo.h 는 50Hz 고정(ESP32 시절 setPeriodHertz 용)
#define BLINK_MS    300
#define WAVE_MS     30
#define LINE_LEN    25

#define SERVO_CENTER  90
// 2026-10-06 서보 패들 교체 뒤 좌우를 가운데에서 30도씩으로 줄였다(사용자). 이전 180/0(±90°).
#define SERVO_LEFT   120
#define SERVO_RIGHT   60
#define SERVO_STEP_MS  14

// ── 도착 표시 ─────────────────────────────
// bench 실측(2026-07-28)으로 확정한 값이다.
//
// 시간:  150ms(0.9초) → 250ms(1.5초) → 500ms. 앞의 두 값은 너무 짧아 놓치기 쉬웠다.
// 색상:  (0,255,80) 녹색은 SKY와 구분되지 않았다. 무지개·보라·순수 파랑도 시험했으나
//        모두 제외하고, 최종적으로 SKY 단일 색으로 통일했다.
//        초록(GREEN)은 충전 상태 전용으로 예약되어 도착에 쓰지 않는다.
//
// 총 재생 시간 = ARRIVE_BLINK_MS * (2 * ARRIVE_BLINK_COUNT + 1) = 500 * 7 = 3500ms
//   (ON/OFF 각 3회 + 마지막 소등 유지 1프레임)
// [중요] ROS의 arrival_hold_sec는 이 값보다 커야 한다. 작으면 코드 0이 먼저 도착해
//        마지막 프레임이 잘린다. 이 상수를 바꾸면 config도 함께 바꿀 것.
#define ARRIVE_BLINK_MS    500   // ON/OFF 각 500ms
#define ARRIVE_BLINK_COUNT   3   // 3회 점멸 후 자동 NORMAL 복귀

// ── 통신 프로토콜 (숫자 1바이트) ──────────
#define STATE_NORMAL     0
#define STATE_LEFT       1
#define STATE_RIGHT      2
#define STATE_ESTOP      3
#define STATE_LINK_LOST  4   // 수신 중 단절. 젯슨이 보내지 않고 워치독이 자체 발동
#define STATE_ARRIVED    5

// 수신 허용 범위. LINK_LOST는 펌웨어 내부 전용이므로 수신은 5까지 받되
// 4가 들어와도 무시하지 않는다(디버깅용 수동 입력 허용).
#define STATE_MIN  STATE_NORMAL
#define STATE_MAX  STATE_ARRIVED

// ── 워치독: 이 시간 동안 신호 없으면 통신두절로 판단 ──
#define WATCHDOG_TIMEOUT_MS  1500

Adafruit_TiCoServo myServo;
Adafruit_NeoPixel ledA(NUM_LEDS_A, LED_A_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledB(NUM_LEDS_B, LED_B_PIN, NEO_GRB + NEO_KHZ800);

const uint32_t SKY    = Adafruit_NeoPixel::Color(0,   200, 255);
const uint32_t ORANGE = Adafruit_NeoPixel::Color(255, 80,  0  );
const uint32_t RED    = Adafruit_NeoPixel::Color(255, 0,   0  );
// 초록은 충전 상태 전용으로 예약한다(충전 중=점멸, 충전 완료=상시 점등).
// 도착에 초록을 쓰면 "충전 중"과 구분되지 않는다.
const uint32_t GREEN  = Adafruit_NeoPixel::Color(0,   255, 0  );
// 도착 표시는 SKY를 그대로 쓴다(2026-07-28 결정).
// 직진과 색이 같으므로 구분은 전적으로 움직임에 의존한다.
//   직진 = 상시 점등 / 도착 = 3회 점멸 후 직진 상태로 복귀
const uint32_t OFF    = Adafruit_NeoPixel::Color(0,   0,   0  );

enum Mode { NORMAL, WAVE_A, WAVE_B, BLINK_BOTH, BLINK_ARRIVE };
Mode    currentMode  = NORMAL;
uint8_t currentState = STATE_NORMAL;

bool blinkState = false;
int  linePos    = 0;

int servoAngle  = SERVO_CENTER;
int servoTarget = SERVO_CENTER;

unsigned long lastBlink     = 0;
unsigned long lastWave      = 0;
unsigned long lastServoStep = 0;
unsigned long lastRxMillis  = 0;
bool watchdogTripped = false;

// 한 번이라도 정상 수신했는가. false인 동안에는 워치독을 돌리지 않는다.
// 젯슨 부팅·ROS 기동에 걸리는 시간은 환경마다 다르므로 유예 시간을 상수로
// 잡지 않고, "연결된 적 있음"을 기준으로 삼는다.
bool everConnected = false;

// 도착 점멸 재생 횟수. 젯슨이 코드 5를 반복 전송해도 표시는 3회로 고정된다.
uint8_t arriveBlinksLeft = 0;
// 마지막 소등 프레임을 한 주기 유지하기 위한 플래그.
bool arriveTailPending = false;

// ══════════════════════════════════════════════════════════════════
// 초음파 DYP-A22 IIC ×2 — 논블로킹 순차 측정 (2026-08-31 신규)
//
// 설계 정본: docs/handoff_jetson_ultrasonic_i2c.md (§5.3 구조, §6.2 프레임)
// 기존 서보·LED 로직은 한 줄도 건드리지 않는다 — A-2 사고 이력 참조.
//
// 왜 순차인가: 센서 간격 214mm 라 실사용 구간에서 빔이 겹친다. 동시에 쏘면
// 서로의 메아리를 자기 것으로 읽어 "그럴듯한 틀린 거리"가 나온다(§4.2).
//
// 타이밍: 채널당 GAP 5ms → TRIG → WAIT 100ms → READ. 2채널 = 약 210ms,
// 프레임 약 4.8Hz. I2C 트랜잭션은 블로킹이지만 50kHz 에서 1ms 미만이라
// 14ms 서보 스텝을 방해하지 않는다. NeoPixel show()와는 같은 loop()에서
// 순차 실행되므로 겹치지 않는다. 나노에서는 show()가 인터럽트를 끄는 동안 millis()가
// 1~2ms 밀릴 수 있어 WAIT 에 여유(데이터시트 80 + 10ms)를 뒀다. ESP32 의 show()는
// RMT 하드웨어로 보내 인터럽트를 끄지 않지만(Adafruit_NeoPixel esp.c), 여유는 그대로 둔다.
//
// ── 2026-09-22 센서 8개 재주소 (사용자가 usonic_addr_setup 으로 굽고, 10cm 장애물로 자리 실측) ──
//   1 왼쪽 바퀴 옆 0x68 · 2 앞 왼쪽 0x69 · 3 앞 오른쪽 0x6A · 4 오른쪽 바퀴 옆 0x6B
//   5 오른쪽 뒷바퀴 옆 0x6F · 6 후방 오른쪽 0x6E · 7 후방 왼쪽 0x6D · 8 왼쪽 뒷바퀴 옆 0x6C
//   (5~8 은 굽힌 순서와 장착 자리가 달라 주소가 번호 순이 아니다 — 실측이 정본)
//
// ── 2026-09-23 8채널 확장 (사용자 승인) ──────────────────────────────────
// 채널 번호 = 자리 번호 - 1 (ch0 = 1번 왼쪽 바퀴 옆 … ch7 = 8번 왼쪽 뒷바퀴 옆).
// 같은 방향을 보는 센서끼리는 서로의 메아리를 읽으므로(§4.2) 한 번에 쏘지 않는다.
// 대신 **서로 등진 두 개**를 한 라운드로 묶어 동시에 쏜다 — 앞 vs 뒤, 왼쪽 vs 오른쪽은
// 빔이 만날 수 없다. 4라운드 × (GAP 5 + WAIT 100) = 420 ms/바퀴, 채널당 약 2.4 Hz
// (2채널 시절 4.8 Hz 의 절반. 올리려면 08-31 devlog 의 '완료 폴링' 방식).
//   라운드 A: ch1 앞 왼쪽      + ch5 후방 오른쪽
//   라운드 B: ch2 앞 오른쪽    + ch6 후방 왼쪽     → 이 뒤에 옛 2채널 프레임(AA 55) 송신
//   라운드 C: ch0 왼쪽 바퀴 옆 + ch3 오른쪽 바퀴 옆
//   라운드 D: ch7 왼쪽 뒷바퀴 옆 + ch4 오른쪽 뒷바퀴 옆 → 이 뒤에 8채널 프레임(AA 57) 송신
// 옛 2채널 프레임을 계속 보내는 이유: 젯슨 파서가 아직 옛 판인 워크스페이스(다른 브랜치
// 빌드)에서도 앞 두 개는 계속 보이게 하기 위해서다. 새 파서는 AA 57 만 읽고 AA 55 는
// 헤더가 달라 그냥 지나친다. 8채널 파서가 모든 브랜치에 들어가면 US_SEND_LEGACY 를 0 으로.
// 라운드 B 뒤에 보내는 이유: 그 시점의 앞 두 값이 옛 설정 measurement_delay [210, 105]
// 와 정확히 맞는다(ch1 은 210 ms 전, ch2 는 105 ms 전에 트리거).
// ══════════════════════════════════════════════════════════════════
#define US_N          8
#define US_ROUNDS     4
#define US_PER_ROUND  2
#define US_SEND_LEGACY        1   // 1: 라운드 B 뒤 옛 AA 55 프레임(앞 왼쪽·앞 오른쪽)도 보낸다
#define US_LEGACY_AFTER_ROUND 1   // 라운드 B (0 부터 셈)
#define US_TRIG_CMD   0xBC  // 150cm·mm 단위. 2026-08-31 실기 확정 — 0xBD(50cm)는
                            // 0xFFFD 만 반환했고 0xBC 는 실거리를 반환했다(§2.10)
#define US_WAIT_MS    100   // 0xBC 최대 측정시간 90ms + show() 지터 여유
#define US_CLEAR_MM   3001  // "범위 내 에코 없음". 실패(0)와 구분해야 costmap 이
                            // 앞이 뚫렸을 때 부채꼴을 지울 수 있다
#define US_ANGLE_LEVEL 0x03 // 지향각 레벨 3(50°) — 2026-09-01 운영 확정(재확정).
                            // 한때 "50° 과잉 봉쇄 체감"으로 40°에 복귀(e3e72a5)했으나,
                            // 그 봉쇄의 원인은 초음파가 아니라 **라이다가 바닥의
                            // 모니터 선을 장애물로 본 것**으로 판명(사용자 확인) —
                            // 오진을 되돌린다. 9회차 주행 검증값. 다른 조건
                            // (max_range 1.5m·레이어 분리)은 무변경.
                            // ROS fov 0.873 과 일치시킬 것.
                            // (레벨: 1=30°/0.524, 2=40°/0.698, 3=50°/0.873, 4=60°/1.047)
// ── 2026-09-24 채널별 부팅 지향각 (정지 시험 결과) ──────────────────────────
// 바퀴 옆 두 개(ch0 왼쪽·ch3 오른쪽)만 레벨 4(60°). 측면은 지도에 들어가는 센서가 한쪽에
// 하나뿐이라 넓혔다. 정지 시험: 빈 곳 바닥 헛값 0, 박스 30/60/100 cm 에서 50°와 차이
// ≤0.6 cm·놓침 0, 빔 가장자리 꼬깔콘 27~40 % → 약 100 %, 가장자리 필통은 60°에서만 잡힘.
// 나머지는 US_ANGLE_LEVEL(50°) 그대로. 드라이버 ultrasonic_fov_rad_per_channel 과 짝.
const uint8_t US_ANGLE_LEVEL_CH[US_N] = { 4, 3, 3, 4, 3, 3, 3, 3 };
#define US_GAP_MS     5     // 채널 사이 간격. 앞 채널 잔향이 다음 측정에 남지 않게
#define US_REG_DIST   0x02
#define US_REG_CMD    0x10
// 상향 8채널 프레임 20B (2026-09-23): AA 57 seq d0L d0H … d7L d7H xor
//   (거리 mm little-endian, xor = seq ^ 거리 16바이트. 채널 순서 = 자리 번호 순).
// 옛 상향 프레임 8B (호환용, US_SEND_LEGACY): AA 55 seq d0L d0H d1L d1H xor
//   (d0 = 앞 왼쪽 ch1, d1 = 앞 오른쪽 ch2).
// 헤더 0xAA/0x55·0x57 은 하향 상태코드(0~7)와 겹치지 않아 양방향이 섞여도 안전하다.
// 값 규약: 0 = 채널 무효(3회 연속 실패, 젯슨 쪽은 그 채널 미발행)
//          1~3000 = 실거리 mm
//          3001 = 범위 내 에코 없음(clear, 젯슨 쪽은 max_range 로 발행해 부채꼴을 지운다)
#define US_FRAME_H1   0xAA
#define US_FRAME_H2   0x55   // 옛 2채널 프레임
#define US_FRAME8_H2  0x57   // 8채널 프레임
// ── 2026-09-24 측정 결과 통계 프레임 (AA 58, 52B) ─────────────────────────
// 센서가 스스로 알리는 동주파수 간섭(0xFFFE)을 다른 실패와 구분해 세려고 추가했다.
// 지금까지는 0xFFFF·0xFFFE·I2C 실패를 모두 usFail 로 뭉뚱그려 "간섭이 있나"를 알 수 없었다.
// 거리 프레임(AA 57)과 판정 로직은 그대로 두고, 세기만 따로 보낸다.
//   AA 58 seq  [ch0: ok clr ffff fffe oth i2c] … [ch7: …]  xor      = 3 + 8×6 + 1 = 52B
//   ok   = 1~3000 mm 실거리        clr  = 0xFFFD 범위 내 에코 없음
//   ffff = 측정 미완료             fffe = 동주파수 간섭(데이터시트 1.2절)
//   oth  = 그 밖의 범위 밖 값      i2c  = 트리거 쓰기 또는 거리 읽기 실패
// 칸마다 uint8(255 에서 멈춤). US_STAT_EVERY_CYCLES 바퀴마다 보내고 0 으로 되돌린다.
// xor = seq ^ 48바이트. 옛 파서들은 헤더가 달라 1바이트씩 지나친다.
#define US_STAT_H2            0x58
#define US_STAT_KINDS         6
#define US_STAT_EVERY_CYCLES  12   // 12 × 420 ms ≈ 5 s
enum UsStatKind { US_ST_OK, US_ST_CLEAR, US_ST_FFFF, US_ST_FFFE, US_ST_OTHER, US_ST_I2C };
// ── 2026-09-24 레지스터 시험 명령 (하향 1바이트, 정지 시험 전용) ─────────────
// 젯슨 드라이버는 이 값을 보내지 않는다(상태코드 0~7·진동 0x10/0x11 만). 벤치 스크립트
// (firmware/usonic_register_bench.py)가 드라이버를 끈 상태에서 보낸다. 워치독은 건드리지 않는다.
//   0x30        부팅 기본값으로 되돌림(지향각 US_ANGLE_LEVEL_CH, 노이즈 US_NOISE_DEFAULT 전 채널)
//   0x31~0x35   노이즈 저감 레벨(레지스터 0x06) 1~5 를 8채널 모두에
//   0x41~0x44   지향각 레벨(0x07) 1~4 를 바퀴 옆 두 채널(ch0 왼쪽·ch3 오른쪽)에만
// 적용은 다음 라운드 트리거 직전(센서가 쉬는 때)에 하고, 곧바로 두 레지스터를 되읽어
// 설정 확인 프레임(AA 59)을 보낸다. 부팅 때도 한 번 보낸다.
//   AA 59 seq [ch0: angle noise] … [ch7] xor = 3 + 16 + 1 = 20B (되읽기 실패 = 0xFF)
// 레지스터가 전원을 끄면 지워지는지는 데이터시트에 없다 — 부팅 때 기본값을 다시 쓰므로
// 시험 값은 재부팅하면 사라진다.
#define US_CMD_RESET          0x30
#define US_CMD_NOISE_BASE     0x30   // 0x31~0x35 → 레벨 1~5
#define US_CMD_SIDE_ANGLE_BASE 0x40  // 0x41~0x44 → 레벨 1~4
#define US_NOISE_DEFAULT      1      // 데이터시트 출고값(배터리 전원용)
#define US_REG_NOISE          0x06
#define US_REG_ANGLE          0x07
#define US_CFG_H2             0x59
#define US_SIDE_CH_L          0      // ch0 왼쪽 바퀴 옆
#define US_SIDE_CH_R          3      // ch3 오른쪽 바퀴 옆

// ── 터치센서 (ESP32 GPIO14 · 나노 시절 D11, 2026-09-05 인수인계 문서 기준) ─────
// **idle HIGH / 터치 LOW** 인 active-low 타입이다. 2026-09-04 에 "잡으면 HIGH"
// 로 잘못 알고 짰던 것을 문서 실측(idle=HIGH, touch=LOW, idle noise 0/994)
// 으로 바로잡았다.
//
// INPUT_PULLUP 을 쓴다 — OUT 선이 빠지면 핀이 HIGH 로 뜨므로 **단선 = 놓음**이
// 되어 07-30 결정의 NC fail-safe 가 그대로 성립한다. 풀업 없이 INPUT 이면
// 단선 시 값이 떠서 '잡음'으로 오판할 수 있다.
//
// 상향 프레임 5B: AA 56 seq flags xor (xor 는 seq^flags).
// 초음파 프레임(AA 55)에 얹지 않고 헤더를 가른 이유는 두 가지다.
//   * 주기가 다르다. 초음파 4.8Hz 는 측정 시간이 정하는 물리 한계인데, 손 놓음
//     판정 유예는 0.5초라 그사이 샘플이 2~3개뿐이다.
//   * 8B 를 9B 로 늘리면 헤더가 같아, 옛 젯슨 파서가 체크섬 실패와 재동기를
//     반복하며 **초음파까지 함께** 조용히 멈춘다.
//
// [판정은 여기서 하지 않는다] 3초 진입도 0.5초 놓침도 mission_manager 몫이다.
// 이 펌웨어는 "그 순간 잡고 있나"만 20Hz 로 보낸다.
//
// [시간 브리지] 이 센서는 **터치 중에 출력이 LOW/HIGH 로 빠르게 튄다.** 반면
// idle 은 994회 측정에서 흔들림 0 이었다(인수인계 문서 §2). 그래서 디바운스도
// 적분 필터도 실패했다 — 튀는 신호를 세면 여러 번 발동하거나 아예 무반응이 된다.
// 신호를 세지 않고 **마지막으로 LOW 를 본 시각** 하나만 기억한다.
//     touched = (now - lastLowMs < TOUCH_HOLD_MS)
// 평소엔 LOW 가 절대 안 나오므로 LOW 가 한 번이라도 보이면 진짜 터치고, 터치 중
// HIGH 가 끼어들어도 200ms 안에 다음 LOW 가 오면 끊기지 않는다. 채터링을
// 필터링하는 게 아니라 무시하는 구조다. 이것도 판정이 아니라 신호 정리다 —
// 사람 손의 3초·0.5초와는 자릿수가 다르다.
#define TOUCH_PIN         2   // 2026-10-05 나노: LTK-01 OUT. 평소 HIGH·누르면 LOW, 내부 풀업
#define TOUCH_FRAME_H1   0xAA
#define TOUCH_FRAME_H2   0x56
#define TOUCH_FLAG_ON    0x01
#define TOUCH_PERIOD_MS  50    // 20Hz. 판정 유예 0.5초에 10프레임
#define TOUCH_HOLD_MS    200   // LOW 본 뒤 '잡음' 유지. 터치 중 끊기면 올린다

uint8_t       touchSeq     = 0;
unsigned long touchLastLow = 0;      // 마지막으로 LOW(터치)를 본 시각
bool          touchSeenLow = false;  // 부팅 후 LOW 를 한 번이라도 봤나
unsigned long touchSentAt  = 0;

// ── 진동모터 (ESP32 GPIO16 · 09-22~09-30 GPIO4 · 나노 시절 D10) ──────────────────────
// MOSFET 드라이버 게이트에 물려 있다(7/28 계획서 6.3절 회로). MCU GPIO 로 모터를
// 직접 구동하지 않는다 — 전류 초과. 플라이백 다이오드가 드라이버 쪽에 있다.
//
// 울리는 길은 둘이다 (2026-09-30, docs/superpowers/specs/2026-09-28-touch-haptic-
// integration-final.md 3절).
//   * 젯슨 명령 — 0x10/0x11/0x12 바이트. 상태코드(0~7)와 겹치지 않는 별도 바이트라
//     applyState() 를 거치지 않는다 — LED·서보는 그대로다. 미션이 손잡이 찾기·잡음
//     확인·놓침에 쓴다(드라이버 노드가 /vica/haptic_request 를 바이트로 바꿔 보낸다).
//   * 상태 진입 — ESTOP 은 길게 ×1, ARRIVED 는 짧게 ×3. applyState() 는 상태가
//     **바뀔 때만** 돌므로 같은 코드가 10Hz 로 반복 와도 한 번만 떨린다(계획서 6.2절
//     의 edge 트리거). 진동은 알림일 뿐 정지 보증이 아니다.
// 새 명령은 진행 중인 패턴을 덮어쓴다 — 미션이 반복 진동 중에 0x12 를 보내면 긴
// 진동이 곧바로 끊기고 짧은 확인이 한 번 온다. 따로 '멈춤' 명령이 필요 없다.
//
// 패턴은 논블로킹이다. delay() 를 쓰면 서보·LED·초음파·워치독이 그 시간 동안
// 멈춘다.
#define HAPTIC_PIN             7   // 2026-10-05 나노: L9110 A-IA(A-IB=GND, 10kΩ 풀다운). 켜기/끄기만
#define HAPTIC_CMD_SHORT      0x10   // 300ms on/150ms off x3 (도착 패턴)
#define HAPTIC_CMD_LONG       0x11   // 1200ms x1 (손잡이 찾기·비상 패턴)
#define HAPTIC_CMD_TICK       0x12   // 300ms x1 (잡음 확인, 2026-09-30)
#define HAPTIC_SHORT_ON_MS    300   // 2026-09-04 150->300. 모터가 회전 올라올 시간(50~100ms)을 준다
#define HAPTIC_SHORT_OFF_MS   150
#define HAPTIC_SHORT_COUNT    3
#define HAPTIC_LONG_ON_MS     1200  // 2026-09-04 800->1200. 사용자 "더 강하게"

uint8_t       hapticLeft  = 0;      // 남은 ON 횟수
bool          hapticOn    = false;  // 지금 HIGH 인가
unsigned long hapticAt    = 0;      // 마지막 전환 시각
unsigned int  hapticOnMs  = 0;
unsigned int  hapticOffMs = 0;

void hapticStart(uint8_t count, unsigned int onMs, unsigned int offMs) {
  // 진행 중이면 새 명령이 덮어쓴다. 겹쳐 쌓지 않는다 — 누적하면 사용자가
  // 몇 번 떨렸는지로 상황을 못 읽는다.
  hapticLeft  = count;
  hapticOnMs  = onMs;
  hapticOffMs = offMs;
  hapticOn    = true;
  hapticAt    = millis();
  digitalWrite(HAPTIC_PIN, HIGH);
  hapticLeft--;
}

void hapticTask(unsigned long now) {
  if (hapticOn) {
    if (now - hapticAt >= hapticOnMs) {
      digitalWrite(HAPTIC_PIN, LOW);
      hapticOn = false;
      hapticAt = now;
    }
  } else if (hapticLeft > 0 && now - hapticAt >= hapticOffMs) {
    digitalWrite(HAPTIC_PIN, HIGH);
    hapticOn = true;
    hapticAt = now;
    hapticLeft--;
  }
}

// 채널(자리 번호-1) → 7bit 주소. 2026-09-22 10cm 장애물 실측표 그대로.
const uint8_t US_ADDR7[US_N] = {
  0x68,  // ch0 1번 왼쪽 바퀴 옆
  0x69,  // ch1 2번 앞 왼쪽      (옛 front_left)
  0x6A,  // ch2 3번 앞 오른쪽    (옛 front_right)
  0x6B,  // ch3 4번 오른쪽 바퀴 옆
  0x6F,  // ch4 5번 오른쪽 뒷바퀴 옆
  0x6E,  // ch5 6번 후방 오른쪽
  0x6D,  // ch6 7번 후방 왼쪽
  0x6C,  // ch7 8번 왼쪽 뒷바퀴 옆
};
// 라운드별로 동시에 쏘는 채널 쌍. 서로 등진 방향만 묶는다(위 절 참조).
const uint8_t US_ROUND_CH[US_ROUNDS][US_PER_ROUND] = {
  { 1, 5 },  // A: 앞 왼쪽 + 후방 오른쪽
  { 2, 6 },  // B: 앞 오른쪽 + 후방 왼쪽
  { 0, 3 },  // C: 왼쪽 바퀴 옆 + 오른쪽 바퀴 옆
  { 7, 4 },  // D: 왼쪽 뒷바퀴 옆 + 오른쪽 뒷바퀴 옆
};
#define US_LEGACY_CH0 1   // 옛 프레임 d0 = 앞 왼쪽
#define US_LEGACY_CH1 2   // 옛 프레임 d1 = 앞 오른쪽

enum UsPhase { US_TRIG, US_WAIT, US_READ };
UsPhase       usPhase   = US_TRIG;
uint8_t       usRound   = 0;
bool          usTrigOk[US_PER_ROUND] = { false, false };
unsigned long usPhaseAt = 0;
uint8_t       usSeq     = 0;
uint16_t      usDist[US_N]  = { 0 };     // 프레임에 실을 값. 0 = 무효
uint8_t       usFails[US_N] = { 0 };     // 연속 실패 수
uint16_t      usBuf[US_N][3];            // 3점 중앙값용 최근 유효 샘플
uint8_t       usBufN[US_N]  = { 0 };
uint8_t       usStat[US_N][US_STAT_KINDS] = { { 0 } };   // 통계 프레임용(창마다 0)
uint8_t       usStatSeq    = 0;
uint8_t       usAngleLv[US_N];            // 채널별 지향각 레벨(부팅 때 US_ANGLE_LEVEL_CH)
uint8_t       usNoiseLv[US_N];            // 채널별 노이즈 저감 레벨(부팅 때 US_NOISE_DEFAULT)
bool          usCfgPending = false;       // 다음 트리거 직전에 쓰고 되읽을 것
uint8_t       usCfgSeq     = 0;
uint8_t       usStatCycles = 0;

void usCount(uint8_t ch, uint8_t kind) {
  if (usStat[ch][kind] < 255) usStat[ch][kind]++;
}

bool usWrite8(uint8_t addr7, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// 거리 레지스터 0x02 에서 2바이트. 상위 바이트 먼저다(§2.7 big-endian).
bool usReadDist(uint8_t addr7, uint16_t *out) {
  Wire.beginTransmission(addr7);
  Wire.write(US_REG_DIST);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr7, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  *out = ((uint16_t)hi << 8) | lo;
  return true;
}

bool usRead8(uint8_t addr7, uint8_t reg, uint8_t *out) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr7, (uint8_t)1) != 1) return false;
  *out = Wire.read();
  return true;
}

// 채널별 지향각·노이즈 레벨을 쓰고 되읽어 AA 59 로 보낸다. 센서가 쉬는 때만 부른다.
void usApplyConfig() {
  uint8_t f[3 + 2 * US_N + 1];
  f[0] = 0xAA;
  f[1] = US_CFG_H2;
  f[2] = usCfgSeq++;
  uint8_t x = f[2];
  for (uint8_t ch = 0; ch < US_N; ch++) {
    uint8_t a = 0xFF, n = 0xFF;
    uint8_t tries = 3;
    while (tries-- && !usWrite8(US_ADDR7[ch], US_REG_ANGLE, usAngleLv[ch])) delay(20);
    tries = 3;
    while (tries-- && !usWrite8(US_ADDR7[ch], US_REG_NOISE, usNoiseLv[ch])) delay(20);
    // 노이즈 값을 실제로 바꾸면 센서가 잠깐(수십 ms) 응답하지 않는다(09-24 실측: 바꾼 직후
    // 0x07 되읽기가 전 채널 실패, 같은 값을 다시 쓸 때는 정상 → 내부 저장으로 추정).
    // 그래서 되읽기는 짧게 기다리며 몇 번 다시 한다. 부팅(값 그대로)에는 첫 시도에 끝난다.
    tries = 5;
    while (tries-- && !usRead8(US_ADDR7[ch], US_REG_ANGLE, &a)) { a = 0xFF; delay(30); }
    tries = 5;
    while (tries-- && !usRead8(US_ADDR7[ch], US_REG_NOISE, &n)) { n = 0xFF; delay(30); }
    f[3 + 2 * ch] = a;
    f[4 + 2 * ch] = n;
    x ^= a ^ n;
  }
  f[3 + 2 * US_N] = x;
  Serial.write(f, sizeof(f));
}

uint16_t usMedian3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { b = (a > c) ? a : c; }
  return b;
}

void usStore(uint8_t ch, uint16_t mm) {
  usFails[ch] = 0;
  if (usBufN[ch] < 3) {
    usBuf[ch][usBufN[ch]++] = mm;
  } else {
    usBuf[ch][0] = usBuf[ch][1];
    usBuf[ch][1] = usBuf[ch][2];
    usBuf[ch][2] = mm;
  }
  // 3점이 모이기 전에는 최신값을 그대로 쓴다. 기동 직후 공백을 줄인다.
  usDist[ch] = (usBufN[ch] >= 3)
      ? usMedian3(usBuf[ch][0], usBuf[ch][1], usBuf[ch][2])
      : mm;
}

void usFail(uint8_t ch) {
  if (usFails[ch] < 255) usFails[ch]++;
  if (usFails[ch] >= 3) {
    usDist[ch] = 0;   // 무효 표시. 마지막 유효값을 계속 내보내면 costmap 이 낡은
    usBufN[ch] = 0;   // 벽을 믿는다 — 복구 후 낡은 샘플이 중앙값을 오염하지 않게 비운다
  }
}

// 8채널 프레임 20B. 한 바퀴(4라운드)가 끝날 때 한 번 보낸다.
void usSendFrame8() {
  uint8_t f[3 + 2 * US_N + 1];
  f[0] = US_FRAME_H1;
  f[1] = US_FRAME8_H2;
  f[2] = usSeq++;
  uint8_t x = f[2];
  for (uint8_t ch = 0; ch < US_N; ch++) {
    f[3 + 2 * ch]     = usDist[ch] & 0xFF;
    f[4 + 2 * ch]     = usDist[ch] >> 8;
    x ^= f[3 + 2 * ch] ^ f[4 + 2 * ch];
  }
  f[3 + 2 * US_N] = x;
  Serial.write(f, sizeof(f));
}

// 통계 프레임 52B. US_STAT_EVERY_CYCLES 바퀴마다 한 번 보내고 칸을 비운다.
void usSendStatFrame() {
  uint8_t f[3 + US_N * US_STAT_KINDS + 1];
  f[0] = US_FRAME_H1;
  f[1] = US_STAT_H2;
  f[2] = usStatSeq++;
  uint8_t x = f[2];
  for (uint8_t ch = 0; ch < US_N; ch++) {
    for (uint8_t k = 0; k < US_STAT_KINDS; k++) {
      uint8_t v = usStat[ch][k];
      f[3 + ch * US_STAT_KINDS + k] = v;
      x ^= v;
      usStat[ch][k] = 0;
    }
  }
  f[3 + US_N * US_STAT_KINDS] = x;
  Serial.write(f, sizeof(f));
}

#if US_SEND_LEGACY
// 옛 2채널 프레임 8B(호환용). 앞 왼쪽·앞 오른쪽만 싣는다. seq 는 8채널 프레임과 따로 센다 —
// 옛 파서가 순번 누락으로 오해하지 않게.
uint8_t usLegacySeq = 0;
void usSendLegacyFrame() {
  uint8_t f[8];
  f[0] = US_FRAME_H1;
  f[1] = US_FRAME_H2;
  f[2] = usLegacySeq++;
  f[3] = usDist[US_LEGACY_CH0] & 0xFF;
  f[4] = usDist[US_LEGACY_CH0] >> 8;
  f[5] = usDist[US_LEGACY_CH1] & 0xFF;
  f[6] = usDist[US_LEGACY_CH1] >> 8;
  f[7] = f[2] ^ f[3] ^ f[4] ^ f[5] ^ f[6];
  Serial.write(f, 8);
}
#endif

// 터치 원시값을 읽어 시간 브리지만 통과시킨다. 판정은 하지 않는다.
void touchPoll() {
  unsigned long now = millis();

  if (digitalRead(TOUCH_PIN) == LOW) {   // active-low: LOW = 잡음
    touchLastLow = now;
    touchSeenLow = true;
  }
  // 부팅 직후 touchLastLow 가 0 이면 now-0 < HOLD 가 잠깐 참이 되어 "잡음"으로
  // 시작한다. LOW 를 한 번이라도 본 뒤에만 브리지를 적용한다.
  bool touched = touchSeenLow && (now - touchLastLow < TOUCH_HOLD_MS);

  // 주기 송신. 상태가 안 바뀌어도 계속 보낸다 — 젯슨이 "언제까지 살아 있었나"로
  // 상향 신선도를 판정하기 때문이다. 조용하면 끊긴 것과 구분이 안 된다.
  if (now - touchSentAt < TOUCH_PERIOD_MS) return;
  touchSentAt = now;

  uint8_t f[5];
  f[0] = TOUCH_FRAME_H1;
  f[1] = TOUCH_FRAME_H2;
  f[2] = touchSeq++;
  f[3] = touched ? TOUCH_FLAG_ON : 0x00;
  f[4] = f[2] ^ f[3];
  Serial.write(f, 5);
}

// 트리거 실패도 WAIT 를 그대로 거친다 — 센서가 빠져 있어도 주기가 흔들리지
// 않아 프레임이 항상 같은 박자로 나간다(실패 시 시리얼 폭주 방지).
// 한 라운드 = 서로 등진 채널 2개를 연달아 트리거(I2C 쓰기 2번, 1ms 미만) → 100ms 대기 →
// 둘 다 읽기. 라운드 4개가 한 바퀴다.
void usTask(unsigned long now) {
  switch (usPhase) {
    case US_TRIG:
      if (now - usPhaseAt < US_GAP_MS) return;
      if (usCfgPending) {          // 레지스터 시험 명령·부팅 설정은 센서가 쉴 때 적용
        usCfgPending = false;
        usApplyConfig();
      }
      for (uint8_t k = 0; k < US_PER_ROUND; k++) {
        usTrigOk[k] = usWrite8(US_ADDR7[US_ROUND_CH[usRound][k]], US_REG_CMD, US_TRIG_CMD);
      }
      usPhase   = US_WAIT;
      usPhaseAt = now;
      return;

    case US_WAIT:
      if (now - usPhaseAt < US_WAIT_MS) return;
      usPhase = US_READ;
      return;

    case US_READ: {
      // 0xFFFF = 측정 미완료, 0xFFFE = 동주파수 간섭 — 둘 다 거리가 아니다(§2.9)
      // 0xFFFD = 범위 내 에코 없음(2026-08-31 실기 관찰, 데이터시트 미기재)
      //          — 정상 상황이므로 clear 로 저장한다. 실패로 치면 앞이 뚫려
      //          있을 때마다 채널이 무효(0)가 되어 costmap 을 지울 수 없다.
      for (uint8_t k = 0; k < US_PER_ROUND; k++) {
        uint8_t  ch = US_ROUND_CH[usRound][k];
        uint16_t raw;
        if (usTrigOk[k] && usReadDist(US_ADDR7[ch], &raw)) {
          if (raw >= 1 && raw <= 3000)      { usCount(ch, US_ST_OK);    usStore(ch, raw); }
          else if (raw == 0xFFFD)           { usCount(ch, US_ST_CLEAR); usStore(ch, US_CLEAR_MM); }
          else {
            usCount(ch, raw == 0xFFFF ? US_ST_FFFF : raw == 0xFFFE ? US_ST_FFFE : US_ST_OTHER);
            usFail(ch);
          }
        } else {
          usCount(ch, US_ST_I2C);
          usFail(ch);
        }
      }
#if US_SEND_LEGACY
      if (usRound == US_LEGACY_AFTER_ROUND) usSendLegacyFrame();
#endif
      usPhase   = US_TRIG;
      usPhaseAt = now;
      usRound++;
      if (usRound >= US_ROUNDS) {
        usRound = 0;
        usSendFrame8();
        if (++usStatCycles >= US_STAT_EVERY_CYCLES) {
          usStatCycles = 0;
          usSendStatFrame();
        }
      }
      return;
    }
  }
}

// ── LED 갱신 시간 창 (2026-10-05) ──────────────────────────────────────────
// show() 는 LED 1개당 약 30us 동안 인터럽트를 끈다(줄당 21개 ≈ 0.63ms). 표준 Servo.h 는
// 서보 펄스를 Timer1 인터럽트로 끝내므로, 그 순간 show() 가 돌고 있으면 펄스가 길어져
// 서보가 움찔한다. TiCoServo(하드웨어 PWM)로 바꾼 뒤에는 꼭 필요하진 않지만, 기다림이
// 길어야 약 4ms 라 해가 없고 Servo.h 로 되돌릴 때를 대비해 남겨 둔다. 그래서 서보 펄스가 이미 끝나고 다음 펄스가 시작되기 전 구간에서만
// show() 한다. Timer1 은 0.5us 단위로 0→40000(20ms) 을 센다(Servo.h·TiCoServo 둘 다 프리스케일 8):
//   5200 (2.6ms)  = 가장 긴 펄스 2400us 가 끝난 뒤
//   37600(18.8ms) = 0.6ms 짜리 show() 가 다음 주기 시작(40000) 전에 끝나는 마지막 지점
// 창은 20ms 중 약 16ms 라 기다림은 길어야 약 4ms — 30ms 주기 물결엔 보이지 않는다.
// 작업자 touch_test(2026-10-05) 에서 실물 검증된 방식이다. 이것으로도 서보가 움찔하면
// TiCoServo(하드웨어 PWM, D9 그대로) 로 바꾼다 — 메모리 handle-servo-signal-policy.
#define LED_SAFE_TCNT_MIN   5200
#define LED_SAFE_TCNT_MAX  37600
#define LED_SAFE_WAIT_US   25000   // 서보가 안 붙어 Timer1 이 멈춰 있을 때의 안전 탈출

void ledShowSafe(Adafruit_NeoPixel &strip) {
  if (myServo.attached()) {
    unsigned long t0 = micros();
    for (;;) {
      noInterrupts();            // 16비트 TCNT1 을 한 번에 읽는다(Servo ISR 이 0 으로 되돌리므로)
      uint16_t c = TCNT1;
      interrupts();
      if (c >= LED_SAFE_TCNT_MIN && c <= LED_SAFE_TCNT_MAX) break;
      if (micros() - t0 > LED_SAFE_WAIT_US) break;
    }
  }
  strip.show();
}

void setA(uint32_t color) {
  for (int i = 0; i < NUM_LEDS_A; i++) ledA.setPixelColor(i, color);
  ledShowSafe(ledA);
}
void setB(uint32_t color) {
  for (int i = 0; i < NUM_LEDS_B; i++) ledB.setPixelColor(i, color);
  ledShowSafe(ledB);
}
void setBoth(uint32_t color) { setA(color); setB(color); }

void drawLine(Adafruit_NeoPixel &strip, int numLeds, int pos) {
  for (int i = 0; i < numLeds; i++) {
    int dist = pos - (numLeds - 1 - i);
    strip.setPixelColor(i, (dist >= 0 && dist < LINE_LEN) ? ORANGE : OFF);
  }
  ledShowSafe(strip);
}

void servoMoveTo(int target) { servoTarget = target; }

// 상태가 실제로 바뀔 때만 처리 — 같은 상태 재수신은 무시.
// 젯슨이 10Hz로 같은 코드를 계속 보내도 애니메이션이 리셋되지 않는다.
void applyState(uint8_t state) {
  if (state == currentState) return;
  currentState = state;

  blinkState = false;
  linePos    = 0;
  // [주의] 0으로 초기화하면 now - lastBlink 가 millis() 전체 값이 되어 다음
  // loop()에서 곧바로 조건을 통과한다. 첫 점멸이 한 주기를 채우지 못하고
  // 이후 타이밍이 반 박자씩 밀린다(2026-07-28 실측: 도착이 "2.5회"로 보임).
  // 현재 시각으로 초기화해야 첫 주기부터 온전히 유지된다.
  lastBlink  = millis();
  lastWave   = millis();

  switch (state) {
    case STATE_NORMAL:
      currentMode = NORMAL;
      setBoth(SKY);
      servoMoveTo(SERVO_CENTER);
      break;

    // ── 서보: 2026-09-04 에 상수명과 일치시켰다 ──────────────────────
    // [정정] 2026-08-02 부터 여기에는 "서보가 거꾸로 장착돼 있으니 상수명과 반대로
    // 부르는 것이 정상이다. 건드리지 말 것"이라고 적혀 있었다. **더는 맞지 않는다.**
    // 사용자가 실물에서 서보 좌우가 뒤바뀐 것을 확인했고(LED 는 그대로 정상),
    // 그래서 두 case 의 servoMoveTo 인자를 서로 맞바꿨다. 이제 상수명 그대로
    // STATE_LEFT → SERVO_LEFT 다.
    //
    // 뒤집힘이 사라진 이유(장착 방향 변경·서보 교체 등)는 확인하지 않았다. 다만
    // **보정 자리는 여전히 이 파일 한 곳뿐이다.** 다음에 또 반대로 보이면 고칠
    // 곳은 여기이지 ROS 가 아니다 — 아두이노는 상태코드 하나로 LED 와 서보를 같은
    // case 에서 정하므로, 상위에서 뒤집으면 LED 까지 함께 뒤집힌다(2026-08-01 사고).
    //
    // 확인 도구는 이미 있다: firmware/bench_test.py --hold 1 / --hold 2.
    //
    // ── LED: 좌우를 바로잡았다 (2026-08-02) ──────────────────────────
    // [정정 2026-08-02] 2026-07-28 주석의 "bench에서 좌/우 모두 LED 방향과 서보
    // 방향이 일치함을 확인했다"는 **틀렸다.** ROS를 거치지 않고 이 펌웨어에 코드를
    // 직접 넣어 확인했다(firmware/bench_test.py --hold 1 / --hold 2).
    //
    //   코드 1 STATE_LEFT   서보 왼쪽  정상 · 주황 LED 오른쪽  반대
    //   코드 2 STATE_RIGHT  서보 오른쪽 정상 · 주황 LED 왼쪽   반대
    //
    // 즉 D8(A)이 왼쪽이고 D9(B)가 오른쪽이다. 그 전 주석의 (좌측)/(우측) 표기가
    // 반대로 적혀 있었다. 그때 고친 것은 **LED 두 줄만**(currentMode와 setA/setB)
    // 이었고 servoMoveTo 는 2026-09-04 에 별도로 맞바꿨다(위 서보 절 참조).
    // LED 줄은 지금도 그대로 유효하다 — 이번에 건드리지 않았다.
    //
    // currentMode는 주황 흐름선이 흐를 스트립을 정하고 setA/setB는 반대쪽을
    // 하늘색 상시 점등으로 둔다. 둘은 항상 짝을 이뤄 반대여야 한다.
    //
    // 2026-08-02 노트북(x86_64)에서 arduino-cli로 이 소스를 실물에 업로드하고
    // bench_test.py --hold 1 / --hold 2로 확인했다. 코드 1은 주황·서보가 모두
    // 왼쪽, 코드 2는 모두 오른쪽이었다. 소스와 실물이 일치한다.
    //
    // 상위(ROS)에서 뒤집어 때우지 않는다. 2026-08-01에 그렇게 했다가 LED는
    // 맞았지만 서보가 함께 뒤집혔다 — 코드 하나가 LED와 서보를 같이 정하기
    // 때문이다. test_left_cue_sends_left_code가 그 재발을 막는다.
    case STATE_LEFT:
      currentMode = WAVE_A;   // A = 왼쪽 (나노 D8 실측 2026-08-02 → ESP32 GPIO17, 09-22 실물 확인)
      setB(SKY); setA(OFF);   // 주황이 흐르는 A는 끄고 반대쪽 B를 하늘색으로
      servoMoveTo(SERVO_LEFT);    // 2026-09-04 반전. 이전에는 SERVO_RIGHT 였다
      break;

    case STATE_RIGHT:
      currentMode = WAVE_B;   // B = 오른쪽 (나노 D9 실측 2026-08-02 → ESP32 GPIO18, 09-22 실물 확인)
      setA(SKY); setB(OFF);
      servoMoveTo(SERVO_RIGHT);   // 2026-09-04 반전. 이전에는 SERVO_LEFT 였다
      break;

    case STATE_ESTOP:
      currentMode = BLINK_BOTH;
      setBoth(OFF);
      // 로봇이 정지한 상태에서 방향 지시가 남아 있으면 잘못된 안내가 된다.
      // 원본 초안은 "마지막 방향 유지"였으나, 회전 중 E-stop이 걸리면 서보가
      // 기울어진 채(또는 이동 중 임의 각도로) 멈춰 "이쪽으로 도세요"를
      // 계속 가리키게 된다. 아키텍처 12장의 "E-stop 시 서보 중립" 원칙에 맞춘다.
      servoMoveTo(SERVO_CENTER);
      hapticStart(1, HAPTIC_LONG_ON_MS, 0);   // 2026-09-30: 비상정지 진입 = 길게 ×1
      break;

    case STATE_LINK_LOST:
      // 상시 점등이므로 애니메이션 모드를 쓰지 않는다.
      currentMode = NORMAL;
      setBoth(RED);
      servoMoveTo(SERVO_CENTER);
      break;

    case STATE_ARRIVED:
      currentMode       = BLINK_ARRIVE;
      arriveBlinksLeft  = ARRIVE_BLINK_COUNT;
      arriveTailPending = true;
      setBoth(OFF);
      servoMoveTo(SERVO_CENTER);
      // 2026-09-30: 도착 = 짧게 ×3. 홈 복귀 도착에도 울린다 — 해롭지 않아 둔다.
      hapticStart(HAPTIC_SHORT_COUNT, HAPTIC_SHORT_ON_MS, HAPTIC_SHORT_OFF_MS);
      break;
  }
}

void setup() {
  // 진동 핀부터 LOW 로 잡는다. L9110 입력은 비어 있으면 모듈이 HIGH 로 끌어올려
  // 부팅 동안 모터가 돈다(2026-09-30 실측) — 가장 먼저 눌러 둔다.
  pinMode(HAPTIC_PIN, OUTPUT);
  digitalWrite(HAPTIC_PIN, LOW);

  Serial.begin(115200);

  // Servo.h 는 50Hz 고정이다. 펄스폭만 544~2400us 로 명시한다.
  myServo.attach(SERVO_PIN, SERVO_US_MIN, SERVO_US_MAX);
  myServo.write(SERVO_CENTER);

  ledA.begin(); ledA.show();
  ledB.begin(); ledB.show();
  setBoth(SKY);   // 부팅 대기 표시. 첫 수신 전까지 이 상태를 유지한다.

  lastRxMillis = millis();

  // 터치센서. active-low 라 풀업을 켠다 — OUT 선이 빠지면 HIGH 로 떠서
  // '놓음'으로 읽힌다(단선 = 놓음, fail-safe).
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  touchSentAt = millis();

  // 진동모터. 부팅 직후 게이트가 떠서 모터가 헛돌지 않게 먼저 LOW 로 잡는다.
  pinMode(HAPTIC_PIN, OUTPUT);
  digitalWrite(HAPTIC_PIN, LOW);

  // ── 초음파 I2C ──
  // 케이블이 길어 데이터시트 상한(100kHz)의 절반으로 시작한다(§4.2-4).
  // 센서·나노 모두 5V 라 시프터 없이 A4/A5 에 바로 물린다(A22 통신 레벨 = VCC).
  // 타임아웃: setWireTimeout(us, true) — 25ms 넘게 멈추면 트랜잭션을 끝내고 TWI 하드웨어를
  // 리셋해 loop() 가 계속 돈다. 없으면 SDA 락업 한 번에 서보·LED·워치독이 같이 멎는다.
  // AVR Wire 는 WIRE_HAS_TIMEOUT 를 정의하지 않으므로 #if 로 감싸지 말고 바로 부른다.
  Wire.begin();
  Wire.setClock(50000);
  Wire.setWireTimeout(25000, true);

  // 지향각 레벨 1을 전원 인가 시마다 굽는다. 레지스터 휘발 여부가 데이터시트에
  // 없어, 기본값(레벨 4·60°)으로 돌아가면 높이 91.3mm 수평 장착에서 16cm 앞부터
  // 바닥이 장애물로 찍힌다(§3.1). 냉기동 직후 센서 안정화(≤1s) 대비 3회 재시도.
  for (uint8_t i = 0; i < US_N; i++) {
    usAngleLv[i] = US_ANGLE_LEVEL_CH[i];
    usNoiseLv[i] = US_NOISE_DEFAULT;
    uint8_t tries = 3;
    while (tries-- && !usWrite8(US_ADDR7[i], 0x07, US_ANGLE_LEVEL_CH[i])) delay(100);
  }
  usCfgPending = true;   // 첫 라운드 직전에 노이즈 레벨까지 쓰고 되읽어 AA 59 로 알린다
  usPhaseAt = millis();
}

void loop() {
  unsigned long now = millis();

  // ── 시리얼 수신 (1바이트 상태 코드) ────
  if (Serial.available()) {
    int b = Serial.read();
    if (b == HAPTIC_CMD_SHORT) {
      // 햅틱 명령은 워치독을 건드리지 않는다. 링크 생존 판정은 상태코드에만
      // 묶여 있어야 "젯슨이 살아서 상태를 보내고 있다"는 뜻이 유지된다.
      hapticStart(HAPTIC_SHORT_COUNT, HAPTIC_SHORT_ON_MS, HAPTIC_SHORT_OFF_MS);
    } else if (b == HAPTIC_CMD_LONG) {
      hapticStart(1, HAPTIC_LONG_ON_MS, 0);
    } else if (b == HAPTIC_CMD_TICK) {
      hapticStart(1, HAPTIC_SHORT_ON_MS, 0);
    } else if (b == US_CMD_RESET) {
      for (uint8_t ch = 0; ch < US_N; ch++) {
        usAngleLv[ch] = US_ANGLE_LEVEL_CH[ch];
        usNoiseLv[ch] = US_NOISE_DEFAULT;
      }
      usCfgPending = true;
    } else if (b > US_CMD_NOISE_BASE && b <= US_CMD_NOISE_BASE + 5) {
      for (uint8_t ch = 0; ch < US_N; ch++) usNoiseLv[ch] = b - US_CMD_NOISE_BASE;
      usCfgPending = true;
    } else if (b > US_CMD_SIDE_ANGLE_BASE && b <= US_CMD_SIDE_ANGLE_BASE + 4) {
      usAngleLv[US_SIDE_CH_L] = b - US_CMD_SIDE_ANGLE_BASE;
      usAngleLv[US_SIDE_CH_R] = b - US_CMD_SIDE_ANGLE_BASE;
      usCfgPending = true;
    } else if (b >= STATE_MIN && b <= STATE_MAX) {
      lastRxMillis    = now;
      watchdogTripped = false;
      everConnected   = true;   // 최초 1회만 의미 있음
      applyState((uint8_t)b);
    }
    // 그 밖의 값은 버린다. 잘못된 명령으로 오작동하지 않게 하는 안전장치.
  }

  // ── 워치독: 수신 중 단절만 감지 ────────
  // everConnected가 false인 동안(부팅 중)에는 발동하지 않는다.
  // 처음부터 USB가 미연결인 경우는 젯슨 측 포트 open 실패로 감지한다.
  if (everConnected && !watchdogTripped &&
      now - lastRxMillis > WATCHDOG_TIMEOUT_MS) {
    watchdogTripped = true;
    applyState(STATE_LINK_LOST);
  }

  // ── 초음파 순차 측정 (논블로킹) ─────────
  // 링크 상태와 무관하게 돈다. 젯슨이 조용해도 측정·송신은 계속한다.
  usTask(now);

  // ── 터치센서 (논블로킹) ────────────────
  // 초음파와 같은 이유로 링크 상태와 무관하게 돈다. 젯슨은 이 프레임이
  // 끊기는 것으로 상향 두절을 판정하므로, 조용해지면 안 된다.
  touchPoll();

  // ── 진동 패턴 (논블로킹) ────────────────
  hapticTask(now);

  // ── 서보 슬로우 이동 ───────────────────
  // 한 번에 돌리지 않고 14ms마다 1도씩. 사용자 손목에 충격을 주지 않는다.
  if (servoAngle != servoTarget && now - lastServoStep >= SERVO_STEP_MS) {
    lastServoStep = now;
    servoAngle += (servoAngle < servoTarget) ? 1 : -1;
    myServo.write(servoAngle);
  }

  // ── 라인 애니메이션 (방향 안내) ─────────
  if (currentMode == WAVE_A || currentMode == WAVE_B) {
    if (now - lastWave >= WAVE_MS) {
      lastWave = now;
      int numLeds = (currentMode == WAVE_A) ? NUM_LEDS_A : NUM_LEDS_B;
      if (currentMode == WAVE_A) drawLine(ledA, NUM_LEDS_A, linePos);
      else                        drawLine(ledB, NUM_LEDS_B, linePos);
      linePos++;
      if (linePos > numLeds + LINE_LEN) linePos = 0;
    }
  }

  // ── 점멸 애니메이션 (E-stop) ────────────
  if (currentMode == BLINK_BOTH && now - lastBlink >= BLINK_MS) {
    lastBlink  = now;
    blinkState = !blinkState;
    setBoth(blinkState ? ORANGE : OFF);
  }

  // ── 도착 점멸 (3회 후 자동 종료) ────────
  if (currentMode == BLINK_ARRIVE && now - lastBlink >= ARRIVE_BLINK_MS) {
    lastBlink = now;

    // 3회를 다 채웠으면 더 토글하지 않는다. 계속 토글하면 소등을 기다리는
    // 동안 4번째 ON이 생긴다.
    if (arriveBlinksLeft > 0 || blinkState) {
      blinkState = !blinkState;
      setBoth(blinkState ? SKY : OFF);
    }

    // ON이 된 시점을 1회로 센다.
    //
    // [주의] OFF 시점에 세면 마지막 3회차가 온전히 보이지 않는다. 3번째 ON 직후
    // 곧바로 OFF로 내려가면서 같은 프레임에 SKY 상시 점등으로 덮어써지기 때문에,
    // 사용자 눈에는 2회만 깜빡인 것처럼 보인다(2026-07-28 실측).
    // ON에서 세고 종료는 그 다음 OFF 프레임까지 기다려야 3회가 온전히 보인다.
    if (blinkState && arriveBlinksLeft > 0) {
      arriveBlinksLeft--;
    }
    // 마지막 OFF도 한 주기를 온전히 유지한 뒤 복귀한다.
    //
    // [주의] OFF 프레임에서 곧바로 setBoth(SKY)를 실행하면 그 OFF가 0ms가 되어
    // 마지막 점멸이 "짧게 스치고 마는" 것처럼 보인다(2026-07-28 실측).
    // arriveTailPending으로 한 프레임을 더 기다려 소등 시간을 확보한다.
    if (!blinkState && arriveBlinksLeft == 0) {
      if (arriveTailPending) {
        arriveTailPending = false;   // 이번 프레임은 OFF를 그대로 유지
      } else {
        // 도착은 일회성 이벤트다. 계속 켜져 있으면 진행 중으로 오인되므로
        // 3회 뒤 스스로 기본 표시로 돌아간다.
        //
        // [중요] currentState는 STATE_ARRIVED로 남겨둔다.
        // 젯슨이 arrival_hold_sec 동안 코드 5를 계속 보내는데,
        // 여기서 currentState를 NORMAL로 바꾸면 다음 코드 5 수신이
        // "상태 변화"로 인식되어 점멸이 무한 반복된다.
        // 표시만 되돌리고 상태는 유지해야 정확히 3회로 끝난다.
        currentMode = NORMAL;
        setBoth(SKY);
      }
    }
  }
}
