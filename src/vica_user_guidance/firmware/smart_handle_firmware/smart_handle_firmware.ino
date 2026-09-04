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
// [주의] 이 장치는 안내 전용이다. 서보는 로봇을 조향하지 않고,
//        LED 표시는 모터 정지를 보장하지 않는다. 정지 권한은 Safety 계층에 있다.

#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

// ══════════════════════════════════════════
#define NUM_LEDS_A  30
#define NUM_LEDS_B  30
// ══════════════════════════════════════════

#define SERVO_PIN   7
#define LED_A_PIN   8
#define LED_B_PIN   9
#define BLINK_MS    300
#define WAVE_MS     30
#define LINE_LEN    25

#define SERVO_CENTER  90
#define SERVO_LEFT   180
#define SERVO_RIGHT    0
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

Servo myServo;
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
// 순차 실행되므로 겹치지 않는다. show()가 인터럽트를 끄는 동안 millis()가
// 1~2ms 밀릴 수 있어 WAIT 에 여유(데이터시트 80 + 10ms)를 둔다.
//
// 채널 0 = front_left  (7bit 0x68 — usonic_addr_setup 스케치로 주소 굽기)
// 채널 1 = front_right (7bit 0x74 — 공장 기본)
// ══════════════════════════════════════════════════════════════════
#define US_N          2
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
#define US_GAP_MS     5     // 채널 사이 간격. 앞 채널 잔향이 다음 측정에 남지 않게
#define US_REG_DIST   0x02
#define US_REG_CMD    0x10
// 상향 프레임 8B: AA 55 seq d0L d0H d1L d1H xor (거리 mm, little-endian).
// 헤더 0xAA/0x55 는 하향 상태코드(0~7)와 겹치지 않아 양방향이 섞여도 안전하다.
// 값 규약: 0 = 채널 무효(3회 연속 실패, 젯슨 쪽은 그 채널 미발행)
//          1~3000 = 실거리 mm
//          3001 = 범위 내 에코 없음(clear, 젯슨 쪽은 max_range 로 발행해 부채꼴을 지운다)
#define US_FRAME_H1   0xAA
#define US_FRAME_H2   0x55

const uint8_t US_ADDR7[US_N] = { 0x68, 0x74 };

enum UsPhase { US_TRIG, US_WAIT, US_READ };
UsPhase       usPhase   = US_TRIG;
uint8_t       usCh      = 0;
bool          usTrigOk  = false;
unsigned long usPhaseAt = 0;
uint8_t       usSeq     = 0;
uint16_t      usDist[US_N]  = { 0, 0 };  // 프레임에 실을 값. 0 = 무효
uint8_t       usFails[US_N] = { 0, 0 };  // 연속 실패 수
uint16_t      usBuf[US_N][3];            // 3점 중앙값용 최근 유효 샘플
uint8_t       usBufN[US_N]  = { 0, 0 };

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

void usSendFrame() {
  uint8_t f[8];
  f[0] = US_FRAME_H1;
  f[1] = US_FRAME_H2;
  f[2] = usSeq++;
  f[3] = usDist[0] & 0xFF;
  f[4] = usDist[0] >> 8;
  f[5] = usDist[1] & 0xFF;
  f[6] = usDist[1] >> 8;
  f[7] = f[2] ^ f[3] ^ f[4] ^ f[5] ^ f[6];
  Serial.write(f, 8);
}

// 트리거 실패도 WAIT 를 그대로 거친다 — 센서가 빠져 있어도 주기가 흔들리지
// 않아 프레임이 항상 약 5Hz 로 나간다(실패 시 시리얼 폭주 방지).
void usTask(unsigned long now) {
  switch (usPhase) {
    case US_TRIG:
      if (now - usPhaseAt < US_GAP_MS) return;
      usTrigOk  = usWrite8(US_ADDR7[usCh], US_REG_CMD, US_TRIG_CMD);
      usPhase   = US_WAIT;
      usPhaseAt = now;
      return;

    case US_WAIT:
      if (now - usPhaseAt < US_WAIT_MS) return;
      usPhase = US_READ;
      return;

    case US_READ: {
      uint16_t raw;
      // 0xFFFF = 측정 미완료, 0xFFFE = 동주파수 간섭 — 둘 다 거리가 아니다(§2.9)
      // 0xFFFD = 범위 내 에코 없음(2026-08-31 실기 관찰, 데이터시트 미기재)
      //          — 정상 상황이므로 clear 로 저장한다. 실패로 치면 앞이 뚫려
      //          있을 때마다 채널이 무효(0)가 되어 costmap 을 지울 수 없다.
      if (usTrigOk && usReadDist(US_ADDR7[usCh], &raw)) {
        if (raw >= 1 && raw <= 3000)      usStore(usCh, raw);
        else if (raw == 0xFFFD)           usStore(usCh, US_CLEAR_MM);
        else                              usFail(usCh);
      } else {
        usFail(usCh);
      }
      usPhase   = US_TRIG;
      usPhaseAt = now;
      usCh++;
      if (usCh >= US_N) {
        usCh = 0;
        usSendFrame();
      }
      return;
    }
  }
}

void setA(uint32_t color) {
  for (int i = 0; i < NUM_LEDS_A; i++) ledA.setPixelColor(i, color);
  ledA.show();
}
void setB(uint32_t color) {
  for (int i = 0; i < NUM_LEDS_B; i++) ledB.setPixelColor(i, color);
  ledB.show();
}
void setBoth(uint32_t color) { setA(color); setB(color); }

void drawLine(Adafruit_NeoPixel &strip, int numLeds, int pos) {
  for (int i = 0; i < numLeds; i++) {
    int dist = pos - (numLeds - 1 - i);
    strip.setPixelColor(i, (dist >= 0 && dist < LINE_LEN) ? ORANGE : OFF);
  }
  strip.show();
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
      currentMode = WAVE_A;   // D8 = 왼쪽 (2026-08-02 실측)
      setB(SKY); setA(OFF);   // 주황이 흐르는 A는 끄고 반대쪽 B를 하늘색으로
      servoMoveTo(SERVO_LEFT);    // 2026-09-04 반전. 이전에는 SERVO_RIGHT 였다
      break;

    case STATE_RIGHT:
      currentMode = WAVE_B;   // D9 = 오른쪽 (2026-08-02 실측)
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
      break;
  }
}

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_CENTER);

  ledA.begin(); ledA.show();
  ledB.begin(); ledB.show();
  setBoth(SKY);   // 부팅 대기 표시. 첫 수신 전까지 이 상태를 유지한다.

  lastRxMillis = millis();

  // ── 초음파 I2C ──
  // 케이블이 길어 데이터시트 상한(100kHz)의 절반으로 시작한다(§4.2-4).
  // setWireTimeout: SDA 락업 시 25ms 후 자동 복구 — 없으면 loop() 전체가 멎어
  // 서보·LED·워치독까지 같이 죽는다.
  Wire.begin();
  Wire.setClock(50000);
  Wire.setWireTimeout(25000, true);

  // 지향각 레벨 1을 전원 인가 시마다 굽는다. 레지스터 휘발 여부가 데이터시트에
  // 없어, 기본값(레벨 4·60°)으로 돌아가면 높이 91.3mm 수평 장착에서 16cm 앞부터
  // 바닥이 장애물로 찍힌다(§3.1). 냉기동 직후 센서 안정화(≤1s) 대비 3회 재시도.
  for (uint8_t i = 0; i < US_N; i++) {
    uint8_t tries = 3;
    while (tries-- && !usWrite8(US_ADDR7[i], 0x07, US_ANGLE_LEVEL)) delay(100);
  }
  usPhaseAt = millis();
}

void loop() {
  unsigned long now = millis();

  // ── 시리얼 수신 (1바이트 상태 코드) ────
  if (Serial.available()) {
    int b = Serial.read();
    if (b >= STATE_MIN && b <= STATE_MAX) {
      lastRxMillis    = now;
      watchdogTripped = false;
      everConnected   = true;   // 최초 1회만 의미 있음
      applyState((uint8_t)b);
    }
    // 범위 밖 값은 버린다. 잘못된 명령으로 오작동하지 않게 하는 안전장치.
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
