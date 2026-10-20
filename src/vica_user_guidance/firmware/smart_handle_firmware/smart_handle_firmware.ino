#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define NUM_LEDS_A  21
#define NUM_LEDS_B  21

#define SERVO_PIN         8
#define LED_A_PIN         9
#define LED_B_PIN        11
#define SERVO_US_MIN    544
#define SERVO_US_MAX   2400
#define SERVO_HZ         50
#define BLINK_MS    300
#define WAVE_MS     30
#define LINE_LEN    25

#define SERVO_CENTER  90
#define SERVO_LEFT   120
#define SERVO_RIGHT   60
#define SERVO_STEP_MS  14

#define ARRIVE_BLINK_MS    500
#define ARRIVE_BLINK_COUNT   3

#define STATE_NORMAL     0
#define STATE_LEFT       1
#define STATE_RIGHT      2
#define STATE_ESTOP      3
#define STATE_LINK_LOST  4
#define STATE_ARRIVED    5

#define STATE_MIN  STATE_NORMAL
#define STATE_MAX  STATE_ARRIVED

#define WATCHDOG_TIMEOUT_MS  1500

Servo myServo;
Adafruit_NeoPixel ledA(NUM_LEDS_A, LED_A_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ledB(NUM_LEDS_B, LED_B_PIN, NEO_GRB + NEO_KHZ800);

const uint32_t SKY    = Adafruit_NeoPixel::Color(0,   200, 255);
const uint32_t ORANGE = Adafruit_NeoPixel::Color(255, 80,  0  );
const uint32_t RED    = Adafruit_NeoPixel::Color(255, 0,   0  );
const uint32_t GREEN  = Adafruit_NeoPixel::Color(0,   255, 0  );
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

bool everConnected = false;

uint8_t arriveBlinksLeft = 0;
bool arriveTailPending = false;

#define US_N          8
#define US_ROUNDS     4
#define US_PER_ROUND  2
#define US_SEND_LEGACY        1
#define US_LEGACY_AFTER_ROUND 1
#define US_TRIG_CMD   0xBC
#define US_WAIT_MS    100
#define US_CLEAR_MM   3001
#define US_ANGLE_LEVEL 0x03
const uint8_t US_ANGLE_LEVEL_CH[US_N] = { 3, 1, 1, 3, 3, 3, 3, 3 };
#define US_GAP_MS     5
#define US_REG_DIST   0x02
#define US_REG_CMD    0x10
#define US_FRAME_H1   0xAA
#define US_FRAME_H2   0x55
#define US_FRAME8_H2  0x57
#define US_STAT_H2            0x58
#define US_STAT_KINDS         6
#define US_STAT_EVERY_CYCLES  12
enum UsStatKind { US_ST_OK, US_ST_CLEAR, US_ST_FFFF, US_ST_FFFE, US_ST_OTHER, US_ST_I2C };
#define US_CMD_RESET          0x30
#define US_CMD_NOISE_BASE     0x30
#define US_CMD_SIDE_ANGLE_BASE 0x40
#define US_NOISE_DEFAULT      1
#define US_REG_NOISE          0x06
#define US_REG_ANGLE          0x07
#define US_CFG_H2             0x59
#define US_SIDE_CH_L          0
#define US_SIDE_CH_R          3

#define TOUCH_PIN         2
#define TOUCH_FRAME_H1   0xAA
#define TOUCH_FRAME_H2   0x56
#define TOUCH_FLAG_ON    0x01
#define TOUCH_PERIOD_MS  50
#define TOUCH_HOLD_MS    200

uint8_t       touchSeq     = 0;
unsigned long touchLastLow = 0;
bool          touchSeenLow = false;
unsigned long touchSentAt  = 0;

#define HAPTIC_PIN             6
#define HAPTIC_CMD_SHORT      0x10
#define HAPTIC_CMD_LONG       0x11
#define HAPTIC_CMD_TICK       0x12
#define HAPTIC_CMD_LOCATE     0x13
#define HAPTIC_SHORT_ON_MS    300
#define HAPTIC_SHORT_OFF_MS   150
#define HAPTIC_SHORT_COUNT    3
#define HAPTIC_LONG_ON_MS     1200
#define HAPTIC_LOCATE_ON_MS   1000
#define HAPTIC_LOCATE_OFF_MS  1000
#define HAPTIC_LOCATE_COUNT   2

uint8_t       hapticLeft  = 0;
bool          hapticOn    = false;
unsigned long hapticAt    = 0;
unsigned int  hapticOnMs  = 0;
unsigned int  hapticOffMs = 0;

void hapticStart(uint8_t count, unsigned int onMs, unsigned int offMs) {
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

const uint8_t US_ADDR7[US_N] = {
  0x68,
  0x69,
  0x6A,
  0x6B,
  0x6F,
  0x6E,
  0x6D,
  0x6C,
};
const uint8_t US_ROUND_CH[US_ROUNDS][US_PER_ROUND] = {
  { 1, 5 },
  { 2, 6 },
  { 0, 3 },
  { 7, 4 },
};
const uint8_t US_CH_ON[US_N] = { 0, 1, 1, 0, 0, 0, 0, 0 };
#define US_LEGACY_CH0 1
#define US_LEGACY_CH1 2

enum UsPhase { US_TRIG, US_WAIT, US_READ };

bool usRoundOn(uint8_t r) {
  for (uint8_t k = 0; k < US_PER_ROUND; k++) if (US_CH_ON[US_ROUND_CH[r][k]]) return true;
  return false;
}
UsPhase       usPhase   = US_TRIG;
uint8_t       usRound   = 0;
bool          usTrigOk[US_PER_ROUND] = { false, false };
unsigned long usPhaseAt = 0;
uint8_t       usSeq     = 0;
uint16_t      usDist[US_N]  = { 0 };
uint8_t       usFails[US_N] = { 0 };
uint16_t      usBuf[US_N][3];
uint8_t       usBufN[US_N]  = { 0 };
uint8_t       usStat[US_N][US_STAT_KINDS] = { { 0 } };
uint8_t       usStatSeq    = 0;
uint8_t       usAngleLv[US_N];
uint8_t       usNoiseLv[US_N];
bool          usCfgPending = false;
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
  usDist[ch] = (usBufN[ch] >= 3)
      ? usMedian3(usBuf[ch][0], usBuf[ch][1], usBuf[ch][2])
      : mm;
}

void usFail(uint8_t ch) {
  if (usFails[ch] < 255) usFails[ch]++;
  if (usFails[ch] >= 3) {
    usDist[ch] = 0;
    usBufN[ch] = 0;
  }
}

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

void touchPoll() {
  unsigned long now = millis();

  if (digitalRead(TOUCH_PIN) == LOW) {
    touchLastLow = now;
    touchSeenLow = true;
  }
  bool touched = touchSeenLow && (now - touchLastLow < TOUCH_HOLD_MS);

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

void usTask(unsigned long now) {
  switch (usPhase) {
    case US_TRIG:
      if (now - usPhaseAt < US_GAP_MS) return;
      if (usCfgPending) {
        usCfgPending = false;
        usApplyConfig();
      }
      for (uint8_t k = 0; k < US_PER_ROUND; k++) {
        uint8_t ch = US_ROUND_CH[usRound][k];
        usTrigOk[k] = US_CH_ON[ch] && usWrite8(US_ADDR7[ch], US_REG_CMD, US_TRIG_CMD);
      }
      usPhase   = US_WAIT;
      usPhaseAt = now;
      return;

    case US_WAIT:
      if (now - usPhaseAt < US_WAIT_MS) return;
      usPhase = US_READ;
      return;

    case US_READ: {
      for (uint8_t k = 0; k < US_PER_ROUND; k++) {
        uint8_t  ch = US_ROUND_CH[usRound][k];
        uint16_t raw;
        if (!US_CH_ON[ch]) continue;
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
      while (usRound < US_ROUNDS && !usRoundOn(usRound)) usRound++;
      if (usRound >= US_ROUNDS) {
        usRound = 0;
        while (usRound < US_ROUNDS - 1 && !usRoundOn(usRound)) usRound++;
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

#define LED_SAFE_TCNT_MIN   5200
#define LED_SAFE_TCNT_MAX  37600
#define LED_SAFE_WAIT_US   25000

void ledShowSafe(Adafruit_NeoPixel &strip) {
  if (myServo.attached()) {
    unsigned long t0 = micros();
    for (;;) {
      noInterrupts();
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

void applyState(uint8_t state) {
  if (state == currentState) return;
  currentState = state;

  blinkState = false;
  linePos    = 0;
  lastBlink  = millis();
  lastWave   = millis();

  switch (state) {
    case STATE_NORMAL:
      currentMode = NORMAL;
      setBoth(SKY);
      servoMoveTo(SERVO_CENTER);
      break;

    case STATE_LEFT:
      currentMode = WAVE_A;
      setB(SKY); setA(OFF);
      servoMoveTo(SERVO_LEFT);
      break;

    case STATE_RIGHT:
      currentMode = WAVE_B;
      setA(SKY); setB(OFF);
      servoMoveTo(SERVO_RIGHT);
      break;

    case STATE_ESTOP:
      currentMode = BLINK_BOTH;
      setBoth(OFF);
      servoMoveTo(SERVO_CENTER);
      hapticStart(1, HAPTIC_LONG_ON_MS, 0);
      break;

    case STATE_LINK_LOST:
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
      hapticStart(HAPTIC_SHORT_COUNT, HAPTIC_SHORT_ON_MS, HAPTIC_SHORT_OFF_MS);
      break;
  }
}

void setup() {
  pinMode(HAPTIC_PIN, OUTPUT);
  digitalWrite(HAPTIC_PIN, LOW);

  Serial.begin(115200);

  myServo.attach(SERVO_PIN, SERVO_US_MIN, SERVO_US_MAX);
  myServo.write(SERVO_CENTER);

  ledA.begin(); ledA.show();
  ledB.begin(); ledB.show();
  setBoth(SKY);

  lastRxMillis = millis();

  pinMode(TOUCH_PIN, INPUT_PULLUP);
  touchSentAt = millis();

  pinMode(HAPTIC_PIN, OUTPUT);
  digitalWrite(HAPTIC_PIN, LOW);

  Wire.begin();
  Wire.setClock(50000);
  Wire.setWireTimeout(25000, true);

  for (uint8_t i = 0; i < US_N; i++) {
    usAngleLv[i] = US_ANGLE_LEVEL_CH[i];
    usNoiseLv[i] = US_NOISE_DEFAULT;
    uint8_t tries = 3;
    while (tries-- && !usWrite8(US_ADDR7[i], 0x07, US_ANGLE_LEVEL_CH[i])) delay(100);
  }
  usCfgPending = true;
  usPhaseAt = millis();
}

void loop() {
  unsigned long now = millis();

  if (Serial.available()) {
    int b = Serial.read();
    if (b == HAPTIC_CMD_SHORT) {
      hapticStart(HAPTIC_SHORT_COUNT, HAPTIC_SHORT_ON_MS, HAPTIC_SHORT_OFF_MS);
    } else if (b == HAPTIC_CMD_LONG) {
      hapticStart(1, HAPTIC_LONG_ON_MS, 0);
    } else if (b == HAPTIC_CMD_TICK) {
      hapticStart(1, HAPTIC_SHORT_ON_MS, 0);
    } else if (b == HAPTIC_CMD_LOCATE) {
      hapticStart(HAPTIC_LOCATE_COUNT, HAPTIC_LOCATE_ON_MS, HAPTIC_LOCATE_OFF_MS);
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
      everConnected   = true;
      applyState((uint8_t)b);
    }
  }

  if (everConnected && !watchdogTripped &&
      now - lastRxMillis > WATCHDOG_TIMEOUT_MS) {
    watchdogTripped = true;
    applyState(STATE_LINK_LOST);
  }

  usTask(now);

  touchPoll();

  hapticTask(now);

  if (servoAngle != servoTarget && now - lastServoStep >= SERVO_STEP_MS) {
    lastServoStep = now;
    servoAngle += (servoAngle < servoTarget) ? 1 : -1;
    myServo.write(servoAngle);
  }

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

  if (currentMode == BLINK_BOTH && now - lastBlink >= BLINK_MS) {
    lastBlink  = now;
    blinkState = !blinkState;
    setBoth(blinkState ? ORANGE : OFF);
  }

  if (currentMode == BLINK_ARRIVE && now - lastBlink >= ARRIVE_BLINK_MS) {
    lastBlink = now;

    if (arriveBlinksLeft > 0 || blinkState) {
      blinkState = !blinkState;
      setBoth(blinkState ? SKY : OFF);
    }

    if (blinkState && arriveBlinksLeft > 0) {
      arriveBlinksLeft--;
    }
    if (!blinkState && arriveBlinksLeft == 0) {
      if (arriveTailPending) {
        arriveTailPending = false;
      } else {
        currentMode = NORMAL;
        setBoth(SKY);
      }
    }
  }
}
