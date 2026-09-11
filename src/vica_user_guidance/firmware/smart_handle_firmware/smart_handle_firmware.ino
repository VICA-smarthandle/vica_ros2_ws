#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define NUM_LEDS_A  30
#define NUM_LEDS_B  30

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

#define US_N          2
#define US_TRIG_CMD   0xBC
#define US_WAIT_MS    100
#define US_CLEAR_MM   3001
#define US_ANGLE_LEVEL 0x03
#define US_GAP_MS     5
#define US_REG_DIST   0x02
#define US_REG_CMD    0x10
#define US_FRAME_H1   0xAA
#define US_FRAME_H2   0x55

#define TOUCH_PIN        11
#define TOUCH_FRAME_H1   0xAA
#define TOUCH_FRAME_H2   0x56
#define TOUCH_FLAG_ON    0x01
#define TOUCH_PERIOD_MS  50
#define TOUCH_HOLD_MS    200

uint8_t       touchSeq     = 0;
unsigned long touchLastLow = 0;
bool          touchSeenLow = false;
unsigned long touchSentAt  = 0;

#define HAPTIC_PIN            10
#define HAPTIC_CMD_SHORT      0x10
#define HAPTIC_CMD_LONG       0x11
#define HAPTIC_SHORT_ON_MS    300
#define HAPTIC_SHORT_OFF_MS   150
#define HAPTIC_SHORT_COUNT    3
#define HAPTIC_LONG_ON_MS     1200

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

const uint8_t US_ADDR7[US_N] = { 0x68, 0x74 };

enum UsPhase { US_TRIG, US_WAIT, US_READ };
UsPhase       usPhase   = US_TRIG;
uint8_t       usCh      = 0;
bool          usTrigOk  = false;
unsigned long usPhaseAt = 0;
uint8_t       usSeq     = 0;
uint16_t      usDist[US_N]  = { 0, 0 };
uint8_t       usFails[US_N] = { 0, 0 };
uint16_t      usBuf[US_N][3];
uint8_t       usBufN[US_N]  = { 0, 0 };

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
      break;
  }
}

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
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
    uint8_t tries = 3;
    while (tries-- && !usWrite8(US_ADDR7[i], 0x07, US_ANGLE_LEVEL)) delay(100);
  }
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
