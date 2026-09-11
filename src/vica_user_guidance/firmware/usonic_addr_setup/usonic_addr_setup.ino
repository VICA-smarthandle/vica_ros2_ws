#include <Wire.h>

#define DO_BURN_ADDR 0

#define ADDR7_DEFAULT 0x74
#define ADDR8_NEW     0xD0
#define ADDR7_NEW     (ADDR8_NEW >> 1)
#define ANGLE_LEVEL   0x01
#define TRIG_CMD      0xBD
#define WAIT_MS       90

uint8_t found[8];
uint8_t nFound = 0;

void scan(const char* tag) {
  nFound = 0;
  Serial.print(tag);
  Serial.print(F(" scan: "));
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("0x"));
      Serial.print(a, HEX);
      Serial.print(' ');
      if (nFound < sizeof(found)) found[nFound++] = a;
    }
  }
  Serial.println(nFound == 0 ? F("(없음 — 배선·전원·풀업 확인)") : F(""));
}

bool wr8(uint8_t addr7, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool rd16(uint8_t addr7, uint8_t reg, uint16_t *out) {
  Wire.beginTransmission(addr7);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(addr7, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read(), lo = Wire.read();
  *out = ((uint16_t)hi << 8) | lo;
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(50000);
  Wire.setWireTimeout(25000, true);
  delay(1000);

  scan("before");

#if DO_BURN_ADDR
  Serial.print(F("addr write(0x05<-0xD0) ok="));
  Serial.println(wr8(ADDR7_DEFAULT, 0x05, ADDR8_NEW));
  delay(300);
  scan("after ");
#endif

  for (uint8_t i = 0; i < nFound; i++) {
    Serial.print(F("angle L1 -> 0x"));
    Serial.print(found[i], HEX);
    Serial.print(F(" ok="));
    Serial.println(wr8(found[i], 0x07, ANGLE_LEVEL));
  }
  Serial.println(F("--- 1초마다 거리 출력 시작 (0=실패, FFFF=미완료, FFFE=간섭) ---"));
}

const uint8_t TRIGS[] = { 0xBD, 0xBC, 0xB8, 0xB4, 0xB0 };
const uint8_t N_TRIG  = sizeof(TRIGS);

void loop() {
  for (uint8_t t = 0; t < N_TRIG; t++) {
    Serial.print(F("trig 0x"));
    Serial.print(TRIGS[t], HEX);
    Serial.print(F(" | "));
    for (uint8_t i = 0; i < nFound; i++) {
      uint16_t mm = 0;
      bool ok = wr8(found[i], 0x10, TRIGS[t]);
      delay(120);
      if (ok) ok = rd16(found[i], 0x02, &mm);
      Serial.print(F("0x"));
      Serial.print(found[i], HEX);
      Serial.print(F(": "));
      if (!ok)               Serial.print(F("comm-fail"));
      else if (mm == 0xFFFF) Serial.print(F("FFFF"));
      else if (mm == 0xFFFE) Serial.print(F("FFFE"));
      else if (mm == 0xFFFD) Serial.print(F("FFFD"));
      else                 { Serial.print(mm); Serial.print(F(" mm")); }
      Serial.print(F("   "));
    }
    Serial.println();
  }
  Serial.println(F("---"));
  delay(300);
}
