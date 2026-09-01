// DYP-A22 IIC 초음파 — 센서 점검·주소 굽기 스케치 (본 펌웨어와 별개, 1회용 도구)
//
// 용도 3가지. docs/handoff_jetson_ultrasonic_i2c.md §7.1-3·§9 절차의 실행 도구다.
//   1. 스캔     : 버스에 붙은 센서 주소 확인 (공장 기본 7bit 0x74)
//   2. 주소 굽기: front_left 센서를 0x68 로 변경 — DO_BURN_ADDR 1 로 바꿔 업로드.
//                 ⚠️ 반드시 센서를 "하나만" 연결한 상태에서. 둘 다 연결하면
//                 기본 주소 0x74 가 충돌해 굽기가 어느 쪽에 갈지 알 수 없다.
//   3. 거리 점검: 붙어 있는 센서 전부를 1초마다 측정해 시리얼로 출력
//
// 절차 (시리얼 모니터 115200):
//   ① front_left 센서만 연결 → DO_BURN_ADDR 1 로 업로드 → "after scan: 0x68" 확인
//   ② 전원을 껐다 켠 뒤 다시 확인 → 0x68 유지되면 굽기 성공 (휘발성이면 여기서 드러남)
//   ③ 센서에 라벨을 붙인다 — "FL(0x68)". 안 붙이면 나중에 구분할 방법이 없다
//   ④ DO_BURN_ADDR 0 으로 되돌려 업로드 → front_right(0x74)도 연결 → 스캔에
//      0x68 과 0x74 가 함께 뜨고 두 채널 거리가 출력되면 점검 완료
//
// 지향각: 레벨 1(약 30°) — 2026-08-31 틸트 불가 확정에 따른 차선책(§5.1).
//         높이 91.3mm 수평 장착에서 바닥 에코를 0.30m 밖으로 밀기 위해 좁힌다.
//         ROS 쪽 field_of_view 0.524 rad 와 반드시 일치시킬 것.

#include <Wire.h>

#define DO_BURN_ADDR 0       // front_left 주소 굽기 시에만 1 (센서 하나만 연결!)

#define ADDR7_DEFAULT 0x74   // 0xE8>>1 공장 기본 → front_right 로 쓴다
#define ADDR8_NEW     0xD0   // 8bit 표기. 레지스터 0x05 에는 8bit 값을 쓴다
#define ADDR7_NEW     (ADDR8_NEW >> 1)   // = 0x68 → front_left
#define ANGLE_LEVEL   0x01   // 레벨 1 = 약 30° (§5.1 차선책. 1→0.524 rad)
#define TRIG_CMD      0xBD   // 50cm·mm. 0xFFFF 지속 시 0xBC 로 바꿔 시험(§2.10)
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
  Wire.setClock(50000);              // 케이블이 길다. 상한 100k 의 절반부터(§4.2-4)
  Wire.setWireTimeout(25000, true);  // I2C 락업 자동 복구
  delay(1000);                       // 센서 전원 안정화 ≤1000ms (§2.3)

  scan("before");

#if DO_BURN_ADDR
  // 주소 굽기 — 센서 하나만 연결돼 있어야 한다
  Serial.print(F("addr write(0x05<-0xD0) ok="));
  Serial.println(wr8(ADDR7_DEFAULT, 0x05, ADDR8_NEW));
  delay(300);
  scan("after ");                    // 0x68 로 바뀌어야 한다
#endif

  // 지향각 설정 — 스캔에 잡힌 센서 전부에 적용 (레벨 1 = 약 30°)
  for (uint8_t i = 0; i < nFound; i++) {
    Serial.print(F("angle L1 -> 0x"));
    Serial.print(found[i], HEX);
    Serial.print(F(" ok="));
    Serial.println(wr8(found[i], 0x07, ANGLE_LEVEL));
  }
  Serial.println(F("--- 1초마다 거리 출력 시작 (0=실패, FFFF=미완료, FFFE=간섭) ---"));
}

// 2026-08-31 실기: 0xBD 가 전 채널 0xFFFD(데이터시트에 없는 값)를 반환했다.
// 트리거 후보를 전부 순환 시험해 실제로 먹는 명령을 찾는다(§2.10 실기 확정).
const uint8_t TRIGS[] = { 0xBD, 0xBC, 0xB8, 0xB4, 0xB0 };  // 50/150/250/350cm + 예제의 0xB0
const uint8_t N_TRIG  = sizeof(TRIGS);

void loop() {
  for (uint8_t t = 0; t < N_TRIG; t++) {
    Serial.print(F("trig 0x"));
    Serial.print(TRIGS[t], HEX);
    Serial.print(F(" | "));
    for (uint8_t i = 0; i < nFound; i++) {
      uint16_t mm = 0;
      bool ok = wr8(found[i], 0x10, TRIGS[t]);
      delay(120);   // 최장 명령(350cm) 기준 110ms + 여유
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
