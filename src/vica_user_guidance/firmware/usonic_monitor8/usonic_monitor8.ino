// [TEST ONLY] DYP-A22 IIC 초음파 8개(7bit 0x68~0x6F) 순차 측정 모니터 — 본 펌웨어와 별개, 점검용
//
// 용도: 어느 주소가 어느 자리에 달려 있는지 확인한다. 센서 하나 앞 10cm 에 장애물을 두고
//       시리얼(115200)에서 어느 주소가 100mm 아래로 떨어지는지 본다. 2026-09-22 에 이 방법으로
//       8개 자리↔주소 표를 확정했다(devlog/2026-09-22-esp32-스마트핸들-이식.md).
// 핀·타임아웃은 본 펌웨어와 같다(SDA 22 / SCL 21, 50kHz, 25ms).
// 빌드: arduino-cli compile --fqbn esp32:esp32:esp32 firmware/usonic_monitor8
// 끝나면 본 펌웨어를 다시 올릴 것 — 이 스케치는 젯슨 프레임(AA55/AA56)을 내보내지 않는다.
#include <Wire.h>
const uint8_t ADDR[8] = {0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F};
bool wr8(uint8_t a, uint8_t r, uint8_t v){ Wire.beginTransmission(a); Wire.write(r); Wire.write(v); return Wire.endTransmission()==0; }
bool rd16(uint8_t a, uint16_t *o){ Wire.beginTransmission(a); Wire.write(0x02); if(Wire.endTransmission()!=0) return false; if(Wire.requestFrom(a,(uint8_t)2)!=2) return false; uint8_t h=Wire.read(), l=Wire.read(); *o=((uint16_t)h<<8)|l; return true; }
void setup(){ Serial.begin(115200); Wire.begin(22,21); Wire.setClock(50000); Wire.setTimeOut(25); delay(800); }
void loop(){
  for(int i=0;i<8;i++){
    uint16_t mm=0; bool ok=wr8(ADDR[i],0x10,0xBC); delay(100); if(ok) ok=rd16(ADDR[i],&mm);   // 0xBC = 150cm 모드
    Serial.printf("0x%02X:", ADDR[i]);
    if(!ok) Serial.print("---- "); else if(mm==0xFFFD) Serial.print("clr  "); else if(mm>=0xFFFE) Serial.print("err  "); else Serial.printf("%4d ", mm);
    delay(5);   // 채널 간격: 앞 채널 잔향이 남지 않게
  }
  Serial.println();
}
