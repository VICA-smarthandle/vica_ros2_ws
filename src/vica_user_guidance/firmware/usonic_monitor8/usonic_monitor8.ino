#include <Wire.h>
const uint8_t ADDR[8] = {0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F};
bool wr8(uint8_t a, uint8_t r, uint8_t v){ Wire.beginTransmission(a); Wire.write(r); Wire.write(v); return Wire.endTransmission()==0; }
bool rd16(uint8_t a, uint16_t *o){ Wire.beginTransmission(a); Wire.write(0x02); if(Wire.endTransmission()!=0) return false; if(Wire.requestFrom(a,(uint8_t)2)!=2) return false; uint8_t h=Wire.read(), l=Wire.read(); *o=((uint16_t)h<<8)|l; return true; }
void setup(){ Serial.begin(115200); Wire.begin(22,21); Wire.setClock(50000); Wire.setTimeOut(25); delay(800); }
void loop(){
  for(int i=0;i<8;i++){
    uint16_t mm=0; bool ok=wr8(ADDR[i],0x10,0xBC); delay(100); if(ok) ok=rd16(ADDR[i],&mm);
    Serial.printf("0x%02X:", ADDR[i]);
    if(!ok) Serial.print("---- "); else if(mm==0xFFFD) Serial.print("clr  "); else if(mm>=0xFFFE) Serial.print("err  "); else Serial.printf("%4d ", mm);
    delay(5);
  }
  Serial.println();
}
