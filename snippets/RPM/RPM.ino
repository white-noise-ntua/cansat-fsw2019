#include <Wire.h>

#define RPM_ADDR 10
#define NUM_BYTES 3

double getRPM() {
  byte data[3];
  Wire.requestFrom(RPM_ADDR, 3);
  for (byte i=0; i<NUM_BYTES; i++)
    data[i] = Wire.read();

  uint32_t d = ((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + ((uint32_t)data[2]);
  return (double)(60.0 / (d * 0.000004));
}

uint32_t timer;
double rpm;

void setup(){
  Serial.begin(9600);
  timer = millis();
}

void loop(){
  if(millis() - timer >= 500){
    rpm = getRPM();
    Serial.println("RPM: "+String(rpm));
  }
}
