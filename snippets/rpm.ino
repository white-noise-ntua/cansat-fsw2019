#include <Wire.h>

double getRPM() {
  byte data[3];
  Wire.requestFrom(RPM_ADDR, 3);                            
  for (byte i=0; i<NUM_BYTES; i++) 
    data[i] = Wire.read();          

  uint32_t d = ((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + ((uint32_t)data[2]);
  return (double)(60.0 / (d * 0.000004));
}
