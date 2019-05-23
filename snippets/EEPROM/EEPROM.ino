#include  <EEPROM.h>

uint32_t i;
float pi = 3.14159, f = 73.7373;
float num;
String printout;

void setup(){
  Serial.begin(9600);

  // Testing integers
  storeInt(0,737373);
  storeInt(42,1001001);
  storeInt(117,73);

  i = readInt(0);
  printout = "Read number " + String(i) + " from address " + String(0);
  Serial.println(printout);

  i = readInt(42);
  printout = "Read number " + String(i) + " from address " + String(42);
  Serial.println(printout);

  i = readInt(117);
  printout = "Read number " + String(i) + " from address " + String(117);
  Serial.println(printout);

  // Testing floats
  writeFloat(200,pi);
  writeFloat(210,f);

  readFloat(200,num);
  Serial.print("Read number "); Serial.print(num,6);
  Serial.print(" from address "); Serial.println(200);

  readFloat(210,num);
  Serial.print("Read number "); Serial.print(num,6);
  Serial.print(" from address "); Serial.println(210);

}

void loop(){
}

// Helper functions for EEPROM

void storeInt(int addr, uint32_t num){
  // store num in EEPROM starting in address addr
  // EEPROM cells are 1 byte, integers are 4 bytes

  uint32_t byte1,byte2,byte3,byte4;

  byte1 = num & 0xFF;
  byte2 = (num>>8) & 0xFF;
  byte3 = (num>>16) & 0xFF;
  byte4 = (num>>24) & 0xFF;

  // little endian
  EEPROM.write(addr,byte1);
  EEPROM.write(addr+1,byte2);
  EEPROM.write(addr+2,byte3);
  EEPROM.write(addr+3,byte4);
}

uint32_t readInt(int addr){
  // read an integer stored in mem[addr..addr+3] in EEPROM
  // using little endian
  uint32_t result;
  uint32_t byte1,byte2,byte3,byte4;

  byte1 = (uint32_t) EEPROM.read(addr);
  byte2 = (uint32_t) EEPROM.read(addr+1);
  byte3 = (uint32_t) EEPROM.read(addr+2);
  byte4 = (uint32_t) EEPROM.read(addr+3);

  result = 0;
  result = result | byte1;
  result = result | ( (byte2<<8) & 0xFF00 );
  result = result | ( (byte3<<16) & 0xFF0000 );
  result = result | ( (byte4<<24) & 0xFF000000 );

  return result;
}

void writeFloat(int addr, float &num){
  const byte* p = (const byte*)(const void*) &num;

  for(unsigned int i=0;i<sizeof(num);i++){
    EEPROM.write(addr++,*p++);
  }
}


void readFloat(int addr, float &num){
  byte* p = (byte*)(void*) &num;
  for(unsigned int i=0;i<sizeof(num);i++){
    *p++ = EEPROM.read(addr++);
  }
}
