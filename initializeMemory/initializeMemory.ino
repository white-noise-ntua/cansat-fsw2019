#include <EEPROM.h>

// Memory map
// Addresses -> Value
//
// 10 -> prevState
// (0,1,2,3) or 255 if the software runs for the first time

// 15 -> isNichromeBurned
// 20 -> areSensorsCalibrated
// -- 0 for false, 1 for true


// 25..29 -> packetCount
// 40..44 -> pitch
// 50..54 -> roll
// 60..64 -> yaw
// 70..74 -> altitude offset
// 80..84 -> gpsAltitude at surface

#define EEPROM_ADDR_PREVSTATE 10
#define EEPROM_ADDR_NICHROME 15
#define EEPROM_ADDR_CALIBRATION 20
#define EEPROM_ADDR_PACKET_COUNT 25
#define EEPROM_ADDR_PITCH 40
#define EEPROM_ADDR_ROLL 50
#define EEPROM_ADDR_YAW 60
#define EEPROM_ADDR_ALTITUDE 70
#define EEPROM_ADDR_GPS_ALT 80


float zero = 0.0;
int prevState = 255;
float num;
int i;

void setup(){
  Serial.begin(9600);
  Serial.println("Initializing EEPROM memory...");
  Serial.println("Writing values...");

  // writing prevState
  EEPROM.write(EEPROM_ADDR_PREVSTATE,prevState);

  // writing isNichromeBurned
  EEPROM.write(EEPROM_ADDR_NICHROME,0);

  // writing areSensorsCalibrated
  EEPROM.write(EEPROM_ADDR_CALIBRATION,0);

  // writing packetCount
  storeInt(EEPROM_ADDR_PACKET_COUNT,0);

  // writing pitch
  writeFloat(EEPROM_ADDR_PITCH,zero);

  // writing roll
  writeFloat(EEPROM_ADDR_ROLL,zero);

  // writing yaw
  writeFloat(EEPROM_ADDR_YAW,zero);

  // writing pressure level
  writeFloat(EEPROM_ADDR_ALTITUDE,zero);

  // writing gps altitude
  writeFloat(EEPROM_ADDR_GPS_ALT,zero);

  Serial.println("OK!");

}

void loop(){
Serial.println("Checking values written...");

  i = EEPROM.read(EEPROM_ADDR_PREVSTATE);
  if(i==prevState) Serial.println("Previous state is OK!");
  else Serial.println("Error writing previous state!");

  i = EEPROM.read(EEPROM_ADDR_NICHROME);
  if(i==0) Serial.println("isNichromeBurned is OK!");
  else Serial.println("Error writing isNichromeBurned!");

  i = EEPROM.read(EEPROM_ADDR_CALIBRATION);
  if(i==0) Serial.println("areSensorsCalibrated is OK!");
  else Serial.println("Error writing areSensorsCalibrated!");

  i = readInt(EEPROM_ADDR_PACKET_COUNT);
  if(i==0) Serial.println("Packet Count is OK!");
  else Serial.println("Error writing packet count!");

  readFloat(EEPROM_ADDR_PITCH,num);
  if(num==0.0) Serial.println("Pitch is OK!");
  else Serial.println("Error writing pitch!");

  readFloat(EEPROM_ADDR_ROLL,num);
  if(num==0.0) Serial.println("Roll is OK!");
  else Serial.println("Error writing roll!");

  readFloat(EEPROM_ADDR_YAW,num);
  if(num==0.0) Serial.println("Yaw is OK!");
  else Serial.println("Error writing yaw!");

  readFloat(EEPROM_ADDR_ALTITUDE,num);
  if(num==0.0) Serial.println("Altitude offset is OK!");
  else Serial.println("Error writing altitude offset!");

  readFloat(EEPROM_ADDR_GPS_ALT,num);
  if(num==0.0) Serial.println("GPS Altitude is OK!");
  else Serial.println("Error writing GPS altitude!");

  Serial.println("Checks finised! Do not forget to upload the flight software!");

  delay(2000);
}

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
