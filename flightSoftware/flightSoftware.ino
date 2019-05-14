//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BuzzerPin 17
#define R

const int teamId = 4440;
int packetCount;
float altitude;
float pressure;
float temperature;
float voltage;
// mission time format(?) RTC?
// GPS Time Format
float latitude;
float longitude;
float gpsAltitude;
int gpsSats;
float spinRate;

imu::Vector<3> euler;

int STATE;

// BONUS DIRECTION (?)

void setup(){

}

void loop(){
  switch(STATE){
    case 0:
      runState0();
      break;
    case 1:
      runState1();
      break;
    case 2:
      runState2();
      break;
    case 3:
      runState3();
      break;
  }
}


void runState0(){
  Serial.println("0");
}

void runState1(){
  Serial.println("1");
}

void runState2(){
  Serial.println("2");
}

void runState3(){
//  Activate Audio Beacon/
  while(73){
    digitalWrite(BuzzerPin,HIGH);
    delay(2000);
    digitalWrite(BuzzerPin,LOW);
    delay(2000);
  }
}
