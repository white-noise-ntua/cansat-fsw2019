//#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BuzzerPin 17

#define FinsServo1 3
#define FinsServo2 4
#define FinsServo3 5

#define VoltageSensor 16
// A2

#define CameraPin 20
#define CameraServo1 21
#define CameraServo2 22
#define CameraServo3 23

// checkpoints
#define NUMBER_OF_TRIES 10
#define ALTITUDE_CHECKPOINT_STATE0 500

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
int TC;
// BONUS DIRECTION (?)

// Global Varriables for state = 0
bool sensorsCalibrated = false;


void setup(){
  pinMode(BuzzerPin,OUTPUT);

  pinMode(FinsServo1,OUTPUT);
  pinMode(FinsServo2,OUTPUT);
  pinMode(FinsServo3,OUTPUT);

  pinMode(CameraServo1,OUTPUT);
  pinMode(CameraServo2,OUTPUT);
  pinMode(CameraServo3,OUTPUT);
  pinMode(CameraPin,OUTPUT);

  Serial2.begin(9600); // XBee
  Serial3.begin(9600); // GPS

  //find state

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
  while(!sensorsCalibrated){
    //USE CODE FROM WIND TUNNEL TEST
    //FOR TALKING TO XBEE
    if(Serial2.available()){
      // calibrate sensors
      sensorsCalibrated = true;
    }

    // take measurements
    // handle telemetry

  }
  
  TC = NUMBER_OF_TRIES;
  
  while(TC > 0){
    // take measurements
    // handle telemetry
    if(altitude >= ALTITUDE_CHECKPOINT_STATE0 ){ //500m
      TC--;
    }
  }

  STATE = 1;
}

void runState1(){
  TC = NUMBER_OF_TRIES;
  
  while(TC > 0){
    // take measurements
    // handle telemetry
    if(altitude <= ALTITUDE_CHECKPOINT_STATE1 ){ //455m
      TC--;
    }
  }

  //burn nichrome wire
  //save isNichromeBurned = true in EEPROM
  
  STATE = 2;
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

void readVoltage(){
  voltage = analogRead(2);
}
