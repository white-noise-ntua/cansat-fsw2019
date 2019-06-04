#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Servo.h>
#include <EEPROM.h>


#define BuzzerPin 17
#define GPSSerial Serial3

#define FinsServo1 3
#define FinsServo2 4
#define FinsServo3 5

#define NichromeWire 6
#define VoltageSensor 16
// A2

#define CameraPin 20
#define CameraServo1 21
#define CameraServo2 22
#define CameraServo3 23


// Periods in milliseconds
#define TELEMETRY_PERIOD 1000
#define SENSITIVE_POLLING 500
#define BNO_POLLING 20

// Nichrome
#define NICHROME_INTENSITY 30
#define NICHROME_DURATION 2000 // milliseconds

// RPM
#define RPM_ADDR 10
#define NUM_BYTES 3

// checkpoints
#define NUMBER_OF_TRIES 5
#define ALTITUDE_CHECKPOINT_STATE0 500
#define ALTITUDE_CHECKPOINT_STATE1 455
#define ALTITUDE_CHECKPOINT_STATE2 5

// Addresses in EEPROM
#define EEPROM_ADDR_PREVSTATE 10
#define EEPROM_ADDR_NICHROME 15
#define EEPROM_ADDR_CALIBRATION 20
#define EEPROM_ADDR_PACKET_COUNT 25
#define EEPROM_ADDR_PITCH 40
#define EEPROM_ADDR_ROLL 50
#define EEPROM_ADDR_YAW 60
#define EEPROM_ADDR_ALTITUDE 70
#define EEPROM_ADDR_GPS_ALT 80


Adafruit_GPS GPS(&GPSSerial);
Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055();


const int teamID = 4440;
int packetCount;
float alt;
float pressure;
float temperature;
float voltage;
int missionTime;
String GPSTime;
float latitude;
float longitude;
float gpsAltitude;
int gpsSats;
double spinRate;
float bonusDirection;
float pitch,roll;


int STATE;
int TC;

// Global Varriables for calibration
float pitchOffset=0,rollOffset=0;
float gpsAltitudeOffset=0;
float altitudeOffset=0;
float surfacePressure = 1019.66;

// Global Varriables for 2-way communication
char receivedChar;
bool newDataReceived = false;


// Global Varriables for control

uint32_t lastControlMeasurement;
float moments;
float K[2] = {0.01,0.0028}; // Kp, Kd
// Fins configurations
const float FINS_OFFSET[3] = {0, 0, 0};
const float FINS_MIN[3] = {70, 70, 70};
const float FINS_MAX[3] = {110, 110, 110};
const int finPins[3] = {FinsServo1, FinsServo2, FinsServo3};
const float SAMPLING_PERIOD = 0.02;
Servo fins[3];

imu::Vector<3> euler;

float finPos[3];      // fin positions
float x[2] = {0,0};  // state varriables: psi, psi_dot

typedef struct coords_t {
  float psi;
  float psi_dot;

} coordinates;

coordinates coords;
coordinates prev_coords;

// Global Varriables for state = 0
bool sensorsCalibrated = false;

// Global Varriables for handleTelemetry
uint32_t lastTransmit;

// Global Varriables for GPS
uint32_t GPStimer;

// Global Varriables for RTC
int hours,minutes,seconds;

// Global Varriables for getMeasurements
uint32_t lastSensitivePoll,lastSensitive2;
bool newDataAvailable = false;

// Values that will be written to EEPROM
int prevState = -1;
bool isNichromeBurned = false;

void setup(){
  packetCount = readInt(EEPROM_ADDR_PACKET_COUNT);
  pinMode(BuzzerPin,OUTPUT);
  pinMode(CameraPin,OUTPUT);
  pinMode(NichromeWire,OUTPUT);

  for (int i = 0; i < 3; i++) {
    fins[i].attach(finPins[i]);
    fins[i].write(FINS_OFFSET[i]);
  }

  pinMode(CameraServo1,OUTPUT);
  pinMode(CameraServo2,OUTPUT);
  pinMode(CameraServo3,OUTPUT);

  Serial2.begin(115200); // XBee

  delay(2000);
  // GPS Setup

  Serial3.begin(9600); // 9600 is the default baud rate
  Serial3.print("$PMTK251,115200*1F");  //change gps baudrate to 115200
  Serial3.write('\r');
  Serial3.write('\n');

  Serial3.flush();
  Serial3.end();

  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // packet type
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update frequency

  // BMP Setup
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode.       */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling    */
                  Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering.            */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time.         */


  // BNO Setup
  bno.begin();

  lastTransmit = lastSensitivePoll = lastControlMeasurement = lastSensitive2 = millis(); // initialize time counters

  findState();

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
  sensorsCalibrated = EEPROM.read(EEPROM_ADDR_CALIBRATION);
  if(sensorsCalibrated){
    readFloat(EEPROM_ADDR_PITCH,pitchOffset);
    readFloat(EEPROM_ADDR_ROLL,rollOffset);
    readFloat(EEPROM_ADDR_ALTITUDE,altitudeOffset);
    readFloat(EEPROM_ADDR_GPS_ALT,gpsAltitudeOffset);
  }
  while(!sensorsCalibrated){
    getMeasurements();
    handleTelemetry();
    recvOneChar();

    if(newDataReceived){
      // calibrate sensors
      int numberOfmeasurements = 0;
      float measuringAlt=0;
      while(numberOfmeasurements < 10){ // sample sensors for approx 5 seconds
        getMeasurements();
        if(newDataAvailable){
          pitchOffset += pitch;
          rollOffset += roll;
          // yaw calibration is not needed
          measuringAlt += alt;
          gpsAltitudeOffset += gpsAltitude;

          numberOfmeasurements++;
          newDataAvailable = false;
        }
      }

      pitchOffset /= numberOfmeasurements;
      rollOffset /= numberOfmeasurements;
      altitudeOffset = measuringAlt/numberOfmeasurements;
      gpsAltitudeOffset /= numberOfmeasurements;

      writeFloat(EEPROM_ADDR_PITCH,pitchOffset);
      writeFloat(EEPROM_ADDR_ROLL,rollOffset);
      writeFloat(EEPROM_ADDR_ALTITUDE,altitudeOffset);
      writeFloat(EEPROM_ADDR_GPS_ALT,gpsAltitudeOffset);

      sensorsCalibrated = true;
      EEPROM.write(EEPROM_ADDR_CALIBRATION,1);
    }

  }

  TC = NUMBER_OF_TRIES;

  while(TC > 0){
    getMeasurements();
    handleTelemetry();
    if(newDataAvailable && alt >= ALTITUDE_CHECKPOINT_STATE0 ){ //500m
      TC--;
      newDataAvailable = false;
    }
  }

  STATE = 1;
}

void runState1(){
  TC = NUMBER_OF_TRIES;

  while(TC > 0){
    getMeasurements();
    handleTelemetry();
    if(newDataAvailable && alt <= ALTITUDE_CHECKPOINT_STATE1 ){ //455m
      TC--;
      newDataAvailable = false;
    }
  }

  // releasing payload
  analogWrite(NichromeWire,NICHROME_INTENSITY);
  delay(NICHROME_DURATION);
  analogWrite(NichromeWire,0);

  EEPROM.write(EEPROM_ADDR_NICHROME,1);
  //save isNichromeBurned = true in EEPROM

  STATE = 2;
}

void runState2(){
  digitalWrite(CameraPin,HIGH);

  TC = NUMBER_OF_TRIES;

  while(TC > 0){

    getMeasurements();

    if(millis() - lastControlMeasurement >= SAMPLING_PERIOD * 1000){
      lastControlMeasurement = millis();
      control();
      //gimbal response
    }

    handleTelemetry();

    if(newDataAvailable && alt <= ALTITUDE_CHECKPOINT_STATE2 ){ //5m
      TC--;
      newDataAvailable = false;
    }
  }

  STATE = 3;
}

void runState3(){
//  Activate Audio Beacon
  unsigned long duration = 1000;
  while(73){
    tone(BuzzerPin,2048,duration);
    delay(2*duration);
  }
}

void findState(){
  prevState = EEPROM.read(EEPROM_ADDR_PREVSTATE);
  isNichromeBurned = EEPROM.read(EEPROM_ADDR_NICHROME);

  // check if Altitude is increasing
  float heightMeasurements[8];
  bool isAltIncreasing, isAltConstant;
  int numberOfmeasurements = 0;

  while(numberOfmeasurements < 8){
    getMeasurements();
    if(newDataAvailable){
      heightMeasurements[numberOfmeasurements] = alt;
      newDataAvailable = false;
      numberOfmeasurements++;
    }
  }

  int increasing = 0;
  for(int i=1;i<8;i++){
    if(heightMeasurements[i] > heightMeasurements[i-1] + 1){
      increasing++;
    }
  }

  isAltIncreasing = (increasing >= 6);
  // 6 out of 7 are increasing, tollerant to 1 falsy measurement in 8

  if(isAltIncreasing){
    isAltConstant = false;
  }
  else{
    float threshold = heightMeasurements[0];
    int constant = 0;
    for(int i=0;i<8;i++){
      if((heightMeasurements[i] >= threshold-2) &&  (heightMeasurements[i] <= threshold+2)){
        constant++;
      }
    }
    isAltConstant = (constant >= 7);
    // also tolerant to 1 falsy measurement
  }


  if(prevState==255){ // != 0,1,2,3
    STATE = 0;
  }
  else if(isNichromeBurned){
    STATE = 2;
  }
  else if(isAltIncreasing){
    STATE = prevState;
  }
  else if(isAltConstant){
    STATE = 0;
  }
  else{
    STATE = 1;
  }
}

void handleTelemetry(){

  if(millis() - lastTransmit >= TELEMETRY_PERIOD){
    Serial2.print(teamID); Serial2.print(",");
    Serial2.print(missionTime); Serial2.print(",");
    Serial2.print(packetCount); Serial2.print(",");
    Serial2.print(alt,1); Serial2.print(",");
    Serial2.print(pressure,0); Serial2.print(",");
    Serial2.print(temperature,0); Serial2.print(",");
    Serial2.print(voltage,2); Serial2.print(",");
    Serial2.print(GPSTime); Serial2.print(",");
    Serial2.print(latitude,4); Serial2.print(",");
    Serial2.print(longitude,4); Serial2.print(",");
    Serial2.print(gpsAltitude,1); Serial2.print(",");
    Serial2.print(gpsSats); Serial2.print(",");
    Serial2.print(pitch,0); Serial2.print(",");
    Serial2.print(roll,0); Serial2.print(",");
    Serial2.print(spinRate,0); Serial2.print(",");
    Serial2.println(bonusDirection,0);
    ;
    lastTransmit = millis();
    packetCount++;
    storeInt(EEPROM_ADDR_PACKET_COUNT,packetCount);
  }
}

void readVoltage(){
  voltage = analogRead(2);
}

// === GPS Functions ===

double convertToDecimalDegrees(float deg){
  double minutes = 0.0;
  double decDeg = 0.0;

  minutes = fmod((double)deg, 100.0);
  deg = (int) (deg / 100);
  decDeg = deg + (minutes/60);

  return decDeg;
}

void readGPS(){
  GPS.read();
  if(GPS.newNMEAreceived()){
    GPS.parse(GPS.lastNMEA()); // parse the new packet

    // update measurements
    latitude = convertToDecimalDegrees(GPS.latitude);
    longitude = convertToDecimalDegrees(GPS.longitude);
    gpsSats = GPS.satellites;
    gpsAltitude = GPS.altitude - gpsAltitudeOffset;
    GPSTime = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
  }
}

// ======================

void readTempPress() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  alt = bmp.readAltitude(1019.66) - altitudeOffset;
}

void readGyro(){
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = -euler.y()-pitchOffset;
  roll = euler.z()-rollOffset;
  transform_coords();
}

void readRPM() {
  byte data[3];
  Wire.requestFrom(RPM_ADDR, 3);
  for (byte i=0; i<NUM_BYTES; i++)
    data[i] = Wire.read();

  uint32_t d = ((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + ((uint32_t)data[2]);
  spinRate =  (double)(60.0 / (d * 0.000004));
}

// === Functions for RTC ===

void readTime(){
  tmElements_t tm;
  RTC.read(tm);
  hours = tm.Hour;
  minutes = tm.Minute;
  seconds = tm.Second;
}

int secondsElapsed(int h1,int m1,int s1,int h2, int m2, int s2){
  return (h2-h1)*3600+(m2-m1)*60+(s2-s1);
}


// ======================


void getMeasurements(){
  readGPS();

  if(millis() - lastSensitivePoll >= SENSITIVE_POLLING){
    lastSensitivePoll = millis();
    readVoltage();
    readTempPress();
    readRPM();
    readTime();
    newDataAvailable = true;
    // get startupTime from EEPROM
    // missionTime = secondsElapsed(...startupTime,hours,minutes,seconds);
  }
  if(millis() - lastSensitive2 >= BNO_POLLING){
    lastSensitive2 = millis();
    readGyro();
  }
}

// === Functions for control ===

void inverse_fins(float M){
  finPos[0] = finPos[1] = finPos[2] = map(M, -0.06,0.06, 70, 110);
  finPos[0] += FINS_OFFSET[0];
  finPos[1] += FINS_OFFSET[1];
  finPos[2] += FINS_OFFSET[2];
}


void transform_coords() {
  coords.psi = deg_to_rad(- wrap_angle(euler.x()));
  coords.psi_dot = (coords.psi - prev_coords.psi) / SAMPLING_PERIOD;
}

float deg_to_rad(float deg) {
  return PI / 180 * deg;
}

float wrap_angle(float x) {
  if (x >= 180) {
    return x - 360;
  }
  else {
    return x;
  }
}

void control(){
  x[0] = coords.psi;
  x[1] = coords.psi_dot;

  moments = K[0]*x[0]+K[1]*x[1];
  inverse_fins(moments);

  for(int j=0; j<3; j++){
    fins[j].write(int(constrain(finPos[j], FINS_MIN[j], FINS_MAX[j])));
  }

  prev_coords = coords;
}

// =============================

// === EEPROM functions ===


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
// 70..74 -> pressure at surface
// 80..84 -> gpsAltitude at surface

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

// ========================


// 2way-communication
void recvOneChar(){
  if (Serial2.available() > 0){
    receivedChar = Serial2.read();
    if(receivedChar == 'C'){
      newDataReceived = true;
    }
  }
}
