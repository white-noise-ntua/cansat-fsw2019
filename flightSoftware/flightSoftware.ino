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

#define VoltageSensor 16
// A2

#define CameraPin 20
#define CameraServo1 21
#define CameraServo2 22
#define CameraServo3 23

// RPM
#define RPM_ADDR 10
#define NUM_BYTES 3

// checkpoints
#define NUMBER_OF_TRIES 10
#define ALTITUDE_CHECKPOINT_STATE0 500
#define ALTITUDE_CHECKPOINT_STATE1 455
#define ALTITUDE_CHECKPOINT_STATE2 5

Adafruit_GPS GPS(&GPSSerial);
Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055();


const int teamID = 4440;
int packetCount;
float altitude;
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
uint32_t lastSensitivePoll;
bool newDataAvailable = false;

// Values that will be written to EEPROM
int prevState = -1;
bool isNichromeBurned = false;

void setup(){
  pinMode(BuzzerPin,OUTPUT);

  for (int i = 0; i < 3; i++) {
    fins[i].attach(finPins[i]);
    fins[i].write(FINS_OFFSET[i]);
  }

  pinMode(CameraServo1,OUTPUT);
  pinMode(CameraServo2,OUTPUT);
  pinMode(CameraServo3,OUTPUT);
  pinMode(CameraPin,OUTPUT);

  Serial2.begin(9600); // XBee

  delay(10000);
  // GPS Setup

  Serial3.begin(9600); // 9600 is the default baud rate
  Serial1.print("$PMTK251,115200*1F");  //change gps baudrate to 115200
  Serial1.write('\r');
  Serial1.write('\n');

  Serial1.flush();
  Serial1.end();

  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // packet type
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ); // 2 Hz update frequency
  // delay(1000); needed for GPS?

  // BMP Setup
  bmp.begin();
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode.       */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling    */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering.            */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time.         */


  // BNO Setup
  bno.begin();

  lastTransmit = lastSensitivePoll = lastControlMeasurement = millis(); // initialize time counters

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
  while(!sensorsCalibrated){
    //USE CODE FROM WIND TUNNEL TEST
    //FOR TALKING TO XBEE
    if(Serial2.available()){
      // calibrate sensors
      sensorsCalibrated = true;
    }

    getMeasurements();
    handleTelemetry();

  }

  TC = NUMBER_OF_TRIES;

  while(TC > 0){
    getMeasurements();
    handleTelemetry();
    if(newDataAvailable && altitude >= ALTITUDE_CHECKPOINT_STATE0 ){ //500m
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
    if(newDataAvailable && altitude <= ALTITUDE_CHECKPOINT_STATE1 ){ //455m
      TC--;
      newDataAvailable = false;
    }
  }

  //burn nichrome wire
  //save isNichromeBurned = true in EEPROM

  STATE = 2;
}

void runState2(){
  digitalWrite(CameraPin,HIGH);

  TC = NUMBER_OF_TRIES;

  while(TC > 0){

    if(millis() - lastControlMeasurement >= SAMPLING_PERIOD * 1000){
      lastControlMeasurement = millis();
      getMeasurements();
      control();
      //gimbal response
    }

    handleTelemetry();

    if(newDataAvailable && altitude <= ALTITUDE_CHECKPOINT_STATE2 ){ //5m
      TC--;
      newDataAvailable = false;
    }
  }

  STATE = 3;
}

void runState3(){
//  Activate Audio Beacon
  while(73){
    digitalWrite(BuzzerPin,HIGH);
    delay(2000);
    digitalWrite(BuzzerPin,LOW);
    delay(2000);
  }
}

void findState(){
  // read from EEPROM
  // take altitude measurements
  // set isAltitude increasing and isAltitude constant
  bool isAltIncreasing, isAltConstant;

  if(prevState==-1){ // != 0,1,2,3
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

  if(millis() - lastTransmit >= 1000){
    Serial2.print(teamID); Serial2.print(",");
    Serial2.print(missionTime); Serial2.print(",");
    Serial2.print(packetCount); Serial2.print(",");
    Serial2.print(altitude,1); Serial2.print(",");
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

    lastTransmit = millis();
    packetCount++;
    //store packetCount in EEPROM
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
  if(millis() - GPStimer >= 1000){ // read from GPS every 1 sec
    GPStimer = millis();
    if(GPS.newNMEAreceived()){
      GPS.parse(GPS.lastNMEA()); // parse the new packet

      // update measurements
      latitude = convertToDecimalDegrees(GPS.latitude);
      longitude = convertToDecimalDegrees(GPS.longitude);
      gpsSats = GPS.satellites;
      gpsAltitude = GPS.altitude;
      GPSTime = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
    }
  }
}

// ======================

void readTempPress() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1019.66); // give pressure at surface
}

void readGyro(){
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  pitch = -euler.y();
  roll = euler.z();
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
  readGyro();
  readGPS();

  // pitch,roll = euler.(?)
  if(millis() - lastSensitivePoll >= 500){
    lastSensitivePoll = millis();
    readVoltage();
    readTempPress();
    readRPM();
    readTime();
    newDataAvailable = true;
    // get startupTime from EEPROM
    // missionTime = secondsElapsed(...startupTime,hours,minutes,seconds);
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

  for(int i=0;i<sizeof(num);i++){
    EEPROM.write(addr++,*p++);
  }
}


void readFloat(int addr, float &num){
  byte* p = (byte*)(void*) &num;
  for(int i=0;i<sizeof(num);i++){
    *p++ = EEPROM.read(addr++);
  }
}

// ========================
