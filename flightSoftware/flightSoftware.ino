#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

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

// checkpoints
#define NUMBER_OF_TRIES 10
#define ALTITUDE_CHECKPOINT_STATE0 500
#define ALTITUDE_CHECKPOINT_STATE1 455
#define ALTITUDE_CHECKPOINT_STATE2 5

Adafruit_GPS GPS(&GPSSerial);
Adafruit_BMP280 bmp;

const int teamID = 4440;
int packetCount;
float altitude;
float pressure;
float temperature;
float voltage;
float missionTime; // mission time format(?) RTC?
String GPSTime; // GPS Time Format
float latitude;
float longitude;
float gpsAltitude;
int gpsSats;
float spinRate;

float pitch,roll; // Change them with the code from control

imu::Vector<3> euler;

int STATE;
int TC;
float bonusDirection; // BONUS DIRECTION (?)

// Global Varriables for state = 0
bool sensorsCalibrated = false;

// Global Varriables for handleTelemetry
uint32_t lastTransmit;

// Global Varriables for GPS
uint32_t GPStimer;

// Values that will be written to EEPROM
int prevState = -1;
bool isNichromeBurned = false;

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

  lastTransmit = millis();

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

    // take measurements
    handleTelemetry();

  }

  TC = NUMBER_OF_TRIES;

  while(TC > 0){
    // take measurements
    handleTelemetry();
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
    handleTelemetry();
    if(altitude <= ALTITUDE_CHECKPOINT_STATE1 ){ //455m
      TC--;
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
    // take measurements

    //control

    //gimbal response

    handleTelemetry();

    if(altitude <= ALTITUDE_CHECKPOINT_STATE2 ){ //5m
      TC--;
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
    String packet =
      String(teamID) + "," +
      String(missionTime) + "," +
      String(packetCount) + "," +
      String(altitude) + "," +
      String(pressure) + "," +
      String(temperature) + "," +
      String(voltage) + "," +
      String(GPSTime) + "," +
      String(latitude) + "," +
      String(longitude) + "," +
      String(gpsAltitude) + "," +
      String(gpsSats) + "," +
      String(pitch) + "," +
      String(roll) + "," +
      String(spinRate) + "," +
      String(bonusDirection) + "\n";

    Serial2.print(packet); // transmit telemetry packet
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
