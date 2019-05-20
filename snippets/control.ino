#include <Wire.h>
#include <Servo.h>

// bmp ---
#include <Adafruit_BMP280.h>
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C


//  bno ---
#define RPM_ADDR 10
#define BNO055_ADDR 0x28
#define NUM_BYTES 3

#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E

#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_EULER_H_MSB_ADDR 0x1B
#define BNO055_EULER_R_LSB_ADDR 0x1C
#define BNO055_EULER_R_MSB_ADDR 0x1D
#define BNO055_EULER_P_LSB_ADDR 0x1E
#define BNO055_EULER_P_MSB_ADDR 0x1F

#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0x19

#define BNO055_ID 0xA0
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_SYS_TRIGGER_ADDR 0X3F
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define POWER_MODE_NORMAL 0x00
#define OPERATION_MODE_CONFIG 0x00
#define OPERATION_MODE_NDOF 0x0C

struct vector {
  double x;
  double y;
  double z;
};

bool write8(byte reg, byte value) {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
  return true;
}

byte read8(byte reg) {
  byte value = 0;

  Wire.beginTransmission(BNO055_ADDR);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom(BNO055_ADDR, (byte)1);
  value = Wire.read();

  return value;
}


bool beginBNO() {
  /* Switch to config mode (just in case since this is the default) */
  write8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
  /* Reset */
  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    delay(10);
  }
  delay(50);
  /* Set to normal power mode */
  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  delay(10);

  write8(BNO055_PAGE_ID_ADDR, 0);
  write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  write8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  delay(20);

  return true;
}

vector getEuler() {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write((uint8_t)BNO055_EULER_H_LSB_ADDR);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDR, (byte)6);

  byte buf[6];
  for (uint8_t i = 0; i < 6; i++) {
    buf[i] = Wire.read();
  }

  int16_t x = ((int16_t)buf[0]) | (((int16_t)buf[1]) << 8);
  int16_t y = ((int16_t)buf[2]) | (((int16_t)buf[3]) << 8);
  int16_t z = ((int16_t)buf[4]) | (((int16_t)buf[5]) << 8);

  vector v;
  v.x = (((double)x) / 16.0);
  v.y = (((double)y) / 16.0);
  v.z = (((double)z) / 16.0);
  return v;
}

vector getGyro() {
  Wire.beginTransmission(BNO055_ADDR);
  Wire.write((uint8_t)BNO055_GYRO_DATA_X_LSB_ADDR);
  Wire.endTransmission();
  Wire.requestFrom(BNO055_ADDR, (byte)6);

  byte buf[6];
  for (uint8_t i = 0; i < 6; i++) {
    buf[i] = Wire.read();
  }

  int16_t x = ((int16_t)buf[0]) | (((int16_t)buf[1]) << 8);
  int16_t y = ((int16_t)buf[2]) | (((int16_t)buf[3]) << 8);
  int16_t z = ((int16_t)buf[4]) | (((int16_t)buf[5]) << 8);

  vector v;
  v.x = (((double)x) / 16.0);
  v.y = (((double)y) / 16.0);
  v.z = (((double)z) / 16.0);
  return v;
}

// RPM ---

double getRPM() {
  byte data[3];
  Wire.requestFrom(RPM_ADDR, 3);
  for (byte i=0; i<NUM_BYTES; i++)
    data[i] = Wire.read();

  uint32_t d = ((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + ((uint32_t)data[2]);
  return (double)(60.0 / (d * 0.000004));
}


// Moments of Inertia


long int LastMeasurement,previous;
float moments;

float K[2] = {0.01,0.0028}; // Kp, Kd

// Fins configurations

const float FINS_OFFSET[3] = {0, 0, 0};
const float FINS_MIN[3] = {70, 70, 70};
const float FINS_MAX[3] = {110, 110, 110};
const int finPins[3] = {20, 21, 23};
Servo fins[3];


// Sampling period
const float SAMPLING_PERIOD = 0.02;

// IMU Structs
vector euler;
//imu::Vector<3> gyroscope;

// Moments (in Nm)
float M[3];

// Fin Positions (in degrees)
float finPos[3];

float x[3] = {0,0,0};  // theta theta_dot theta_integral

// Struct to hold coordinates to our reference frame


//inverse fins model
void inverse_fins(float M){
 // a = map(M, -0.06,0.06, -20, 20);
  finPos[0] = finPos[1] = finPos[2] = map(M, -0.06,0.06, 70, 110);

  finPos[0] += FINS_OFFSET[0];
  finPos[1] += FINS_OFFSET[1];
  finPos[2] += FINS_OFFSET[2];
}


void transform_coords() {

  coords.psi = deg_to_rad(- wrap_angle(euler.x);
  coords.psi_dot = (coords.psi - prev_coords.psi) / SAMPLING_PERIOD;

}



float deg_to_rad(float deg) {
  return PI / 180 * deg;
}


float wrap_angle(float x) {
  if (x >= 180) {
    return x - 360;
  } else {
    return x;
  }

}



/* YAW CONTROL */

void get_measurements() {
  // Get raw measurements from BNO055
  euler = getEuler();
  transform_coords();
}


/* MAIN CONTROL ROUTINES */
void control() {
  // Main controller routine

  // Get measurements from IMU and filter them

  get_measurements();

  x[0] = coords.psi;
  x[1] = coords.psi_dot;


  moments = K[0]*x[0]+K[1]*x[1];

  inverse_fins(moments);

  for(int j=0; j<3; j++){
    fins[j].write(int(constrain(finPos[j], FINS_MIN[j], FINS_MAX[j])));
  }
  prev_coords = coords;
}

// #######################################
const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
char commandd[numChars] = {0};

boolean newData = false;
boolean transmit = false;
// #######################################

void setup() {

  Wire.begin();
  Serial2.begin(115200);
  Serial.begin(9600);

  for (int i = 0; i < 3; i++) {
    fins[i].attach(finPins[i]);
  }
  for(int j = 0; j<3; j++){
  fins[j].write(FINS_OFFSET[j]);
  }

  beginBNO();
   Serial.println("BMP280 test");

  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  previous = millis();
  Serial.print("Hello bitches -- Initialization complete");
}

void loop() {

  if(millis()-previous >= SAMPLING_PERIOD*1000 && transmit){

    Serial2.print(" T:  ");
    Serial2.print(millis());
    Serial2.print(" theta: ");
    Serial2.print(coords.theta*180/3.1516);
    Serial2.print(" phi: ");
    Serial2.print(coords.phi*180/3.1516);
    Serial2.print(" psi: ");
    Serial2.print(coords.psi*180/3.1516);
    Serial2.print(" RPM: ");
    Serial2.print(getRPM());
//    Serial2.print(" M:  ");
//    Serial2.print(moments);
    Serial2.print(" fP:  ");
    Serial2.print(finPos[1]);
    Serial2.print("  P:  ");
    Serial2.println(bmp.readPressure());
    control();
    previous = millis();
  }

  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    showParsedData();
    newData = false;
   }
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial2.available() > 0 && newData == false) {
        rc = Serial2.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(tempChars,",");
    strcpy(commandd, strtokIndx); // copy it to messageFromPC

    if (strcmp(commandd, "start") == 0) {
      Serial2.println(commandd);
      transmit = true;
    }
    else if (strcmp(commandd, "change") == 0) {
      Serial2.println(commandd);
      transmit = false;
      strtokIndx = strtok(NULL,",");
      K[0] = atof(strtokIndx);
      strtokIndx = strtok(NULL, ",");
      K[1] = atof(strtokIndx);
      strtokIndx = strtok(NULL, ",");
      K[2] = atof(strtokIndx);
    }
}

void showParsedData() {
    Serial2.print("f1 ");
    Serial2.println(K[0],4);
    Serial2.print("f2 ");
    Serial2.println(K[1],4);
    Serial2.print("f3 ");
    Serial2.println(K[2],4);
}
