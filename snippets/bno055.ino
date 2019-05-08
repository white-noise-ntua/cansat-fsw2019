#include <Wire.h>

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