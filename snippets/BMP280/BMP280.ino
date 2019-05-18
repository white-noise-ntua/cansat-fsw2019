#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

uint32_t timer;
float temperature;
float pressure;

void setup(){
  Serial1.begin(9600); // start serial monitor

  if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
  }
  timer = millis();
}

void loop(){
  if(millis() - timer >= 1000){
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    String packet = "Temperature: " + String(temperature) + ",  Pressure: " + String(pressure);
    Serial1.println(packet);

  }
}
