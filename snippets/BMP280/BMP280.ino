#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

uint32_t timer;
float temperature;
float pressure;

void setup(){
  Serial.begin(9600); // start serial monitor

  if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  timer = millis();
}

void loop(){
  if(millis() - timer >= 1000){
    timer = millis();
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();
    String packet = "Temperature: " + String(temperature) + ",  Pressure: " + String(pressure);
    Serial.println(packet);

  }
}
