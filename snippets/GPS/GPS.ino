#include <Adafruit_GPS.h>

#define GPSSerial Serial3

Adafruit_GPS GPS(&GPSSerial);

String GPSTime;
float latitude;
float longitude;
float gpsAltitude;
int gpsSats;

uint32_t GPStimer;
uint32_t sampleTimer;

void setup(){

  // begin serial monitor
  Serial2.begin(115200);
  
  // GPS Setup
  delay(2000);
  GPSSerial.begin(9600); // 9600 is the default baud rate
  GPSSerial.print("$PMTK251,115200*1F");  //change gps baudrate to 115200
  GPSSerial.write('\r');
  GPSSerial.write('\n');

  GPSSerial.flush();
  GPSSerial.end();

  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // packet type
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ); // 2 Hz update frequency
  delay(1000); //needed for GPS?
  Serial.println("Initialization completed!");
  sampleTimer = millis();
}

void loop(){
    readGPS();
    if(millis() - sampleTimer >= 1000){
      Serial2.print("GPS IS FIXED:\t"); Serial2.println(GPS.fix);
      if(GPS.fix){
          Serial2.print(latitude,4); Serial2.print(",");
          Serial2.print(longitude,4); Serial2.print(",");
          Serial2.print(gpsSats); Serial2.print(",");
          Serial2.print(gpsAltitude,1); Serial2.print(",");
          Serial2.println(GPSTime);
      }
      sampleTimer = millis();
    }
}

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
      gpsAltitude = GPS.altitude;
      GPSTime = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
    }
}
