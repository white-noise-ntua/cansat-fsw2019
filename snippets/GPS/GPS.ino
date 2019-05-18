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
  Serial1.begin(9600);

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

  sampleTimer = millis();
}

void loop(){
  if(millis() - sampleTimer >= 1000){
    readGPS();
    String printout = String(latitude) + "," +
                      String(longitude) + "," +
                      String(gpsSats) + "," +
                      String(gpsAltitude) + "," +
                      GPSTime + "\n";
    Serial1.print(printout);
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
