#include <Wire.h>
#include <RTCx.h>

void setup(void)
{
	Serial.begin(9600);
	Wire.begin();
	Serial.println();
	Serial.println("Autoprobing for a RTC...");
	if (rtc.autoprobe()) {
		// Found something, hopefully a clock.
		Serial.print("Autoprobe found ");
		Serial.print(rtc.getDeviceName());
		Serial.print(" at 0x");
		Serial.println(rtc.getAddress(), HEX);
	}
	else {
		// Nothing found at any of the addresses listed.
		Serial.println("No RTCx found, cannot continue");
		while (1)
			;
	}
	// Ensure the oscillator is running.
	//rtc.startClock();
}

void loop(void)
{
	struct RTCx::tm tm;
	if (millis() - last > 500) {
		last = millis();
		rtc.readClock(tm);

//		RTCx::printIsotime(Serial, tm).println()/;
		RTCx::time_t t = RTCx::mktime(&tm);

		Serial.print("unixtime = ");
		Serial.println(t);
		Serial.println("-----");
	}

//	while (Serial.available()) {
//		char c = Serial.read();
//		if ((c == '\r' || c == '\n' || c == '\0')) {
//			if (bufPos <= bufLen && buffer[0] == 'C') {
//				// Check time error
//				buffer[bufPos] = '\0';
//				RTCx::time_t pcTime = atol(&(buffer[1]));
//				rtc.readClock(&tm);
//				RTCx::time_t mcuTime = RTCx::mktime(&tm);
//				Serial.print("MCU clock error: ");
//				Serial.print(mcuTime - pcTime);
//				Serial.println(" s");
//				Serial.println("~~~~~");
//			}
//			if (bufPos <= bufLen && buffer[0] == 'T') {
//				// Set time
//				buffer[bufPos] = '\0';
//				RTCx::time_t t = atol(&(buffer[1]));
//				RTCx::gmtime_r(&t, &tm);
//				rtc.setClock(&tm);
//				Serial.println("Clock set");
//				Serial.println(&(buffer[0]));
//				RTCx::printIsotime(Serial, tm);
//				Serial.println("~~~~~");
//			}
//			if (bufPos <= bufLen && buffer[0] == 'X') {
//				// Set calibration value
//				buffer[bufPos] = '\0';
//				if (rtc.getDevice() == RTCx::MCP7941x) {
//					int8_t oldCal = rtc.getCalibration();
//					char *endptr;
//					long cal = strtol(&(buffer[1]), &endptr, 0);
//					if (cal >= -127 && cal <= 127 && endptr == &buffer[bufPos]) {
//						Serial.print("Previous calibration: ");
//						Serial.println(oldCal, DEC);
//						Serial.print("Calibration: ");
//						Serial.println(cal, DEC);
//						rtc.setCalibration(cal);
//					}
//					else
//						Serial.println("Bad value for calibration");
//				}
//				else {
//					Serial.println("Cannot set calibration: not a MCP7941x");
//				}
//			}
//			if (bufPos <= bufLen && buffer[0] == 'M') {
//				// Set SQW mode
//				buffer[bufPos] = '\0';
//				char *endptr;
//				long mode = strtol(&(buffer[1]), &endptr, 0);
//				if (mode >= RTCx::freq1Hz && mode <= RTCx::freqCalibration
//					&& endptr == &buffer[bufPos]) {
//					if (rtc.setSQW((RTCx::freq_t)mode)) {
//						Serial.print("SQW: ");
//						Serial.println(mode, DEC);
//					}
//					else
//						Serial.println("Could not set SQW");
//				}
//				else
//					Serial.println("Bad value for SQW");
//			}
//			bufPos = 0;
//		}
//		else if (bufPos < bufLen)
//			// Store character
//			buffer[bufPos++] = c;
//	}
}
