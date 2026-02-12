#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Your wiring:
// GPS TX -> Arduino D5
// GPS RX -> Arduino D4
SoftwareSerial mySerial(5, 4);
Adafruit_GPS GPS(&mySerial);

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(9600);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // needed for fix + location
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // stable for UNO
  delay(1000);

  Serial.println(F("Lat, Lon (decimal degrees)"));
}

void loop() {
  // Continuously read GPS
  GPS.read();

  // Parse complete NMEA sentences
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  // Print once per second
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    if (GPS.fix) {
      Serial.print(GPS.latitudeDegrees, 6);
      Serial.print(F(", "));
      Serial.println(GPS.longitudeDegrees, 6);
    } else {
      Serial.println(F("No fix"));
    }
  }
}
