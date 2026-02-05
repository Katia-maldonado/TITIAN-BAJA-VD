// **Hall Effect Sensor for CVT Dyno (Auto-increment log)**
// CSUF Baja SAE – Vehicle Dynamics

#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;
const int hallPin = 2;
const int maxCnt = 10;
File logFile;

void setup() {
  Serial.begin(9600);
  pinMode(hallPin, INPUT);

  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("FAILED!");
    while (1);
  }
  Serial.println("OK");

  // ---- Auto-increment file naming ----
  char filename[13];  // 8.3 format: e.g., RUN001.CSV
  int fileIndex = 1;
  do {
    sprintf(filename, "RUN%03d.CSV", fileIndex);
    fileIndex++;
  } while (SD.exists(filename) && fileIndex < 999);

  Serial.print("Logging to: ");
  Serial.println(filename);

  logFile = SD.open(filename, FILE_WRITE);
  if (logFile) {
    logFile.println("Time_s,RPM");
    logFile.flush();
  } else {
    Serial.println("File open error!");
  }
}

void loop() {
  unsigned long start = micros();
  int old = 1;
  int cnt = 0;

  while (cnt < maxCnt) {
    int val = digitalRead(hallPin);
    if (!val && val != old) cnt++;
    old = val;
  }

  float seconds = (micros() - start) / 1000000.0;
  float rpm = cnt / seconds * 60.0;
  float timeNow = millis() / 1000.0;

  // ---- Write to SD card ----
  if (logFile) {
    logFile.print(timeNow, 3);
    logFile.print(",");
    logFile.println(rpm);
    logFile.flush();  // ensures data is written immediately
  }

  // optional Serial echo
  Serial.print(timeNow, 3);
  Serial.print(",");
  Serial.println(rpm);
}
