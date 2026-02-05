// CSUF BAJA SAE – VD/DAQ (Simple Version)
// Logs Primary & Secondary RPM with RTC time/date to SD.
// Button D7<->GND toggles logging (INPUT_PULLUP). LED D8 ON while logging.
// SD CS=D10. Hall sensors on D2/D3. RTC uses A4(SDA)/A5(SCL).

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

RTC_DS3231 rtc;

// -------- Pins --------
const int SD_CS        = 10;
const int hallPrimary  = 2;
const int hallSecondary = 3;
const int ledPin       = 8;
const int buttonPin    = 7;

// -------- Settings --------
const unsigned long SAMPLE_MS   = 50;  // sample window in ms
const uint8_t       PPR         = 1;    // pulses per rev (magnets per rev)
const unsigned long DEBOUNCE_MS = 50;   // button debounce

// -------- Counters / State --------
volatile unsigned long cntP = 0, cntS = 0;

bool logging     = false;
bool lastStable  = HIGH;   // button idle (INPUT_PULLUP)
bool debouncing  = false;
unsigned long tDebounce = 0;

bool sdOk  = false;
bool rtcOk = false;
unsigned long t0_ms = 0;   // millis at power-up (for fallback time)
char filename[32];

// --- ISRs (Hall Sensors) ---
void isrP() {
  cntP++;
}
void isrS() {
  cntS++;
}

// --- Build new file name ---
// Use simple 8.3 format: RUN001.CSV, RUN002.CSV, etc.
void buildNewRunFilename() {
  int run = 1;
  do {
    snprintf(filename, sizeof(filename), "RUN%03d.CSV", run);  // RUN001.CSV
    run++;
  } while (SD.exists(filename));
}

// --- Write CSV header row ---
void writeHeader() {
  File localFile = SD.open(filename, FILE_WRITE);
  if (localFile) {
    if (rtcOk) {
      localFile.println("Date,Time,Primary_RPM,Secondary_RPM");
    } else {
      localFile.println("Time_s,Primary_RPM,Secondary_RPM");
    }
    localFile.close();
    Serial.println("✅ Header written");
  } else {
    Serial.println("❌ Header write failed (open)");
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(hallPrimary, INPUT_PULLUP);
  pinMode(hallSecondary, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(hallPrimary), isrP, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallSecondary), isrS, FALLING);

  Serial.println("Initializing DAQ...");

  // --- SD card ---
  sdOk = SD.begin(SD_CS);
  Serial.print("SD: ");
  Serial.println(sdOk ? "OK" : "FAIL");

  // --- RTC ---
  rtcOk = rtc.begin();
  Serial.print("RTC: ");
  Serial.println(rtcOk ? "OK" : "FAIL");

  /* initial commit so RTC has accurate time, this forces the RTC to reset with the computer everytime upload code
  
  if (rtcOk) {
  Serial.println("Setting RTC to compile time");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
} 

  */

  // only resets the RTC time  if the battery dies or it loses power
  if (rtcOk && rtc.lostPower()) {
  Serial.println("RTC lostPower -> setting compile time");
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

  t0_ms = millis();
  Serial.println("DAQ ready. Press button to START/STOP logging.");
}

void loop() {
  unsigned long nowMs = millis();

  // --- Debounced pushbutton toggle (on RELEASE) ---
  bool raw = digitalRead(buttonPin);  // HIGH idle, LOW pressed
  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);
    if (raw2 != lastStable) {
      // toggle on release: LOW -> HIGH
      if (lastStable == LOW && raw2 == HIGH) {
        if (!logging) {
          // Turning logging ON
          if (!sdOk) {
            sdOk = SD.begin(SD_CS);
            Serial.print("Retry SD: ");
            Serial.println(sdOk ? "OK" : "FAIL");
          }
          if (sdOk) {
            buildNewRunFilename();
            writeHeader();
            logging = true;
            digitalWrite(ledPin, HIGH);
            Serial.print("▶ LOG ON -> ");
            Serial.println(filename);
          } else {
            Serial.println("⛔ Cannot start logging (SD not OK).");
          }
        } else {
          // Turning logging OFF
          logging = false;
          digitalWrite(ledPin, LOW);
          Serial.println("■ LOG OFF");
        }
      }
      lastStable = raw2;
    }
    debouncing = false;
  }

  // --- Periodic sample & write ---
  static unsigned long lastSample = 0;
  if (logging && (nowMs - lastSample >= SAMPLE_MS)) {
    lastSample = nowMs;

    // Copy and reset counts safely
    noInterrupts();
    unsigned long nP = cntP;
    unsigned long nS = cntS;
    cntP = 0;
    cntS = 0;
    interrupts();

    // RPM calculations
    float rpmP = (nP * (60000.0 / SAMPLE_MS)) / PPR;
    float rpmS = (nS * (60000.0 / SAMPLE_MS)) / PPR;

    File localFile = SD.open(filename, FILE_WRITE);
    if (localFile) {
      // --- Time/Date Column(s) ---
      if (rtcOk) {
        DateTime dt = rtc.now();
        // Date: YYYY/MM/DD
        localFile.print(dt.year());
        localFile.print('/');
        if (dt.month() < 10) localFile.print('0');
        localFile.print(dt.month());
        localFile.print('/');
        if (dt.day() < 10) localFile.print('0');
        localFile.print(dt.day());
        localFile.print(',');

        // Time: HH:MM:SS
        if (dt.hour() < 10) localFile.print('0');
        localFile.print(dt.hour());
        localFile.print(':');
        if (dt.minute() < 10) localFile.print('0');
        localFile.print(dt.minute());
        localFile.print(':');
        if (dt.second() < 10) localFile.print('0');
        localFile.print(dt.second());
        localFile.print(',');
      } else {
        // Fallback: time since power-up (seconds)
        float tsec = (millis() - t0_ms) / 1000.0;
        localFile.print(tsec, 3);
        localFile.print(',');
      }

      // --- RPM Data Columns ---
      localFile.print(rpmP, 2);
      localFile.print(',');
      localFile.println(rpmS, 2);

      localFile.close();
      Serial.println("💾 write ok");
    } else {
      Serial.println("❌ file open fail");
    }
  }
}
