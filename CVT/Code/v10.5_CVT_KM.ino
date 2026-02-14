/*
Version: v10.5
Author: Katia Maldonado
System: CVT

- Serial Monitor: one-line RPM output ONLY while logging
- SD Card: CSV with separate Date and Time columns (Excel-friendly)
- Button toggles logging on RELEASE
- LED ON while logging
- Hot-swap SD: SD.begin() re-runs every time you start logging
- 3rd RPM sensor uses Pin Change Interrupt on D4
- RTC validity check (prevents 2000-00-00 / bad timestamps)

IMPORTANT:
- If your RTC time is wrong, set FORCE_SET_RTC_TO_COMPILE_TIME = true,
  upload ONCE, then set it back to false and upload again.
*/

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

// -------- RTC --------
RTC_DS3231 rtc;

// Set this true to force RTC = compile time (upload once), then set back to false
// double check this line 
////////////////////////////////////////////////////////////////////////////////////////////
///////////////////RTC/////////////////////////////////////////////////////////////////////
const bool FORCE_SET_RTC_TO_COMPILE_TIME = false;

// Consider RTC valid only if year >= this
const int RTC_VALID_YEAR_MIN = 2023;

// -------- Pins --------
const int SD_CS         = 10;
const int hallPrimary   = 2;
const int hallSecondary = 3;
const int hallBrake     = 4;
const int ledPin        = 8;
const int buttonPin     = 7;

// -------- Settings --------
const unsigned long SAMPLE_MS   = 50;   // 20 Hz RPM calc / SD logging rate
const unsigned long SERIAL_MS   = 250;  // Serial pretty display rate (only when logging)
const uint8_t       PPR         = 1;
const unsigned long DEBOUNCE_MS = 50;

// -------- Counters / State --------
volatile unsigned long cntP = 0, cntS = 0, cntB = 0;

bool logging     = false;
bool lastStable  = HIGH;
bool debouncing  = false;
unsigned long tDebounce = 0;

bool sdOk  = false;
bool rtcOk = false;
unsigned long t0_ms = 0;          // fallback time base if RTC invalid
char filename[13];                // "RUN001.CSV" + null

// Global RPM variables
float rpmP = 0;
float rpmS = 0;
float rpmB = 0;

// --- ISRs ---
void isrP() { cntP++; }
void isrS() { cntS++; }

// Pin Change ISR for D4 (Brake RPM)
ISR(PCINT2_vect) {
  static bool lastBrakeState = HIGH;
  bool state = (PIND & (1 << PIND4));     // read D4 directly
  if (lastBrakeState == HIGH && state == LOW) {
    cntB++;
  }
  lastBrakeState = state;
}

// --- Helper Functions ---
void buildNewRunFilename() {
  int run = 1;
  do {
    sprintf(filename, "RUN%03d.CSV", run);
    run++;
  } while (SD.exists(filename));
}

bool rtcLooksValid(const DateTime &dt) {
  return (dt.year() >= RTC_VALID_YEAR_MIN) && (dt.month() >= 1 && dt.month() <= 12) && (dt.day() >= 1 && dt.day() <= 31);
}

void writeHeader() {
  File localFile = SD.open(filename, FILE_WRITE);
  if (localFile) {
    // Always keep same columns so Excel is happy
    localFile.println(F("Date,Time,P_RPM,S_RPM,B_RPM"));
    localFile.close();
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
  pinMode(hallBrake, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(hallPrimary), isrP, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallSecondary), isrS, FALLING);

  // Enable PCINT for D4 (PCINT20)
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);

  // RTC init
  rtcOk = rtc.begin();

  if (rtcOk) {
    if (FORCE_SET_RTC_TO_COMPILE_TIME) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    } else if (rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // Print RTC at boot so you can confirm it is real
    DateTime dt = rtc.now();
    Serial.print(F("RTC boot read: "));
    Serial.print(dt.year()); Serial.print('-');
    Serial.print(dt.month()); Serial.print('-');
    Serial.print(dt.day()); Serial.print(' ');
    Serial.print(dt.hour()); Serial.print(':');
    Serial.print(dt.minute()); Serial.print(':');
    Serial.println(dt.second());
  } else {
    Serial.println(F("RTC begin() FAILED"));
  }

  t0_ms = millis();
  Serial.println(F("DAQ Ready. Press button to START/STOP logging."));
}

void loop() {
  unsigned long nowMs = millis();

  // --- 1) Button Logic (toggle logging on RELEASE) ---
  bool raw = digitalRead(buttonPin);

  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }

  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);

    if (raw2 != lastStable) {
      // Toggle on release: LOW -> HIGH
      if (lastStable == LOW && raw2 == HIGH) {

        if (!logging) {
          // START LOGGING (hot-swap: re-init SD each time)
          sdOk = SD.begin(SD_CS);

          if (sdOk) {
            buildNewRunFilename();
            writeHeader();

            logging = true;
            digitalWrite(ledPin, HIGH);

            Serial.print(F("Start of "));
            Serial.println(filename);
            Serial.println();
          } else {
            Serial.println(F("SD Fail"));
            for (int i = 0; i < 3; i++) {
              digitalWrite(ledPin, HIGH); delay(100);
              digitalWrite(ledPin, LOW);  delay(100);
            }
          }

        } else {
          // STOP LOGGING
          logging = false;
          digitalWrite(ledPin, LOW);

          Serial.print(F("End of "));
          Serial.println(filename);
          Serial.println();
        }
      }

      lastStable = raw2;
    }

    debouncing = false;
  }

  // --- 2) Sampling Logic ---
  static unsigned long lastSample = 0;
  if (nowMs - lastSample >= SAMPLE_MS) {
    lastSample = nowMs;

    // Atomic read + reset
    noInterrupts();
    unsigned long nP = cntP;
    unsigned long nS = cntS;
    unsigned long nB = cntB;
    cntP = 0; cntS = 0; cntB = 0;
    interrupts();

    // RPM calculations
    rpmP = (nP * (60000.0 / SAMPLE_MS)) / PPR;
    rpmS = (nS * (60000.0 / SAMPLE_MS)) / PPR;
    rpmB = (nB * (60000.0 / SAMPLE_MS)) / PPR;

    // --- 3) SD Logging (ONLY if active) ---
    if (logging) {
      File localFile = SD.open(filename, FILE_WRITE);
      if (localFile) {

        bool usedRtc = false;
        if (rtcOk) {
          DateTime dt = rtc.now();
          if (rtcLooksValid(dt)) {
            usedRtc = true;

            // Date: YYYY-MM-DD (Excel friendly)
            localFile.print(dt.year()); localFile.print('-');
            if (dt.month() < 10) localFile.print('0');
            localFile.print(dt.month()); localFile.print('-');
            if (dt.day() < 10) localFile.print('0');
            localFile.print(dt.day());
            localFile.print(',');

            // Time: HH:MM:SS
            if (dt.hour() < 10) localFile.print('0');
            localFile.print(dt.hour()); localFile.print(':');
            if (dt.minute() < 10) localFile.print('0');
            localFile.print(dt.minute()); localFile.print(':');
            if (dt.second() < 10) localFile.print('0');
            localFile.print(dt.second());
            localFile.print(',');
          }
        }

        // Fallback if RTC invalid
        if (!usedRtc) {
          // Put Date/Time columns as blanks and log Time_s in place of Date
          // (keeps the SAME number of columns every row)
          float tsec = (millis() - t0_ms) / 1000.0;
          localFile.print(tsec, 3);  // goes into Date column when RTC is bad
          localFile.print(',');
          localFile.print(F("NO_RTC"));
          localFile.print(',');
        }

        localFile.print(rpmP, 0); localFile.print(',');
        localFile.print(rpmS, 0); localFile.print(',');
        localFile.println(rpmB, 0);

        localFile.close();
      } else {
        logging = false;
        digitalWrite(ledPin, LOW);
        sdOk = false;
        Serial.println(F("SD write fail -> logging stopped"));
      }
    }
  }

  // --- 4) Serial Pretty One-Line Output (ONLY when logging) ---
  static unsigned long lastSerial = 0;
  if (logging && (nowMs - lastSerial >= SERIAL_MS)) {
    lastSerial = nowMs;

    Serial.print(F("Primary - "));
    Serial.print((int)rpmP);

    Serial.print(F("    |    Secondary - "));
    Serial.print((int)rpmS);

    Serial.print(F("    |    Brake - "));
    Serial.println((int)rpmB);
  }
}
