/*
Version: v11
Author: Leon Nguyen
System: DAQ

Description:
- Primary RPM (Pin 2): Changed to Rising Edge logic with 2ms Debounce for AiM Adapter (Requires 12V->3.3V Divider).
- Secondary RPM (Pin 3): Logic updated for 5V Push-Pull Hall Sensor (Requires 5V->3.3V Divider).
- Brake Sensors Implementation: Added Front/Rear logging for DataQ 2000 PSI sensors (Requires 5V->3.3V Divider).
- IR Sensor Implementation: Added AiM Infrared Analog Sensor logic (Requires 5V->3.3V Divider).
- GPS speed-tracker Implentation
- Optimized Primary and Secondary RPM logic
- Enabled Hardware Analog Averaging to smooth out analog data
- Optimized constant calculations and buffer arrays

Sensors Included:
- Spark Plug RPM Adapter (Primary RPM) [CHANGED]
- Hall Effect Sensor (Secondary RPM) [CHANGED]
- DS3231 RTC (Real Time Clock)
- MPU-6050 (Accelerometer & Gyroscope)
- GPS Module (NMEA 9600 baud)
- Brake Pressure Sensor (Front) [NEW]
- Brake Pressure Sensor (Rear) [NEW]
- CVT Belt Temp Sensor (Analog AiM) [NEW]

TEENSY 4.1 HARDWARE WIRING GUIDE

WARNING: Teensy 4.1 is STRICTLY 3.3V. Do not plug 5V or 12V directly into pins, use Voltage Dividers!
Ensure Voltage Dividers are in place for all 5V/12V sensors.

--- COMMUNICATION PINS ---
- I2C SDA (IMU & RTC)     -> Pin 18 (A4)
- I2C SCL (IMU & RTC)     -> Pin 19 (A5)
- GPS RX (Serial1 RX1)    -> Pin 0  (Connects to GPS module's TX pin)
- GPS TX (Serial1 TX1)    -> Pin 1  (Connects to GPS module's RX pin)

--- ANALOG PINS (Require 5V to 3.3V Divider per sensor) ---
- Front Brake Pressure    -> Pin 14 (A0)
- Rear Brake Pressure     -> Pin 15 (A1)
- CVT Belt Temp (AiM IR)  -> Pin 16 (A2)

--- DIGITAL PINS ---
- Primary RPM (Spark)     -> Pin 2  (Requires 12V to 3.3V Divider)
- Secondary RPM (Hall)    -> Pin 3  (Requires 5V to 3.3V Divider)
- Push Button             -> Pin 7  (Wire other side to Ground, uses internal pullup)
- LED Indicator           -> Pin 8  (Use inline resistor to Ground)

--- IMU SPECIFIC WIRING ---
- MPU6050 AD0 Pin         -> Wire to 3.3V power (Sets address to 0x69 for code)

TOTAL VOLTAGE DIVIDERS : 5 (4: 5V -> 3.3V, 1: 12V -> 3.3V, 9x 2.2k and 1x 10k resistors)
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>

RTC_DS3231 rtc;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

// -------- Pins --------
const int pinRPM_Primary   = 2;   // Renamed from hallPrimary (AiM Adapter) [NEW]
const int pinRPM_Secondary = 3;   // Renamed from hallSecondary (Hall Sensor) [NEW]
const int ledPin           = 8;
const int buttonPin        = 7;

// Analog Pins (Must be behind Voltage Dividers!) [NEW]
const int pinBrakeF        = 14;  // A0 - Front Brake (DataQ) [NEW]
const int pinBrakeR        = 15;  // A1 - Rear Brake (DataQ) [NEW]
const int pinCVT           = 16;  // A2 - CVT Temp (AiM Analog) [NEW]

// -------- Settings --------
const unsigned long SAMPLE_MS     = 50;   // 20Hz Sampling
const uint8_t       PPR           = 1;    // Pulses Per Rev
const unsigned long DEBOUNCE_MS   = 50;   // Button Debounce
const unsigned long FLUSH_MS      = 1000; // Flush to SD every 1 second
const unsigned long RPM_DEBOUNCE  = 2000; // 2ms Lockout for Spark Noise [NEW]

// Voltage Divider Math (Assuming 2.2k resistors, ANALOG) [NEW]
// Ratio = 2200 / (2200 + 2200) = 0.5 [NEW]
// WARNING!!! : For 12v Primary Pin, use 10k TOP and 2.2k BOTTOM, DIGITAL) [NEW]
const float DIVIDER_RATIO = 0.5;          // [NEW]

//RPM Math [NEW]
const float MINUTE_MICROS_PPR = 60000000.0 / PPR;

// -------- Counters / State --------
// Swaped old pulse counters for period tracking [REWORKED]
volatile unsigned long lastTimeP = 0;
volatile unsigned long periodP = 0;

volatile unsigned long lastTimeS = 0;
volatile unsigned long periodS = 0;

bool logging      = false;
bool lastStable   = HIGH; 
bool debouncing   = false;
unsigned long tDebounce = 0;

bool sdOk = false, rtcOk = false, mpuOk = false;
unsigned long t0_ms = 0; 
char filename[32]; 

File logFile;

// --- ISRs ---
// Primary RPM (AiM Adapter) - Rising Edge with Debounce  [NEW]
void isrP() { 
  unsigned long now = micros();
  // Using 2000us (2ms) lockout for Spark Noise
  if (now - lastTimeP > RPM_DEBOUNCE) { 
    periodP = now - lastTimeP; 
    lastTimeP = now;
  }
}
// Secondary RPM (Hall) - Falling Edge [NEW]
void isrS() { 
  unsigned long now = micros();
  // Using a 1000us (1ms) debounce for the Hall sensor to prevent noise double-triggers
  if (now - lastTimeS > 1000) { 
    periodS = now - lastTimeS;
    lastTimeS = now;
  }
}

// --- GPS Parsing ---
void updateGPS() {
  while (Serial1.available() > 0) { 
    gps.encode(Serial1.read());
  }
}

// --- Build File Name ---
void buildNewRunFilename() {
  int run = 1;
  do {
    sprintf(filename, "RUN%03d.CSV", run); 
    run++;
  } while (SD.exists(filename));
}

// --- Write Header ---
void writeHeader() {
  // Added Speed, Brakes, and CVT columns to header
  logFile.println(F("Date,Time,P_RPM,S_RPM,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Lat,Lon,Speed_MPH,BrakeF_PSI,BrakeR_PSI,CVT_TempC"));
  logFile.flush();
  Serial.println(F("Header written"));
}

// --- Helper Functions --- [NEW]

float readPressurePSI(int pin) { // [NEW]
  int raw = analogRead(pin);
  float pinV = raw * (3.3 / 1023.0); // Convert bits to 3.3V scale [NEW]
  float sensorV = pinV / DIVIDER_RATIO; // Reverse voltage divider [NEW]
  
  if (sensorV < 0.5) return 0.0; // Floor 0.5V offset [NEW]
  return (sensorV - 0.5) * (500.0); // DataQ Scaling [NEW]
}

float readTempC(int pin) { // [NEW]
  int raw = analogRead(pin);
  float pinV = raw * (3.3 / 1023.0); 
  float sensorV = pinV / DIVIDER_RATIO; // Reverse voltage divider [NEW]
  
  // AiM Spec: 820mV = -20C, 2880mV = +250C [NEW]
  if (sensorV < 0.8) return -20.0;
  float tempC = (sensorV - 0.82) * 131.07 - 20.0; // [NEW]
  return tempC;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000); 
  
  Serial1.begin(9600);   

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  
  // RPM Inputs changed to INPUT (No Pullup) due to external Dividers [NEW]
  pinMode(pinRPM_Primary, INPUT); // [NEW]
  pinMode(pinRPM_Secondary, INPUT); // [NEW]

  // Analog Inputs [NEW]
  pinMode(pinBrakeF, INPUT); // [NEW]
  pinMode(pinBrakeR, INPUT); // [NEW]
  pinMode(pinCVT, INPUT); // [NEW]
  analogReadResolution(10); // Ensure 0-1023 range [NEW]
  analogReadAveraging(16); // Hardware averages 16 samples for clean analog data

  attachInterrupt(digitalPinToInterrupt(pinRPM_Primary), isrP, RISING); // Changed to RISING for AiM [NEW]
  attachInterrupt(digitalPinToInterrupt(pinRPM_Secondary), isrS, FALLING);

  Serial.println(F("Initializing Teensy 4.1 DAQ"));
  
  sdOk = SD.begin(BUILTIN_SDCARD);
  Serial.print(F("SD: "));
  Serial.println(sdOk ? F("OK") : F("FAIL"));

  rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.print(F("RTC: ")); Serial.println(rtcOk ? F("OK") : F("FAIL"));

  if (!mpu.begin(0x69)) { 
    Serial.println(F("MPU6050 FAIL - Check AD0 pin!"));
    mpuOk = false;
  } else {
    Serial.println(F("MPU6050 OK"));
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);      
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  t0_ms = millis();
  Serial.println(F("Ready. Press Button."));
}

void loop() {
  unsigned long nowMs = millis();
  updateGPS();

  // --- Button Logic ---
  bool raw = digitalRead(buttonPin);
  if (!debouncing && raw != lastStable) { 
    debouncing = true; 
    tDebounce = nowMs;
  }
  
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);
    if (raw2 != lastStable) {
      if (lastStable == LOW && raw2 == HIGH) { 
        if (!logging) {
          sdOk = SD.begin(BUILTIN_SDCARD);
          if (sdOk) {
            buildNewRunFilename();
            logFile = SD.open(filename, FILE_WRITE);
            if (logFile) {
              writeHeader();
              logging = true;
              digitalWrite(ledPin, HIGH);
              Serial.print(F("LOG ON -> ")); Serial.println(filename);
            }
          } else {
             for(int i=0; i<5; i++) { 
               digitalWrite(ledPin, HIGH); delay(50); 
               digitalWrite(ledPin, LOW); delay(50); 
             }
          }
        } else {
          logging = false;
          digitalWrite(ledPin, LOW);
          if (logFile) logFile.close(); 
          Serial.println(F("LOG OFF"));
        }
      }
      lastStable = raw2;
    }
    debouncing = false;
  }

 // --- Sampling Loop --- [REWORKED]
  static unsigned long lastSample = 0;
  if (logging && (nowMs - lastSample >= SAMPLE_MS)) {
    lastSample = nowMs;

    // 1. Safely grab the latest periods and timestamps from interrupts
    noInterrupts();
    unsigned long pP = periodP;
    unsigned long tP = lastTimeP;
    unsigned long pS = periodS;
    unsigned long tS = lastTimeS;
    interrupts();

    // 2. Read Analog & IMU Sensors
    sensors_event_t a, g, temp;
    if (mpuOk) {
      mpu.getEvent(&a, &g, &temp);
    }
    float psiF = readPressurePSI(pinBrakeF);
    float psiR = readPressurePSI(pinBrakeR);
    float cvtTemp = readTempC(pinCVT);

    // 3. Calculate high-resolution RPM
    float rpmP = 0.0;
    float rpmS = 0.0;
    unsigned long nowMicro = micros();
    
    // Convert microseconds to minutes: 60,000,000us in a minute
    // Add a timeout: If half a second (500,000us) passes without a pulse, assume engine stopped
    if (pP > 0 && (nowMicro - tP < 500000)) {
        rpmP = MINUTE_MICROS_PPR / (float)pP;
    }
    if (pS > 0 && (nowMicro - tS < 500000)) {
        rpmS = MINUTE_MICROS_PPR / (float)pS; // Update PPR logic if secondary wheel has multiple magnets
    }

    // 4. Write to SD Card
    // Static buffer allocation for reducing memory frags and overheard [NEW]
    if (logFile) {
      static char dataLine[256]; // Increased buffer size [NEW]
      static char timeStr[32];
      static char imuStr[64];
      static char extraStr[64]; // Buffer for Brakes/Temp [NEW]

      if (rtcOk) {
        DateTime dt = rtc.now();
        snprintf(timeStr, sizeof(timeStr), "%04d/%02d/%02d,%02d:%02d:%02d", 
                 dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
      } else {
        snprintf(timeStr, sizeof(timeStr), "%.3f", (nowMs - t0_ms) / 1000.0);
      }

      if (mpuOk) {
        snprintf(imuStr, sizeof(imuStr), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
                 a.acceleration.x, a.acceleration.y, a.acceleration.z, 
                 g.gyro.x, g.gyro.y, g.gyro.z);
      } else {
        snprintf(imuStr, sizeof(imuStr), "0,0,0,0,0,0");
      }

      // Format Analog Sensors [NEW]
      snprintf(extraStr, sizeof(extraStr), "%.0f,%.0f,%.1f", psiF, psiR, cvtTemp); // [NEW]

      // Compile the entire line including GPS Speed [NEW]
      if (gps.location.isValid()) {
         snprintf(dataLine, sizeof(dataLine), "%s,%.0f,%.0f,%s,%.6f,%.6f,%.2f,%s", 
                  timeStr, rpmP, rpmS, imuStr, gps.location.lat(), gps.location.lng(), gps.speed.mph(), extraStr);
      } else {
         snprintf(dataLine, sizeof(dataLine), "%s,%.0f,%.0f,%s,0.000000,0.000000,0.00,%s", 
                  timeStr, rpmP, rpmS, imuStr, extraStr);
      }
      
      logFile.println(dataLine);
      Serial.println(dataLine); 

      static unsigned long lastFlush = 0;
      if (nowMs - lastFlush >= FLUSH_MS) {
        lastFlush = nowMs;
        logFile.flush(); 
      }
      
    } else {
      Serial.println(F("File Write Fail"));
      logging = false; 
      digitalWrite(ledPin, LOW);
    }
  }