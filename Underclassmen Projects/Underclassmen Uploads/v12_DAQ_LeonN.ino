/*
Version: v12
Author: Leon Nguyen
System: DAQ

Description:
- Changed pin assignments in order to optimize physical wiring layout
- Configured for XBee Pro 900HP (Live Transmission, 57600 baud)
- Implemented GPS Status Failsafe via LED Indicator (Pin 29)
- Implemented I2C Timeout Failsafe (Teensy Freeze Protection)
- Implemented System Freeze Failsafe (Watchdog Timer)
- Implemented MPU6050 Failsafe (Main Loop Halt Protection)
- Reworked SD Card Logic (Data Loss Protection and Optimization)

Sensors Included:
- Spark Plug RPM Adapter (Primary RPM)
- Hall Effect Sensor (Secondary RPM)
- DS3231 RTC (Real Time Clock)
- MPU-6050 (Accelerometer & Gyroscope)
- GPS Module (NMEA 9600 baud)
- Brake Pressure Sensor (Analog Front)
- Brake Pressure Sensor (Analog Rear)
- CVT Belt Temp Sensor (Analog AiM)

TEENSY 4.1 HARDWARE WIRING GUIDE

WARNING: Teensy 4.1 is STRICTLY 3.3V. Do not plug 5V or 12V directly into pins, use Voltage Dividers!
         Ensure Voltage Dividers are in place for all 5V/12V sensors!!!

--- COMMUNICATION PINS --- [REWORKED]
- I2C SDA (IMU & RTC)     -> Pin 18 (A4)
- I2C SCL (IMU & RTC)     -> Pin 19 (A5)
- GPS RX (Serial8 RX8)    -> Pin 34 (Connects to GPS module's TX pin)
- GPS TX (Serial8 TX8)    -> Pin 35 (Connects to GPS module's RX pin)
- XBee RX (Serial1 RX1)   -> Pin 0 (Connects to XBee's TX(DOUT) pin) [NEW]
- XBee TX (Serial1 TX1)   -> Pin 1 (Connects to XBee's RX(DIN) pin) [NEW]

--- ANALOG PINS (Require 5V to 3.3V Divider per sensor) --- [REWORKED]
- Front Brake Pressure    -> Pin 24 (A0)
- Rear Brake Pressure     -> Pin 26 (A1)
- CVT Belt Temp (AiM IR)  -> Pin 38 (A2)

--- DIGITAL PINS --- [REWORKED]
- Primary RPM (Spark)     -> Pin 10 (Requires 12V to 3.3V Divider)
- Secondary RPM (Hall)    -> Pin 12 (Requires 5V to 3.3V Divider)
- LED Indicator           -> Pin 29 (Operates on 5-12v, requires MOSFET + 1x 10k resistor to prevent float states)
- Push Button             -> Pin 30 (Wire other side to Ground, uses internal pullup)

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
#include <Watchdog_t4.h>

RTC_DS3231 rtc;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
WDT_T4<WDT1> wdt;

// -------- Pins -------- [REWORKED]
const int pinRPM_Primary   = 10;   // AiM Adapter [12V DIVIDER]
const int pinRPM_Secondary = 12;   // Hall Sensor [5V DIVIDER]
const int ledPin           = 29;
const int buttonPin        = 30;

// Analog Pins [REWORKED]
const int pinBrakeF        = 24;  // A0 - Front Brake (DataQ) [5V DIVIDER]
const int pinBrakeR        = 26;  // A1 - Rear Brake (DataQ) [5V DIVIDER]
const int pinCVT           = 38;  // A2 - CVT Temp (AiM Analog) [5V DIVIDER]

// -------- Settings --------
const unsigned long SAMPLE_MS     = 50;   // 20Hz Sampling
const uint8_t       PPR           = 1;    // Pulses Per Rev
const unsigned long DEBOUNCE_MS   = 50;   // Button Debounce
const unsigned long RPM_DEBOUNCE  = 2000; // 2ms Lockout for Spark Noise

// --- Ring Buffer Settings --- [NEW]
const int BUFFER_SIZE = 100; // Hold up to 100 lines of telemetry
char logBuffer[BUFFER_SIZE][256];
volatile int head = 0; // Where new data is written to
volatile int tail = 0; // Where data is read and saved to SD

// Voltage Divider Math (2.2k resistors)
// Ratio = 2200 / (2200 + 2200) = 0.5
// WARNING!!! For 12V Primary Pin, use 10k TOP and 2.2k BOTTOM resistors
const float DIVIDER_RATIO = 0.5;  

//RPM Math
const float MINUTE_MICROS_PPR = 60000000.0 / PPR;

// -------- Counters / State --------
// Pulse counters for period tracking
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
// Primary RPM (AiM Adapter) - Rising Edge with Debounce
void isrP() { 
  unsigned long now = micros();
  // Using 2000us (2ms) lockout for Spark Noise
  if (now - lastTimeP > RPM_DEBOUNCE) { 
    periodP = now - lastTimeP; 
    lastTimeP = now;
  }
}
// Secondary RPM (Hall) - Falling Edge
void isrS() { 
  unsigned long now = micros();
  // Using a 1000us (1ms) debounce for the Hall sensor to prevent noise double-triggers
  if (now - lastTimeS > 1000) { 
    periodS = now - lastTimeS;
    lastTimeS = now;
  }
}

// --- GPS Parsing --- [REWORKED]
void updateGPS() {
  while (Serial8.available() > 0) { 
    gps.encode(Serial8.read());
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

// --- Helper Functions ---
float readPressurePSI(int pin) {
  int raw = analogRead(pin);
  float pinV = raw * (3.3 / 1023.0); // Convert bits to 3.3V scale
  float sensorV = pinV / DIVIDER_RATIO; // Reverse voltage divider
  
  if (sensorV < 0.5) return 0.0; // Floor 0.5V offset
  return (sensorV - 0.5) * (500.0); // DataQ Scaling
}

float readTempC(int pin) {
  int raw = analogRead(pin);
  float pinV = raw * (3.3 / 1023.0); 
  float sensorV = pinV / DIVIDER_RATIO; // Reverse voltage divider
  
  // AiM Spec: 820mV = -20C, 2880mV = +250C
  if (sensorV < 0.8) return -20.0;
  float tempC = (sensorV - 0.82) * 131.07 - 20.0;
  return tempC;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  // I2C Timeout Failsafe [NEW]
  // No response in 10,000 microseconds (10ms) will timeout bus instead of freezing the Teensy
  Wire.setTimeout(10000); 
  
  Serial1.begin(57600); // Swapped for XBee (RX1 = 0, TX1 = 1) [REWORKED]
  Serial8.begin(9600); // GPS module (RX8 = 34, TX8 = 35) [NEW]

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  
  // RPM Inputs changed to INPUT (No Pullup) due to external Dividers
  pinMode(pinRPM_Primary, INPUT);
  pinMode(pinRPM_Secondary, INPUT);

  // Analog Inputs
  pinMode(pinBrakeF, INPUT);
  pinMode(pinBrakeR, INPUT);
  pinMode(pinCVT, INPUT);
  analogReadResolution(10); 
  analogReadAveraging(16); // Hardware averages 16 samples for clean analog data

  attachInterrupt(digitalPinToInterrupt(pinRPM_Primary), isrP, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRPM_Secondary), isrS, FALLING);

  Serial.println(F("Initializing Teensy 4.1 DAQ..."));
  
  sdOk = SD.begin(BUILTIN_SDCARD);
  Serial.print(F("SD: "));
  Serial.println(sdOk ? F("OK") : F("FAIL"));

  rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.print(F("RTC: ")); Serial.println(rtcOk ? F("OK") : F("FAIL"));

  if (!mpu.begin(0x69)) { 
    Serial.println(F("MPU6050 FAIL! Check the AD0 pin."));
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

// Initialize Watchdog Timer [NEW] 
  WDT_timings_t config;
  config.trigger = 2;   // 2 second timeout
  config.timeout = 3;   // 3 second hard reboot
  wdt.begin(config);
  Serial.println(F("Watchdog Armed."));
}

void loop() {
  wdt.feed(); // Reset the watchdog countdown [NEW] 
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

 // --- Sampling Loop ---
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
    // Safety Upgrade: Only attempt read if the sensor was found during setup [NEW]
    if (mpuOk) {
      // getEvent returns TRUE if successful, FALSE if the bus timed out
      if (!mpu.getEvent(&a, &g, &temp)) {
      }
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
    // Static buffer allocation for reducing memory fragmentation and overhead
    if (logFile) {
      static char dataLine[256];
      static char timeStr[32];
      static char imuStr[64];
      static char extraStr[64]; // Buffer for Brakes/Temp

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

      // Format Analog Sensors
      snprintf(extraStr, sizeof(extraStr), "%.0f,%.0f,%.1f", psiF, psiR, cvtTemp);

      // Compile the entire line including GPS Speed
      if (gps.location.isValid()) {
         snprintf(dataLine, sizeof(dataLine), "%s,%.0f,%.0f,%s,%.6f,%.6f,%.2f,%s", 
                  timeStr, rpmP, rpmS, imuStr, gps.location.lat(), gps.location.lng(), gps.speed.mph(), extraStr);
      } else {
         snprintf(dataLine, sizeof(dataLine), "%s,%.0f,%.0f,%s,0.000000,0.000000,0.00,%s", 
                  timeStr, rpmP, rpmS, imuStr, extraStr);
      }
      
      // Instead of logFile.println(dataLine), copy the formatted line into RAM buffer [NEW]
      int nextHead = (head + 1) % BUFFER_SIZE;
      if (nextHead != tail) { 
          strncpy(logBuffer[head], dataLine, 255);
          head = nextHead;
      } else {
          Serial.println(F("Buffer OVERRUN! SD card is too SLOW."));
      }

      Serial.println(dataLine);
      Serial1.println(dataLine); // Send to XBee [NEW]

    } else {
      Serial.println(F("File Write FAIL!"));
      logging = false; 
      digitalWrite(ledPin, LOW);
    }
  }
  // --- GPS Status Failsafe --- [NEW]
  if (!logging) {
      static unsigned long lastBlink = 0;
      // Fast blink (200ms) = GPS searching for satellite
      // Slow blink (1000ms) = GPS connected to satellite
      unsigned long blinkRate = gps.location.isValid() ? 1000 : 200; 

      if (millis() - lastBlink >= blinkRate) {
          lastBlink = millis();
          digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED on Pin 29
      }
  }
  // --- Asynchronous SD Write --- [NEW]
  static unsigned long lastForceFlush = 0;

  if (logging && logFile && (head != tail)) {
      logFile.println(logBuffer[tail]);
      tail = (tail + 1) % BUFFER_SIZE;
          
      // Logic: Flush if buffer is empty OR if 500ms has passed
      if ((head == tail) || (millis() - lastForceFlush > 500)) {
          logFile.flush();
          lastForceFlush = millis();
      }
  }
} // Oopsies xd
