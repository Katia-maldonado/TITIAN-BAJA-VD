/*
Version: v14
Author: Leon Nguyen
System: DAQ

Description:
- Remapped hardware pins, moving GPS to Serial1 and XBee to Serial3 [REWORKED]
- Transitioned main loop timing to Teensy's internal hardware RTC (VBAT backed) for zero-latency timestamping [MAJOR]
- Repurposed DS3231 for precise boot sync, 1Hz SQW hardware heartbeat, and ambient enclosure temperature monitoring [NEW]
- Implemented interrupt-driven IMU polling via INT pin to prevent reading stale data [NEW]
- Replaced standard IMU library calls with a custom 14-byte I2C Burst Read to maximize throughput [NEW]
- Increased GPS update rate to 10Hz and added GPS PPS hardware interrupt for sub-millisecond video synchronization and clock drift measurement (PPS_Delta) [NEW]
- Allocated massive dedicated memory block for XBee Serial3 TX to completely eliminate telemetry bottlenecks [REWORKED]
- Moved all Interrupt Service Routines (ISRs) to ITCM RAM using FASTRUN macro for deterministic, zero-latency execution [NEW]
- Optimized GPS serial payload ("NMEA Diet") by restricting output to strictly RMC sentences [REWORKED]
- Added strategic Watchdog Timer feeding prior to SD card mounting to prevent false hard-reboots [NEW]
- Upgraded hardware ADC resolution to 12-bit (4096 steps) for 4x increased sensitivity on analog sensors [NEW]

Sensors Included:
- Spark Plug RPM Adapter (Primary RPM)
- Hall Effect Sensor (Secondary RPM)
- DS3231 RTC (Time, 1Hz SQW Failsafe, Enclosure Temp)
- MPU-6050 (Accelerometer & Gyroscope - Interrupt Driven)
- Adafruit Ultimate GPS Module (NMEA 115200 baud, 10Hz, PPS)
- Brake Pressure Sensor (Analog Front)
- Brake Pressure Sensor (Analog Rear)
- CVT Belt Temp Sensor (Analog AiM)
- XBee-PRO 900HP (S3B) (Live-Telemetry Module)

TEENSY 4.1 HARDWARE WIRING GUIDE:
--- COMMUNICATION PINS ---
- IMU & RTC (SDA)        -> Pin 18
- IMU & RTC (SCL)        -> Pin 19
- GPS RX (Serial1 RX1)   -> Pin 0
- GPS TX (Serial1 TX1)   -> Pin 1
- XBee RX (Serial3 RX3)  -> Pin 15
- XBee TX (Serial3 TX3)  -> Pin 14

--- ANALOG PINS ---
- Front Brake Pressure   -> Pin 39 (Shunt Resistor)
- Rear Brake Pressure    -> Pin 40 (Shunt Resistor)
- CVT Belt Temp (AiM IR) -> Pin 27 (5V to 3.3V Divider)

--- DIGITAL PINS ---
- Primary RPM (Spark)    -> Pin 32 (12V to 3.3V Divider)
- Secondary RPM (Hall)   -> Pin 33 (INTERNAL_PULLUP)
- LED Indicator          -> Pin 31 (MOSFET)
- Push Button            -> Pin 23 (INTERNAL_PULLUP)

--- AUXILIARY / SYNC PINS ---
- GPS PPS Pin            -> Pin 2  (Hardware Sync)
- IMU INT Pin            -> Pin 20 (Hardware Sync)
- RTC SQW Pin            -> Pin 17 (Hardware Failsafe Sync)
- MPU6050 AD0 Pin        -> Wire to 3.3V (Address 0x69)
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <TimeLib.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <Watchdog_t4.h>

RTC_DS3231 rtc;
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
WDT_T4<WDT1> wdt;

// -------- Pins -------- 
const int pinRPM_Primary   = 32;
const int pinRPM_Secondary = 33; 
const int ledPin           = 31;
const int buttonPin        = 23;
const int pinBrakeF        = 39;
const int pinBrakeR        = 40;
const int pinCVT           = 27;
const int pinGPS_PPS       = 2;
const int pinIMU_INT       = 20;
const int pinRTC_SQW       = 17;

// -------- Settings --------
const unsigned long SAMPLE_MS     = 50;
const uint8_t       PPR           = 1;
const unsigned long DEBOUNCE_MS   = 50;   
const unsigned long RPM_DEBOUNCE  = 2000;

// --- Ring Buffer Settings ---
const int BUFFER_SIZE = 100;
char logBuffer[BUFFER_SIZE][256];
volatile int head = 0;
volatile int tail = 0;
volatile int xbeeTail = 0;

const float DIVIDER_MULTIPLIER = 2.0;
const float MINUTE_MICROS_PPR = 60000000.0 / PPR;

// -------- Counters / State --------
volatile unsigned long lastTimeP = 0;
volatile unsigned long periodP = 0;  

volatile unsigned long lastTimeS = 0;
volatile unsigned long periodS = 0;

volatile unsigned long lastPPSMicros = 0; 
volatile unsigned long ppsDelta = 0;

// Tracks microsecond drift
volatile unsigned long lastSQWMicros = 0; 

bool logging      = false;
bool lastStable   = HIGH;
bool debouncing   = false;
unsigned long tDebounce = 0;

bool sdOk = false, rtcOk = false, mpuOk = false;
unsigned long t0_ms = 0; 
char filename[32]; 

File logFile;

// -------- Sensor Data Globals --------
float carAccX = 0, carAccY = 0, carAccZ = 0;
float carGyroX = 0, carGyroY = 0, carGyroZ = 0;

// --- FASTRUN ISRs (Loaded into ITCM RAM for zero latency) ---
void FASTRUN isrP() { 
  unsigned long now = micros();
  if (now - lastTimeP > RPM_DEBOUNCE) {
    periodP = now - lastTimeP;
    lastTimeP = now;
  }
}

void FASTRUN isrS() { 
  unsigned long now = micros();
  if (now - lastTimeS > 1000) {
    periodS = now - lastTimeS;
    lastTimeS = now;
  }
}

void FASTRUN isrPPS() { 
  unsigned long now = micros();
  ppsDelta = now - lastPPSMicros;
  // Calculate exact drift
  lastPPSMicros = now; 
}

void FASTRUN isrSQW() { 
  lastSQWMicros = micros();
}

// -----------------------------------------------------------

void updateGPS() {
  while (Serial1.available() > 0) gps.encode(Serial1.read());
}

void buildNewRunFilename() {
  int run = 1;
  do {
    sprintf(filename, "RUN%03d.CSV", run);
    run++;
  } while (SD.exists(filename));
}

void writeHeader() {
  // Added PPS_Delta to track clock drift
  logFile.println(F("Date,Time,PPS_uS,PPS_Delta,SQW_uS,P_RPM,S_RPM,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Lat,Lon,Speed_MPH,BrakeF_PSI,BrakeR_PSI,CVT_TempC,Enc_TempC"));
  logFile.flush();
  Serial.println(F("Header written"));
}

float readPressurePSI(int pin) {
  int val = analogRead(pin);
  // OPTIMIZED: 12-bit ADC resolution
  float pinVoltage = (val / 4095.0) * 3.3; 
  if (pinVoltage < 0.55) return 0.0;
  float psi = ((pinVoltage - 0.6) / 2.4) * 2000.0;
  return (psi < 0.0) ? 0.0 : psi;
}

float readTempC(int pin) {
  int raw = analogRead(pin);
  // 12-bit ADC resolution
  float pinV = raw * (3.3 / 4095.0); 
  float sensorV = pinV * DIVIDER_MULTIPLIER;
  if (sensorV < 0.8) return -20.0;
  return (sensorV - 0.82) * 131.07 - 20.0;
}

// --- Custom I2C Burst Read for MPU-6050 ---
void readIMUBurst() {
  Wire.beginTransmission(0x69); // 0x69 is your specific MPU address
  Wire.write(0x3B); 
  Wire.endTransmission(false);  // 'false' keeps the I2C connection open/active

  Wire.requestFrom((uint8_t)0x69, (uint8_t)14); 

  if (Wire.available() == 14) {
    int16_t rawAccX = Wire.read() << 8 | Wire.read();
    int16_t rawAccY = Wire.read() << 8 | Wire.read();
    int16_t rawAccZ = Wire.read() << 8 | Wire.read();
    int16_t rawTemp = Wire.read() << 8 | Wire.read();
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

// Convert the raw hardware integers into your float variables
    // --- ORIENTATION MAPPING: Pins UP, Top Face REAR ---
    // (X points DOWN, Y points RIGHT, Z points REAR)
    
    // ACCELEROMETER
    carAccX = -(rawAccZ / 4096.0); // Car Forward/Braking (Negative Z)
    carAccY =  (rawAccY / 4096.0); // Car Cornering Right (Positive Y)
    carAccZ = -(rawAccX / 4096.0); // Car Jumps/Vertical Up (Negative X)
    
    // GYROSCOPE (Right-Hand Rule matched to axes)
    carGyroX = -(rawGyroZ / 65.5); // Roll (Body lean left/right)
    carGyroY =  (rawGyroY / 65.5); // Pitch (Nose dive/squat)
    carGyroZ = -(rawGyroX / 65.5); // Yaw (Car spinning/sliding)
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(10000);

  static char gpsBuffer[256];
  Serial1.addMemoryForRead(gpsBuffer, sizeof(gpsBuffer));

  static char xbeeBuffer[1024];
  Serial3.addMemoryForWrite(xbeeBuffer, sizeof(xbeeBuffer)); 

  Serial3.begin(115200); 
  
  Serial1.begin(9600);
  Serial1.println(F("$PMTK251,115200*1F"));
  delay(100);
  Serial1.begin(115200);
  // "NMEA Diet": Configured to output strictly RMC sentences for reduced CPU load
  Serial1.println(F("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"));
  delay(100);
  Serial1.println(F("$PMTK220,100*2F"));

  pinMode(ledPin, OUTPUT);
  digitalWriteFast(ledPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pinRPM_Primary, INPUT);
  pinMode(pinRPM_Secondary, INPUT_PULLUP);
  pinMode(pinBrakeF, INPUT);
  pinMode(pinBrakeR, INPUT);
  pinMode(pinCVT, INPUT);
  
  pinMode(pinIMU_INT, INPUT); 
  pinMode(pinRTC_SQW, INPUT_PULLUP);
  pinMode(pinGPS_PPS, INPUT);
  
  // 12-bit ADC for higher analog sensitivity
  analogReadResolution(12);
  analogReadAveraging(16);

  attachInterrupt(digitalPinToInterrupt(pinRPM_Primary), isrP, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRPM_Secondary), isrS, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinGPS_PPS), isrPPS, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRTC_SQW), isrSQW, FALLING);

  wdt.feed(); 
  sdOk = SD.begin(BUILTIN_SDCARD);

  // --- RTC Boot Sync & 1Hz Setup ---
  rtcOk = rtc.begin();
  if (rtcOk) {
    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Teensy3Clock.set(rtc.now().unixtime());
    setTime(Teensy3Clock.get());
    rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
  }

  // --- IMU Configuration ---
  if (!mpu.begin(0x69)) {
    mpuOk = false;
  } else {
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Wire.beginTransmission(0x69);
    Wire.write(0x38); 
    Wire.write(0x01); 
    Wire.endTransmission();
  }

  t0_ms = millis();
  WDT_timings_t config;
  config.trigger = 2;
  config.timeout = 3;
  wdt.begin(config);
}

void loop() {
  wdt.feed();
  unsigned long nowMs = millis();
  updateGPS();

  bool raw = digitalReadFast(buttonPin);
  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }
  
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalReadFast(buttonPin);
    if (raw2 != lastStable) {
      if (lastStable == LOW && raw2 == HIGH) {
        if (!logging) {
          wdt.feed();
          if (SD.begin(BUILTIN_SDCARD)) {
            buildNewRunFilename();
            logFile = SD.open(filename, FILE_WRITE);
            if (logFile) { writeHeader(); logging = true; digitalWriteFast(ledPin, HIGH); }
          }
        } else {
          logging = false;
          digitalWriteFast(ledPin, LOW);
          if (logFile) logFile.close();
        }
      }
      lastStable = raw2;
    }
    debouncing = false;
  }

  // --- Enclosure Temperature Polling (Every 5 seconds) ---
  static unsigned long lastTempRead = 0;
  static float enclosureTemp = 0.0;
  if (nowMs - lastTempRead >= 5000) {
      if (rtcOk) enclosureTemp = rtc.getTemperature();
      lastTempRead = nowMs;
  }

  // --- Main 20Hz Sampling Loop ---
  static unsigned long lastSample = 0;
  if (logging && (nowMs - lastSample >= SAMPLE_MS)) {
    lastSample = nowMs;

    noInterrupts();
    unsigned long pP = periodP; unsigned long tP = lastTimeP;
    unsigned long pS = periodS; unsigned long tS = lastTimeS;
    unsigned long ppsSync = lastPPSMicros; 
    unsigned long syncDelta = ppsDelta; 
    unsigned long sqwSync = lastSQWMicros; 
    interrupts();
    
    if (mpuOk && digitalReadFast(pinIMU_INT) == HIGH) {
        readIMUBurst();
    }

    float rpmP = (pP > 0 && (micros() - tP < 500000)) ? MINUTE_MICROS_PPR / (float)pP : 0.0;
    float rpmS = (pS > 0 && (micros() - tS < 500000)) ? MINUTE_MICROS_PPR / (float)pS : 0.0;

    if (logFile) {
      static char dataLine[256]; static char timeStr[32];
      time_t t = Teensy3Clock.get(); 
      snprintf(timeStr, sizeof(timeStr), "%04d/%02d/%02d,%02d:%02d:%02d", 
               year(t), month(t), day(t), hour(t), minute(t), second(t));
               
      snprintf(dataLine, sizeof(dataLine), "%s,%lu,%lu,%lu,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.0f,%.0f,%.1f,%.2f", 
               timeStr, ppsSync, syncDelta, sqwSync, rpmP, rpmS, carAccX, carAccY, carAccZ, carGyroX, carGyroY, carGyroZ, gps.location.lat(), gps.location.lng(), gps.speed.mph(), readPressurePSI(pinBrakeF), readPressurePSI(pinBrakeR), readTempC(pinCVT), enclosureTemp);

      int nextHead = (head + 1) % BUFFER_SIZE;
      if (nextHead != tail && nextHead != xbeeTail) {
          strncpy(logBuffer[head], dataLine, 255);
          logBuffer[head][255] = '\0'; head = nextHead;
      } else {
          tail = (tail + 1) % BUFFER_SIZE;
          xbeeTail = (xbeeTail + 1) % BUFFER_SIZE;
          strncpy(logBuffer[head], dataLine, 255); logBuffer[head][255] = '\0'; head = nextHead;
      }
    }
  }
  
  if (!logging) {
      unsigned long blinkInterval = gps.location.isValid() ? 1000 : 200;
      if ((millis() % blinkInterval) < 50) digitalWriteFast(ledPin, HIGH); else digitalWriteFast(ledPin, LOW);
  }
  
  static unsigned long lastForceFlush = 0;
  if (logging && logFile) {
      if (head != tail) { logFile.println(logBuffer[tail]);
      tail = (tail + 1) % BUFFER_SIZE; }
      if (millis() - lastForceFlush > 2000) { logFile.flush();
      lastForceFlush = millis(); }
  }

  if (logging && (head != xbeeTail)) {
      Serial3.println(logBuffer[xbeeTail]);
      xbeeTail = (xbeeTail + 1) % BUFFER_SIZE;
  }
}