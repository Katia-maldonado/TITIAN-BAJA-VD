/*
Version: v15
Author: Leon Nguyen
System: DAQ

Description:
- Retained GPS PPS hardware interrupt for silent background tracking, but omitted data from CSV for clean Excel logs [UPDATED]
- Updated IMU software axis mapping to match new physical orientation (X=Up, Y=Left, Z=Rear) [UPDATED]
- Removed DS3231 RTC module dependencies, and shifted entirely to Teensy 4.1 internal hardware RTC (VBAT ON) for zero-latency timestamping [NEW]
- Removed SQW Interrupt hardware sync and Enclosure Temperature tracking [UPDATED]
- Remapped hardware pins across the board, GPS moved to Serial8, XBee moved to Serial5 [UPDATED] 
- IMU address restored to default 0x68 (AD0 pulled low/floating) since it is now the sole device on the bus [UPDATED]
= Removed DIVIDER_MULTIPLIER math for CVT readings [UPDATED]

Sensors Included:
- Spark Plug RPM Adapter (Primary RPM)
- Hall Effect Sensor (Secondary RPM)
- MPU-6050 (Accelerometer & Gyroscope - Interrupt Driven)
- Adafruit Ultimate GPS Module (NMEA 115200 baud, 10Hz, PPS)
- Brake Pressure Sensor (Analog Front)
- Brake Pressure Sensor (Analog Rear)
- CVT Belt Temp Sensor (Analog AiM)
- XBee-PRO 900HP (S3B) (Live-Telemetry Module)

TEENSY 4.1 HARDWARE WIRING GUIDE:
--- COMMUNICATION PINS ---
- IMU(SDA / SCL)         -> Pin 18 / Pin 19
- GPS RX (Serial8 RX8)   -> Pin 34
- GPS TX (Serial8 TX8)   -> Pin 35
- XBee RX (Serial5 RX5)  -> Pin 21
- XBee TX (Serial5 TX5)  -> Pin 20

--- ANALOG PINS ---
- Front Brake Pressure   -> Pin 40 (Shunt Resistor)
- Rear Brake Pressure    -> Pin 41 (Shunt Resistor)
- CVT Belt Temp (AiM IR) -> Pin 38 (5V to 3.3V Divider)

--- DIGITAL PINS ---
- Primary RPM (Spark)    -> Pin 37 (12V to 3.3V Divider)
- Secondary RPM (Hall)   -> Pin 36 (INTERNAL_PULLUP)
- LED Indicator          -> Pin 39 (MOSFET)
- Push Button            -> Pin 23 (INTERNAL_PULLUP)

--- AUXILIARY / SYNC PINS ---
- GPS PPS Pin            -> Pin 33 (Hardware Sync)
- IMU INT Pin            -> Pin 22 (Hardware Sync)
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <Watchdog_t4.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
WDT_T4<WDT1> wdt;

// -------- Pins -------- 
const int pinRPM_Primary   = 37;
const int pinRPM_Secondary = 36;
const int ledPin           = 39;
const int buttonPin        = 23;
const int pinBrakeF        = 40;
const int pinBrakeR        = 41;
const int pinCVT           = 38;
const int pinGPS_PPS       = 33;
const int pinIMU_INT       = 22;

// -------- Settings --------
const unsigned long SAMPLE_MS     = 50;
const uint8_t       PPR           = 2;
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
volatile unsigned long ppsDelta = 0; // Tracks microsecond drift

bool logging      = false;
bool lastStable   = HIGH;
bool debouncing   = false;
unsigned long tDebounce = 0;

bool sdOk = false, mpuOk = false;
unsigned long t0_ms = 0; 
char filename[32]; 

File logFile;

// -------- Sensor Data Globals --------
float carAccX = 0, carAccY = 0, carAccZ = 0;
float carGyroX = 0, carGyroY = 0, carGyroZ = 0;

// --- Time Sync Helper Function ---
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

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
  ppsDelta = now - lastPPSMicros; // Calculate exact drift
  lastPPSMicros = now; 
}

// -----------------------------------------------------------

void updateGPS() {
  while (Serial8.available() > 0) gps.encode(Serial8.read());
}

void buildNewRunFilename() {
  int run = 1;
  do {
    sprintf(filename, "RUN%03d.CSV", run);
    run++;
  } while (SD.exists(filename));
}

void writeHeader() {
  // PPS columns intentionally omitted to keep CSV clean
  logFile.println(F("Date,Time,P_RPM,S_RPM,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Lat,Lon,Speed_MPH,BrakeF_PSI,BrakeR_PSI,CVT_TempC"));
  logFile.flush();
  Serial.println(F("Header written"));
}

float readPressurePSI(int pin) {
  int val = analogRead(pin);
  // OPTIMIZED: 12-bit ADC resolution
  float pinVoltage = (val / 4095.0) * 3.3;
  // Calculate raw PSI based on the theoretical 0.60V baseline
  float psi = ((pinVoltage - 0.6) / 2.4) * 2000.0;
  return psi;
}

float readTempC(int pin) {
  // 1. Read the raw 12-bit hardware value (0 to 4095)
  int raw = analogRead(pin);
  
  // 2. Create 'pinV' and calculates the voltage
  float pinV = raw * (3.3 / 4095.0); 
  
  // 3. Now the compiler knows what pinV is
  float sensorV = pinV; 
  
  // 4. Safety check (sensor baseline is 820mV at -20C)
  if (sensorV < 0.8) return -20.0;
  
  // 5. Calculate temp based on the 131.07 °C/V slope
  return (sensorV - 0.82) * 131.07 - 20.0;
}

// --- Custom I2C Burst Read for MPU-6050 ---
void readIMUBurst() {
  Wire.beginTransmission(0x68); // 0x68 is the default MPU address
  Wire.write(0x3B); 
  Wire.endTransmission(false);  // 'false' keeps the I2C connection open/active

  Wire.requestFrom((uint8_t)0x68, (uint8_t)14);
  if (Wire.available() == 14) {
    int16_t rawAccX = Wire.read() << 8 | Wire.read();
    int16_t rawAccY = Wire.read() << 8 | Wire.read();
    int16_t rawAccZ = Wire.read() << 8 | Wire.read();
    int16_t rawTemp = Wire.read() << 8 | Wire.read();
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
    
    // --- ORIENTATION MAPPING ---
    // (X points UP, Y points LEFT, Z points REAR)
    // ACCELEROMETER
    carAccX = -(rawAccZ / 4096.0); // Car Forward/Braking (Z is Rear -> Forward is -Z)
    carAccY = -(rawAccY / 4096.0); // Car Cornering Right (Y is Left -> Right is -Y)
    carAccZ =  (rawAccX / 4096.0); // Car Jumps/Vertical Up (X is Up -> Up is +X)
    
    // GYROSCOPE (Right-Hand Rule matched to axes)
    carGyroX = -(rawGyroZ / 65.5); // Roll (Rotation around Forward axis)
    carGyroY = -(rawGyroY / 65.5); // Pitch (Rotation around Right axis)
    carGyroZ =  (rawGyroX / 65.5); // Yaw (Rotation around Up axis)
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(10000);

  static char gpsBuffer[256];
  Serial8.addMemoryForRead(gpsBuffer, sizeof(gpsBuffer));
  static char xbeeBuffer[1024];
  Serial5.addMemoryForWrite(xbeeBuffer, sizeof(xbeeBuffer)); 

  Serial5.begin(115200); 
  
  delay(1000);
  Serial8.begin(9600);
  Serial8.println(F("$PMTK251,115200*1F"));
  delay(100);
  Serial8.begin(115200);
  // "NMEA Diet": Configured to output strictly RMC sentences for reduced CPU load
  Serial8.println(F("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"));
  delay(100);
  Serial8.println(F("$PMTK220,100*2F"));

  pinMode(ledPin, OUTPUT);
  digitalWriteFast(ledPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(pinRPM_Primary, INPUT);
  pinMode(pinRPM_Secondary, INPUT_PULLUP);

  pinMode(pinIMU_INT, INPUT); 
  pinMode(pinGPS_PPS, INPUT);
  
  // 12-bit ADC for higher analog sensitivity
  analogReadResolution(12);
  analogReadAveraging(16);

  attachInterrupt(digitalPinToInterrupt(pinRPM_Primary), isrP, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRPM_Secondary), isrS, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinGPS_PPS), isrPPS, RISING);

  wdt.feed();
  sdOk = SD.begin(BUILTIN_SDCARD);

  // --- Teensy Internal RTC Setup ---
  setSyncProvider(getTeensy3Time);
  if (timeStatus() != timeSet) {
    Serial.println(F("Unable to sync with the Teensy RTC"));
  } else {
    Serial.println(F("Teensy RTC has set the system time"));
  }

  // --- IMU Configuration ---
  if (!mpu.begin(0x68)) {
    mpuOk = false;
  } else {
    mpuOk = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Wire.beginTransmission(0x68);
    Wire.write(0x37); 
    Wire.write(0x10); 
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

  // --- Main 20Hz Sampling Loop ---
  static unsigned long lastSample = 0;
  if (logging && (nowMs - lastSample >= SAMPLE_MS)) {
    lastSample = nowMs;

    noInterrupts();
    unsigned long pP = periodP; unsigned long tP = lastTimeP;
    unsigned long pS = periodS; unsigned long tS = lastTimeS;
    
    // Track the PPS values in the background in case we need them later
    unsigned long ppsSync = lastPPSMicros; 
    unsigned long syncDelta = ppsDelta; 
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
               
      // Removed PPS values from the string to keep the CSV clean
      snprintf(dataLine, sizeof(dataLine), "%s,%.0f,%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.0f,%.0f,%.1f", 
               timeStr, rpmP, rpmS, carAccX, carAccY, carAccZ, carGyroX, carGyroY, carGyroZ, gps.location.lat(), gps.location.lng(), gps.speed.mph(), readPressurePSI(pinBrakeF), readPressurePSI(pinBrakeR), readTempC(pinCVT));
               
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
      Serial5.println(logBuffer[xbeeTail]);
      xbeeTail = (xbeeTail + 1) % BUFFER_SIZE;
  }
}