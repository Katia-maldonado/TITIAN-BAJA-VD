/*
Version: v13.5
Author: Leon Nguyen
System: DAQ

Description:
- Increased XBee baud rate to 115200 to prevent bottlenecks [REWORKED]
- Added dedicated XBee ring buffer logic for asynchronous transmission [NEW]
- Improved buffer overrun handling to safely drop old data instead of freezing [REWORKED]
- Optimized math operations (multiplication instead of division) for more efficient sensor readings [REWORKED]
- Optimized SD card flush interval to reduce loop stutters and preserve SD health [REWORKED]
- Added PMTK commands to configure GPS for 5Hz update rate for smoothness [NEW]
- Adjusted GPS Status Failsafe for improved visibility [REWORKED]
- Adjusted IMU axis for mounting in Teensy 4.1 enclosure [NEW]
- Utilizing Fast variants of digitalWrite and digitalRead for optimization. [REWORKED]
- Reworked BPS code as inital BPS was the incorrect model used [MAJOR]

Sensors Included:
- Spark Plug RPM Adapter (Primary RPM)
- Hall Effect Sensor (Secondary RPM)
- DS3231 RTC (Real Time Clock)
- MPU-6050 (Accelerometer & Gyroscope)
- Adafruit Ultimate GPS Module (NMEA 9600 baud, 5Hz)
- Brake Pressure Sensor (Analog Front)
- Brake Pressure Sensor (Analog Rear)
- CVT Belt Temp Sensor (Analog AiM)

TEENSY 4.1 HARDWARE WIRING GUIDE

WARNING: Teensy 4.1 is STRICTLY 3.3V.
Do not plug 5V or 12V directly into pins, use Voltage Dividers!

--- COMMUNICATION PINS --- 
- IMU & RTC (SDA)        -> Pin 18 
- IMU & RTC (SCL)        -> Pin 19 
- GPS RX (Serial8 RX8)   -> Pin 34 (Connects to GPS module's TX pin)
- GPS TX (Serial8 TX8)   -> Pin 35 (Connects to GPS module's RX pin)
- XBee RX (Serial1 RX1)  -> Pin 0  (Connects to XBee's TX(DOUT) pin) 
- XBee TX (Serial1 TX1)  -> Pin 1  (Connects to XBee's RX(DIN) pin) 

--- ANALOG PINS --- 
- Front Brake Pressure   -> Pin 24 (Requires Shunt Resistor)
- Rear Brake Pressure    -> Pin 26 (Requires Shunt Resistor)
- CVT Belt Temp (AiM IR) -> Pin 38 (Requires 5V to 3.3V Divider)

--- DIGITAL PINS --- 
- Primary RPM (Spark)    -> Pin 10 (Requires 12V to 3.3V Divider)
- Secondary RPM (Hall)   -> Pin 12 (Requires 5V to 3.3V Divider)
- LED Indicator          -> Pin 29 (Requires MOSFET + 1x 10k resistor)
- Push Button            -> Pin 30 (Wire other to GND, uses internal pullup)

--- IMU SPECIFIC WIRING ---
- MPU6050 AD0 Pin        -> Wire to 3.3V power (Sets address to 0x69 for code)
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

// -------- Pins -------- 
const int pinRPM_Primary   = 10;   // AiM Adapter [12V DIVIDER]
const int pinRPM_Secondary = 12;   // Hall Sensor [5V DIVIDER]
const int ledPin           = 29;
const int buttonPin        = 30;

// Analog Pins 
const int pinBrakeF        = 24;   // Front Brake (DataQ)   [SHUNT RESISTOR]
const int pinBrakeR        = 26;   // Rear Brake (DataQ)    [SHUNT RESISTOR]
const int pinCVT           = 38;   // CVT Temp (AiM Analog) [5V DIVIDER]

// -------- Settings --------
const unsigned long SAMPLE_MS     = 50;   // 20Hz Sampling
const uint8_t       PPR           = 1;    // Pulses Per Rev
const unsigned long DEBOUNCE_MS   = 50;   // Button Debounce
const unsigned long RPM_DEBOUNCE  = 2000; // 2ms Lockout for Spark Noise

// --- Ring Buffer Settings --- 
const int BUFFER_SIZE = 100;       // Hold up to 100 lines of telemetry
char logBuffer[BUFFER_SIZE][256];
volatile int head = 0;             // Where new data is written to
volatile int tail = 0;             // Where data is read and saved to SD
volatile int xbeeTail = 0;         // Where data is read and sent to XBee [NEW]

// Voltage Divider Math (2.2k resistors)
// Ratio = 2200 / (2200 + 2200) = 0.5. To reverse it, we multiply by 2.0.
// WARNING!!! For 12V Primary Pin, use 10k TOP and 2.2k BOTTOM resistors
const float DIVIDER_MULTIPLIER = 2.0; // Math optimization (1 / 0.5 = 2.0) [REWORKED]
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

// --- GPS Parsing --- 
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

// --- Helper Functions --
// BPS model outputs 4-20mA [MAJOR]
float readPressurePSI(int pin) {
  int val = analogRead(pin);
  // Convert 10-bit ADC value to voltage based on 3.3V logic
  float pinVoltage = (val / 1023.0) * 3.3; 
  
  // If voltage is below 0.55V (just below the 4mA/0.6V baseline), 
  // the sensor is likely reading 0 PSI or is disconnected.
  if (pinVoltage < 0.55) {
    return 0.0; 
  }

  // Map the 0.6V - 3.0V range directly to 0 - 2000 PSI
  // (3.0V max - 0.6V min = 2.4V span)
  float psi = ((pinVoltage - 0.6) / 2.4) * 2000.0; 
  
  // Final safety check to prevent any negative readings due to minor voltage fluctuations
  if (psi < 0.0) return 0.0; 
  return psi;
}

float readTempC(int pin) {
  int raw = analogRead(pin);
  float pinV = raw * (3.3 / 1023.0);
  float sensorV = pinV * DIVIDER_MULTIPLIER; // Reverse voltage divider (Optimized to multiply) [REWORKED]
  
  // AiM Spec: 820mV = -20C, 2880mV = +250C
  if (sensorV < 0.8) return -20.0;
  float tempC = (sensorV - 0.82) * 131.07 - 20.0;
  return tempC;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  
  // I2C Timeout Failsafe 
  // No response in 10,000 microseconds (10ms) will timeout bus instead of freezing the Teensy
  Wire.setTimeout(10000);
  
  Serial1.begin(115200); // Swapped for XBee (RX1 = 0, TX1 = 1) [REWORKED]
  Serial8.begin(9600);   // GPS module (RX8 = 34, TX8 = 35) 

  // Configure Adafruit Ultimate GPS to 5Hz and limit sentences to RMC & GGA [NEW]
  Serial8.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
  delay(100);
  Serial8.println(F("$PMTK220,200*2C"));

  pinMode(ledPin, OUTPUT);
  digitalWriteFast(ledPin, LOW);
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

  // Initialize Watchdog Timer 
  WDT_timings_t config;
  config.trigger = 2;   // 2 second timeout
  config.timeout = 3;   // 3 second hard reboot
  wdt.begin(config);
  Serial.println(F("Watchdog Armed."));
}

void loop() {
  wdt.feed(); // Reset the watchdog countdown 
  unsigned long nowMs = millis();
  updateGPS();
  
  // --- Button Logic ---
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
          sdOk = SD.begin(BUILTIN_SDCARD);
          if (sdOk) {
            buildNewRunFilename();
            logFile = SD.open(filename, FILE_WRITE);
            if (logFile) {
              writeHeader();
              logging = true;
              digitalWriteFast(ledPin, HIGH);
              Serial.print(F("LOG ON -> ")); Serial.println(filename);
            }
          } else {
             for(int i=0; i<5; i++) { 
               digitalWriteFast(ledPin, HIGH);
               delay(50); 
               digitalWriteFast(ledPin, LOW); delay(50); 
             }
          }
        } else {
          logging = false;
          digitalWriteFast(ledPin, LOW);
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
    if (mpuOk) {
      if (!mpu.getEvent(&a, &g, &temp)) {
          // Bus timeout protected by Wire.setTimeout
      }
    }

    // --- AXIS REMAPPING --- [NEW]
    // Mapping Sensor to Car: Car X=Forward, Car Y=Lateral, Car Z=Vertical
    
    float carAccX = -a.acceleration.z; // Negative Z because +Z is rearward
    float carAccY =  a.acceleration.y; // Lateral
    float carAccZ =  a.acceleration.x; // Vertical (Gravity ~9.8 here)

    float carGyroX = -g.gyro.z;
    float carGyroY =  g.gyro.y;
    float carGyroZ =  g.gyro.x;

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
        rpmS = MINUTE_MICROS_PPR / (float)pS;
    }

    // 4. Format the Data
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
        snprintf(timeStr, sizeof(timeStr), "NoDate,%.3f", (nowMs - t0_ms) / 1000.0);
      }

      if (mpuOk) {
        // Use the CAR variables here for proper CSV display [NEW]
        snprintf(imuStr, sizeof(imuStr), "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", 
                 carAccX, carAccY, carAccZ, carGyroX, carGyroY, carGyroZ);
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
      
      // Copy the formatted line into RAM buffer 
      int nextHead = (head + 1) % BUFFER_SIZE;
      if (nextHead != tail && nextHead != xbeeTail) { 
          strncpy(logBuffer[head], dataLine, 255);
          head = nextHead;
      } else {
          Serial.println(F("Buffer OVERRUN! SD card is too SLOW. Dropping oldest data."));
          // Advance tails to forcefully drop the oldest frame and make room [REWORKED]
          if (nextHead == tail) tail = (tail + 1) % BUFFER_SIZE;
          if (nextHead == xbeeTail) xbeeTail = (xbeeTail + 1) % BUFFER_SIZE;

          strncpy(logBuffer[head], dataLine, 255);
          head = nextHead;
      }

      Serial.println(dataLine);
      // Serial1.println() was removed from here to prevent lagging the 50ms loop [REWORKED]

    } else {
      Serial.println(F("File Write FAIL!"));
      logging = false; 
      digitalWriteFast(ledPin, LOW);
    }
  }
  
  // --- GPS Status Failsafe --- [REWORKED]
  if (!logging) {
      // 1000ms cycle = Locked. 200ms cycle = Searching.
      unsigned long blinkInterval = gps.location.isValid() ? 1000 : 200; 
      
      // Modulo math creates a repeating cycle from 0 to the blinkInterval
      unsigned long currentPhase = millis() % blinkInterval;

      // Turn the LED ON for a sharp 50ms flash at the start of every cycle, then turn it completely OFF
      if (currentPhase < 50) {
          digitalWriteFast(ledPin, HIGH);
      } else {
          digitalWriteFast(ledPin, LOW);
      }
  }
  
 // --- Asynchronous SD Write --- 
  static unsigned long lastForceFlush = 0;
  if (logging && logFile) {
      // 1. Write one line to the SD card if the buffer has data
      if (head != tail) {
          logFile.println(logBuffer[tail]);
          tail = (tail + 1) % BUFFER_SIZE;
      }
          
      // 2. Force FAT table update ONLY every 2000ms to protect SD health [REWORKED]
      if (millis() - lastForceFlush > 2000) { 
          logFile.flush();
          lastForceFlush = millis();
      }
  }

  // --- Asynchronous XBee Transmission --- [NEW]
  // Sends data to the XBee whenever the Teensy has "free time", keeping the main loop fast
  if (logging && (head != xbeeTail)) {
      Serial1.println(logBuffer[xbeeTail]);
      xbeeTail = (xbeeTail + 1) % BUFFER_SIZE;
  }
}
