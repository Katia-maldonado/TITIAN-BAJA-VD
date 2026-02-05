// CSUF BAJA SAE – VD/DAQ (Robust Version)
// Logs even if RTC or IMU init fails.
// Button D7<->GND toggles logging (INPUT_PULLUP). LED D8 ON while logging.
// SD CS=D10. Hall sensors on D2/D3. RTC & IMU share A4(SDA)/A5(SCL). IMU AD0->5V preferred (0x69).

#include <Wire.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>
// #include <Adafruit_MPU6050.h> // <-- REMOVED: Using manual I2C read
// #include <Adafruit_Sensor.h>  // <-- REMOVED: Using manual I2C read

RTC_DS3231 rtc;
// Adafruit_MPU6050 mpu;       // <-- REMOVED: Using manual I2C read

// -------- MPU6050 Manual Read Constants (from working test sketch) --------
const byte MPU_I2C_ADDRESS = 0x69;     // The address your chip is responding to
const byte MPU6050_ID_REG = 0x75;      // WHO_AM_I register
const byte MPU6050_ID_EXPECTED = 0x98; // The ID your specific chip returns
const byte MPU6050_PWR_MGMT_1 = 0x6B;  // Power Management Register
const byte MPU6050_ACCEL_CONFIG = 0x1C; // Accel Config Register
const byte MPU6050_GYRO_CONFIG = 0x1B; // Gyro Config Register
const byte MPU6050_ACCEL_OUT = 0x3B;    // Base register for all sensor data

// Scaling factors for converting raw data to physical units (2g / 250 deg/s ranges)
const float ACCEL_SCALE_FACTOR = 0.000598755; // (2 * 9.80665 / 16384) in m/s^2 per LSB
const float GYRO_SCALE_FACTOR = 0.007633587;  // (250 / 131.0) in deg/s per LSB
// -------------------------------------------------------------------------

// -------- Pins --------
const int SD_CS = 10;
const int hallPrimary = 2;
const int hallSecondary = 3;
const int ledPin = 8;
const int buttonPin = 7;

// -------- Settings --------
const unsigned long SAMPLE_MS = 500;  // use 5000 for hand tests
const uint8_t PPR = 1;                // pulses per rev (magnets per rev)
const unsigned long DEBOUNCE_MS = 50;

volatile unsigned long cntP = 0, cntS = 0;
bool logging = false;
bool lastStable = HIGH, debouncing = false;
unsigned long tDebounce = 0;

bool sdOk = false, rtcOk = false, imuOk = false;
unsigned long t0_ms = 0;  // millis at power-up (fallback time base)
char filename[32];


// --- MPU6050 Manual I2C Functions ---

// Function to manually read one or more registers from the MPU
void readRegisters(byte reg, byte length, byte *data) {
  Wire.beginTransmission(MPU_I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false); // Send restart
  Wire.requestFrom(MPU_I2C_ADDRESS, length);
  int i = 0;
  while (Wire.available() && i < length) {
    data[i++] = Wire.read();
  }
}

// Function to write one byte to an MPU register
void writeRegister(byte reg, byte data) {
  Wire.beginTransmission(MPU_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

// Custom initialization routine that bypasses the Adafruit ID check
bool initMPU6050() {
  byte chipId = 0x00;
  byte data[1];

  // 1. Check WHO_AM_I ID
  readRegisters(MPU6050_ID_REG, 1, data);
  chipId = data[0];
  Serial.print("MPU WHO_AM_I ID @ 0x69: 0x"); Serial.println(chipId, HEX);

  if (chipId != MPU6050_ID_EXPECTED) {
    Serial.print("ERROR: MPU ID 0x"); Serial.print(chipId, HEX);
    Serial.println(" does not match expected 0x98.");
    return false;
  }

  // 2. Wake up the MPU (clear sleep bit 6, set clock source to PLL with X-axis gyro)
  writeRegister(MPU6050_PWR_MGMT_1, 0x01);

  // 3. Set Gyroscope Configuration (250 deg/s range, bits 4 & 3 are 0)
  writeRegister(MPU6050_GYRO_CONFIG, 0x00);

  // 4. Set Accelerometer Configuration (2g range, bits 4 & 3 are 0)
  writeRegister(MPU6050_ACCEL_CONFIG, 0x00);

  // 5. Success!
  return true;
}


// --- ISRs ---
void isrP() {
  cntP++;
}
void isrS() {
  cntS++;
}

// --- Helpers ---
void buildNewRunFilename() {
  if (rtcOk) {
    DateTime now = rtc.now();
    int run = 1;
    do {
      snprintf(filename, sizeof(filename), "%04d-%02d-%02d_RUN%d.CSV",
               now.year(), now.month(), now.day(), run);
      run++;
    } while (SD.exists(filename));
  } else {
    int run = 1;
    do {
      snprintf(filename, sizeof(filename), "RUN%03d.CSV", run);
      run++;
    } while (SD.exists(filename));
  }
}

void writeHeader() {
  File localFile = SD.open(filename, FILE_WRITE);  // **USE LOCAL FILE HANDLE**
  if (localFile) {
    // NOTE: Header now includes all IMU data (Accel, Gyro)
    if (rtcOk) localFile.println("Date,Time,Primary_RPM,Secondary_RPM,Ax_mps2,Ay_mps2,Az_mps2,Gx_dps,Gy_dps,Gz_dps,Temp_C");
    else localFile.println("Time_s,Primary_RPM,Secondary_RPM,Ax_mps2,Ay_mps2,Az_mps2,Gx_dps,Gy_dps,Gz_dps,Temp_C");
    localFile.close();  // **ALWAYS CLOSE LOCAL HANDLE**
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

  // SD first (required for logging)
  sdOk = SD.begin(SD_CS);
  Serial.print("SD: ");
  Serial.println(sdOk ? "OK" : "FAIL");

  // RTC (don’t block if it fails)
  rtcOk = rtc.begin();
  Serial.print("RTC: ");
  Serial.println(rtcOk ? "OK" : "FAIL");
  if (rtcOk && rtc.lostPower()) {
    Serial.println("RTC lostPower -> setting compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // IMU (using manual function)
  Serial.println("--- IMU Initialization ---");
  imuOk = initMPU6050();

  Serial.print("IMU: ");
  Serial.println(imuOk ? "OK" : "FAIL");
  
  // The configuration is now handled within initMPU6050(), 
  // assuming 2g and 250 deg/s ranges.
  // Removed: mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // Removed: mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Removed: mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  t0_ms = millis();
  Serial.println("DAQ ready. Press button to START/STOP logging.");
}

void loop() {
  unsigned long nowMs = millis();

  // --- Debounced toggle on RELEASE ---
  bool raw = digitalRead(buttonPin);  // HIGH idle, LOW pressed
  if (!debouncing && raw != lastStable) {
    debouncing = true;
    tDebounce = nowMs;
  }
  if (debouncing && (nowMs - tDebounce) > DEBOUNCE_MS) {
    bool raw2 = digitalRead(buttonPin);
    if (raw2 != lastStable) {
      if (lastStable == LOW && raw2 == HIGH) {  // toggle on release
        if (!logging) {
          if (!sdOk) {  // try to (re)start SD if it failed earlier
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

    noInterrupts();
    unsigned long nP = cntP, nS = cntS;
    cntP = 0;
    cntS = 0;
    interrupts();

    float rpmP = (nP * (60000.0 / SAMPLE_MS)) / PPR;
    float rpmS = (nS * (60000.0 / SAMPLE_MS)) / PPR;

    // IMU sample (zeros if IMU not OK)
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, tempC = 0;
    
    if (imuOk) {
      byte rawData[14]; // Raw data block: Accel(X,Y,Z), Temp, Gyro(X,Y,Z)
      readRegisters(MPU6050_ACCEL_OUT, 14, rawData);

      // Convert 14 bytes into 7 signed 16-bit integers
      int16_t accX_raw = (rawData[0] << 8) | rawData[1];
      int16_t accY_raw = (rawData[2] << 8) | rawData[3];
      int16_t accZ_raw = (rawData[4] << 8) | rawData[5];
      int16_t temp_raw = (rawData[6] << 8) | rawData[7];
      int16_t gyroX_raw = (rawData[8] << 8) | rawData[9];
      int16_t gyroY_raw = (rawData[10] << 8) | rawData[11];
      int16_t gyroZ_raw = (rawData[12] << 8) | rawData[13];

      // Convert raw data to physical units (using 2g/250dps ranges set in init)
      ax = accX_raw * ACCEL_SCALE_FACTOR;
      ay = accY_raw * ACCEL_SCALE_FACTOR;
      az = accZ_raw * ACCEL_SCALE_FACTOR;

      gx = gyroX_raw * GYRO_SCALE_FACTOR;
      gy = gyroY_raw * GYRO_SCALE_FACTOR;
      gz = gyroZ_raw * GYRO_SCALE_FACTOR;
      
      // Temperature conversion formula for MPU6050
      tempC = (temp_raw / 340.0) + 36.53;
    }

    File localFile = SD.open(filename, FILE_WRITE);  // **USE LOCAL FILE HANDLE**
    if (localFile) {
      // --- Time/Date Column ---
      if (rtcOk) {
        DateTime dt = rtc.now();
        localFile.print(dt.year());
        localFile.print('/');
        if (dt.month() < 10) localFile.print('0');
        localFile.print(dt.month());
        localFile.print('/');
        if (dt.day() < 10) localFile.print('0');
        localFile.print(dt.day());
        localFile.print(',');
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
        float tsec = (millis() - t0_ms) / 1000.0;
        localFile.print(tsec, 3);
        localFile.print(',');
      }

      // --- RPM and IMU Data Columns ---
      localFile.print(rpmP, 2);
      localFile.print(',');
      localFile.print(rpmS, 2);
      localFile.print(',');
      localFile.print(ax, 3);
      localFile.print(',');
      localFile.print(ay, 3);
      localFile.print(',');
      localFile.print(az, 3);
      localFile.print(',');
      localFile.print(gx, 3);
      localFile.print(',');
      localFile.print(gy, 3);
      localFile.print(',');
      localFile.print(gz, 3);
      localFile.print(',');
      localFile.println(tempC, 2); // Temp is the last column

      localFile.close();  // **ALWAYS CLOSE LOCAL HANDLE**
      Serial.println("💾 write ok");
    } else {
      Serial.println("❌ file open fail");
    }
  }
}