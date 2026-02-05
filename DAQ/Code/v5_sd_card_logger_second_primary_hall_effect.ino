// **hall effect sensor for cvt dyno with SD card logger**
// CSUF BAJA SAE - Vehicle Dynamics

// assignments to arduino
// - Hall effect sensor A - primary
// - Hall effect sensor B - secondary

// - SD module CS pin - 10
// - File format: RUN#.CSV - adds in increments
// - Logs: Time_s,RPM_A,RPM_B - when in SD card it is an excel

// Wiring for sensors (make sure GND and VCC is hooked up correct)
//   S -> D2 and D3
//   VCC -> 5V
//   - -> GND
//   10k pull-up resistor for sensors

#include <SPI.h> // need library to use SD logger
#include <SD.h>

// pin assaignments
const uint8_t hallPrimary = 2;   // Sensor A - D2
const uint8_t hallSecondary = 3;   // Sensor B - D3
const uint8_t SD_CS = 10;

// settings
const uint16_t SAMPLE_MS = 500; // measures how many pulses were counted every 500 ms
const uint8_t PPR = 1;          // pulses per revolution, can change to any number, 1 magnet is in use

// variables
volatile uint32_t pulsesPrimary = 0; // volatile is a variable value that can change unexpectdly
volatile uint32_t pulsesSecondary = 0;
File logFile; // logFile represents when the .csv file is open on the SD card

// interrupts for each sensor
void onPulsePrimary() { pulsesPrimary++; }
void onPulseSecondary() { pulsesSecondary++; }

// to find next available RUN#.CSV on SD card
// sensor goes low when magnet passes

// finds next available file name for SD card
// returns first unused name, name goes up on each ran
String nextFileName() {
  for (int i = 1; i < 1000; i++) {
    String name = "RUN" + String(i) + ".CSV";
    if (!SD.exists(name)) return name;
  }
  return "RUN999.CSV";  // fallback
}

// runs when arduino gets power
// checks status of SD card to prevent loss of data
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing SD card...");

  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed! Check wiring.");
    while (1);
  }
  Serial.println("SD init OK.");

  // create new run file for data
  // if can not open file it stops
  // if it can open then says what filename is being used
  String filename = nextFileName();
  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.println("Error opening file!");
    while (1);
  }

  Serial.print("Logging to: ");
  Serial.println(filename);

  // headers for file
  logFile.println("Time_s,RPM_Primary,RPM_Secondary"); // column names for excel
  logFile.flush(); // makes the data go to SD card immediately

  // set up for hall sensors
  pinMode(hallPrimary, INPUT);
  pinMode(hallSecondary, INPUT);
  attachInterrupt(digitalPinToInterrupt(hallPrimary), onPulsePrimary, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallSecondary), onPulseSecondary, FALLING);

  Serial.println("Time_s,RPM_Primary,RPM_Secondary");
}

void loop() {
  // reset counters
  noInterrupts();
  pulsesPrimary = 0;
  pulsesSecondary = 0;
  interrupts();

  unsigned long t0 = millis();
  delay(SAMPLE_MS);
  unsigned long t1 = millis();

  noInterrupts();
  uint32_t cPrimary = pulsesPrimary;
  uint32_t cSecondary = pulsesSecondary;
  interrupts();

  float dt_s = (t1 - t0) / 1000.0f;
  float rpmPrimary = (cPrimary / (float)PPR) / dt_s * 60.0f;
  float rpmSecondary = (cSecondary / (float)PPR) / dt_s * 60.0f;
  float timeNow = millis() / 1000.0f;

  // outputs to serial
  Serial.print(timeNow, 3); Serial.print(",");
  Serial.print(rpmPrimary, 1);    Serial.print(",");
  Serial.println(rpmSecondary, 1);

  // logs to data
  logFile.print(timeNow, 3); logFile.print(",");
  logFile.print(rpmPrimary, 1);    logFile.print(",");
  logFile.println(rpmSecondary, 1);
  logFile.flush();  // ensures data is saved every cycle
}
