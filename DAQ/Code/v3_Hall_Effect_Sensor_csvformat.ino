// **hall effect sensor for cvt dyno**
// CSUF BAJA SAE - Vehicle Dynamics


// code based off from ex: https://www.makerguides.com/arduino-tachometer-a3144-hall-effect-sensor/


// works by reading 100 pulses from the hall effect sensor and then converts to RPM, using 1 magnet for sensor to read

// using  A3144L Hall Effect sensor
// safe voltage range: 4.5 V-24 V -> max voltage 28 V
// operating temp: 302 F (thats for A3144L sensor -> need to use that sensor)
// magnet type: use N35-N52
// needs to be close to air, gap of 1-3 mm

const int hallPin = 2;   // sensor output connected to arduino
const int maxCnt = 100;  // the number of pulses we want before calculating RPM

void setup() {         // runs when power turns on
  Serial.begin(9600);  // 9600 bits/sec
  pinMode(hallPin, INPUT);
  Serial.println("Time_s,RPM");  // prints out header to be in csv format
}

void loop() {                        // repeats forever
  unsigned long start = micros();    // micros() returns microsecs since power turns on
  int old = 1;                       // remembers previous state, High = 1, Low = 0
                                     // A3144 output is normally high w/no magnet
  int cnt = 0;                       // each time it goes from HIGH -> LOW, value increases
  while (cnt < maxCnt) {             // goes until 100 which is the mxCnt we choice
    int val = digitalRead(hallPin);  // returns 0 for low and 1 for high
    if (!val && val != old) cnt++;   // if reading low and the current is reading different from previous reading then increase count
    old = val;
  }
  float seconds = (micros() - start) / 1000000.0;  // converts microsecs to secs
  float rpm = cnt / seconds * 60.0;                // calculates RPM by pulses/sec * 60

  // prints out in correct format
  float timeNow = millis() / 1000.0;  // gives time stamp for each RPM reading
  Serial.print(timeNow, 3);           // prints timeNow to 3rd decimal place
  Serial.print(",");
  Serial.println(rpm);  // prints RPM
}