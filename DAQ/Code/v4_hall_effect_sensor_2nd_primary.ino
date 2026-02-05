// **hall effect sensor for cvt dyno**
// CSUF BAJA SAE - Vehicle Dynamics

// Two channels: A on D2, B on D3 (A3144 open-collector to 5V with 10k pull-up)

const uint8_t hallA = 2;   // Sensor A output (D2 / INT0)
const uint8_t hallB = 3;   // Sensor B output (D3 / INT1)

const uint16_t SAMPLE_MS = 500; // sampling window (ms)
const uint8_t  PPR = 1;         // pulses per revolution (magnets per rev)

volatile uint32_t pulsesA = 0;
volatile uint32_t pulsesB = 0;

void onPulseA() { pulsesA++; }   // trigger on falling edge (active-low pulse)
void onPulseB() { pulsesB++; }

void setup() {
  Serial.begin(9600);
  // Use INPUT if you wired 10k pull-ups to 5V; use INPUT_PULLUP if you didn't.
  pinMode(hallA, INPUT);
  pinMode(hallB, INPUT);

  attachInterrupt(digitalPinToInterrupt(hallA), onPulseA, FALLING);
  attachInterrupt(digitalPinToInterrupt(hallB), onPulseB, FALLING);

  Serial.println("Time_s,RPM_A,RPM_B");  // when runs outputs columns in format easy for tables
}

void loop() {
  // zero counts and measure over a fixed window
  noInterrupts();
  pulsesA = 0;
  pulsesB = 0;
  interrupts();

  unsigned long t0 = millis();
  delay(SAMPLE_MS);  // collect pulses during this window
  unsigned long t1 = millis();

  noInterrupts();
  uint32_t cA = pulsesA;
  uint32_t cB = pulsesB;
  interrupts();

  float dt_s = (t1 - t0) / 1000.0f;
  float rpmA = (cA / (float)PPR) / dt_s * 60.0f;
  float rpmB = (cB / (float)PPR) / dt_s * 60.0f;

  float timeNow = millis() / 1000.0f;
  Serial.print(timeNow, 3); Serial.print(",");
  Serial.print(rpmA, 1);    Serial.print(",");
  Serial.println(rpmB, 1);
}
