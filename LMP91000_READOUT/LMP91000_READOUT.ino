#include <LMP91000.h>
#include <Wire.h>

// Configuration
#define SDA_PIN 6
#define SCL_PIN 7
#define LED_PIN 9
const int MENB_PINS[4] = {2, 3, 4, 5};  // TIA0, TIA1, TIA2, TIA3
// Current R_External: 100M, 10M, 0, 100k

#define DEBUG true
#define ACTIVE_SENSOR 2  // 0-3 (LMP1-LMP4), currently testing LMP2

LMP91000 sensors[4] = {LMP91000(), LMP91000(), LMP91000(), LMP91000()};

void configureLMP(LMP91000 &sensor, int menbPin, const char* name) {
  Serial.print("Configuring "); Serial.println(name);
  sensor.standby();
  sensor.setMENB(menbPin);
  sensor.setTwoLead();
  sensor.setGain(1);
  sensor.setRLoad(3);
  sensor.setExtRefSource();
  sensor.setIntZ(3);
  sensor.disableFET();
  sensor.setNegBias();
  sensor.setBias(0);
  sensor.disable();
  Serial.print(name); Serial.println(" configured.");
}

void scanI2C() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("No I2C devices found.");
}

void testGain(int sensorIdx, int gain) {
  sensors[sensorIdx].enable();
  delay(50);
  sensors[sensorIdx].setGain(gain);
  double actualGain = sensors[sensorIdx].getGain();
  Serial.print("LMP"); Serial.print(sensorIdx + 1);
  Serial.print(" Gain "); Serial.print(gain);
  Serial.print(" = "); Serial.println(actualGain, 4);
  sensors[sensorIdx].disable();
}

void blinkLED(int times, int delayTime) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(delayTime);
    digitalWrite(LED_PIN, LOW);
    delay(delayTime);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  pinMode(MENB_PINS[0], OUTPUT);
  pinMode(MENB_PINS[1], OUTPUT);
  pinMode(MENB_PINS[2], OUTPUT);
  pinMode(MENB_PINS[3], OUTPUT);

  configureLMP(sensors[0], MENB_PINS[0], "LMP1");
  configureLMP(sensors[1], MENB_PINS[1], "LMP2");
  configureLMP(sensors[2], MENB_PINS[2], "LMP3");
  configureLMP(sensors[3], MENB_PINS[3], "LMP4");
  Serial.println("All LMPs configured");
  
  Serial.println("Ready.\n");
  pinMode(LED_PIN, OUTPUT);
  blinkLED(3, 100); // Blink 3 times to indicate ready
}

void loop() {
  // Test single sensor with gain sweep
  for (int gain = 0; gain < 8; gain++) {
    testGain(ACTIVE_SENSOR, gain);
    blinkLED(10, 100); // Blink 1 time to indicate gain change
    delay(5000);
  }
}