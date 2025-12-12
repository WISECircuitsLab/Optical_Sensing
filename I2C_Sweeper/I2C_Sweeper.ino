#include <Wire.h>

// Board Information, please consult to verify pin mappings:
//  - https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/

// --- I2C pins using board label style ---
#define SDA_PIN D4
#define SCL_PIN D5

// --- MENB pins (active LOW on LMP91000) ---
const int MENB_PINS[4] = {D10, D1, D2, D3};

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nI2C sweep — all MENB pins driven LOW");

  // Drive all MENB pins LOW so devices are enabled
  for (int i = 0; i < 4; ++i) {
    pinMode(MENB_PINS[i], OUTPUT);
    digitalWrite(MENB_PINS[i], LOW);
  }

  // Start I2C on chosen pins (using board labels)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
}

void loop() {
  Serial.println("Starting bus scan...");
  byte found = 0;

  for (byte addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    byte err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      ++found;
      delay(2);
    }
  }

  if (found == 0) {
    Serial.println("No I2C devices detected.");
  } else {
    Serial.print("Scan complete. Devices found: ");
    Serial.println(found);
  }

  Serial.println();
  delay(2000);
}
