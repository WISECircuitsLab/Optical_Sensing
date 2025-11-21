#include <LMP91000.h>
#include <Wire.h>

// ----------- I2C Pins (adjust if needed) -----------
#define SDA_PIN 6
#define SCL_PIN 7

// ---- TIA configuration ----
LMP91000 pStat1 = LMP91000();
LMP91000 pStat2 = LMP91000();
LMP91000 pStat3 = LMP91000();
LMP91000 pStat4 = LMP91000();

// ----------- LED + MENB Pins -----------
#define LED_PIN 9
const int MENB_PINS[4] = {2, 3, 4, 5};


// ---- Analog input to MCU ----
//const int TIA_OUT_PIN = 34;    // ESP32 ADC-capable pin connected to LMP91000 OUT
//const float ADC_REF = 3.3;     // ESP32 ADC reference voltage


// Helper: configure one LMP91000 instance
void configureLMP(LMP91000 &pStat, int menbPin, const char *name) {
  Serial.print("Configuring "); Serial.println(name);
  pStat.standby();
  pStat.setMENB(menbPin);
  pStat.setTwoLead();
  pStat.setGain(1);
  pStat.getGain();
  pStat.setRLoad(3);
  pStat.setExtRefSource();
  pStat.setIntZ(3);
  pStat.disableFET();
  pStat.setNegBias();
  pStat.setBias(0);
  pStat.disable();
  Serial.print(name); Serial.println(" configured.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---- Configure each LMP ----
  configureLMP(pStat1, MENB_PINS[0], "LMP1");
  configureLMP(pStat2, MENB_PINS[1], "LMP2");
  configureLMP(pStat3, MENB_PINS[2], "LMP3");
  configureLMP(pStat4, MENB_PINS[3], "LMP4");
  Serial.println("All LMPs configured");

  // --- LMP91000 basic setup ---
  // pStat1.standby();
  // pStat1.setMENB(MENB1);
  // pStat1.setTwoLead();
  // pStat1.setRLoad(3);        // 10 Ω load
  // pStat1.setExtRefSource();
  // pStat1.setIntZ(3);         // 50 % internal zero
  // pStat1.disableFET();
  // pStat1.setNegBias();
  // pStat1.setBias(0);
  // pStat1.enable();

  // pinMode(TIA_OUT_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
}

void loop() {
  Serial.println("Stating void loop...");

  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
      count++;
      delay(5);
    }
  }

  if (count == 0)
    Serial.println("⚠️  No I2C devices detected.");
  else
    Serial.println("✅  Scan complete.");


  // put your main code here, to run repeatedly:
  for (int i = 0; i < 8; i++){
    Serial.println(i);
    // pStat1.enable();
    // pStat1.setGain(i);
    // double gain1 = pStat1.getGain();
    // Serial.print("LMP1 Gain");
    // Serial.println(gain1);
    // pStat1.disable();
    // delay(5000);


    

    // pStat2.setGain(i);
    // double gain2 = pStat2.getGain();
    // Serial.print("LMP2 Gain");
    // Serial.println(gain2);
    // delay(1000);
    
    pStat3.enable();
    pStat3.setGain(i);
    double gain3 = pStat3.getGain();
    Serial.print("LMP3 Gain");
    Serial.println(gain3);
    pStat3.disable();
    delay(5000);

    // pStat4.setGain(i);
    // double gain4 = pStat4.getGain();
    // Serial.print("LMP4 Gain");
    // Serial.println(gain4);
    // pStat3.disable();
    // delay(1000);

    // for (int j = 0; j < 4; j++){
    //   int raw = analogRead(A0); 
    //   float volts = raw * (5.0 / 1023.0);
    //   Serial.print("Sensor ");  
    //   Serial.print(j);  
    //   Serial.print(": ");  
    //   Serial.print(volts, 3);  
    //   Serial.print(" V");  
    //   if (i < 3) Serial.print(" \t");
    // }
  
  }

  // blink LED to show end of cycle  
  // digitalWrite(LED_PIN, HIGH);  
  // delay(50);  
  // digitalWrite(LED_PIN, LOW); 
  
}

// //__________I2C Sweeper __________
//  #include <Wire.h>
//  #define SDA_PIN 6
//  #define SCL_PIN 7
// const int MENB_PINS[4] = {2, 3, 4, 5};  // adjust to your wiring

// void setup() {
//   Serial.begin(115200);
//   delay(500);
//   Serial.println("\nI2C scan with MENB forced LOW...");

//   // enable all LMPs (ACTIVE-LOW)
//   for (int i = 0; i < 4; i++) {
//     pinMode(MENB_PINS[i], OUTPUT);
//     digitalWrite(MENB_PINS[i], HIGH);
//   }

//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(100000);
// }

// void loop() {
//   Serial.println("Loop alive");
//   delay(1000);
//   digitalWrite(MENB_PINS[0], LOW);

//   byte count = 0;
//   for (byte addr = 1; addr < 127; addr++) {
//     Wire.beginTransmission(addr);
//     if (Wire.endTransmission() == 0) {
//       Serial.print("Found device at 0x");
//       Serial.println(addr, HEX);
//       count++;
//       delay(5);
//     }
//   }

//   if (count == 0)
//     Serial.println("⚠️  No I2C devices detected.");
//   else
//     Serial.println("✅  Scan complete.");
// }

