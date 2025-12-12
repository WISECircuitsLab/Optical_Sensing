/*
 * ============================================================================
 * LMP91000 READOUT SYSTEM - Serial Command Interface
 * ============================================================================
 * 
 * AVAILABLE SERIAL COMMANDS:
 * -------------------------
 * 'u' - Up gain level (increment internal gain, range: 0-7)
 * 'd' - Down gain level (decrement internal gain, range: 0-7)
 * 'n' - Next TIA (switch to next TIA, wraps around: 0->1->2->3->0)
 * 'b' - Back TIA (switch to previous TIA, wraps around: 0->3->2->1->0)
 * 'o' - LED ON (turn LED on)
 * 'f' - LED OFF (turn LED off)
 * 'l,hertz,duration_ms' - Blink LED (e.g., "l,2.5,1000" = 2.5 Hz for 1000ms)
 * 'C' - Toggle continuous A0 voltage stream mode (reads analog input from pin A0)
 * 
 * GAIN LOGIC:
 * -----------
 * - Gain Level 0 (EXT_GAIN): Internal gain is open circuit, only external gain used
 * - Gain Level 1-7: Internal gain is in parallel with external gain
 *   * If external gain is -1 (open circuit), only internal gain is used
 *   * If external gain is valid, total gain = R_internal || R_external (parallel)
 * 
 * INTERNAL GAIN VALUES (kOhm):
 * -----------------------------
 * 0 - External only (internal open)
 * 1 - 2.75 kOhm
 * 2 - 3.5 kOhm
 * 3 - 7 kOhm
 * 4 - 14 kOhm
 * 5 - 35 kOhm
 * 6 - 120 kOhm
 * 7 - 350 kOhm
 * 
 * EXTERNAL GAIN VALUES (Ohm):
 * ---------------------------
 * TIA0: 100 MOhm (100,000,000)
 * TIA1: 10 MOhm (10,000,000)
 * TIA2: Open circuit (-1)
 * TIA3: 100 kOhm (100,000)
 * 
 * CSV OUTPUT FORMAT:
 * ------------------
 * Timestamp(ms),Command,TIA,Gain_Level,R_External(Ohm),R_Internal(Ohm),R_Total(Ohm),LED_State
 * 
 * ============================================================================
 */

#include <LMP91000.h>
#include <Wire.h>

// Configuration
#define SDA_PIN D4
#define SCL_PIN D5
#define LED_PIN D9
const int MENB_PINS[4] = {D10, D1, D2, D3};  // TIA0, TIA1, TIA2, TIA3 (TIA0 not connected on either board, reset to D10, not used)
const float R_External[4] = {-1.0, 10000000.0, -1.0, 100000.0}; // -1 indicates no external gain, open circuit

#define DEBUG true
#define MAX_GAIN 7
#define MIN_GAIN 0
#define EXT_GAIN 0 // set gain to 0 if wanting to use only the external gain
#define NUM_TIAS 4

LMP91000 sensors[4] = {LMP91000(), LMP91000(), LMP91000(), LMP91000()};

// State variables
int currentTIA = 0;  // 0-3 (LMP1-LMP4)
int currentGain = 4; // 0-7 (initial: 14 kOhm)
bool ledState = false; // LED state
unsigned long startTime = 0; // System start time for timestamps

// Continuous stream mode variables
bool continuousMode = false; // Continuous stream mode state
float avgVoltage = 0.0; // Exponential weighted average voltage
const float ALPHA = 0.1; // Exponential averaging factor (0.0 to 1.0, lower = more smoothing)
const int A0_PIN = A0; // Analog input pin

void configureLMP(LMP91000 &sensor, int menbPin, const char* name) {
  Serial.print("Configuring "); Serial.println(name);
  sensor.standby();

  sensor.setMENB(menbPin);

  sensor.setThreeLead(); //the LMP91000 for 3-electrode potentiometric measurements.
  sensor.setGain(4); // Initial TIA gain set to setting 4: i.e. 14 Kohm
  //param - value - RLoad: 0 - 00 - 10 Ohm; 1 - 01 - 33 Ohm; 2 - 10 - 50 Ohm; 3 - 11 - 100 Ohm
  sensor.setRLoad(0); // setting internal load to 10ohms

  // Setting reference voltage for TAI
  sensor.setIntRefSource(); // set bias source to Vdd not Vextern
  //0 - 00 - 20%; /1 - 01 - 50%; 2 - 10 - 67%; 3 - 11 - bypassed
  sensor.setIntZ(1); // sets the TIA bias to 50% of Vdd

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
//param - value - gain resistor
//0 - 000 - External resistor
//1 - 001 - 2.75 kOhm
//2 - 010 - 3.5 kOhm
//3 - 011 - 7 kOhm
//4 - 100 - 14 kOhm
//5 - 101 - 35 kOhm
//6 - 110 - 120 kOhm
//7 - 111 - 350 kOhm
  sensors[sensorIdx].enable();
  delay(50);
  sensors[sensorIdx].setGain(gain);
  double actualGain = sensors[sensorIdx].getGain();
  Serial.print("LMP"); Serial.print(sensorIdx + 1);
  Serial.print(" Gain "); Serial.print(gain);
  Serial.print(" = "); Serial.println(actualGain, 4);
  sensors[sensorIdx].disable();
}

// LED Control Functions
void blinkLED(float hertz, unsigned long durationMs) {
  unsigned long periodMs = (unsigned long)(1000.0 / hertz);
  unsigned long halfPeriod = periodMs / 2;
  unsigned long startTime = millis();
  
  while (millis() - startTime < durationMs) {
    digitalWrite(LED_PIN, HIGH);
    delay(halfPeriod);
    digitalWrite(LED_PIN, LOW);
    delay(halfPeriod);
  }
}

void LEDON() {
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
}

void LEDOFF() {
  digitalWrite(LED_PIN, LOW);
  ledState = false;
}

// Internal gain resistance values in Ohms
const float R_Internal[8] = {
  0.0,      // 0 - External only (open circuit)
  2750.0,   // 1 - 2.75 kOhm = 2750 Ohms
  3500.0,   // 2 - 3.5 kOhm = 3500 Ohms
  7000.0,   // 3 - 7 kOhm = 7000 Ohms
  14000.0,  // 4 - 14 kOhm = 14000 Ohms
  35000.0,  // 5 - 35 kOhm = 35000 Ohms
  120000.0, // 6 - 120 kOhm = 120000 Ohms
  350000.0  // 7 - 350 kOhm = 350000 Ohms
};

// Calculate total parallel resistance
float calculateTotalResistance(int tia, int gainLevel) {
  float rInt = R_Internal[gainLevel];
  float rExt = R_External[tia];
  
  // If gain is 0 (EXT_GAIN), internal is open, use only external
  if (gainLevel == EXT_GAIN) {
    if (rExt > 0) {
      return rExt;
    } else {
      return -1.0; // Both open circuit
    }
  }
  
  // If external is open circuit (-1), use only internal
  if (rExt < 0) {
    return rInt;
  }
  
  // Both are valid, calculate parallel resistance: R_total = (R1 * R2) / (R1 + R2)
  return (rInt * rExt) / (rInt + rExt);
}

// TIA and Gain Control Functions
void upGain() {
  if (currentGain < MAX_GAIN) {
    currentGain++;
    // If gain is 0, we want external only, so set sensor to 0
    // If gain is 1-7, set sensor to that gain level
    sensors[currentTIA].setGain(currentGain);
    printStatus();
  } else {
    Serial.println("Already at maximum gain (7)");
  }
}

void downGain() {
  if (currentGain > MIN_GAIN) {
    currentGain--;
    sensors[currentTIA].setGain(currentGain);
    printStatus();
  } else {
    Serial.println("Already at minimum gain (0)");
  }
}

void nextTIA() {
  sensors[currentTIA].disable();
  currentTIA = (currentTIA + 1) % NUM_TIAS;
  sensors[currentTIA].enable();
  delay(50);
  sensors[currentTIA].setGain(currentGain);
  printStatus();
}

void backTIA() {
  sensors[currentTIA].disable();
  currentTIA = (currentTIA - 1 + NUM_TIAS) % NUM_TIAS;
  sensors[currentTIA].enable();
  delay(50);
  sensors[currentTIA].setGain(currentGain);
  printStatus();
}

void printStatus() {
  float rExt = R_External[currentTIA];
  float rInt = R_Internal[currentGain];
  float rTotal = calculateTotalResistance(currentTIA, currentGain);
  
  Serial.println("\n========================================");
  Serial.print("TIA: "); Serial.println(currentTIA);
  Serial.print("Gain Level: "); Serial.println(currentGain);
  
  // Print External Resistance
  Serial.print("R_External: ");
  if (rExt < 0) {
    Serial.println("Open Circuit");
  } else if (rExt >= 1000000.0) {
    Serial.print(rExt / 1000000.0, 2); Serial.println(" MOhm");
  } else if (rExt >= 1000.0) {
    Serial.print(rExt / 1000.0, 2); Serial.println(" kOhm");
  } else {
    Serial.print(rExt, 2); Serial.println(" Ohm");
  }
  
  // Print Internal Resistance
  Serial.print("R_Internal: ");
  if (currentGain == EXT_GAIN) {
    Serial.println("Open Circuit (External Only Mode)");
  } else {
    Serial.print("Level "); Serial.print(currentGain); Serial.print(" = ");
    if (rInt >= 1000.0) {
      Serial.print(rInt / 1000.0, 2); Serial.println(" kOhm");
    } else {
      Serial.print(rInt, 2); Serial.println(" Ohm");
    }
  }
  
  // Print Total Resistance
  Serial.print("R_Total (Parallel): ");
  if (rTotal < 0) {
    Serial.println("Open Circuit");
  } else if (rTotal >= 1000000.0) {
    Serial.print(rTotal / 1000000.0, 4); Serial.println(" MOhm");
  } else if (rTotal >= 1000.0) {
    Serial.print(rTotal / 1000.0, 4); Serial.println(" kOhm");
  } else {
    Serial.print(rTotal, 4); Serial.println(" Ohm");
  }
  Serial.println("========================================\n");
}

// Read and print A0 voltage with exponential weighted average
void readAndPrintA0() {
  int analogValue = analogRead(A0_PIN);
  // 12-bit adc, at vdd=3.3V
  float voltage = (analogValue / 4095.0) * 3.3; // Assuming 3.3V reference, adjust if needed
  
  // Update exponential weighted average
  if (avgVoltage == 0.0) {
    // First reading, initialize average
    avgVoltage = voltage;
  } else {
    // Exponential weighted average: avg = alpha * new + (1 - alpha) * old
    avgVoltage = ALPHA * voltage + (1.0 - ALPHA) * avgVoltage;
  }
  
  // Print current voltage and average voltage
  Serial.print("A0 Voltage: ");
  Serial.print(voltage, 4);
  Serial.print(" V, Avg: ");
  Serial.print(avgVoltage, 4);
  Serial.println(" V");
}

// CSV Logging Function
void printCSVLog(const char* command) {
  unsigned long timestamp = millis() - startTime;
  float rExt = R_External[currentTIA];
  float rInt = R_Internal[currentGain];
  float rTotal = calculateTotalResistance(currentTIA, currentGain);
  
  // CSV format: Timestamp(ms), Command, TIA, Gain_Level, R_External(Ohm), R_Internal(Ohm), R_Total(Ohm), LED_State
  Serial.print(timestamp);
  Serial.print(",");
  Serial.print(command);
  Serial.print(",");
  Serial.print(currentTIA);
  Serial.print(",");
  Serial.print(currentGain);
  Serial.print(",");
  if (rExt < 0) {
    Serial.print("OPEN");
  } else {
    Serial.print(rExt, 2);
  }
  Serial.print(",");
  if (currentGain == EXT_GAIN) {
    Serial.print("OPEN");
  } else {
    Serial.print(rInt, 2);
  }
  Serial.print(",");
  if (rTotal < 0) {
    Serial.print("OPEN");
  } else {
    Serial.print(rTotal, 4);
  }
  Serial.print(",");
  Serial.println(ledState ? "ON" : "OFF");
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
  
  // Enable initial TIA
  sensors[currentTIA].enable();
  delay(50);
  sensors[currentTIA].setGain(currentGain);
  
  pinMode(LED_PIN, OUTPUT);
  startTime = millis();
  
  Serial.println("\nReady. Commands:");
  Serial.println("  'u' - Up gain level");
  Serial.println("  'd' - Down gain level");
  Serial.println("  'n' - Next TIA");
  Serial.println("  'b' - Previous TIA");
  Serial.println("  'o' - LED ON");
  Serial.println("  'f' - LED OFF");
  Serial.println("  'l' - Blink LED (format: l,hertz,duration_ms)");
  Serial.println("  'C' - Toggle continuous A0 voltage stream mode");
  
  // Print CSV header
  Serial.println("\nCSV Log:");
  Serial.println("Timestamp(ms),Command,TIA,Gain_Level,R_External(Ohm),R_Internal(Ohm),R_Total(Ohm),LED_State");
  
  printStatus();
  blinkLED(2.0, 500); // Blink at 2 Hz for 500ms
  printCSVLog("INIT"); // Log initial state
}

void loop() {
  // Check if continuous stream mode is active
  if (continuousMode) {
    readAndPrintA0();
    delay(100); // Small delay to prevent overwhelming serial output (adjust as needed)
  }
  
  if (Serial.available() > 0) {
    Serial.setTimeout(100);
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() == 0) return;
    
    char command = input.charAt(0);
    
    switch (command) {
      case 'u':
        upGain();
        printCSVLog("UP_GAIN");
        break;
      case 'd':
        downGain();
        printCSVLog("DOWN_GAIN");
        break;
      case 'n':
        nextTIA();
        printCSVLog("NEXT_TIA");
        break;
      case 'b':
        backTIA();
        printCSVLog("BACK_TIA");
        break;
      case 'o':
        LEDON();
        Serial.println("LED ON");
        printCSVLog("LED_ON");
        break;
      case 'f':
        LEDOFF();
        Serial.println("LED OFF");
        printCSVLog("LED_OFF");
        break;
      case 'l': {
        // Parse: l,hertz,duration_ms
        int commaIdx = input.indexOf(',');
        if (commaIdx > 0) {
          int commaIdx2 = input.indexOf(',', commaIdx + 1);
          if (commaIdx2 > commaIdx) {
            float hertz = input.substring(commaIdx + 1, commaIdx2).toFloat();
            unsigned long duration = input.substring(commaIdx2 + 1).toInt();
            Serial.print("Blinking LED at "); Serial.print(hertz);
            Serial.print(" Hz for "); Serial.print(duration); Serial.println(" ms");
            blinkLED(hertz, duration);
            // LED state returns to previous after blinking
            printCSVLog("BLINK_LED");
          } else {
            Serial.println("Invalid format. Use: l,hertz,duration_ms");
            printCSVLog("ERROR");
          }
        } else {
          Serial.println("Invalid format. Use: l,hertz,duration_ms");
          printCSVLog("ERROR");
        }
        break;
      }
      case 'C': {
        // Toggle continuous stream mode
        continuousMode = !continuousMode;
        if (continuousMode) {
          // Starting new session, reset average
          avgVoltage = 0.0;
          Serial.println("Continuous A0 voltage stream mode ON");
          printCSVLog("CONTINUOUS_ON");
        } else {
          Serial.println("Continuous A0 voltage stream mode OFF");
          printCSVLog("CONTINUOUS_OFF");
        }
        break;
      }
      default:
        Serial.print("Unknown command: '"); Serial.print(command); Serial.println("'");
        printCSVLog("UNKNOWN");
        break;
    }
  }
}