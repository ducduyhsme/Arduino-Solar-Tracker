/*
 * Project Group 4 SEMI - Dual-Axis Solar Tracker
 * Hardware: Arduino Uno R3, 2 Servos, 4 LDRs, Potentiometer, 2 Pushbuttons
 * Features: Auto/Manual modes, PLX-DAQ Excel Data Logging, Night Reset
 * * Based on project documentation [cite: 2, 3, 20]
 */

#include <Servo.h>

// --- Pin Definitions [cite: 22, 23, 25, 27] ---
#define LDR_TL_PIN A0  // Top-Left LDR
#define LDR_TR_PIN A1  // Top-Right LDR
#define LDR_BL_PIN A2  // Bottom-Left LDR
#define LDR_BR_PIN A3  // Bottom-Right LDR

#define POT_PIN A4     // Potentiometer for Manual Control
#define VOLT_PIN A5    // PV Panel Voltage Divider Input

#define SERVO_LR_PIN 6 // Left-Right Servo (Azimuth) [cite: 23]
#define SERVO_UD_PIN 5 // Up-Down Servo (Elevation) [cite: 23]

#define BTN_MODE_PIN 12 // Switch between Auto/Manual [cite: 26, 42]
#define BTN_AXIS_PIN 11 // Switch between LR/UD Servo in Manual [cite: 25, 44]

// --- Constants & Configuration ---
#define TOLERANCE 10    // Deadband range [-10, 10] to stabilize motors [cite: 48]
#define NIGHT_LIMIT 8   // Low light threshold to trigger night return [cite: 51]
#define LOAD_RESISTANCE 10.0 // Value of the load resistor in Ohms (Update this to your actual resistor value)

Servo servoLR; // Horizontal (East-West)
Servo servoUD; // Vertical (North-South)

// --- Variables ---
bool isAutoMode = true;      // true = Automatic, false = Manual
bool manualControlLR = true; // true = Controlling Left-Right, false = Up-Down

int posLR = 90; // Initial position
int posUD = 45; // Initial position

// Button State Tracking (for debouncing)
int lastBtnModeState = HIGH;
int lastBtnAxisState = HIGH;

void setup() {
  // Initialize Serial for PLX-DAQ 
  Serial.begin(9600);
  
  // Send PLX-DAQ header commands
  Serial.println("CLEARDATA"); 
  Serial.println("LABEL,Time,Mode,Voltage(V),Current(mA),Power(mW)"); 

  // Attach Servos
  servoLR.attach(SERVO_LR_PIN);
  servoUD.attach(SERVO_UD_PIN);
  
  // Set Pin Modes
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_AXIS_PIN, INPUT_PULLUP);
  
  // Move to initial position
  servoLR.write(posLR);
  servoUD.write(posUD);
  
  delay(500);
}

void loop() {
  // --- 1. Check Mode Switch Button [cite: 42] ---
  int btnModeState = digitalRead(BTN_MODE_PIN);
  if (btnModeState == LOW && lastBtnModeState == HIGH) {
    isAutoMode = !isAutoMode; // Toggle Mode
    delay(200); // Simple debounce
  }
  lastBtnModeState = btnModeState;

  // --- 2. Execute Mode Logic ---
  if (isAutoMode) {
    runAutomaticMode();
  } else {
    runManualMode();
  }

  // --- 3. Data Logging to Excel [cite: 56] ---
  logDataToExcel();
  
  delay(100); // Stability delay
}

// --- Automatic Mode Logic [cite: 45] ---
void runAutomaticMode() {
  // Read LDR values [cite: 46]
  int tl = analogRead(LDR_TL_PIN);
  int tr = analogRead(LDR_TR_PIN);
  int bl = analogRead(LDR_BL_PIN);
  int br = analogRead(LDR_BR_PIN);

  // Calculate Averages [cite: 47]
  int avgTop = (tl + tr) / 2;
  int avgBot = (bl + br) / 2;
  int avgLeft = (tl + bl) / 2;
  int avgRight = (tr + br) / 2;
  int avgSum = (avgTop + avgBot + avgLeft + avgRight) / 4;

  // Check for Night Mode [cite: 51, 52]
  if (avgSum < NIGHT_LIMIT) {
    // Return to sunrise position
    servoLR.write(0);   // East [cite: 53]
    servoUD.write(30);  // Horizon [cite: 53]
    return;
  }

  // Calculate Differences
  int diffElev = avgTop - avgBot;
  int diffAzi = avgRight - avgLeft; // [cite: 47]

  // --- Azimuth Control (Left-Right) ---
  if (abs(diffAzi) > TOLERANCE) { // [cite: 48]
    if (diffAzi > 0) {
      posLR = posLR + 1; // Move Right [cite: 50]
    } else {
      posLR = posLR - 1; // Move Left
    }
  }

  // --- Elevation Control (Up-Down) ---
  if (abs(diffElev) > TOLERANCE) { // [cite: 51]
    if (diffElev > 0) {
      posUD = posUD + 1; // Move Up
    } else {
      posUD = posUD - 1; // Move Down
    }
  }

  // Constrain angles to servo limits (0-180)
  posLR = constrain(posLR, 0, 180);
  posUD = constrain(posUD, 0, 180);

  // Write to Servos [cite: 54]
  servoLR.write(posLR);
  servoUD.write(posUD);
}

// --- Manual Mode Logic [cite: 41, 43] ---
void runManualMode() {
  // Check Axis Switch Button [cite: 44]
  int btnAxisState = digitalRead(BTN_AXIS_PIN);
  if (btnAxisState == LOW && lastBtnAxisState == HIGH) {
    manualControlLR = !manualControlLR; // Toggle controlled servo
    delay(200);
  }
  lastBtnAxisState = btnAxisState;

  // Read Potentiometer
  int potVal = analogRead(POT_PIN);
  int angle = map(potVal, 0, 1023, 0, 180);

  if (manualControlLR) {
    servoLR.write(angle);
    posLR = angle; // Update current pos for smooth transition back to auto
  } else {
    servoUD.write(angle);
    posUD = angle;
  }
}

// --- Data Logging Logic [cite: 55, 60] ---
void logDataToExcel() {
  // Read Voltage from Divider
  // Note: ADC maps 0-5V to 0-1023. 
  // If using a voltage divider for >5V panels, adjust logic below.
  // Assuming direct scale or 1:1 for small panel logic mentioned in doc.
  float sensorValue = analogRead(VOLT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0); 

  // Compute Current (I = V / R) and Power (P = V * I)
  // [cite: 27] states current is calculated since resistor is known.
  float current = (voltage / LOAD_RESISTANCE) * 1000; // in mA
  float power = voltage * current; // in mW

  // Print to Serial for PLX-DAQ
  // Format: "DATA,TIME,Mode,Volts,Current,Power"
  Serial.print("DATA,TIME,");
  
  if (isAutoMode) Serial.print("Auto");
  else Serial.print("Manual");
  
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(current);
  Serial.print(",");
  Serial.println(power);
}
