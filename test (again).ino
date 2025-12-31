/*
 * Project Group 4 SEMI - Dual-Axis Solar Tracker (Continuous LR Servo Version)
 * Hardware: Arduino Uno R3, 1 Standard Servo (UD), 1 Continuous Servo (LR)
 * 4 LDRs, Potentiometer, 2 Pushbuttons
 * Features: Auto/Manual modes, PLX-DAQ Excel Data Logging
 * Note: Left-Right Servo is CONTINUOUS. Up-Down is STANDARD.
 * Based on project documentation [cite: 2, 3, 20]
 */

#include <Servo.h>

// --- Pin Definitions [cite: 22, 23, 25, 27] ---
#define LDR_TL_PIN A0  // Top-Left LDR
#define LDR_TR_PIN A1  // Top-Right LDR
#define LDR_BL_PIN A2  // Bottom-Left LDR
#define LDR_BR_PIN A3  // Bottom-Right LDR

#define POT_PIN A4     // Potentiometer for Manual Control
#define VOLT_PIN A5    // PV Panel Voltage Divider Input

#define SERVO_LR_PIN 6 // Left-Right Servo (Continuous Rotation) [cite: 23]
#define SERVO_UD_PIN 5 // Up-Down Servo (Standard Positional) [cite: 23]

#define BTN_MODE_PIN 12 // Switch between Auto/Manual [cite: 26]
#define BTN_AXIS_PIN 11 // Switch between LR/UD Servo in Manual [cite: 25]

// --- Constants & Configuration ---
#define TOLERANCE 50    // Higher tolerance for continuous servo to stop jitter
#define NIGHT_LIMIT 8   // Low light threshold [cite: 51]
#define LOAD_RESISTANCE 10.0 // Value of load resistor in Ohms

Servo servoLR; // Continuous (Speed controlled: 0=Left, 90=Stop, 180=Right)
Servo servoUD; // Standard (Angle controlled: 0-180)

// --- Variables ---
bool isAutoMode = true;      // true = Automatic, false = Manual
bool manualControlLR = true; // true = Controlling Left-Right, false = Up-Down

int posUD = 45; // Initial position for Up-Down servo

// Button State Tracking
int lastBtnModeState = HIGH;
int lastBtnAxisState = HIGH;

void setup() {
  Serial.begin(9600);
  
  // Excel PLX-DAQ Header
  Serial.println("CLEARDATA"); 
  Serial.println("LABEL,Time,Mode,Voltage(V),Current(mA),Power(mW)"); 

  servoLR.attach(SERVO_LR_PIN);
  servoUD.attach(SERVO_UD_PIN);
  
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_AXIS_PIN, INPUT_PULLUP);
  
  // Initial Positions
  servoUD.write(posUD); // Set vertical to 45 deg
  servoLR.write(90);    // STOP continuous servo (90 is usually stop)
  
  delay(500);
}

void loop() {
  // --- 1. Check Mode Switch Button [cite: 42] ---
  int btnModeState = digitalRead(BTN_MODE_PIN);
  if (btnModeState == LOW && lastBtnModeState == HIGH) {
    isAutoMode = !isAutoMode; 
    servoLR.write(90); // Safety stop when switching modes
    delay(200); 
  }
  lastBtnModeState = btnModeState;

  // --- 2. Execute Mode Logic ---
  if (isAutoMode) {
    runAutomaticMode();
  } else {
    runManualMode();
  }

  // --- 3. Data Logging [cite: 56] ---
  logDataToExcel();
  
  delay(100); 
}

// --- Automatic Mode Logic ---
void runAutomaticMode() {
  int tl = analogRead(LDR_TL_PIN);
  int tr = analogRead(LDR_TR_PIN);
  int bl = analogRead(LDR_BL_PIN);
  int br = analogRead(LDR_BR_PIN);

  int avgTop = (tl + tr) / 2;
  int avgBot = (bl + br) / 2;
  int avgLeft = (tl + bl) / 2;
  int avgRight = (tr + br) / 2;
  int avgSum = (avgTop + avgBot + avgLeft + avgRight) / 4;

  // Night Mode [cite: 51]
  if (avgSum < NIGHT_LIMIT) {
    servoUD.write(30); 
    // For continuous servo, we can't easily "return to 0" without a sensor.
    // We just stop it to save power.
    servoLR.write(90); 
    return;
  }

  int diffElev = avgTop - avgBot;
  int diffAzi = avgRight - avgLeft; 

  // --- Continuous Left-Right Logic ---
  // Instead of angles, we pulse the motor for short bursts
  if (abs(diffAzi) > TOLERANCE) {
    if (diffAzi > 0) {
       // Sun is to the RIGHT
       servoLR.write(180); // Full speed one way
       delay(50);          // Spin for very short time
       servoLR.write(90);  // Stop
    } else {
       // Sun is to the LEFT
       servoLR.write(0);   // Full speed other way
       delay(50);          // Spin for very short time
       servoLR.write(90);  // Stop
    }
  } else {
    servoLR.write(90); // Ensure stopped if within tolerance
  }

  // --- Standard Up-Down Logic ---
  if (abs(diffElev) > TOLERANCE) { 
    if (diffElev > 0) {
      posUD++; 
    } else {
      posUD--; 
    }
  }
  posUD = constrain(posUD, 0, 180);
  servoUD.write(posUD);
}

// --- Manual Mode Logic [cite: 41] ---
void runManualMode() {
  int btnAxisState = digitalRead(BTN_AXIS_PIN);
  if (btnAxisState == LOW && lastBtnAxisState == HIGH) {
    manualControlLR = !manualControlLR; 
    servoLR.write(90); // Stop LR motor when switching control
    delay(200);
  }
  lastBtnAxisState = btnAxisState;

  int potVal = analogRead(POT_PIN);

  if (manualControlLR) {
    // Continuous Control: Pot center (approx 512) stops motor.
    // Left (<500) spins left, Right (>524) spins right.
    if (potVal > 550) {
      servoLR.write(180); // Spin Right
    } else if (potVal < 470) {
      servoLR.write(0);   // Spin Left
    } else {
      servoLR.write(90);  // Stop (Deadzone in middle)
    }
    
  } else {
    // Standard Control: Map pot directly to angle
    int angle = map(potVal, 0, 1023, 0, 180);
    servoUD.write(angle);
    posUD = angle;
  }
}

// --- Data Logging Logic [cite: 55] ---
void logDataToExcel() {
  float sensorValue = analogRead(VOLT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0); 
  float current = (voltage / LOAD_RESISTANCE) * 1000; 
  float power = voltage * current; 

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
