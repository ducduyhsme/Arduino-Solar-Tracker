/*
 * Project Group 4 SEMI - Dual-Axis Solar Tracker
 * Hardware:  Arduino Uno R3, 2 Servos (LR = Continuous, UD = Standard), 4 LDRs
 * Features: Automatic Mode (Flowchart-based), Manual Mode, Excel Data Logging
 * 
 * FIX APPLIED: LDR pin assignments rotated 90 degrees clockwise
 * to correct sensor orientation mismatch
 */

#include <Servo.h>

// --- Pin Definitions ---
// =============================================================
// FIXED: LDR Pin Assignments - Rotated 90 degrees clockwise
// =============================================================
// Original mapping (what code expected):
//   TL=A0  TR=A1
//   BL=A2  BR=A3
//
// Your physical sensor layout (rotated 90° left from code's expectation):
// What code called "Top-Left" is actually your "Top-Right"
// What code called "Top-Right" is actually your "Bottom-Right"
// What code called "Bottom-Left" is actually your "Top-Left"
// What code called "Bottom-Right" is actually your "Bottom-Left"
//
// NEW mapping to correct the 90-degree rotation:
#define LDR_TL_PIN A2  // Physical Top-Left (was connected as Bottom-Left)
#define LDR_TR_PIN A0  // Physical Top-Right (was connected as Top-Left)
#define LDR_BL_PIN A3  // Physical Bottom-Left (was connected as Bottom-Right)
#define LDR_BR_PIN A1  // Physical Bottom-Right (was connected as Top-Right)

#define POT_PIN A4     // Potentiometer for Manual Control
#define VOLT_PIN A5    // PV Panel Voltage Divider Input

#define SERVO_LR_PIN 6 // Left-Right Servo (Continuous Rotation)
#define SERVO_UD_PIN 5 // Up-Down Servo (Standard 180)

#define BTN_MODE_PIN 12 // Switch between Auto/Manual
#define BTN_AXIS_PIN 11 // Switch between LR/UD Servo in Manual

// --- Constants & Configuration ---
#define TOLERANCE 10    // Value from flowchart
#define NIGHT_LIMIT 8   // Value from flowchart
#define LOAD_RESISTANCE 10. 0 

// Speeds for Continuous Servo (90 is Stop)
#define LR_SPEED_RIGHT 80 // Move Right (Adjust for speed)
#define LR_SPEED_LEFT 100 // Move Left (Adjust for speed)
#define LR_STOP 90

// --- Left-Right Servo Position Limits ---
// These constants define the rotation limits for the LR servo
#define LR_MAX_RIGHT 90   // Maximum 90 degrees to the right from center
#define LR_MAX_LEFT -90   // Maximum 90 degrees to the left from center
#define LR_CENTER 0       // Center position (starting point)

Servo servoLR; // Continuous Rotation Servo
Servo servoUD; // Standard Servo

// --- Variables ---
bool isAutoMode = true;      
bool manualControlLR = true; 

int posUD = 45; // Initial position for UD

// --- Position tracking for LR continuous servo ---
// This variable tracks the estimated position of the LR servo in degrees
// Positive values = rotated right, Negative values = rotated left
int posLR = LR_CENTER;

// --- Timing variables for position estimation ---
// Used to calculate how far the servo has rotated based on time
unsigned long lastLRUpdateTime = 0;
#define DEGREES_PER_MS 0.06  // Estimated rotation speed:  ~60 degrees per second (adjust based on your servo)

// Button State Tracking
int lastBtnModeState = HIGH;
int lastBtnAxisState = HIGH;

// --- Function declaration for LR servo control with limits ---
void moveLRServoWithLimits(int direction);

void setup() {
  Serial.begin(9600);
  Serial.println("CLEARDATA"); 
  Serial.println("LABEL,Time,Mode,Voltage(V),Current(mA),Power(mW),LR_Pos,UD_Pos"); 

  servoLR.attach(SERVO_LR_PIN);
  servoUD.attach(SERVO_UD_PIN);
  
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_AXIS_PIN, INPUT_PULLUP);
  
  // Initial positions
  servoLR. write(LR_STOP); 
  servoUD.write(posUD);
  
  // Initialize timing for position tracking
  lastLRUpdateTime = millis();
  
  delay(500);
  
  // =============================================================
  // DEBUG: Print LDR pin mapping at startup
  // =============================================================
  Serial.println("--- LDR Pin Mapping (FIXED) ---");
  Serial.println("Physical Top-Left    -> A2");
  Serial.println("Physical Top-Right   -> A0");
  Serial.println("Physical Bottom-Left -> A3");
  Serial.println("Physical Bottom-Right-> A1");
  Serial.println("-------------------------------");
}

void loop() {
  // --- 1. Check Mode Switch Button ---
  int btnModeState = digitalRead(BTN_MODE_PIN);
  if (btnModeState == LOW && lastBtnModeState == HIGH) {
    isAutoMode = !isAutoMode; 
    servoLR.write(LR_STOP); // Safety stop
    delay(200); 
  }
  lastBtnModeState = btnModeState;

  // --- 2. Execute Mode Logic ---
  if (isAutoMode) {
    runAutomaticMode();
  } else {
    runManualMode();
  }

  // --- 3. Data Logging to Excel ---
  logDataToExcel();
  
  delay(50); 
}

// ============================================================
// FUNCTION: Move LR Servo with 90-degree limits in each direction
// ============================================================
// This function controls the Left-Right continuous rotation servo
// while enforcing position limits of 90 degrees right and 90 degrees left
// 
// Parameters:
//   direction: 1 = move right, -1 = move left, 0 = stop
// 
// How it works:
//   - Tracks estimated position based on rotation time
//   - Stops the servo when it reaches the limit (±90 degrees)
//   - Allows movement in the opposite direction when at a limit
// ============================================================
void moveLRServoWithLimits(int direction) {
  // Get current time for position calculation
  unsigned long currentTime = millis();
  
  // Calculate time elapsed since last update
  unsigned long elapsedTime = currentTime - lastLRUpdateTime;
  
  // Get the current servo command to determine if it was moving
  // and update the estimated position accordingly
  static int lastDirection = 0;  // Tracks previous movement direction
  
  // Update position estimate based on previous movement
  if (lastDirection == 1) {
    // Was moving right - add to position
    posLR = posLR + (int)(elapsedTime * DEGREES_PER_MS);
  } else if (lastDirection == -1) {
    // Was moving left - subtract from position
    posLR = posLR - (int)(elapsedTime * DEGREES_PER_MS);
  }
  
  // Constrain the estimated position to valid range
  posLR = constrain(posLR, LR_MAX_LEFT, LR_MAX_RIGHT);
  
  // Update the timestamp
  lastLRUpdateTime = currentTime;
  
  // Now decide what to do based on requested direction and current position
  if (direction == 0) {
    // Stop requested
    servoLR.write(LR_STOP);
    lastDirection = 0;
  }
  else if (direction == 1) {
    // Move RIGHT requested - check if we've reached the right limit
    if (posLR >= LR_MAX_RIGHT) {
      // At right limit - stop the servo, cannot go further right
      servoLR. write(LR_STOP);
      lastDirection = 0;
    } else {
      // Not at limit - allow movement to the right
      servoLR.write(LR_SPEED_RIGHT);
      lastDirection = 1;
    }
  }
  else if (direction == -1) {
    // Move LEFT requested - check if we've reached the left limit
    if (posLR <= LR_MAX_LEFT) {
      // At left limit - stop the servo, cannot go further left
      servoLR.write(LR_STOP);
      lastDirection = 0;
    } else {
      // Not at limit - allow movement to the left
      servoLR.write(LR_SPEED_LEFT);
      lastDirection = -1;
    }
  }
}

// ============================================================
// FUNCTION: Reset LR position to center (for night mode)
// ============================================================
// This function resets the LR position tracking to center
// Call this when returning to initial position at night
// ============================================================
void resetLRPosition() {
  posLR = LR_CENTER;
  lastLRUpdateTime = millis();
}

// --- Automatic Mode Logic based on Flowchart ---
void runAutomaticMode() {
  // =============================================================
  // 1. Read the analog value from each LDR sensor
  // =============================================================
  // These readings now correctly map to physical sensor positions
  // after the pin reassignment fix
  int topleft = analogRead(LDR_TL_PIN);   // Now reads from A2 (physical top-left)
  int topright = analogRead(LDR_TR_PIN);  // Now reads from A0 (physical top-right)
  int botleft = analogRead(LDR_BL_PIN);   // Now reads from A3 (physical bottom-left)
  int botright = analogRead(LDR_BR_PIN);  // Now reads from A1 (physical bottom-right)

  // =============================================================
  // 2. Calculate the average values
  // =============================================================
  // Average of top sensors (for elevation calculation)
  int avgtop = (topright + topleft) / 2;
  // Average of bottom sensors (for elevation calculation)
  int avgbot = (botright + botleft) / 2;
  // Average of right sensors (for azimuth calculation)
  int avgright = (topright + botright) / 2;
  // Average of left sensors (for azimuth calculation)
  int avgleft = (topleft + botleft) / 2;

  // =============================================================
  // 3. Calculate the differences (Azimuth & elevation)
  // =============================================================
  // Elevation difference:  positive = more light on top, panel should tilt UP
  int diffelev = avgtop - avgbot;
  // Azimuth difference: positive = more light on right, panel should rotate RIGHT
  int diffazi = avgright - avgleft;
  // Overall average for night detection
  int avgsum = (avgtop + avgbot + avgright + avgleft) / 4;

  // =============================================================
  // DEBUG: Print sensor values to Serial Monitor for troubleshooting
  // =============================================================
  // Uncomment the following lines to see sensor readings in real-time: 
  /*
  Serial.print("TL: "); Serial.print(topleft);
  Serial.print(" TR: "); Serial.print(topright);
  Serial.print(" BL:"); Serial.print(botleft);
  Serial.print(" BR:"); Serial.print(botright);
  Serial.print(" | diffAzi:"); Serial.print(diffazi);
  Serial.print(" diffElev:"); Serial.print(diffelev);
  Serial.print(" avgSum:"); Serial.println(avgsum);
  */

  // =============================================================
  // 4. Check Night Mode (avgsum < 8)
  // =============================================================
  if (avgsum < NIGHT_LIMIT) {
    // "Rotate the Servo Motors to the initial position"
    servoLR.write(LR_STOP); // Stop LR servo
    resetLRPosition();      // Reset LR position tracking to center
    posUD = 30;             // Set UD to sunrise/horizon position
    servoUD.write(posUD);
    return; // Go back to Start
  }

  // =============================================================
  // 5. Azimuth (Left-Right) Control with position limits
  // =============================================================
  // Check |diffazi| <= 10
  if (abs(diffazi) <= TOLERANCE) {
    // "Stop the Left-Right Servo Motor" - light is balanced horizontally
    moveLRServoWithLimits(0);  // 0 = stop
  } 
  else {
    // Check diffazi > 0
    if (diffazi > 0) {
      // More light on right side -> "Left-Right Servo Motor move PV panel Right"
      moveLRServoWithLimits(1);  // 1 = move right
    } else {
      // More light on left side -> "Left-Right Servo Motor move PV panel Left"
      moveLRServoWithLimits(-1); // -1 = move left
    }
  }

  // =============================================================
  // 6. Elevation (Up-Down) Control
  // =============================================================
  // Check |diffelev| <= 10
  if (abs(diffelev) <= TOLERANCE) {
    // "Stop the Up-Down Servo Motor" - light is balanced vertically
    // Do nothing (position doesn't change)
  } 
  else {
    // Check diffelev > 0
    if (diffelev > 0) {
      // More light on top -> "Up-Down Servo Motor move PV panel Up"
      posUD = posUD + 1;
    } else {
      // More light on bottom -> "Up-Down Servo Motor move PV panel Down"
      posUD = posUD - 1;
    }
    // Write the new position for the standard servo
    posUD = constrain(posUD, 0, 180);
    servoUD.write(posUD);
  }
  // The logic then goes back to Start
}

// --- Manual Mode Logic ---
void runManualMode() {
  int btnAxisState = digitalRead(BTN_AXIS_PIN);
  if (btnAxisState == LOW && lastBtnAxisState == HIGH) {
    manualControlLR = !manualControlLR; 
    servoLR. write(LR_STOP); // Stop when switching axis
    delay(200);
  }
  lastBtnAxisState = btnAxisState;

  int potVal = analogRead(POT_PIN);
  int angle = map(potVal, 0, 1023, 0, 180);

  if (manualControlLR) {
    // Potentiometer controls speed/direction of continuous servo with limits
    if (angle > 95) {
      // Moving left (pot turned right of center)
      moveLRServoWithLimits(-1);
    } else if (angle < 85) {
      // Moving right (pot turned left of center)
      moveLRServoWithLimits(1);
    } else {
      // Dead zone around center - stop
      moveLRServoWithLimits(0);
    }
  } else {
    // Potentiometer controls position of standard servo
    servoUD.write(angle);
    posUD = angle;
  }
}

// --- Data Logging Logic ---
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
  Serial.print(power);
  Serial.print(",");
  Serial.print(posLR);  // Added:  LR position for debugging
  Serial.print(",");
  Serial.println(posUD); // Added: UD position for debugging
}
