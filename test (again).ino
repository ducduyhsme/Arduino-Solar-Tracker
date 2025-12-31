/*
 * Project Group 4 SEMI - Dual-Axis Solar Tracker (CONTINUOUS SERVO TEST)
 * Modified to create an infinite loop: 2s CW -> 2s CCW
 */

#include <Servo.h>

// --- Pin Definitions ---
#define LDR_TL_PIN A0 
#define LDR_TR_PIN A1 
#define LDR_BL_PIN A2 
#define LDR_BR_PIN A3 
#define POT_PIN A4     
#define VOLT_PIN A5    
#define SERVO_LR_PIN 6 // Continuous Rotation Servo
#define SERVO_UD_PIN 5 
#define BTN_MODE_PIN 12 
#define BTN_AXIS_PIN 11 

// --- Constants ---
#define LOAD_RESISTANCE 10.0 

Servo servoLR; 
Servo servoUD; 

void setup() {
  Serial.begin(9600);
  Serial.println("CLEARDATA"); 
  Serial.println("LABEL,Time,Action,Voltage(V)"); 

  // Attach Servos
  servoLR.attach(SERVO_LR_PIN);
  servoUD.attach(SERVO_UD_PIN);
  
  // Set Pin Modes
  pinMode(BTN_MODE_PIN, INPUT_PULLUP);
  pinMode(BTN_AXIS_PIN, INPUT_PULLUP);
  
  // Initialize UD servo to a fixed position (e.g., 45 degrees) so it stays still
  servoUD.write(45); 

  // Ensure LR servo is stopped initially
  servoLR.write(90); 
  
  delay(1000); // Wait 1 second before starting loop
}

void loop() {
  // --- STEP 1: Rotate Clockwise (approx 180 deg) ---
  // For continuous servo: 180 is usually max speed one way, 0 is max speed the other.
  // If this rotates the wrong way, swap 180 with 0.
  Serial.println("MSG,Rotating CW");
  servoLR.write(180);  
  
  // Run for 2 seconds
  delay(2000); 

  // --- STEP 2: Stop Briefly ---
  // It is good practice to stop briefly before reversing direction to protect gears
  servoLR.write(90);   // 90 is Stop
  delay(500);          // Stop for 0.5 seconds

  // --- STEP 3: Rotate Counter-Clockwise (approx 180 deg) ---
  Serial.println("MSG,Rotating CCW");
  servoLR.write(0);    // Opposite direction
  
  // Run for 2 seconds
  delay(2000); 

  // --- STEP 4: Stop Briefly ---
  servoLR.write(90);   // Stop
  delay(500); 

  // --- Optional: Log Data during the loop ---
  // (This will only log once per cycle)
  logDataToExcel();
}

// --- Data Logging Logic ---
void logDataToExcel() {
  float sensorValue = analogRead(VOLT_PIN);
  float voltage = sensorValue * (5.0 / 1023.0); 

  Serial.print("DATA,TIME,Cycling,");
  Serial.println(voltage);
}
