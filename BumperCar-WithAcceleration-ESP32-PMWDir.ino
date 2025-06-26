// GoBabyGo - ESP32 Version - Arcade Style Control with Ramping
// Samuel Hartpence, 2025

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <CytronMotorDriver.h> // Corrected library include

BluetoothSerial SerialBT;

// --- Joystick Pin Definitions ---
const int joystickXPin = 32;
const int joystickYPin = 33;

// --- MDDS30 Pin Definitions for PWM-DIR mode ---
// Left Motor Pins
const int LEFT_PWM_PIN = 18; // Connect to AN1 on MDDS30
const int LEFT_DIR_PIN = 17; // Connect to IN1 on MDDS30

// Right Motor Pins
const int RIGHT_PWM_PIN = 19; // Connect to AN2 on MDDS30
const int RIGHT_DIR_PIN = 16; // Connect to IN2 on MDDS30

// Create motor driver objects
// The first parameter is the mode (PWM_DIR for Sign-Magnitude)
// The second is the PWM pin, the third is the Direction pin
// PWM_DIR is an enum provided by the CytronMotorDriver library
CytronMD motorLeft(PWM_DIR, LEFT_PWM_PIN, LEFT_DIR_PIN); // Changed from CytronMotorDriver
CytronMD motorRight(PWM_DIR, RIGHT_PWM_PIN, RIGHT_DIR_PIN); // Changed from CytronMotorDriver

// --- Joystick Calibration & Control Parameters ---
int centerX, centerY;
const int deadzone = 50; // Adjust as needed
const int analogMax = 4095; // ESP32 ADC resolution (12-bit)

// Ramping parameters
const int accelerationRate = 5; // How much speed changes per loop iteration during acceleration
const int decelerationRate = 10; // How much speed changes per loop iteration during deceleration

// Current motor power levels (0-255 for full forward, -255 for full reverse)
int currentLeftPower = 0;
int currentRightPower = 0;

// Function to process raw joystick axis values
int processAxis(int rawValue, int centerValue) {
  int processedValue = rawValue - centerValue;

  // Apply deadzone
  if (abs(processedValue) < deadzone) {
    return 0;
  }

  // Scale to -255 to 255 range
  if (processedValue > 0) {
    return map(processedValue, deadzone, (analogMax - centerValue), 0, 255);
  } else {
    return map(processedValue, -(analogMax - centerValue), -deadzone, -255, 0);
  }
}

// Function to average analog readings for calibration
int averageAnalog(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  return sum / 10;
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("GoBabyGo_ESP32");

  Serial.println("GoBabyGo ESP32 Initializing...");

  pinMode(joystickXPin, INPUT);
  pinMode(joystickYPin, INPUT);

  // Calibrate joystick on startup
  // Ensure joysticks are at center when powering up/resetting the ESP32
  centerX = averageAnalog(joystickXPin);
  centerY = averageAnalog(joystickYPin);

  Serial.print("Calibrated centerX: ");
  Serial.print(centerX);
  Serial.print(" | centerY: ");
  Serial.println(centerY);

  // Initialize motors to neutral/stop state using the library
  // The setSpeed(0) will send 0% duty cycle on PWM and the correct DIR state
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  delay(5000); // Small delay to ensure neutral is sent and MDDS30 is ready

  Serial.println("Setup Complete. Ready for joystick input.");
}

void loop() {
  int rawX = analogRead(joystickXPin);
  int rawY = analogRead(joystickYPin);

  int x = processAxis(rawX, centerX);
  int y = processAxis(rawY, centerY);

  // Calculate target power for each motor based on joystick input
  // Differential drive logic:
  // Forward/Backward (Y-axis) affects both
  // Left/Right (X-axis) subtracts from one, adds to other for turning
  int targetLeftPower = constrain(y + x, -255, 255);
  int targetRightPower = constrain(y - x, -255, 255);

  // --- Ramping Logic ---
  // Smoothly adjust current motor power towards target power
  if (currentLeftPower < targetLeftPower) {
    currentLeftPower += accelerationRate;
  } else if (currentLeftPower > targetLeftPower) {
    currentLeftPower -= decelerationRate;
  }
  // Ensure we don't overshoot the target due to large ramping steps
  if (abs(currentLeftPower - targetLeftPower) < max(accelerationRate, decelerationRate)) {
      currentLeftPower = targetLeftPower;
  }

  if (currentRightPower < targetRightPower) {
    currentRightPower += accelerationRate;
  } else if (currentRightPower > targetRightPower) {
    currentRightPower -= decelerationRate;
  }
  if (abs(currentRightPower - targetRightPower) < max(accelerationRate, decelerationRate)) {
      currentRightPower = targetRightPower;
  }

  // Constrain final power values to valid range (-255 to 255)
  currentLeftPower = constrain(currentLeftPower, -255, 255);
  currentRightPower = constrain(currentRightPower, -255, 255);

  // Set motor speed using the CytronMotorDriver library
  // The library handles converting the -255 to 255 range to appropriate PWM/DIR signals
  motorLeft.setSpeed(currentLeftPower);
  motorRight.setSpeed(currentRightPower);

  // --- Debug Output ---
  String debug = "RawX: " + String(rawX) + " | RawY: " + String(rawY) +
                 " | ProcX: " + String(x) + " | ProcY: " + String(y) +
                 " | TgtL: " + String(targetLeftPower) + " | TgtR: " + String(targetRightPower) +
                 " | CurrL: " + String(currentLeftPower) + " | CurrR: " + String(currentRightPower);

  Serial.println(debug);
  SerialBT.println(debug);

  delay(50); // Small delay to prevent overwhelming the serial buffer and for loop stability
}