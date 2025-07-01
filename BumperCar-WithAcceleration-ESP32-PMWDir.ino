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
const int deadzone = 100; // Adjust as needed
const int analogMax = 4095; // ESP32 ADC resolution (12-bit)

// Ramping parameters
const int accelerationRate = 5; // How much speed changes per loop iteration during acceleration
const int decelerationRate = 40; // How much speed changes per loop iteration during deceleration

// Current motor power levels (0-255 for full forward, -255 for full reverse)
int currentLeftPower = 0;
int currentRightPower = 0;

// --- Filtering Parameters for live readings (Moving Average) ---
const int FILTER_SAMPLES = 10; // Number of samples for the moving average filter
int x_readings[FILTER_SAMPLES]; // Array to hold last X readings
int y_readings[FILTER_SAMPLES]; // Array to hold last Y readings
int x_read_index = 0; // Current index in the X array
int y_read_index = 0; // Current index in the Y array
long x_sum = 0; // Sum of X readings for average
long y_sum = 0; // Sum of Y readings for average

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

// Function to average analog readings for calibration (used once in setup())
int averageAnalog(int pin) {
  long sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += analogRead(pin);
    delay(1); // Small delay to allow ADC to settle
  }
  return sum / 100;
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
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
  delay(5000); // Small delay to ensure neutral is sent and MDDS30 is ready

  Serial.println("Setup Complete. Ready for joystick input.");

  // --- Initialize filter arrays for loop() ---
  // Fill filter arrays with initial center values to prevent glitches on startup
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    x_readings[i] = centerX;
    y_readings[i] = centerY;
  }
  x_sum = (long)centerX * FILTER_SAMPLES; // Cast to long to prevent overflow
  y_sum = (long)centerY * FILTER_SAMPLES;
}

void loop() {
  // --- Live Analog Read with Moving Average Filter ---
  // Read new raw value from joystick
  int newRawX = analogRead(joystickXPin);
  int newRawY = analogRead(joystickYPin);

  // Update the moving average filter for X-axis
  x_sum = x_sum - x_readings[x_read_index] + newRawX; // Subtract oldest, add newest
  x_readings[x_read_index] = newRawX; // Store newest reading
  x_read_index = (x_read_index + 1) % FILTER_SAMPLES; // Move to next index (wraps around)
  int filteredX = x_sum / FILTER_SAMPLES; // Calculate new average

  // Update the moving average filter for Y-axis
  y_sum = y_sum - y_readings[y_read_index] + newRawY;
  y_readings[y_read_index] = newRawY;
  y_read_index = (y_read_index + 1) % FILTER_SAMPLES;
  int filteredY = y_sum / FILTER_SAMPLES;

  // Use filtered values for processing
  int x = processAxis(filteredX, centerX);
  int y = processAxis(filteredY, centerY);

  // Calculate target power for each motor based on joystick input
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

  // --- IMPORTANT NEW BLOCK: Force stop if joystick is truly neutral ---
  // This explicitly sets motor power to 0 if the processed (filtered and deadzoned)
  // joystick input is exactly at the center. This should eliminate any lingering
  // small signals that might keep one motor "on" according to the controller.
  if (x == 0 && y == 0) {
    currentLeftPower = 0;
    currentRightPower = 0;
  }

  // Set motor speed using the CytronMotorDriver library
  motorLeft.setSpeed(currentLeftPower);
  motorRight.setSpeed(currentRightPower);

  // --- Debug Output ---
  // Added filteredX and filteredY to debug output for monitoring
  String debug = "RawX: " + String(newRawX) + " | RawY: " + String(newRawY) +
                 " | FltX: " + String(filteredX) + " | FltY: " + String(filteredY) +
                 " | ProcX: " + String(x) + " | ProcY: " + String(y) +
                 " | TgtL: " + String(targetLeftPower) + " | TgtR: " + String(targetRightPower) +
                 " | CurrL: " + String(currentLeftPower) + " | CurrR: " + String(currentRightPower);

  Serial.println(debug);
  SerialBT.println(debug);

  delay(50); // Small delay to prevent overwhelming the serial buffer and for loop stability
}