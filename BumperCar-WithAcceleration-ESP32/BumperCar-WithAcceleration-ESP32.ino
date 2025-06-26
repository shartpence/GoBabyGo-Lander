// GoBabyGo - ESP32 Version - Arcade Style Control with Ramping
// Samuel Hartpence, 2025

#include <Arduino.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// --- PWM Configuration ---
// Standard hobby servo PWM frequency
const int PWM_FREQ = 50;        // Hz (20ms period)
// 16-bit resolution means 2^16 = 65536 steps (0 to 65535)
const int PWM_RESOLUTION = 16;  
const int MAX_PWM_VALUE = pow(2, PWM_RESOLUTION) - 1; // 65535

// --- Pin Definitions ---
const int joystickXPin = 32;
const int joystickYPin = 33;

const int leftMotorPin = 18; // Connect to RC1 on MDDS30
const int rightMotorPin = 19; // Connect to RC2 on MDDS30

// --- Joystick Calibration & Deadzone ---
const int deadzone = 20; // Joystick analogRead values within this range of center are ignored
int centerX = 530;       // Will be calibrated in setup()
int centerY = 531;       // Will be calibrated in setup()

// --- Ramping Parameters ---
const int accelerationRate = 5;  // Adjust for desired acceleration smoothness
const int decelerationRate = 20; // Adjust for desired deceleration smoothness (often faster than accel)

// --- Current Motor Power States (for ramping) ---
int currentLeftPower = 0;  // Range -255 to 255
int currentRightPower = 0; // Range -255 to 255

// --- Functions ---

// Maps motor power (-255 to 255) to a microsecond pulse width (1000-2000 µs)
// Then maps the pulse width to the appropriate 16-bit duty cycle value for ledcWrite
int pwmFromPower(int power) {
  // Map power (-255 to 255) to standard RC servo microseconds (1000 to 2000)
  // 1000 us = full reverse, 1500 us = neutral, 2000 us = full forward
  int microseconds = map(power, -255, 255, 1000, 2000);

  // Map microseconds to 16-bit duty cycle value for ESP32's ledcWrite
  // Period for 50Hz PWM is 20,000 microseconds (1 second / 50 Hz = 0.02 seconds = 20,000 µs)
  // Duty Cycle = (Pulse Width in µs / Period in µs) * MAX_PWM_VALUE
  // E.g., for 1000µs: (1000 / 20000) * 65535 = 0.05 * 65535 = 3276.75
  // E.g., for 1500µs: (1500 / 20000) * 65535 = 0.075 * 65535 = 4915.125
  // E.g., for 2000µs: (2000 / 20000) * 65535 = 0.10 * 65535 = 6553.5
  return map(microseconds, 1000, 2000, 3277, 6554); // Using rounded values for clarity
}


void setup() {
  Serial.begin(115200); // Higher baud rate for faster debug output
  SerialBT.begin("GoBabyGo_ESP32"); // Initialize Bluetooth Serial

  Serial.println("GoBabyGo ESP32 Initializing...");

  // Initialize PWM channels using the NEW API
  // ledcAttach(pin, freq, resolution_bits)
  ledcAttach(leftMotorPin, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(rightMotorPin, PWM_FREQ, PWM_RESOLUTION);

  // Force ESCs to neutral at startup for calibration/arming
  int neutralPWM = pwmFromPower(0); // Power 0 maps to 1500 µs equivalent
  Serial.print("Sending neutral signal (PWM value: ");
  Serial.print(neutralPWM);
  Serial.println(") to ESCs...");

  // Send neutral for a good amount of time for ESCs to arm.
  // Many ESCs require ~2-5 seconds of neutral signal upon power-up.
  // Use the pin number directly with ledcWrite
  ledcWrite(leftMotorPin, neutralPWM); 
  ledcWrite(rightMotorPin, neutralPWM);
  delay(10000); //for ESC arming

  // Calibrate joystick - read center values after initial power up.
  // This is crucial for consistent deadzone and movement.
  centerX = analogRead(joystickXPin);
  centerY = analogRead(joystickYPin);

  Serial.print("Calibrated centerX: ");
  Serial.print(centerX);
  Serial.print(" | centerY: ");
  Serial.println(centerY);
  Serial.println("Setup Complete. Ready for joystick input.");
}

// Processes raw analog joystick input to a scaled motor power value (-255 to 255)
int processAxis(int raw, int center) {
  int adjusted = raw - center; // Offset from center
  if (abs(adjusted) < deadzone) return 0; // Apply deadzone
  
  // Constrain adjusted value to expected joystick range. 
  // Our joysticks return ~0-1023, center ~512. So +/- 512 from center.
  adjusted = constrain(adjusted, -512, 512); 

  // Map the adjusted joystick range (-512 to 512) to motor power (-255 to 255)
  return map(adjusted, -512, 512, -255, 255);
}

void loop() {
  int rawX = analogRead(joystickXPin);
  int rawY = analogRead(joystickYPin);

  int x = processAxis(rawX, centerX); // Processed X (-255 to 255)
  int y = processAxis(rawY, centerY); // Processed Y (-255 to 255)

  // Store previous processed values for ramping logic
  static int prevX = x;
  static int prevY = y;

  // Calculate target power for each motor based on arcade drive
  // Forward/Backward (Y) + Steering (X)
  int targetLeftPower = constrain(y + x, -255, 255);
  int targetRightPower = constrain(y - x, -255, 255);

  // --- Ramping Logic ---
  // Apply ramping to `currentLeftPower`
  if (currentLeftPower < targetLeftPower) {
    currentLeftPower += accelerationRate;
  } else if (currentLeftPower > targetLeftPower) {
    currentLeftPower -= decelerationRate;
  }
  // If we're very close to target, snap to target to avoid oscillation
  if (abs(currentLeftPower - targetLeftPower) < max(accelerationRate, decelerationRate)) {
      currentLeftPower = targetLeftPower;
  }
  
  // Apply ramping to `currentRightPower`
  if (currentRightPower < targetRightPower) {
    currentRightPower += accelerationRate;
  } else if (currentRightPower > targetRightPower) {
    currentRightPower -= decelerationRate;
  }
  // If we're very close to target, snap to target to avoid oscillation
  if (abs(currentRightPower - targetRightPower) < max(accelerationRate, decelerationRate)) {
      currentRightPower = targetRightPower;
  }

  // Ensure current power values stay within -255 to 255
  currentLeftPower = constrain(currentLeftPower, -255, 255);
  currentRightPower = constrain(currentRightPower, -255, 255);

  // Convert current power to PWM duty cycle values
  int leftPWM_val = pwmFromPower(currentLeftPower);
  int rightPWM_val = pwmFromPower(currentRightPower); 

  // Write PWM signals to motors using the pin number directly
  ledcWrite(leftMotorPin, leftPWM_val);
  ledcWrite(rightMotorPin, rightPWM_val);

  // --- Debug Output ---
  String debug = "RawX: " + String(rawX) + " | RawY: " + String(rawY) +
                 " | ProcX: " + String(x) + " | ProcY: " + String(y) +
                 " | TgtL: " + String(targetLeftPower) + " | TgtR: " + String(targetRightPower) +
                 " | CurrL: " + String(currentLeftPower) + " | CurrR: " + String(currentRightPower) +
                 " | PWM_L: " + String(leftPWM_val) + " | PWM_R: " + String(rightPWM_val);

  Serial.println(debug);
  SerialBT.println(debug);

  // Update previous values for next loop iteration's ramping
  prevX = x;
  prevY = y;

  // Small delay to prevent overwhelming the serial output and for stable PWM
  delay(10);
}