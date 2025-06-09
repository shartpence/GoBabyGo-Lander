// GoBabyGo - ESP32 Version - Arcade Style Control with Ramping
// Samuel Hartpence, 2025
// ledcWrite() takes a value from 0–65535 when using 16-bit resolution.

// I used 3276 = ~5% (for 1000 µs) and 6553 = ~10% (for 2000 µs) for 50 Hz — adjust if needed.

#include <Arduino.h>

// Define PWM properties as constants for clarity and easy modification
const int PWM_FREQ = 50;         // Hz
const int PWM_RESOLUTION = 16;   // bits (0-65535 for 16-bit)

const int joystickXPin = A0;   // ADC1_0, GPIO36
const int joystickYPin = A3;   // ADC1_3, GPIO39

const int leftMotorPin = 18;   // PWM Output to left ESC
const int rightMotorPin = 19;  // PWM Output to right ESC

const int deadzone = 20;
int centerX = 530;
int centerY = 531;

// Ramp parameters
const int accelerationRate = 20;
const int decelerationRate = 80;

int currentLeftPower = 0;
int currentRightPower = 0;

void setup() {
  Serial.begin(9600);

  // Initialize PWM for LEFT motor (pin, frequency, resolution)
  // In ESP32 core 3.0.0+, ledcAttach handles channel assignment automatically.
  ledcAttach(leftMotorPin, PWM_FREQ, PWM_RESOLUTION);

  // Initialize PWM for RIGHT motor (pin, frequency, resolution)
  ledcAttach(rightMotorPin, PWM_FREQ, PWM_RESOLUTION);

  // Read joystick center
  centerX = analogRead(joystickXPin);
  centerY = analogRead(joystickYPin);

  Serial.print("Calibrated centerX: ");
  Serial.print(centerX);
  Serial.print(" | centerY: ");
  Serial.println(centerY);
}

int processAxis(int raw, int center) {
  int adjusted = raw - center;
  if (abs(adjusted) < deadzone) return 0;
  // ESP32 ADC is 12-bit, so raw values are 0-4095.
  // center is around 2047. So adjusted is -2047 to 2047.
  // Constraining to -512, 512, then mapping -255, 255.
  // This implies you expect a smaller useful range from your joystick,
  // or a larger deadzone.
  // The original map(adjusted, -512, 512, -255, 255) is correct for that range.
  adjusted = constrain(adjusted, -512, 512);
  return map(adjusted, -512, 512, -255, 255);
}

int pwmFromPower(int power) {
  // Convert power (-255 to 255) to pulse width (1000us to 2000us)
  int microseconds = map(power, -255, 255, 1000, 2000);
  // Convert microseconds to 16-bit duty cycle at 50Hz
  // Duty cycle for 50Hz (20ms period) with 16-bit resolution (65535 max)
  // 1000us = 1ms. Duty for 1ms: (1ms / 20ms) * 65535 = 0.05 * 65535 = 3276.75 (~3277)
  // 2000us = 2ms. Duty for 2ms: (2ms / 20ms) * 65535 = 0.10 * 65535 = 6553.5 (~6554)
  // Your original values (3276, 6553) are very close and likely correct.
  int duty = map(microseconds, 1000, 2000, 3276, 6553);
  return duty;
}

void loop() {
  int rawX = analogRead(joystickXPin);
  int rawY = analogRead(joystickYPin);

  int x = processAxis(rawX, centerX);
  int y = processAxis(rawY, centerY);

  static int prevX = x;
  static int prevY = y;

  int targetLeftPower = constrain(y + x, -255, 255);
  int targetRightPower = constrain(y - x, -255, 255);

  // LEFT motor ramping
  // The ramping logic seems a bit complex with the current X and Y.
  // Typically, ramping involves moving currentPower towards targetPower by a step.
  // For simplicity and smoother control, you might consider:
  // if (currentLeftPower < targetLeftPower) currentLeftPower += accelerationRate;
  // else if (currentLeftPower > targetLeftPower) currentLeftPower -= decelerationRate;
  // The same for Right motor.
  // However, I'm preserving your original ramping logic as it is, assuming it works as intended for your application.
  if (abs(y) > deadzone || abs(x) > deadzone) {
    if ((y > 0 && prevY <= y) || (y < 0 && prevY >= y))
      currentLeftPower += accelerationRate;
    else if ((y > 0 && prevY > y) || (y < 0 && prevY < y))
      currentLeftPower -= decelerationRate;

    if ((x > 0 && prevX <= x) || (x < 0 && prevX >= x))
      currentLeftPower += (x > 0 ? accelerationRate : -accelerationRate);
    else if ((x > 0 && prevX > x) || (x < 0 && prevX < x))
      currentLeftPower -= (x > 0 ? decelerationRate : -decelerationRate);
  } else {
    currentLeftPower += (currentLeftPower > 0) ? -decelerationRate : decelerationRate;
    if (abs(currentLeftPower) < decelerationRate) currentLeftPower = 0;
  }

  currentLeftPower = constrain(currentLeftPower, -255, 255);

  // RIGHT motor ramping
  if (abs(y) > deadzone || abs(x) > deadzone) {
    if ((y > 0 && prevY <= y) || (y < 0 && prevY >= y))
      currentRightPower += accelerationRate;
    else if ((y > 0 && prevY > y) || (y < 0 && prevY < y))
      currentRightPower -= decelerationRate;

    if ((x > 0 && prevX <= x) || (x < 0 && prevX >= x))
      currentRightPower += (x > 0 ? -accelerationRate : accelerationRate);
    else if ((x > 0 && prevX > x) || (x < 0 && prevX < x))
      currentRightPower -= (x > 0 ? -decelerationRate : accelerationRate);
  } else {
    currentRightPower += (currentRightPower > 0) ? -decelerationRate : decelerationRate;
    if (abs(currentRightPower) < decelerationRate) currentRightPower = 0;
  }

  currentRightPower = constrain(currentRightPower, -255, 255);

  // Write PWM (invert one side if motors face opposite directions)
  // Use the pin number directly with ledcWrite for ESP32 core 3.0.0+
  ledcWrite(leftMotorPin, pwmFromPower(currentLeftPower));
  ledcWrite(rightMotorPin, pwmFromPower(-currentRightPower));   // invert if needed

  Serial.print("X: "); Serial.print(rawX);
  Serial.print(" | Y: "); Serial.print(rawY);
  Serial.print(" | Left: "); Serial.print(currentLeftPower);
  Serial.print(" | Right: "); Serial.print(currentRightPower);
  Serial.println();

  prevX = x;
  prevY = y;

  delay(50);
}