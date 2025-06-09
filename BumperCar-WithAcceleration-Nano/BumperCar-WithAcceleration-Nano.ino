/*code developed by Samuel Hartpence
This implements an ‘arcade style’ drive with refined joystick-direction-aware ramp control
for a two motor tank configuration
Project motivated by Go Baby GO lander project 2025
Originally developed to work with REV Spark Mini Motor Controllers
PMW needs to be scaled appropriately if using another style controller
For SPRAK Minis,
PMW values:
1000 μs → Full reverse
1500 μs → Neutral (zero RPM)
2000 μs → Full forward
*/

#include <Servo.h>

const int joystickXPin = A0;
const int joystickYPin = A1;

const int deadzone = 20;

Servo leftMotor;
Servo rightMotor;

int centerX = 530;
int centerY = 531;

// Ramp control parameters
const int accelerationRate = 20;   // Adjust this value
const int decelerationRate = 80;  // Adjust this value

// Current motor power levels (normalized -255 to 255)
int currentLeftPower = 0;
int currentRightPower = 0;

void setup() {
  Serial.begin(9600);

  leftMotor.attach(9);   // PWM pin for left motor
  rightMotor.attach(10); // PWM pin for right motor

  // Read joystick center at startup
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
  adjusted = constrain(adjusted, -512, 512);
  return map(adjusted, -512, 512, -255, 255);
}

int pwmFromPower(int power) {
  return map(power, -255, 255, 1000, 2000); // 1500 = neutral
}

void loop() {
  int rawX = analogRead(joystickXPin);
  int rawY = analogRead(joystickYPin);

  int x = processAxis(rawX, centerX);
  int y = processAxis(rawY, centerY);

  static int prevX = x;
  static int prevY = y;

  // Arcade drive: y = fwd/back, x = turn
  int targetLeftPower = y + x;
  int targetRightPower = y - x;

  targetLeftPower = constrain(targetLeftPower, -255, 255);
  targetRightPower = constrain(targetRightPower, -255, 255);

  // Ramp control for left motor
  if (abs(y) > deadzone || abs(x) > deadzone) {
    // Y-axis ramp
    if (abs(y) > deadzone) {
      if ((y > 0 && prevY <= y) || (y < 0 && prevY >= y)) {
        currentLeftPower += accelerationRate;
      } else if ((y > 0 && prevY > y) || (y < 0 && prevY < y)) {
        currentLeftPower -= decelerationRate;
      }
    }
    // X-axis ramp (additive for turning)
    if (abs(x) > deadzone) {
      if ((x > 0 && prevX <= x) || (x < 0 && prevX >= x)) {
        currentLeftPower += (x > 0 ? accelerationRate : -accelerationRate);
      } else if ((x > 0 && prevX > x) || (x < 0 && prevX < x)) {
        currentLeftPower -= (x > 0 ? decelerationRate : -decelerationRate);
      }
    }
  } else {
    if (currentLeftPower > 0) {
      currentLeftPower -= decelerationRate;
      if (currentLeftPower < 0) currentLeftPower = 0;
    } else if (currentLeftPower < 0) {
      currentLeftPower += decelerationRate;
      if (currentLeftPower > 0) currentLeftPower = 0;
    }
  }
  currentLeftPower = constrain(currentLeftPower, -255, 255);

  // Ramp control for right motor
  if (abs(y) > deadzone || abs(x) > deadzone) {
    // Y-axis ramp
    if (abs(y) > deadzone) {
      if ((y > 0 && prevY <= y) || (y < 0 && prevY >= y)) {
        currentRightPower += accelerationRate;
      } else if ((y > 0 && prevY > y) || (y < 0 && prevY < y)) {
        currentRightPower -= decelerationRate;
      }
    }
    // X-axis ramp (additive for turning, opposite direction)
    if (abs(x) > deadzone) {
      if ((x > 0 && prevX <= x) || (x < 0 && prevX >= x)) {
        currentRightPower += (x > 0 ? -accelerationRate : accelerationRate);
      } else if ((x > 0 && prevX > x) || (x < 0 && prevX < x)) {
        currentRightPower -= (x > 0 ? -decelerationRate : accelerationRate);
      }
    }
  } else {
    if (currentRightPower > 0) {
      currentRightPower -= decelerationRate;
      if (currentRightPower < 0) currentRightPower = 0;
    } else if (currentRightPower < 0) {
      currentRightPower += decelerationRate;
      if (currentRightPower > 0) currentRightPower = 0;
    }
  }
  currentRightPower = constrain(currentRightPower, -255, 255);

  int leftPWM = pwmFromPower(currentLeftPower);
  int rightPWM = pwmFromPower(currentRightPower * -1);

  leftMotor.writeMicroseconds(leftPWM);
  rightMotor.writeMicroseconds(rightPWM);

  Serial.print("Joystick X: "); Serial.print(rawX);
  Serial.print(" | Y: "); Serial.print(rawY);
  Serial.print(" | x: "); Serial.print(x);
  Serial.print(" | y: "); Serial.print(y);
  Serial.print(" | prevX: "); Serial.print(prevX);
  Serial.print(" | prevY: "); Serial.print(prevY);
  Serial.print(" | Target Left Power: "); Serial.print(targetLeftPower);
  Serial.print(" | Current Left Power: "); Serial.print(currentLeftPower);
  Serial.print(" | Left PWM: "); Serial.print(leftPWM);
  Serial.print(" | Target Right Power: "); Serial.print(targetRightPower);
  Serial.print(" | Current Right Power: "); Serial.print(currentRightPower);
  Serial.print(" | Right PWM: "); Serial.println(rightPWM);

  prevX = x;
  prevY = y;
  delay(50);
}