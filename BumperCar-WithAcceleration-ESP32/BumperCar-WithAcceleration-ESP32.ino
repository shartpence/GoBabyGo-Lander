// GoBabyGo - ESP32 Version - Arcade Style Control with Ramping
// Samuel Hartpence, 2025

#include <Arduino.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;


const int PWM_FREQ = 50;
const int PWM_RESOLUTION = 16;

const int joystickXPin = 32;
const int joystickYPin = 33;

const int leftMotorPin = 18;
const int rightMotorPin = 19;

const int deadzone = 20;
int centerX = 530;
int centerY = 531;

const int accelerationRate = 20;
const int decelerationRate = 80;

int currentLeftPower = 0;
int currentRightPower = 0;

void setup() {
  Serial.begin(9600);

  // Initialize PWM channels
  ledcAttach(leftMotorPin, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(rightMotorPin, PWM_FREQ, PWM_RESOLUTION);

  // Force ESCs to neutral at startup
  int neutralPWM = pwmFromPower(0);  // Usually ~1500 µs equivalent
  ledcWrite(leftMotorPin, neutralPWM);
  ledcWrite(rightMotorPin, neutralPWM);

  Serial.println("Sending neutral signal to ESCs...");
  delay(5000);  // Give ESCs time to initialize

  // Calibrate joystick
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
  int microseconds = map(power, -255, 255, 1000, 2000);
  return map(microseconds, 1000, 2000, 3276, 6553);
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

  ledcWrite(leftMotorPin, pwmFromPower(currentLeftPower));
  ledcWrite(rightMotorPin, pwmFromPower(-currentRightPower)); // invert if needed

  // Output to both USB and Bluetooth
  String debug = "X: " + String(rawX) + " | Y: " + String(rawY) +
                 " | Left: " + String(currentLeftPower) +
                 " | Right: " + String(currentRightPower);

  Serial.println(debug);
  SerialBT.println(debug);

  prevX = x;
  prevY = y;

  delay(50);
}
