#include <Arduino.h>

// --------------------
// PINS (PWM-DIR MODE)
// --------------------
const int joystickXPin = 32;
const int joystickYPin = 33;
const int LEFT_PWM_PIN  = 18; 
const int LEFT_DIR_PIN  = 17; 
const int RIGHT_PWM_PIN = 19; 
const int RIGHT_DIR_PIN = 16; 

// --------------------
// PARAMETERS
// --------------------
const int deadzone = 80;     // Adjusted for your specific joystick
float powerCutFactor = 0.75;  // Power limit (0.0 to 1.0)

// --------------------
// STATE VARIABLES
// --------------------
int centerX, centerY;
int currentLeftPower = 0;
int currentRightPower = 0;
const int rampStep = 4; // How much power can change per 20ms loop

// --------------------
// FUNCTIONS
// --------------------
int processAxis(int raw, int center) {
  int adjusted = raw - center;
  if (abs(adjusted) < deadzone) return 0;
  
  // Maps the stick travel to motor power range
  // We constrain to 2000 to allow for stick variance
  adjusted = constrain(adjusted, -2000, 2000);
  return map(adjusted, -2000, 2000, -255, 255);
}

void setMotor(int pwmPin, int dirPin, int power) {
  power = constrain(power, -255, 255);
  digitalWrite(dirPin, (power >= 0) ? HIGH : LOW);
  ledcWrite(pwmPin, abs(power));
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  ledcAttach(LEFT_PWM_PIN, 1000, 8); 
  ledcAttach(RIGHT_PWM_PIN, 1000, 8);

  // Initial Calibration
  centerX = 0; centerY = 0;
  for(int i=0; i<50; i++) {
    centerX += analogRead(joystickXPin);
    centerY += analogRead(joystickYPin);
    delay(2);
  }
  centerX /= 50;
  centerY /= 50;

  Serial.println("System Ready.");
}

void loop() {
  int rawX = analogRead(joystickXPin);
  int rawY = analogRead(joystickYPin);

  int x = -processAxis(rawX, centerX);
  int y = -processAxis(rawY, centerY); // Inverted Y to match standard drive

  // Watchdog: If inside deadzone, stop motors immediately
  if (x == 0 && y == 0) {
    ledcWrite(LEFT_PWM_PIN, 0);
    ledcWrite(RIGHT_PWM_PIN, 0);
    currentLeftPower = 0;  // Reset ramping memory
    currentRightPower = 0; // Reset ramping memory
  } else {
    // Arcade Drive Math
    int targetLeft  = constrain((y + x) * powerCutFactor, -255, 255);
    int targetRight = constrain((y - x) * powerCutFactor, -255, 255);

    // Soft Ramping
    if (currentLeftPower < targetLeft) currentLeftPower += rampStep;
    else if (currentLeftPower > targetLeft) currentLeftPower -= rampStep;

    if (currentRightPower < targetRight) currentRightPower += rampStep;
    else if (currentRightPower > targetRight) currentRightPower -= rampStep;

    // Send the ramped power to the motors
    setMotor(LEFT_PWM_PIN, LEFT_DIR_PIN, currentLeftPower);
    setMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN, currentRightPower);
  }

  // Debugging
  Serial.print("X: "); Serial.print(x);
  Serial.print(" Y: "); Serial.println(y);
  delay(20); 
}