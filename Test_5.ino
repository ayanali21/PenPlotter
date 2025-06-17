// Motor Pins
#define AIN1 16
#define AIN2 17
#define BIN1 18
#define BIN2 19

// Servo Pin
#define SERVO_PIN 21

// Limit Switch Pins
#define X_LIMIT_LEFT 12
#define X_LIMIT_RIGHT 14
#define Y_LIMIT_BOTTOM 27
#define Y_LIMIT_TOP 26

// Encoder Pinsws
#define ENCODER_X 25
#define ENCODER_Y 33

// PWM Config
#define PWM_FREQ 1000
#define PWM_RES 8
#define PWM_DUTY 255

// Servo PWM Config
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RES 16
#define SERVO_CHANNEL 4
#define SERVO_MIN_US 500   // 0 deg
#define SERVO_MAX_US 2500  // 180 deg

// Ramp Config
const int RAMP_STEP = 30;
const int RAMP_DELAY_US = 1000;

// Servo Angles
int currentServoAngle = 140;

// Movement Direction
volatile int moveXDir = 0;  // -1 = left, 1 = right
volatile int moveYDir = 0;  // -1 = down, 1 = up

// Encoder Positions
volatile long encoderXCount = 0;
volatile long encoderYCount = 0;
const float DIST_PER_PULSE_MM = 1.739;  // <-- Change this based on your actual measurement

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 200;  // in ms

void IRAM_ATTR encoderXISR() {
  encoderXCount += moveXDir;  // Increment or decrement depending on direction
}

void IRAM_ATTR encoderYISR() {
  encoderYCount += moveYDir;
}

void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Limit Switches
  pinMode(X_LIMIT_LEFT, INPUT_PULLUP);
  pinMode(X_LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(Y_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(Y_LIMIT_TOP, INPUT_PULLUP);

  // Encoder pins
  pinMode(ENCODER_X, INPUT_PULLUP);
  pinMode(ENCODER_Y, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_X), encoderXISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_Y), encoderYISR, RISING);

  // PWM Channels
  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcAttachPin(AIN1, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(AIN2, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcAttachPin(BIN1, 2);
  ledcSetup(3, PWM_FREQ, PWM_RES);
  ledcAttachPin(BIN2, 3);

  // Servo setup
  ledcSetup(SERVO_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);
  moveServoToAngle(currentServoAngle);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    switch (cmd) {
      case 'w':
        moveYDir = 1;
        moveXDir = 0;
        moveY();
        break;
      case 's':
        moveYDir = -1;
        moveXDir = 0;
        moveY();
        break;
      case 'a':
        moveXDir = -1;
        moveYDir = 0;
        moveX();
        break;
      case 'd':
        moveXDir = 1;
        moveYDir = 0;
        moveX();
        break;
      case 'x': stopMotors(); break;
      case 'u': moveServoToAngle(140); break;
      case 'l': moveServoToAngle(90); break;
      case 'p': printEncoderPositions(); break;
      case 'g': gotoXY(30.0, 20.0); break;  // Move to X=30mm, Y=20mm
      default: Serial.println("Invalid command"); break;
    }
  }

  // Continuously monitor limit switches and maintain motion if needed
  if (moveXDir != 0) moveX();
  if (moveYDir != 0) moveY();
}

void moveX() {
  if (moveXDir == -1 && digitalRead(X_LIMIT_LEFT) == LOW) {
    Serial.println("Left limit hit!");
    stopMotors();
    return;
  }
  if (moveXDir == 1 && digitalRead(X_LIMIT_RIGHT) == LOW) {
    Serial.println("Right limit hit!");
    stopMotors();
    return;
  }

  ledcWrite(0, 0);
  ledcWrite(1, 0);  // Stop Y axis

  for (int duty = 0; duty <= PWM_DUTY; duty += RAMP_STEP) {
    ledcWrite(2, (moveXDir == -1) ? duty : 0);
    ledcWrite(3, (moveXDir == 1) ? duty : 0);
    delayMicroseconds(RAMP_DELAY_US);
  }
}

void moveY() {
  if (moveYDir == -1 && digitalRead(Y_LIMIT_BOTTOM) == LOW) {
    Serial.println("Bottom limit hit!");
    stopMotors();
    return;
  }
  if (moveYDir == 1 && digitalRead(Y_LIMIT_TOP) == LOW) {
    Serial.println("Top limit hit!");
    stopMotors();
    return;
  }

  ledcWrite(2, 0);
  ledcWrite(3, 0);  // Stop X axis

  for (int duty = 0; duty <= PWM_DUTY; duty += RAMP_STEP) {
    ledcWrite(0, (moveYDir == -1) ? duty : 0);
    ledcWrite(1, (moveYDir == 1) ? duty : 0);
    delayMicroseconds(RAMP_DELAY_US);
  }

  // Live position printing every 200ms
  unsigned long now = millis();
  if (now - lastPrintTime > printInterval) {
    lastPrintTime = now;
    Serial.print("X: ");
    Serial.print(encoderXCount);
    Serial.print(" (");
    Serial.print(encoderXCount * DIST_PER_PULSE_MM);
    Serial.print(" mm) | Y: ");
    Serial.print(encoderYCount);
    Serial.print(" (");
    Serial.print(encoderYCount * DIST_PER_PULSE_MM);
    Serial.println(" mm)");
  }
}

void stopMotors() {
  for (int duty = PWM_DUTY; duty >= 0; duty -= RAMP_STEP) {
    ledcWrite(0, duty);
    ledcWrite(1, duty);
    ledcWrite(2, duty);
    ledcWrite(3, duty);
    delayMicroseconds(RAMP_DELAY_US);
  }
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  moveXDir = 0;
  moveYDir = 0;
}

void moveServoToAngle(int angle) {
  int us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  int duty = (us * (1 << SERVO_PWM_RES)) / (1000000 / SERVO_PWM_FREQ);
  ledcWrite(SERVO_CHANNEL, duty);
  currentServoAngle = angle;
}

void printEncoderPositions() {
  Serial.print("X Count: ");
  Serial.print(encoderXCount);
  Serial.print(" (");
  Serial.print(encoderXCount * DIST_PER_PULSE_MM);
  Serial.print(" mm)");

  Serial.print(" | Y Count: ");
  Serial.print(encoderYCount);
  Serial.print(" (");
  Serial.print(encoderYCount * DIST_PER_PULSE_MM);
  Serial.println(" mm)");
}

// Check if X limit switch is hit for given direction
bool isXLimitHit(int dir) {
  return (dir == -1 && digitalRead(X_LIMIT_LEFT) == LOW) || (dir == 1 && digitalRead(X_LIMIT_RIGHT) == LOW);
}

// Check if Y limit switch is hit for given direction
bool isYLimitHit(int dir) {
  return (dir == -1 && digitalRead(Y_LIMIT_BOTTOM) == LOW) || (dir == 1 && digitalRead(Y_LIMIT_TOP) == LOW);
}

void gotoXY(float targetX_mm, float targetY_mm) {
  long targetXCount = targetX_mm / DIST_PER_PULSE_MM;
  long targetYCount = targetY_mm / DIST_PER_PULSE_MM;

  long errorX = targetXCount - encoderXCount;
  long errorY = targetYCount - encoderYCount;

  moveXDir = (errorX > 0) ? 1 : (errorX < 0) ? -1
                                             : 0;
  moveYDir = (errorY > 0) ? 1 : (errorY < 0) ? -1
                                             : 0;

  while (abs(encoderXCount - targetXCount) > 0 || abs(encoderYCount - targetYCount) > 0) {
    if (moveXDir != 0 && !isXLimitHit(moveXDir)) {
      ledcWrite(2, (moveXDir == -1) ? PWM_DUTY : 0);
      ledcWrite(3, (moveXDir == 1) ? PWM_DUTY : 0);
    } else {
      ledcWrite(2, 0);
      ledcWrite(3, 0);
      moveXDir = 0;
    }

    if (moveYDir != 0 && !isYLimitHit(moveYDir)) {
      ledcWrite(0, (moveYDir == -1) ? PWM_DUTY : 0);
      ledcWrite(1, (moveYDir == 1) ? PWM_DUTY : 0);
    } else {
      ledcWrite(0, 0);
      ledcWrite(1, 0);
      moveYDir = 0;
    }

    // Update error values
    errorX = targetXCount - encoderXCount;
    errorY = targetYCount - encoderYCount;

    // Update directions
    moveXDir = (errorX > 0) ? 1 : (errorX < 0) ? -1
                                               : 0;
    moveYDir = (errorY > 0) ? 1 : (errorY < 0) ? -1
                                               : 0;

    // Print current position every 200ms
    unsigned long now = millis();
    if (now - lastPrintTime > printInterval) {
      lastPrintTime = now;
      Serial.print("GotoX: ");
      Serial.print(encoderXCount * DIST_PER_PULSE_MM, 2);
      Serial.print(" mm, GotoY: ");
      Serial.print(encoderYCount * DIST_PER_PULSE_MM, 2);
      Serial.println(" mm");
    }

    delay(10);
  }

  stopMotors();
  Serial.println("Reached target position.");
