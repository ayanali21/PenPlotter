#include "driver/ledc.h"

// Motor Pins
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19

// Limit switches
#define Y_LIMIT_BOTTOM 26
#define Y_LIMIT_TOP 27
#define X_LIMIT_RIGHT 12
#define X_LIMIT_LEFT 14

// Emergency Stop
#define EMERGENCY_STOP_PIN 22

// Servo
#define SERVO_PIN 21
#define SERVO_CHANNEL LEDC_CHANNEL_4
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_FREQ 50
#define SERVO_RES LEDC_TIMER_16_BIT
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_STEP 2
#define SERVO_DELAY_MS 5

// Encoder Pins
#define ENCODER_X_PIN 25
#define ENCODER_Y_PIN 33

// Distance per pulse in mm (calibrate this!)
#define MM_PER_PULSE 1.540

// Limit switch debounce flags
bool limitTriggeredX = false;
bool limitTriggeredY = false;
unsigned long lastDebounceTimeX = 0;
unsigned long lastDebounceTimeY = 0;
const unsigned long debounceDelay = 50;


volatile long encoderXCount = 0;
volatile long encoderYCount = 0;

int currentServoAngle = 140;

// PWM setup
#define PWM_FREQ 1000
#define PWM_RESOLUTION_MOTOR LEDC_TIMER_8_BIT
#define PWM_TIMER LEDC_TIMER_0
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE

// Motor channels
#define IN1_CHANNEL LEDC_CHANNEL_0
#define IN2_CHANNEL LEDC_CHANNEL_1
#define IN3_CHANNEL LEDC_CHANNEL_2
#define IN4_CHANNEL LEDC_CHANNEL_3

bool motorRunning = false;

void IRAM_ATTR encoderXISR() {
encoderXCount++;
}

void IRAM_ATTR encoderYISR() {
encoderYCount++;
}

void setup() {
Serial.begin(115200);
Serial.println("Ready! w/x/a/d to move, u/l for servo, q to stop, p to print pos");

// Limit switches
pinMode(Y_LIMIT_TOP, INPUT_PULLUP);
pinMode(Y_LIMIT_BOTTOM, INPUT_PULLUP);
pinMode(X_LIMIT_LEFT, INPUT_PULLUP);
pinMode(X_LIMIT_RIGHT, INPUT_PULLUP);

pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

// Encoder pins
pinMode(ENCODER_X_PIN, INPUT_PULLUP);
pinMode(ENCODER_Y_PIN, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(ENCODER_X_PIN), encoderXISR, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER_Y_PIN), encoderYISR, RISING);

// Setup motor PWM timer
ledc_timer_config_t motor_timer = {
.speed_mode = PWM_SPEED_MODE,
.duty_resolution = PWM_RESOLUTION_MOTOR,
.timer_num = PWM_TIMER,
.freq_hz = PWM_FREQ,
.clk_cfg = LEDC_AUTO_CLK
};
ledc_timer_config(&motor_timer);

// Motor PWM channels
setupPWMChannel(IN1, IN1_CHANNEL);
setupPWMChannel(IN2, IN2_CHANNEL);
setupPWMChannel(IN3, IN3_CHANNEL);
setupPWMChannel(IN4, IN4_CHANNEL);

// Servo PWM setup
ledc_timer_config_t servo_timer = {
.speed_mode = PWM_SPEED_MODE,
.duty_resolution = SERVO_RES,
.timer_num = SERVO_TIMER,
.freq_hz = SERVO_FREQ,
.clk_cfg = LEDC_AUTO_CLK
};
ledc_timer_config(&servo_timer);

ledc_channel_config_t servo_channel = {
.gpio_num = SERVO_PIN,
.speed_mode = PWM_SPEED_MODE,
.channel = SERVO_CHANNEL,
.intr_type = LEDC_INTR_DISABLE,
.timer_sel = SERVO_TIMER,
.duty = 0,
.hpoint = 0
};
ledc_channel_config(&servo_channel);

moveServoSmooth(currentServoAngle);
}

void setupPWMChannel(int pin, ledc_channel_t channel) {
ledc_channel_config_t channelConfig = {
.gpio_num = pin,
.speed_mode = PWM_SPEED_MODE,
.channel = channel,
.intr_type = LEDC_INTR_DISABLE,
.timer_sel = PWM_TIMER,
.duty = 0,
.hpoint = 0
};
ledc_channel_config(&channelConfig);
}

void stopMotor() {
for (int ch = 0; ch <= 3; ch++) {
ledc_set_duty(PWM_SPEED_MODE, (ledc_channel_t)ch, 0);
ledc_update_duty(PWM_SPEED_MODE, (ledc_channel_t)ch);
}
motorRunning = false;
Serial.println("All motors stopped");
}

void rampUpForwardY() {
Serial.println("Y-axis forward...");
for (int duty = 0; duty <= 255; duty += 5) {
if (digitalRead(Y_LIMIT_TOP) == LOW || digitalRead(EMERGENCY_STOP_PIN) == LOW) {
stopMotor();
Serial.println("Y limit top or Emergency stop triggered");
return;
}
ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, duty);
ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
delay(30);
}
}

void rampUpBackwardY() {
Serial.println("Y-axis backward...");
for (int duty = 0; duty <= 255; duty += 5) {
if (digitalRead(Y_LIMIT_BOTTOM) == LOW || digitalRead(EMERGENCY_STOP_PIN) == LOW) {
stopMotor();
Serial.println("Y limit bottom or Emergency stop triggered");
return;
}
ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, duty);
ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
delay(30);
}
}

void rampUpLeftX() {
Serial.println("X-axis left...");
for (int duty = 0; duty <= 255; duty += 5) {
if (digitalRead(X_LIMIT_LEFT) == LOW || digitalRead(EMERGENCY_STOP_PIN) == LOW) {
stopMotor();
Serial.println("X limit left or Emergency stop triggered");
return;
}
ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, duty);
ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
delay(30);
}
}

void rampUpRightX() {
Serial.println("X-axis right...");
for (int duty = 0; duty <= 255; duty += 5) {
if (digitalRead(X_LIMIT_RIGHT) == LOW || digitalRead(EMERGENCY_STOP_PIN) == LOW) {
stopMotor();
Serial.println("X limit right or Emergency stop triggered");
return;
}
ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, duty);
ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
delay(30);
}
}

void moveServoSmooth(int targetAngle) {
int step = (targetAngle > currentServoAngle) ? SERVO_STEP : -SERVO_STEP;

for (int angle = currentServoAngle; angle != targetAngle; angle += step) {
int us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
int duty = (us * (1 << 16)) / (1000000 / SERVO_FREQ);
ledc_set_duty(PWM_SPEED_MODE, SERVO_CHANNEL, duty);
ledc_update_duty(PWM_SPEED_MODE, SERVO_CHANNEL);
delay(SERVO_DELAY_MS);
}

int us = map(targetAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
int duty = (us * (1 << 16)) / (1000000 / SERVO_FREQ);
ledc_set_duty(PWM_SPEED_MODE, SERVO_CHANNEL, duty);
ledc_update_duty(PWM_SPEED_MODE, SERVO_CHANNEL);
currentServoAngle = targetAngle;

Serial.print("Servo angle: ");
Serial.println(currentServoAngle);
}

void printPosition() {
float x_mm = encoderXCount * MM_PER_PULSE;
float y_mm = encoderYCount * MM_PER_PULSE;

Serial.print("X = ");
Serial.print(encoderXCount);
Serial.print(" (");
Serial.print(x_mm, 2);
Serial.print(" mm), Y = ");
Serial.print(encoderYCount);
Serial.print(" (");
Serial.print(y_mm, 2);
Serial.println(" mm)");
}

void homeAxes() {
Serial.println("Starting homing sequence...");

// --------------------
// Home Y Axis (BOTTOM)
// --------------------

// Fast approach
while (digitalRead(Y_LIMIT_BOTTOM) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, 180); // Fast speed
ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
delay(5);
}

// Stop Y motor
ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
delay(100);

// Back off
for (int i = 0; i < 20; i++) {
ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 100);
ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
delay(5);
}

// Stop
ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
delay(100);

// Slow approach
while (digitalRead(Y_LIMIT_BOTTOM) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, 100); // Slow speed
ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
delay(5);
}

// Stop Y motor
ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
delay(100);

// --------------------
// Home X Axis (LEFT)
// --------------------

// Fast approach
while (digitalRead(X_LIMIT_LEFT) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 180); // Fast speed
ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
delay(5);
}

// Stop X motor
ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
delay(100);

// Back off
for (int i = 0; i < 20; i++) {
ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, 100);
ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
delay(5);
}

// Stop
ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
delay(100);

// Slow approach
while (digitalRead(X_LIMIT_LEFT) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 100); // Slow speed
ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
delay(5);
}

// Stop X motor
ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 0);
ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
delay(100);

// --------------------
// Reset encoder counts
encoderXCount = 0;
encoderYCount = 0;

Serial.println("Reached home (double-hit)");
}

// Nikolaus haus
void drawNikolausHouse() {
  Serial.println("Starting Nikolaus House drawing...");

  // Go to home position first
  Serial.println("Homing before drawing");
  while (digitalRead(Y_LIMIT_BOTTOM) != LOW || digitalRead(X_LIMIT_LEFT) != LOW) {
    if (digitalRead(Y_LIMIT_BOTTOM) != LOW) {
      ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 150); // move Y backward
      ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
      ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, 0);
      ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
    } else {
      ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 0);
      ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
    }
    if (digitalRead(X_LIMIT_LEFT) != LOW) {
      ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, 0);
      ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
      ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 150); // move X left
      ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
    } else {
      ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 0);
      ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
    }
    delay(10);
  }
  stopMotor();
  encoderXCount = 0;
  encoderYCount = 0;
  Serial.println("Reached home position");

  // Put pen down
  moveServoSmooth(90);

  // Define key points (X,Y) in mm
  float baseWidth = 50;
  float baseHeight = 35;
  float roofHeight = 35;

  float points[][2] = {
    {0, 0},
    {0, baseHeight},
    {baseWidth/2, baseHeight + roofHeight},
    {baseWidth, baseHeight},
    {baseWidth, 0},
    {0, 0},
    {baseWidth, baseHeight},
    {0, baseHeight},
    {baseWidth, 0},
    {baseWidth/2, baseHeight + roofHeight},
  };

  long startX = encoderXCount;
  long startY = encoderYCount;

  for (int i = 1; i < sizeof(points)/sizeof(points[0]); i++) {
    float dx = points[i][0] - points[i-1][0];
    float dy = points[i][1] - points[i-1][1];
    long targetX = encoderXCount + dx / MM_PER_PULSE;
    long targetY = encoderYCount + dy / MM_PER_PULSE;

    // Move X
    if (dx != 0) {
      if (dx > 0) rampUpRightX(); else rampUpLeftX();
    }
    // Move Y
    if (dy != 0) {
      if (dy > 0) rampUpForwardY(); else rampUpBackwardY();
    }

    // simulate step move delay
    delay(200);
  }

  // Lift pen up
  moveServoSmooth(140);

  Serial.println("Nikolaus House drawing complete.");
}

void loop() {
if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
stopMotor();
return;
}

unsigned long now = millis();

bool xHit = (digitalRead(X_LIMIT_LEFT) == LOW || digitalRead(X_LIMIT_RIGHT) == LOW);
bool yHit = (digitalRead(Y_LIMIT_TOP) == LOW || digitalRead(Y_LIMIT_BOTTOM) == LOW);

if (xHit) {
  if (!limitTriggeredX && now - lastDebounceTimeX > debounceDelay) {
    stopMotor();
    limitTriggeredX = true;
    lastDebounceTimeX = now;
    Serial.println("Limit X triggered");
  }
} else {
  limitTriggeredX = false;
}

if (yHit) {
  if (!limitTriggeredY && now - lastDebounceTimeY > debounceDelay) {
    stopMotor();
    limitTriggeredY = true;
    lastDebounceTimeY = now;
    Serial.println("Limit Y triggered");
  }
} else {
  limitTriggeredY = false;
}

if (Serial.available()) {
char command = Serial.read();
switch (command) {
case 'w': rampUpForwardY(); motorRunning = true; break;
case 'x': rampUpBackwardY(); motorRunning = true; break;
case 'a': rampUpLeftX(); motorRunning = true; break;
case 'd': rampUpRightX(); motorRunning = true; break;
case 'q': stopMotor(); break;
case 'u': moveServoSmooth(140); break;
case 'l': moveServoSmooth(90); break;
case 'p': printPosition(); break;
case 'h': homeAxes(); break;
case 'n': drawNikolausHouse(); break;
default: break;
}
}
}
