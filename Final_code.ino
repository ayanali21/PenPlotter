#include "driver/ledc.h"

// Motor Pins
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19

// Limit switches
#define Y_LIMIT_BOTTOM     26
#define Y_LIMIT_TOP        27
#define X_LIMIT_RIGHT      12
#define X_LIMIT_LEFT       14

// Emergency Stop
#define EMERGENCY_STOP_PIN 22

// Servo
#define SERVO_PIN        21
#define SERVO_CHANNEL    LEDC_CHANNEL_4
#define SERVO_TIMER      LEDC_TIMER_1
#define SERVO_FREQ       50
#define SERVO_RES        LEDC_TIMER_16_BIT
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2500
#define SERVO_STEP       2
#define SERVO_DELAY_MS   5

int currentServoAngle = 140;

// Motor PWM Setup
#define PWM_FREQ 1000
#define PWM_RESOLUTION_MOTOR LEDC_TIMER_8_BIT
#define PWM_TIMER LEDC_TIMER_0
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE

#define IN1_CHANNEL LEDC_CHANNEL_0
#define IN2_CHANNEL LEDC_CHANNEL_1
#define IN3_CHANNEL LEDC_CHANNEL_2
#define IN4_CHANNEL LEDC_CHANNEL_3

// Speed Sensor Pins
#define ENCODER_Y_PIN 33
#define ENCODER_X_PIN 25

// Calibration
#define Y_TRAVEL_DISTANCE 117.3  // Y axis travel distance in mm
#define X_TRAVEL_DISTANCE 106.5  // X axis travel distance in mm

// Global variables for RPM and timing
const int slotsPerRevolution = 20; // Number of slots on the disc (H206 has 20 slots)
unsigned long lastYTime = 0;
unsigned long lastXTime = 0;

float yRPM = 0;  // RPM for Y axis
float xRPM = 0;  // RPM for X axis

// Target RPM for both axes
#define TARGET_RPM 35

// PID control variables
float Kp = 2.0;  // Proportional constant
float Ki = 0.5;  // Integral constant
float Kd = 1.0;  // Derivative constant

float previousErrorY = 0;
float integralY = 0;

float previousErrorX = 0;
float integralX = 0;

// Movement flags for controlling motor direction
bool moveForwardY = false;
bool moveBackwardY = false;
bool moveLeftX = false;
bool moveRightX = false;
bool shouldStop = false;

// Interrupts for the H206 Speed Sensor
void IRAM_ATTR encoderYISR() {
  unsigned long currentTime = millis();
  yRPM = (60.0 / (currentTime - lastYTime)) * slotsPerRevolution;  // RPM calculation for Y axis
  lastYTime = currentTime;
}

void IRAM_ATTR encoderXISR() {
  unsigned long currentTime = millis();
  xRPM = (60.0 / (currentTime - lastXTime)) * slotsPerRevolution;  // RPM calculation for X axis
  lastXTime = currentTime;
}

void autoHome() {
  Serial.println("Homing to bottom-left position...");

  // Move X to left
  while (digitalRead(X_LIMIT_LEFT) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
    ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, 180);
    ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);
    ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, 0);
    ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
  }
  stopMotor();

  // Move Y to bottom
  while (digitalRead(Y_LIMIT_BOTTOM) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
    ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, 180);
    ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);
    ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, 0);
    ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
  }
  stopMotor();

  Serial.println("Homing complete. Plotter is at bottom-left position (0,0).");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Ready! w/x/a/d to move, u/l for servo, q to stop");

  pinMode(Y_LIMIT_TOP, INPUT_PULLUP);
  pinMode(Y_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(X_LIMIT_LEFT, INPUT_PULLUP);
  pinMode(X_LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

  pinMode(ENCODER_Y_PIN, INPUT_PULLUP);
  pinMode(ENCODER_X_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_Y_PIN), encoderYISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_X_PIN), encoderXISR, FALLING);

  ledc_timer_config_t motor_timer = {
    .speed_mode = PWM_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION_MOTOR,
    .timer_num = PWM_TIMER,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&motor_timer);

  setupPWMChannel(IN1, IN1_CHANNEL);
  setupPWMChannel(IN2, IN2_CHANNEL);
  setupPWMChannel(IN3, IN3_CHANNEL);
  setupPWMChannel(IN4, IN4_CHANNEL);

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
  autoHome();
}

void setupPWMChannel(int pin, ledc_channel_t channel) {
  ledc_channel_config_t config = {
    .gpio_num = pin,
    .speed_mode = PWM_SPEED_MODE,
    .channel = channel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = PWM_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&config);
}

void stopMotor() {
  for (int ch = 0; ch <= 3; ch++) {
    ledc_set_duty(PWM_SPEED_MODE, (ledc_channel_t)ch, 0);
    ledc_update_duty(PWM_SPEED_MODE, (ledc_channel_t)ch);
  }

  // Reset RPM to 0 when motors stop
  yRPM = 0;
  xRPM = 0;

  Serial.println("Motor stopped");
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
}

// Add the motor driving functions with PID adjustments
void driveY(int dir) {
  if (dir > 0 && digitalRead(Y_LIMIT_TOP) == LOW) return;
  if (dir < 0 && digitalRead(Y_LIMIT_BOTTOM) == LOW) return;

  // PID control for Y axis speed
  float errorY = TARGET_RPM - yRPM;
  integralY += errorY;
  float derivativeY = errorY - previousErrorY;
  float outputY = Kp * errorY + Ki * integralY + Kd * derivativeY;

  // Adjust speed based on PID output
  int speedY = constrain(outputY, 0, 255);  // Adjust max PWM duty cycle
  ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, dir > 0 ? speedY : 0);
  ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);
  ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, dir < 0 ? speedY : 0);
  ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);

  previousErrorY = errorY;
}

void driveX(int dir) {
  if (dir > 0 && digitalRead(X_LIMIT_RIGHT) == LOW) return;
  if (dir < 0 && digitalRead(X_LIMIT_LEFT) == LOW) return;

  // PID control for X axis speed
  float errorX = TARGET_RPM - xRPM;
  integralX += errorX;
  float derivativeX = errorX - previousErrorX;
  float outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;

  // Adjust speed based on PID output
  int speedX = constrain(outputX, 0, 255);  // Adjust max PWM duty cycle
  ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, dir > 0 ? speedX : 0);
  ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);
  ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, dir < 0 ? speedX : 0);
  ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);

  previousErrorX = errorX;
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    moveForwardY = (cmd == 'w');
    moveBackwardY = (cmd == 'x');
    moveLeftX = (cmd == 'a');
    moveRightX = (cmd == 'd');
    shouldStop = (cmd == 'q');
    if (cmd == 'u') moveServoSmooth(140);
    if (cmd == 'l') moveServoSmooth(90);
  }

  if (shouldStop || digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    stopMotor();
    return;
  }

  if (moveForwardY && digitalRead(Y_LIMIT_TOP) == HIGH) {
    driveY(1);
  } else if (moveBackwardY && digitalRead(Y_LIMIT_BOTTOM) == HIGH) {
    driveY(-1);
  } else if (moveLeftX && digitalRead(X_LIMIT_LEFT) == HIGH) {
    driveX(-1);
  } else if (moveRightX && digitalRead(X_LIMIT_RIGHT) == HIGH) {
    driveX(1);
  } else {
    stopMotor();
  }

  // Send Y and X RPM to Serial Plotter for visualizing
  Serial.print("Y_RPM:");
  Serial.print(yRPM);
  Serial.print(",X_RPM:");
  Serial.println(xRPM);

  delay(10);  // For responsiveness
}