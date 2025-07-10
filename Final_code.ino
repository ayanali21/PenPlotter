#include "driver/ledc.h"
#include <Preferences.h>

// ------------------------ PIN CONFIGURATION ------------------------
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19

#define Y_LIMIT_BOTTOM 32
#define Y_LIMIT_TOP 33
#define X_LIMIT_LEFT 25
#define X_LIMIT_RIGHT 26

#define EMERGENCY_STOP_PIN 27
#define SERVO_PIN 21

#define ENCODER_Y_PIN 34
#define ENCODER_X_PIN 35

#define LED_GREEN 4
#define LED_YELLOW 2
#define LED_RED 13

#define LID_SENSOR_PIN 22

// ------------------------ MOVEMENT & CALIBRATION ------------------------
Preferences preferences;
volatile long encoderCountX = 0;
volatile long encoderCountY = 0;
float ticksPerMM = 6.0;  // default, can be calibrated

#define SIDE_MM 20  // Length of house side in mm
#define RPM_DEFAULT 150

// ------------------------ SERVO CONFIGURATION ------------------------
#define SERVO_CHANNEL LEDC_CHANNEL_4
#define SERVO_TIMER LEDC_TIMER_1
#define SERVO_FREQ 50
#define SERVO_RES LEDC_TIMER_16_BIT
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define PEN_UP_ANGLE 60
#define PEN_DOWN_ANGLE 0

int currentServoAngle = PEN_UP_ANGLE;

// ------------------------ LED ------------------------
void setLEDState(bool green, bool yellow, bool red) {
  digitalWrite(LED_GREEN, green);
  digitalWrite(LED_YELLOW, yellow);
  digitalWrite(LED_RED, red);
}

// ------------------------ ENCODERS ------------------------
void IRAM_ATTR encoderXISR() {
  encoderCountX++;
}

void IRAM_ATTR encoderYISR() {
  encoderCountY++;
}

// ------------------------ SERVO CONTROL ------------------------
void moveServo(int targetAngle) {
  int us = map(targetAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  int duty = (us * (1 << 16)) / (1000000 / SERVO_FREQ);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, SERVO_CHANNEL, duty);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, SERVO_CHANNEL);
  delay(300);
}

// ------------------------ MOTOR CONTROL ------------------------
void driveMotor(int ch1, int ch2, int speed, int direction) {
  int duty = constrain(speed, 0, 255);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch1, direction > 0 ? duty : 0);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch2, direction < 0 ? duty : 0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch1);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch2);
}

void stopMotors() {
  for (int ch = 0; ch <= 3; ch++) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)ch);
  }
}

void moveToEncoderCounts(long targetX, long targetY, int dirX, int dirY) {
  encoderCountX = 0;
  encoderCountY = 0;

  while ((abs(encoderCountX) < abs(targetX)) || (abs(encoderCountY) < abs(targetY))) {
    if (abs(encoderCountX) < abs(targetX)) {
      driveMotor(IN3_CHANNEL, IN4_CHANNEL, 100, dirX);
    } else {
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, IN3_CHANNEL, 0);
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, IN4_CHANNEL, 0);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, IN3_CHANNEL);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, IN4_CHANNEL);
    }

    if (abs(encoderCountY) < abs(targetY)) {
      driveMotor(IN1_CHANNEL, IN2_CHANNEL, 100, dirY);
    } else {
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, IN1_CHANNEL, 0);
      ledc_set_duty(LEDC_HIGH_SPEED_MODE, IN2_CHANNEL, 0);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, IN1_CHANNEL);
      ledc_update_duty(LEDC_HIGH_SPEED_MODE, IN2_CHANNEL);
    }

    delay(1);
  }

  stopMotors();
}

void homeAxisX() {
  while (digitalRead(X_LIMIT_LEFT) == HIGH) {
    driveMotor(IN3_CHANNEL, IN4_CHANNEL, 100, -1);
    delay(5);
  }
  stopMotors(); delay(200);
}

void homeAxisY() {
  while (digitalRead(Y_LIMIT_BOTTOM) == HIGH) {
    driveMotor(IN1_CHANNEL, IN2_CHANNEL, 100, -1);
    delay(5);
  }
  stopMotors(); delay(200);
}

void autoHome() {
  setLEDState(false, true, false);
  homeAxisX();
  homeAxisY();
  stopMotors();
  setLEDState(true, false, false);
}

void drawNikolausHouse() {
  Serial.println("Drawing Nikolaus House...");
  autoHome();
  delay(500);

  moveServo(PEN_UP_ANGLE);
  delay(500);

  moveToEncoderCounts(0, ticksPerMM * SIDE_MM, 0, 1);
  delay(200);
  moveToEncoderCounts(ticksPerMM * 10, 0, 1, 0);
  delay(200);

  moveServo(PEN_DOWN_ANGLE);
  delay(500);

  // Draw square
  moveToEncoderCounts(ticksPerMM * SIDE_MM, 0, 1, 0);
  delay(200);
  moveToEncoderCounts(0, ticksPerMM * SIDE_MM, 0, 1);
  delay(200);
  moveToEncoderCounts(-ticksPerMM * SIDE_MM, 0, -1, 0);
  delay(200);
  moveToEncoderCounts(0, -ticksPerMM * SIDE_MM, 0, -1);
  delay(200);

  // Diagonals
  moveToEncoderCounts(ticksPerMM * SIDE_MM, ticksPerMM * SIDE_MM, 1, 1);
  delay(200);
  moveToEncoderCounts(-ticksPerMM * SIDE_MM, ticksPerMM * SIDE_MM, -1, 1);
  delay(200);
  moveToEncoderCounts(0, -ticksPerMM * SIDE_MM, 0, -1);
  delay(200);
  moveToEncoderCounts(ticksPerMM * SIDE_MM, -ticksPerMM * SIDE_MM, 1, -1);
  delay(200);

  moveServo(PEN_UP_ANGLE);
  delay(500);
  autoHome();
  setLEDState(true, false, false);
  Serial.println("Nikolaus House drawing complete.");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Pins
  pinMode(Y_LIMIT_TOP, INPUT_PULLUP);
  pinMode(Y_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(X_LIMIT_LEFT, INPUT_PULLUP);
  pinMode(X_LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(LID_SENSOR_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  setLEDState(true, false, false);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_Y_PIN), encoderYISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_X_PIN), encoderXISR, FALLING);

  // PWM config
  ledc_timer_config_t timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);

  for (int i = 0; i < 4; i++) {
    ledc_channel_config_t ch = {
      .gpio_num = IN1 + i,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel = (ledc_channel_t)i,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_0,
      .duty = 0,
      .hpoint = 0
    };
    ledc_channel_config(&ch);
  }

  // Servo PWM
  ledc_timer_config_t servoTimer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = SERVO_RES,
    .timer_num = SERVO_TIMER,
    .freq_hz = SERVO_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&servoTimer);

  ledc_channel_config_t servoCh = {
    .gpio_num = SERVO_PIN,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = SERVO_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = SERVO_TIMER,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&servoCh);

  moveServo(PEN_UP_ANGLE);

  // Load preferences
  preferences.begin("calibration", true);
  ticksPerMM = preferences.getFloat("ticksPerMM", 6.0);
  preferences.end();

  autoHome();
}

void loop() {
  if (digitalRead(LID_SENSOR_PIN) == LOW) {
    drawNikolausHouse();
  }
  delay(200);
}
