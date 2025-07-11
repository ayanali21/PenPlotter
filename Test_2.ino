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

bool motorRunning = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Ready! w/x/a/d to move, u/l for servo, q to stop");

  pinMode(Y_LIMIT_TOP, INPUT_PULLUP);
  pinMode(Y_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(X_LIMIT_LEFT, INPUT_PULLUP);
  pinMode(X_LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

  // Setup Motor PWM
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

  // Setup Servo PWM
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
  motorRunning = false;
  Serial.println("All motors stopped");
}

bool limitOrEmergencyTriggered() {
  return digitalRead(EMERGENCY_STOP_PIN) == LOW ||
         digitalRead(Y_LIMIT_TOP) == LOW ||
         digitalRead(Y_LIMIT_BOTTOM) == LOW ||
         digitalRead(X_LIMIT_LEFT) == LOW ||
         digitalRead(X_LIMIT_RIGHT) == LOW;
}

void rampMoveMotor(int ch1, int ch2, int limitPin) {
  for (int duty = 0; duty <= 255; duty += 5) {
    if (digitalRead(limitPin) == LOW || digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      stopMotor();
      Serial.println("Limit or emergency stop triggered during ramp");
      return;
    }
    ledc_set_duty(PWM_SPEED_MODE, (ledc_channel_t)ch1, duty);
    ledc_update_duty(PWM_SPEED_MODE, (ledc_channel_t)ch1);
    ledc_set_duty(PWM_SPEED_MODE, (ledc_channel_t)ch2, 0);
    ledc_update_duty(PWM_SPEED_MODE, (ledc_channel_t)ch2);
    delay(20);
  }

  // Keep full speed until limit hit
  while (digitalRead(limitPin) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) {
    delay(10);
  }

  stopMotor();
  Serial.println("Motor auto-stopped due to limit/emergency");
}

// Motion functions
void rampUpForwardY()  { Serial.println("Y+"); rampMoveMotor(IN1_CHANNEL, IN2_CHANNEL, Y_LIMIT_TOP); }
void rampUpBackwardY() { Serial.println("Y-"); rampMoveMotor(IN2_CHANNEL, IN1_CHANNEL, Y_LIMIT_BOTTOM); }
void rampUpRightX()    { Serial.println("X+"); rampMoveMotor(IN3_CHANNEL, IN4_CHANNEL, X_LIMIT_RIGHT); }
void rampUpLeftX()     { Serial.println("X-"); rampMoveMotor(IN4_CHANNEL, IN3_CHANNEL, X_LIMIT_LEFT); }

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

void loop() {
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    stopMotor();
    return;
  }

  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'w': rampUpForwardY();  motorRunning = true; break;
      case 'x': rampUpBackwardY(); motorRunning = true; break;
      case 'a': rampUpLeftX();     motorRunning = true; break;
      case 'd': rampUpRightX();    motorRunning = true; break;
      case 'q': stopMotor(); break;
      case 'u': moveServoSmooth(140); break;
      case 'l': moveServoSmooth(90);  break;
      default: Serial.println("Invalid cmd. Use w/x/a/d/q/u/l"); break;
    }
  }
}
