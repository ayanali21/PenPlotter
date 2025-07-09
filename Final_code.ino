#include "driver/ledc.h"
#include <Preferences.h>

// ------------------------ PIN CONFIGURATION ------------------------
// Motor control pins
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19

// Limit switch pins (X and Y axes)
#define Y_LIMIT_BOTTOM 32
#define Y_LIMIT_TOP 33
#define X_LIMIT_LEFT 25
#define X_LIMIT_RIGHT 26

// Emergency stop and servo control pins
#define EMERGENCY_STOP_PIN 27
#define SERVO_PIN 21

// Encoder pins for X and Y axes
#define ENCODER_Y_PIN 34
#define ENCODER_X_PIN 35

// LED indicators
#define LED_GREEN 4
#define LED_YELLOW 2
#define LED_RED 13

// Lid sensor for detecting when the lid is open
#define LID_SENSOR_PIN 22

// ------------------------ SERVO CONFIGURATION ------------------------
#define SERVO_CHANNEL LEDC_CHANNEL_4       // Channel for servo PWM control
#define SERVO_TIMER LEDC_TIMER_1           // Timer for servo control
#define SERVO_FREQ 50                      // Frequency for servo control
#define SERVO_RES LEDC_TIMER_16_BIT       // Resolution for servo control
#define SERVO_MIN_US 500                  // Minimum pulse width for servo (microseconds)
#define SERVO_MAX_US 2500                 // Maximum pulse width for servo (microseconds)
#define PEN_UP_ANGLE 60                   // Servo angle to lift the pen
#define PEN_DOWN_ANGLE 0                  // Servo angle to lower the pen

int currentServoAngle = PEN_UP_ANGLE;   // Variable to store the current servo angle

// ------------------------ MOTOR PWM CONFIG ------------------------
#define PWM_FREQ 1000                     // PWM frequency for motor control
#define PWM_RESOLUTION_MOTOR LEDC_TIMER_8_BIT // PWM resolution for motor control
#define PWM_TIMER LEDC_TIMER_0            // PWM timer for motor control
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE // Use high-speed mode for PWM
#define IN1_CHANNEL LEDC_CHANNEL_0        // PWM channel for IN1
#define IN2_CHANNEL LEDC_CHANNEL_1        // PWM channel for IN2
#define IN3_CHANNEL LEDC_CHANNEL_2        // PWM channel for IN3
#define IN4_CHANNEL LEDC_CHANNEL_3        // PWM channel for IN4

// ------------------------ ENCODER & PID ------------------------
const int slotsPerRevolution = 20;      // Number of encoder slots per revolution for RPM calculation
unsigned long lastYTime = 0, lastXTime = 0;  // Time stamps for the encoder interrupts
float yRPM = 0, xRPM = 0;              // RPM values for Y and X axes
unsigned long lastPrintTime = 0;       // Time stamp for printing RPM data

#define DEBOUNCE_DELAY 50               // Debounce delay for switches (milliseconds)

// Variables for target RPMs (can change at runtime)
float TARGET_RPM_X = 150;              
float TARGET_RPM_Y = 150;

#define BASE_PWM 80                     // Base PWM value for motor control

// PID control constants (for tuning the speed control)
float Kp = 7.0, Ki = 0.2, Kd = 0.25;  // Proportional, Integral, Derivative constants
float previousErrorY = 0, integralY = 0;
float previousErrorX = 0, integralX = 0;

float movementCorrectionFactor = 1.0;   // Correction factor for movement calibration
Preferences preferences;                // For storing and retrieving preferences (like correction factor)

// ------------------------ DEBOUNCE FUNCTION ------------------------
unsigned long lastDebounceTime = 0;     // Timestamp for debouncing
bool lastYLimitState = HIGH;            // Last state of the Y-axis limit switch
bool lastXLimitState = HIGH;            // Last state of the X-axis limit switch

// ------------------------ LED STATE FUNCTION ------------------------
void setLEDState(bool green, bool yellow, bool red) {
  // Set the states for the green, yellow, and red LEDs based on the input parameters
  digitalWrite(LED_GREEN, green);
  digitalWrite(LED_YELLOW, yellow);
  digitalWrite(LED_RED, red);
}

// ------------------------ ENCODER INTERRUPTS ------------------------
void IRAM_ATTR encoderYISR() {
  unsigned long currentTime = micros(); // Get the current time in microseconds
  if (currentTime - lastYTime > DEBOUNCE_DELAY) {  // Ensure enough time has passed for debounce
    unsigned long delta = currentTime - lastYTime;  // Calculate time since last interrupt
    if (delta > 1000 && delta < 500000)  // Ignore very fast or very slow pulses
      yRPM = (60000000.0 / delta) / slotsPerRevolution;  // Calculate RPM from time difference
    lastYTime = currentTime;  // Update the last time
  }
}

void IRAM_ATTR encoderXISR() {
  unsigned long currentTime = micros(); // Get the current time in microseconds
  if (currentTime - lastXTime > DEBOUNCE_DELAY) {  // Ensure enough time has passed for debounce
    unsigned long delta = currentTime - lastXTime;  // Calculate time since last interrupt
    if (delta > 1000 && delta < 500000)  // Ignore very fast or very slow pulses
      xRPM = (60000000.0 / delta) / slotsPerRevolution;  // Calculate RPM from time difference
    lastXTime = currentTime;  // Update the last time
  }
}

// ------------------------ PWM SETUP ------------------------
void setupPWMChannel(int pin, ledc_channel_t channel) {
  // Configure the PWM channel for motor control
  ledc_channel_config_t config = {
    .gpio_num = pin,                  // GPIO pin number
    .speed_mode = PWM_SPEED_MODE,     // Speed mode for PWM
    .channel = channel,               // Channel number for PWM
    .intr_type = LEDC_INTR_DISABLE,    // No interrupt for PWM
    .timer_sel = PWM_TIMER,           // Timer number for PWM
    .duty = 0,                        // Initial duty cycle (0 means motor is stopped)
    .hpoint = 0                       // Initial duty cycle offset (unused here)
  };
  ledc_channel_config(&config);  // Apply the PWM configuration
}

// ------------------------ MOTOR CONTROL ------------------------
void stopMotor() {
  // Stop all motors by setting their PWM duty cycle to 0
  for (int ch = 0; ch <= 3; ch++) {
    ledc_set_duty(PWM_SPEED_MODE, (ledc_channel_t)ch, 0);
    ledc_update_duty(PWM_SPEED_MODE, (ledc_channel_t)ch);
  }
  Serial.println("Motor stopped");
}

// ------------------------ EMERGENCY STOP ------------------------
void emergencyStop() {
  // Emergency stop function to immediately stop motors and lift the pen
  stopMotor();                         // Stop all motors
  moveServo(PEN_UP_ANGLE);             // Lift the pen to avoid damage
  setLEDState(false, false, true);      // Turn on red LED for emergency indication
  Serial.println("EMERGENCY STOP! Motors stopped and pen lifted.");

  // Reset RPM and PID state to prevent any movement after emergency stop
  yRPM = xRPM = 0;
  integralX = integralY = 0;
  previousErrorX = previousErrorY = 0;

  while (true) delay(1000);            // Stay in this state indefinitely
}

// ------------------------ NORMAL STOP ------------------------
void normalStop() {
  // Normal stop function to stop motors without affecting the pen position
  stopMotor();                        // Stop all motors
  setLEDState(true, false, false);     // Turn on green LED to indicate normal stop
  Serial.println("Normal stop completed.");
}

// ------------------------ MOVE SERVO ------------------------
void moveServo(int targetAngle) {
  // Move the servo to the target angle (pen up or pen down)
  int us = map(targetAngle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);  // Convert angle to microseconds
  int duty = (us * (1 << 16)) / (1000000 / SERVO_FREQ);  // Convert microseconds to duty cycle
  ledc_set_duty(PWM_SPEED_MODE, SERVO_CHANNEL, duty);    // Set the duty cycle for the servo
  ledc_update_duty(PWM_SPEED_MODE, SERVO_CHANNEL);        // Update the PWM duty cycle
  currentServoAngle = targetAngle;                         // Update current servo angle
  delay(300);                                              // Wait for the servo to move
}

// ------------------------ PID CONTROL ------------------------
void applyPID_X(int dir) {
  // Check if the limit switch is pressed for the X axis and stop motor if necessary
  if ((dir > 0 && digitalRead(X_LIMIT_RIGHT) == LOW) || (dir < 0 && digitalRead(X_LIMIT_LEFT) == LOW)) {
    stopMotor(); return;
  }
  // PID control logic for X axis movement
  float error = TARGET_RPM_X - xRPM;                    // Calculate RPM error
  integralX += error;                                    // Accumulate the error for integral term
  integralX = constrain(integralX, -500, 500);           // Prevent integral windup
  float output = Kp * error + Ki * integralX + Kd * (error - previousErrorX);  // Calculate PID output
  int speed = constrain(BASE_PWM + abs(output), 0, 255); // Calculate motor speed
  ledc_set_duty(PWM_SPEED_MODE, IN3_CHANNEL, dir > 0 ? speed : 0);  // Set PWM duty for IN3
  ledc_set_duty(PWM_SPEED_MODE, IN4_CHANNEL, dir < 0 ? speed : 0);  // Set PWM duty for IN4
  ledc_update_duty(PWM_SPEED_MODE, IN3_CHANNEL);          // Update PWM duty cycle
  ledc_update_duty(PWM_SPEED_MODE, IN4_CHANNEL);          // Update PWM duty cycle
  previousErrorX = error;                                // Update previous error for next iteration
}

void applyPID_Y(int dir) {
  // Check if the limit switch is pressed for the Y axis and stop motor if necessary
  if ((dir > 0 && digitalRead(Y_LIMIT_TOP) == LOW) || (dir < 0 && digitalRead(Y_LIMIT_BOTTOM) == LOW)) {
    stopMotor(); return;
  }
  // PID control logic for Y axis movement
  float error = TARGET_RPM_Y - yRPM;                    // Calculate RPM error
  integralY += error;                                    // Accumulate the error for integral term
  integralY = constrain(integralY, -500, 500);           // Prevent integral windup
  float output = Kp * error + Ki * integralY + Kd * (error - previousErrorY);  // Calculate PID output
  int speed = constrain(BASE_PWM + abs(output), 0, 255); // Calculate motor speed
  ledc_set_duty(PWM_SPEED_MODE, IN1_CHANNEL, dir > 0 ? speed : 0);  // Set PWM duty for IN1
  ledc_set_duty(PWM_SPEED_MODE, IN2_CHANNEL, dir < 0 ? speed : 0);  // Set PWM duty for IN2
  ledc_update_duty(PWM_SPEED_MODE, IN1_CHANNEL);          // Update PWM duty cycle
  ledc_update_duty(PWM_SPEED_MODE, IN2_CHANNEL);          // Update PWM duty cycle
  previousErrorY = error;                                // Update previous error for next iteration
}

// Set target RPMs dynamically
void setTargetRPM(float rpmX, float rpmY) {
  TARGET_RPM_X = rpmX;
  TARGET_RPM_Y = rpmY;
}

// Time-based movement with PID speed control
void timedMove(int dirX, int dirY, unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    // Check for emergency stop during movement
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      emergencyStop();  // Trigger emergency stop if the button is pressed
      return;  // Exit the function immediately
    }
    applyPID_X(dirX);   // Apply PID control for X axis
    applyPID_Y(dirY);   // Apply PID control for Y axis
    delay(5);           // Small delay to avoid overloading the system
  }
  stopMotor();  // Stop the motors when the movement is complete
}

void timedMoveDiagonal(int dirX, int dirY, unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    // Check for emergency stop during diagonal movement
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      emergencyStop();  // Trigger emergency stop if the button is pressed
      return;  // Exit the function immediately
    }
    applyPID_X(dirX);   // Apply PID control for X axis
    applyPID_Y(dirY);   // Apply PID control for Y axis
    delay(5);           // Small delay to avoid overloading the system
  }
  stopMotor();  // Stop the motors when the movement is complete
}

void driveX(int dir) { applyPID_X(dir); }
void driveY(int dir) { applyPID_Y(dir); }

void driveXY(int dirX, int dirY) {
  unsigned long moveDuration = 50;
  unsigned long interval = 5;
  unsigned long lastYUpdate = 0;
  float ratio = (float)TARGET_RPM_Y / TARGET_RPM_X;
  unsigned long start = millis();
  while (millis() - start < moveDuration) {
    unsigned long now = millis();
    applyPID_X(dirX);
    if ((now - lastYUpdate) >= (interval * ratio)) {
      applyPID_Y(dirY);
      lastYUpdate = now;
    }
    delay(1);
  }
}

// ------------------------ HOMING ------------------------
void homeAxisY() {
  // Homing process for Y axis
  while (digitalRead(Y_LIMIT_BOTTOM) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) { 
    driveY(-1); delay(5); 
  }
  stopMotor(); delay(100);
  for (int i = 0; i < 10; i++) { driveY(1); delay(5); }  // Move slightly up to clear the bottom switch
  stopMotor(); delay(100);
  // Move down again if necessary to fine-tune position
  while (digitalRead(Y_LIMIT_BOTTOM) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) { 
    driveY(-1); delay(10); 
  }
  stopMotor(); delay(100);
}

void homeAxisX() {
  // Homing process for X axis
  while (digitalRead(X_LIMIT_LEFT) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) { 
    driveX(-1); delay(5); 
  }
  stopMotor(); delay(100);
  for (int i = 0; i < 10; i++) { driveX(1); delay(5); }  // Move slightly right to clear the left switch
  stopMotor(); delay(100);
  // Move left again if necessary to fine-tune position
  while (digitalRead(X_LIMIT_LEFT) == HIGH && digitalRead(EMERGENCY_STOP_PIN) == HIGH) { 
    driveX(-1); delay(10); 
  }
  stopMotor(); delay(100);
}

void autoHome() {
  // Turn on the yellow LED to indicate the homing process is in progress
  setLEDState(false, true, false); // Yellow on
  
  Serial.println("Homing to bottom-left position...");
  homeAxisX(); homeAxisY();
  Serial.println("Homing complete.");
  
  // After homing is done, change the LED to green
  setLEDState(true, false, false); // Green on
}

// ------------------------ CALIBRATION ------------------------
void calibrateMovement() {
  const unsigned long calibrationMoveTime = 500; // Time for calibration move
  Serial.println("Calibration: Drawing a horizontal line of 50mm...");

  // Start homing to make sure plotter is at a known position
  autoHome();  
  // Turn on the yellow LED to indicate the calibration process is in progress
  setLEDState(false, true, false); // Yellow on
  moveServo(PEN_DOWN_ANGLE); delay(300);  // Lower the pen before starting

  unsigned long startMove = millis();
  while (millis() - startMove < calibrationMoveTime) {
    // Check for emergency stop during calibration movement
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      emergencyStop();  // Trigger emergency stop if the button is pressed
      return;  // Exit the function immediately
    }
    driveX(1); delay(10);  // Move in X direction for calibration
  }
  stopMotor(); delay(300); // Stop motor after calibration move
  moveServo(PEN_UP_ANGLE); delay(300);  // Lift the pen after calibration

  // Prompt user for the measured distance
  Serial.println("Enter measured line length in mm:");
  String input = "";
  while (!Serial.available()) delay(10);  // Wait for user input
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') break;
    input += c;
  }

  // Update correction factor based on user input
  float measuredMM = input.toFloat();
  if (measuredMM > 0) {
    movementCorrectionFactor = 50.0 / measuredMM;  // Adjust correction factor
    preferences.putFloat("corrFactor", movementCorrectionFactor);
    Serial.print("New correction factor: ");
    Serial.println(movementCorrectionFactor);
  } else {
    Serial.println("Invalid input.");
  }
  autoHome();  // Move back to the home position after calibration
}

// ------------------------ DRAWING ------------------------
void drawNikolausHouse() {
  const unsigned long stepTime = 480;
  const unsigned long sdiagTime = 230;
  const unsigned long ldiagTime = 485;
  const unsigned long pauseTime = 400;

  Serial.println("Drawing Nikolaus House...");

  // Start homing to make sure the plotter is at a known position
  autoHome();

  // Turn on the yellow LED to indicate the drawing process is in progress
  setLEDState(false, true, false); // Yellow on

  moveServo(PEN_UP_ANGLE);  
  delay(800);

  setTargetRPM(150, 150);
  timedMove(0, 1, 300);  // Move in Y direction
  normalStop(); // Normal stop after Y move
  setLEDState(false, true, false); // Yellow on
  delay(300);

  timedMove(1, 0, 200);  // Move in X direction
  normalStop();          // Normal stop after X move
  setLEDState(false, true, false); // Yellow on
  delay(300);

  setLEDState(false, true, false); // Yellow on

  moveServo(PEN_DOWN_ANGLE);  
  delay(800);

  timedMove(1, 0, stepTime);  // Draw one side of the square
  normalStop();               // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  timedMove(0, 1, stepTime);  // Draw the second side of the square
  normalStop();               // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  timedMove(-1, 0, stepTime);  // Draw the third side of the square
  normalStop();                // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  timedMove(0, -1, stepTime);  // Draw the fourth side of the square
  normalStop();               // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  // Set target RPM and use timedMoveDiagonal for diagonal movements
  setTargetRPM(150, 150);

  // Use timedMoveDiagonal for the long diagonal (lower left to upper right)
  timedMoveDiagonal(1, 1, ldiagTime);  // Diagonal movement (X and Y move simultaneously)
  normalStop();                        // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  // Use timedMoveDiagonal for the small diagonal (upper right to lower left)
  timedMoveDiagonal(-1, 1, sdiagTime);  // Diagonal movement (X moves left, Y moves up)
  normalStop();                         // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  // Use timedMoveDiagonal for the small diagonal (lower left to upper right)
  timedMoveDiagonal(-1, -1, sdiagTime);  // Diagonal movement (X moves left, Y moves down)
  normalStop();                         // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  // Use timedMoveDiagonal for the long diagonal (upper right to lower left)
  timedMoveDiagonal(1, -1, ldiagTime);  // Diagonal movement (X moves right, Y moves down)
  normalStop();                         // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  delay(pauseTime);

  moveServo(PEN_UP_ANGLE);  
  delay(500);

  autoHome();  // Return to home position

  // Turn on the green LED after the drawing process is complete
  setLEDState(true, false, false); // Green on

  Serial.println("Nikolaus House drawing complete.");
}

// ------------------------ SETUP & LOOP ------------------------
void setup() {
  delay(2000);
  Serial.begin(115200);  // Initialize serial communication

  // Initialize pin modes for input and output
  pinMode(Y_LIMIT_TOP, INPUT_PULLUP);
  pinMode(Y_LIMIT_BOTTOM, INPUT_PULLUP);
  pinMode(X_LIMIT_LEFT, INPUT_PULLUP);
  pinMode(X_LIMIT_RIGHT, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(ENCODER_Y_PIN, INPUT_PULLUP);
  pinMode(ENCODER_X_PIN, INPUT_PULLUP);
  pinMode(LID_SENSOR_PIN, INPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  setLEDState(true, false, false); // Default: Green LED on

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_Y_PIN), encoderYISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_X_PIN), encoderXISR, FALLING);

  // Configure PWM for motor control
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

  // Configure PWM for servo control
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

  // Load saved preferences for movement correction factor
  preferences.begin("plotter", false);
  if (preferences.isKey("corrFactor")) {
    movementCorrectionFactor = preferences.getFloat("corrFactor", 1.0);
  }

  autoHome();  // Home the plotter
  delay(2000); // Give time for homing
  setLEDState(true, false, false); // Green LED on after homing
}

void loop() {
  // Continuously check if the lid is open
  bool lidOpen = digitalRead(LID_SENSOR_PIN) == HIGH;

  if (lidOpen) {
    normalStop();  // Stop gracefully if the lid is open
    moveServo(PEN_UP_ANGLE);
    while (digitalRead(LID_SENSOR_PIN) == HIGH) {
      setLEDState(false, false, true);  // Blink red LED to indicate lid is open
      delay(300);
      setLEDState(false, false, false);
      delay(300);
    }
    setLEDState(true, false, false);  // Turn green LED on once lid is closed
  }

    // Continuously check for emergency stop and Hall effect sensor
  if (digitalRead(EMERGENCY_STOP_PIN) == HIGH || digitalRead(LID_SENSOR_PIN) == HIGH) {
    emergencyStop();  // Trigger emergency stop if the button or Hall effect sensor is triggered
  }

  // Listen for serial commands to control the plotter
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'n': drawNikolausHouse(); break;  // Draw the Nikolaus house
      case 'u': moveServo(PEN_UP_ANGLE); break;  // Lift the pen
      case 'l': moveServo(PEN_DOWN_ANGLE); break;  // Lower the pen
      case 'h': autoHome(); break;  // Home the plotter
      case 'c': calibrateMovement(); break;  // Calibrate the plotter
    }
  }
}
