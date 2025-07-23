// ------------------------ Headers ------------------------
#include "driver/ledc.h"
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

// ------------------------ WIFI CONFIG ------------------------
const char* ssid = "ADITYA";
const char* password = "pen_plotter";
WebServer server(80);

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
#define SERVO_CHANNEL LEDC_CHANNEL_4      // Channel for servo PWM control
#define SERVO_TIMER LEDC_TIMER_1          // Timer for servo control
#define SERVO_FREQ 50                     // Frequency for servo control
#define SERVO_RES LEDC_TIMER_16_BIT       // Resolution for servo control
#define SERVO_MIN_US 500                  // Minimum pulse width for servo (microseconds)
#define SERVO_MAX_US 2500                 // Maximum pulse width for servo (microseconds)
#define PEN_UP_ANGLE 60                   // Servo angle to lift the pen
#define PEN_DOWN_ANGLE 0                  // Servo angle to lower the pen

int currentServoAngle = PEN_UP_ANGLE;   // Variable to store the current servo angle

// ------------------------ MOTOR PWM CONFIG ------------------------
int manualDirX = 0;
int manualDirY = 0;
bool manualModeActive = false;
#define RPM_TIMEOUT_US 250000UL                   // 250 milliseconds (in microseconds)
#define PWM_FREQ 1000                             // PWM frequency for motor control
#define PWM_RESOLUTION_MOTOR LEDC_TIMER_8_BIT     // PWM resolution for motor control
#define PWM_TIMER LEDC_TIMER_0                    // PWM timer for motor control
#define PWM_SPEED_MODE LEDC_HIGH_SPEED_MODE       // Use high-speed mode for PWM
#define IN1_CHANNEL LEDC_CHANNEL_0                // PWM channel for IN1
#define IN2_CHANNEL LEDC_CHANNEL_1                // PWM channel for IN2
#define IN3_CHANNEL LEDC_CHANNEL_2                // PWM channel for IN3
#define IN4_CHANNEL LEDC_CHANNEL_3                // PWM channel for IN4

// ------------------------ ENCODER & PID ------------------------
const int slotsPerRevolution = 20;      // Number of encoder slots per revolution for RPM calculation
unsigned long lastYTime = 0, lastXTime = 0;  // Time stamps for the encoder interrupts
float yRPM = 0, xRPM = 0;              // RPM values for Y and X axes
unsigned long lastPrintTime = 0;       // Time stamp for printing RPM data

#define DEBOUNCE_DELAY 50               // Debounce delay for switches (milliseconds)

// Variables for target RPMs (can change at runtime)
float TARGET_RPM_X = 150;              
float TARGET_RPM_Y = 150;

#define BASE_PWM 80   // Base PWM value for motor control
#define MAX_PWM 250  // Safe max PWM to avoid overshoot

// PID control constants (for tuning the speed control)
float Kp = 5.0, Ki = 0.2, Kd = 0.15;  // Proportional, Integral, Derivative constants
float previousErrorY = 0, integralY = 0;
float previousErrorX = 0, integralX = 0;

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
  unsigned long currentTime = micros();
  if (currentTime - lastYTime > DEBOUNCE_DELAY) {
    unsigned long delta = currentTime - lastYTime;
    if (delta > 1000 && delta < 500000) {
      float newRPM = (60000000.0 / delta) / slotsPerRevolution;
      yRPM = 0.8 * yRPM + 0.2 * newRPM;  // Smooth update
    }
    lastYTime = currentTime;
  }
}

void IRAM_ATTR encoderXISR() {
  unsigned long currentTime = micros();
  if (currentTime - lastXTime > DEBOUNCE_DELAY) {
    unsigned long delta = currentTime - lastXTime;
    if (delta > 1000 && delta < 500000) {
      float newRPM = (60000000.0 / delta) / slotsPerRevolution;
      xRPM = 0.8 * xRPM + 0.2 * newRPM;  // Smooth update
    }
    lastXTime = currentTime;
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
}

// ------------------------ EMERGENCY STOP ------------------------
volatile bool emergencyStopRequested = false;
void emergencyStop() {
  emergencyStopRequested = true;

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Emergency Stop");
  Serial.println("Received WiFi Command: Emergency Stop");

  stopMotor();
  moveServo(PEN_UP_ANGLE);
  setLEDState(false, false, true);

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
void applyPID(
  float& currentRPM, float targetRPM,
  float& integral, float& prevError,
  ledc_channel_t chForward, ledc_channel_t chBackward,
  int dir, int limitHigh, int limitLow
) {
  // Check limits
  if ((dir > 0 && digitalRead(limitHigh) == LOW) || 
      (dir < 0 && digitalRead(limitLow) == LOW)) {
    stopMotor(); return;
  }

  // PID Calculation
  float error = targetRPM - currentRPM;
  integral += error;
  float maxIntegral = 1000.0 / (Kp + 0.01);  // Prevent divide by zero
  integral = constrain(integral, -maxIntegral, maxIntegral);
  float output = Kp * error + Ki * integral + Kd * (error - prevError);
  int speed = constrain(BASE_PWM + abs(output), 0, MAX_PWM);

  // Set PWM
  ledc_set_duty(PWM_SPEED_MODE, chForward, dir > 0 ? speed : 0);
  ledc_set_duty(PWM_SPEED_MODE, chBackward, dir < 0 ? speed : 0);
  ledc_update_duty(PWM_SPEED_MODE, chForward);
  ledc_update_duty(PWM_SPEED_MODE, chBackward);

  prevError = error;
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
    // Check RPMs live
    checkRPM();
    // Check for emergency stop during movement
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      emergencyStop();  // Trigger emergency stop if the button is pressed
      return;  // Exit the function immediately
    }
    applyPID(xRPM, TARGET_RPM_X, integralX, previousErrorX, IN3_CHANNEL, IN4_CHANNEL, dirX, X_LIMIT_RIGHT, X_LIMIT_LEFT);
    applyPID(yRPM, TARGET_RPM_Y, integralY, previousErrorY, IN1_CHANNEL, IN2_CHANNEL, dirY, Y_LIMIT_TOP, Y_LIMIT_BOTTOM);
    delay(5); // Small delay to avoid overloading the system
  }
  stopMotor();  // Stop the motors when the movement is complete
}

void driveX(int dir) { applyPID(xRPM, TARGET_RPM_X, integralX, previousErrorX, IN3_CHANNEL, IN4_CHANNEL, dir, X_LIMIT_RIGHT, X_LIMIT_LEFT); }
void driveY(int dir) { applyPID(yRPM, TARGET_RPM_Y, integralY, previousErrorY, IN1_CHANNEL, IN2_CHANNEL, dir, Y_LIMIT_TOP, Y_LIMIT_BOTTOM); }

void driveXY(int dirX, int dirY) {
  unsigned long moveDuration = 50;
  unsigned long interval = 5;
  unsigned long lastYUpdate = 0;
  float ratio = (TARGET_RPM_X != 0) ? ((float)TARGET_RPM_Y / TARGET_RPM_X) : 1.0;
  unsigned long start = millis();
  while (millis() - start < moveDuration) {
    unsigned long now = millis();
    applyPID(xRPM, TARGET_RPM_X, integralX, previousErrorX, IN3_CHANNEL, IN4_CHANNEL, dirX, X_LIMIT_RIGHT, X_LIMIT_LEFT);
    if ((now - lastYUpdate) >= (interval * ratio)) {
      applyPID(yRPM, TARGET_RPM_Y, integralY, previousErrorY, IN1_CHANNEL, IN2_CHANNEL, dirY, Y_LIMIT_TOP, Y_LIMIT_BOTTOM);
      lastYUpdate = now;
    }
    delay(1);
  }
}

void manualMove(int dirX, int dirY) {
  manualModeActive = true;
  while (manualModeActive) {
    if ((dirX > 0 && digitalRead(X_LIMIT_RIGHT) == LOW) || (dirX < 0 && digitalRead(X_LIMIT_LEFT) == LOW)) {
      stopMotor(); break;
    }

    if ((dirY > 0 && digitalRead(Y_LIMIT_TOP) == LOW) || (dirY < 0 && digitalRead(Y_LIMIT_BOTTOM) == LOW)) {
      stopMotor(); break;
    }

    // PID control logic for manual X and Y axis movement
    applyPID(xRPM, TARGET_RPM_X, integralX, previousErrorX, IN3_CHANNEL, IN4_CHANNEL, dirX, X_LIMIT_RIGHT, X_LIMIT_LEFT);
    applyPID(yRPM, TARGET_RPM_Y, integralY, previousErrorY, IN1_CHANNEL, IN2_CHANNEL, dirY, Y_LIMIT_TOP, Y_LIMIT_BOTTOM);
    delay(5);  // Small delay to avoid overloading the system
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
  //UI Integration
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Auto Home");
  Serial.println("Received WiFi Command: Auto Home");
  
  // After homing is done, change the LED to green
  setLEDState(true, false, false); // Green on
}

void handlePenUp() {
  moveServo(PEN_UP_ANGLE);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Pen moved UP");
  Serial.println("Received WiFi Command: Pen UP");
}

void handlePenDown() {
  moveServo(PEN_DOWN_ANGLE);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Pen moved DOWN");
  Serial.println("Received WiFi Command: Pen DOWN");
}

void checkRPM() {
  unsigned long now = micros();

  // Timeout-based decay to 0 RPM if no pulses recently
  if (now - lastXTime > RPM_TIMEOUT_US) xRPM = 0;
  if (now - lastYTime > RPM_TIMEOUT_US) yRPM = 0;


  Serial.print("X RPM: "); Serial.print(xRPM); Serial.print(" / Target: "); Serial.print(TARGET_RPM_X);
  Serial.print("  |  Y RPM: "); Serial.print(yRPM); Serial.print(" / Target: "); Serial.println(TARGET_RPM_Y);

  if (abs(xRPM - TARGET_RPM_X) > 5) Serial.println("Warning: X axis RPM deviates from target!");
  if (abs(yRPM - TARGET_RPM_Y) > 5) Serial.println("Warning: Y axis RPM deviates from target!");
}


// ------------------------ DRAWING ------------------------
void nonBlockingDelay(unsigned long delayTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime) {
    server.handleClient(); // Check for web requests (like emergency stop)
    if (emergencyStopRequested) return; // Exit immediately if emergency stop
    delay(10); // Small delay to prevent overloading
  }
}

void drawNikolausHouse() {
  const unsigned long stepTime = 400;
  const unsigned long sdiagTime = 200;
  const unsigned long ldiagTime = 405;
  const unsigned long pauseTime = 400;

  Serial.println("Drawing Nikolaus House...");

  // Start homing to make sure the plotter is at a known position
  autoHome();

  // Turn on the yellow LED to indicate the drawing process is in progress
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  moveServo(PEN_UP_ANGLE);
  nonBlockingDelay(800);
  if (emergencyStopRequested) return;

  setTargetRPM(150, 150);
  timedMove(0, 1, 1000);  // Move in Y direction
  normalStop(); // Normal stop after Y move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(300);
  if (emergencyStopRequested) return;

  timedMove(1, 0, 500);  // Move in X direction
  normalStop();          // Normal stop after X move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(300);
  if (emergencyStopRequested) return;

  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID

  moveServo(PEN_DOWN_ANGLE);  
  nonBlockingDelay(1000);
  if (emergencyStopRequested) return;

  setTargetRPM(160, 160);
  timedMove(1, 0, 500);  // Draw one side of the square
  normalStop();               // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(pauseTime);
  if (emergencyStopRequested) return;

  setTargetRPM(160, 160);
  timedMove(0, 1, 460);  // Draw the second side of the square
  normalStop();               // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(pauseTime);
  if (emergencyStopRequested) return;

  setTargetRPM(160, 160);
  timedMove(-1, 0, 470);  // Draw the third side of the square
  normalStop();                // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(pauseTime);
  if (emergencyStopRequested) return;

  setTargetRPM(160, 160);
  timedMove(0, -1, 465);  // Draw the fourth side of the square
  normalStop();               // Normal stop after this move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(pauseTime);
  if (emergencyStopRequested) return;
 
  // Use timedMove for the long diagonal (lower left to upper right)
  setTargetRPM(255, 225);
  timedMove(1, 1, 478);  // Diagonal movement (X and Y move simultaneously)
  normalStop();                        // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(800);
  if (emergencyStopRequested) return;
 /*
  // Use timedMove for the small diagonal (upper right to lower left)
  setTargetRPM(130, 130);
  timedMove(-1, 1, sdiagTime);  // Diagonal movement (X moves left, Y moves up)
  normalStop();                         // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(pauseTime);
  if (emergencyStopRequested) return;

  // Use timedMove for the small diagonal (lower left to upper right)
  setTargetRPM(130, 130);
  timedMove(-1, -1, sdiagTime);  // Diagonal movement (X moves left, Y moves down)
  normalStop();                         // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(pauseTime);
  if (emergencyStopRequested) return;
  
  // Use timedMove for the long diagonal (upper right to lower left)
  timedMove(1, -1, ldiagTime);  // Diagonal movement (X moves right, Y moves down)
  normalStop();                         // Normal stop after the diagonal move
  setLEDState(false, true, false); // Yellow on
  checkLidStatus(); //Check status of LID
  checkRPM();
  if (emergencyStopRequested) return;
  nonBlockingDelay(pauseTime);
  if (emergencyStopRequested) return;
*/
  moveServo(PEN_UP_ANGLE);  
  nonBlockingDelay(500);
  if (emergencyStopRequested) return;
  checkLidStatus(); //Check status of LID
  autoHome();  // Return to home position

  // Turn on the green LED after the drawing process is complete
  setLEDState(true, false, false); // Green on

  Serial.println("Nikolaus House drawing complete.");
}

void waitUntilSafeToProceed() {
  // 1. Emergency button check (must NOT be pressed)
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    Serial.println("Emergency button pressed. Halting.");
    setLEDState(false, false, true);  // Solid red
    emergencyStop();  // Full emergency stop
  }

  // 2. Hall sensor check (must have magnet nearby)
  while (digitalRead(LID_SENSOR_PIN) == LOW) {  // No magnet detected
    Serial.println("Magnet not detected. Waiting...");
    setLEDState(false, false, true);  // Red ON
    delay(300);
    setLEDState(false, false, false); // Red OFF
    delay(300);
  }

  // Once both conditions are satisfied
  Serial.println("Safety checks passed. Proceeding.");
  setLEDState(true, false, false);  // Green ON
}

void checkLidStatus() {
  static bool wasLidOpen = false;
  bool lidOpen = digitalRead(LID_SENSOR_PIN) == LOW;

  if (lidOpen && !wasLidOpen) {
    // Lid just opened — stop and lift pen
    normalStop();
    moveServo(PEN_UP_ANGLE);
    wasLidOpen = true;
  }

  while (digitalRead(LID_SENSOR_PIN) == LOW) {
    setLEDState(false, false, true);  // Blink red
    delay(300);
    setLEDState(false, false, false);
    delay(300);
  }

  if (wasLidOpen) {
    // Lid just closed
    Serial.println("Magnet detected. Resuming...");
    setLEDState(true, false, false);
    wasLidOpen = false;
  }
}

void handleManualCommand(char cmd) {
  switch (cmd) {
    case 'w': manualDirY = 1; manualDirX = 0; manualModeActive = true; break;
    case 's': manualDirY = -1; manualDirX = 0; manualModeActive = true; break;
    case 'a': manualDirX = -1; manualDirY = 0; manualModeActive = true; break;
    case 'd': manualDirX = 1; manualDirY = 0; manualModeActive = true; break;
    case 'q': manualModeActive = false; stopMotor(); break;
  }
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
  pinMode(ENCODER_Y_PIN, INPUT);
  pinMode(ENCODER_X_PIN, INPUT);
  pinMode(LID_SENSOR_PIN, INPUT_PULLUP);
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

  waitUntilSafeToProceed();  // Ensure safe state before continuing
  autoHome();  // Only start homing once safe
  delay(2000); // Give time for homing
  setLEDState(true, false, false); // Green LED on after homing

  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP address: "); Serial.println(WiFi.localIP());

  // Setup HTTP server routes
  server.on("/pen/up", handlePenUp);
  server.on("/pen/down", handlePenDown);
  server.on("/auto-home", autoHome);
  server.on("/emergency/activate", emergencyStop);
  server.on("/start", []() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Drawing Nikolaus House");
  drawNikolausHouse();
  });
    // Manual motion endpoints
        server.on("/move/up", []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", "Moving up");
        handleManualCommand('w');
        });

        server.on("/move/down", []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", "Moving up");
        handleManualCommand('s');
        });

        server.on("/move/left", []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", "Moving up");
        handleManualCommand('a');
        });

        server.on("/move/right", []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", "Moving up");
        handleManualCommand('d');
        });

        server.on("/move/stop", []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", "Stopped");
        handleManualCommand('q');
        });
  server.begin();
  Serial.println("HTTP server started");

  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "*");
    server.send(204); // No Content
    } else {
    server.send(404, "text/plain", "Not Found");
    }
  });
}

void loop() {
  server.handleClient(); // Check HTTP requests
  checkLidStatus(); //Check status of LID

  // Continuously check for emergency stop only
  if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    emergencyStop();  // Only emergency button triggers full stop
  }

  // Listen for serial commands to control the plotter
  while (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'n': drawNikolausHouse(); break;
      case 'u': moveServo(PEN_UP_ANGLE); break;
      case 'l': moveServo(PEN_DOWN_ANGLE); break;
      case 'h': autoHome(); break;
      case 'w': manualDirY = 1; manualDirX = 0; manualModeActive = true; break;
      case 's': manualDirY = -1; manualDirX = 0; manualModeActive = true; break;
      case 'a': manualDirX = -1; manualDirY = 0; manualModeActive = true; break;
      case 'd': manualDirX = 1; manualDirY = 0; manualModeActive = true; break;
      case 'q': manualModeActive = false; stopMotor(); break;
      default: Serial.print("Unknown command: "); Serial.println(cmd); break;
    }
  }
  // Handle manual movement if active
  if (manualModeActive) {
    if ((manualDirX > 0 && digitalRead(X_LIMIT_RIGHT) == LOW) ||
        (manualDirX < 0 && digitalRead(X_LIMIT_LEFT) == LOW) ||
        (manualDirY > 0 && digitalRead(Y_LIMIT_TOP) == LOW) ||
        (manualDirY < 0 && digitalRead(Y_LIMIT_BOTTOM) == LOW)) {
      stopMotor();
      manualModeActive = false;
    } else {
      applyPID(xRPM, TARGET_RPM_X, integralX, previousErrorX, IN3_CHANNEL, IN4_CHANNEL, manualDirX, X_LIMIT_RIGHT, X_LIMIT_LEFT);
      applyPID(yRPM, TARGET_RPM_Y, integralY, previousErrorY, IN1_CHANNEL, IN2_CHANNEL, manualDirY, Y_LIMIT_TOP, Y_LIMIT_BOTTOM);
    }
    delay(5);
  }
}