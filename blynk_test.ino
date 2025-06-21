// Blynk IoT Configuration
#define BLYNK_TEMPLATE_ID "TMPL4TYNOK31g"
#define BLYNK_TEMPLATE_NAME "DC Motor Control"
#define BLYNK_AUTH_TOKEN "anfkHzntCoge2rrMbzfLu6GogREnNcd6"  // Your actual token

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Wi-Fi credentials (replace with your real network info)
char ssid[] = "AB";
char pass[] = "123456789";

// Motor control pins
const int IN1 = 16;
const int IN2 = 17;

void setup() {
  // Start serial monitor for debugging
  Serial.begin(115200);

  // Set motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Initially turn motor off
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
}

// Virtual Pin V0 controls motor on/off
BLYNK_WRITE(V0) {
  int motorState = param.asInt(); // 1 = ON, 0 = OFF

  if (motorState == 1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);  // Motor forward
    Serial.println("Motor ON (Forward)");
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);  // Motor stop
    Serial.println("Motor OFF");
  }
}

void loop() {
  Blynk.run();
}