#include <Arduino.h>

// Motor command variables (could be PWM duty or desired velocity)
float leftCommand = 0.0f;
float rightCommand = 0.0f;
const unsigned long CMD_TIMEOUT_MS = 500;     // 0.5s command timeout for safety
unsigned long lastCommandTime = 0;

// Encoder count variables (must be updated by ISRs or library as wheels turn)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Timing for sending feedback
const unsigned long FEEDBACK_INTERVAL_MS = 50; // 20 Hz feedback
unsigned long lastFeedbackTime = 0;

void setup() {
  Serial.begin(115200);  // start USB serial
  // Setup motor control pins, encoder interrupts etc. here
  // e.g., pinMode, attachInterrupt for encoders, etc.
  lastCommandTime = millis();
}

void setMotorSpeeds(float leftCommand, float rightCommand) {
  // put stuff here later TODO
}

void loop() {
  // 1. Receive and parse commands from serial (if available)
  if (Serial.available()) {
    // Read a line of input
    static char buf[64];
    int n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = '\0';
      float cmdL, cmdR;
      if (sscanf(buf, "%f %f", &cmdL, &cmdR) == 2) {
        // Successfully parsed two floats
        leftCommand = cmdL;
        rightCommand = cmdR;
        lastCommandTime = millis();
      }
    }
  }

  // 2. Safety check: if no command received for a while, stop motors
  if (millis() - lastCommandTime > CMD_TIMEOUT_MS) {
    leftCommand = 0.0f;
    rightCommand = 0.0f;
  }

  // 3. Apply the latest command to the motors
  setMotorSpeeds(leftCommand, rightCommand);
  // (Implement setMotorSpeeds to control motor driver: e.g., convert velocity 
  // to PWM or use PID if the Teensy is doing closed-loop speed control.)

  // 4. Periodically send encoder feedback over serial
  // unsigned long now = millis();
  // if (now - lastFeedbackTime >= FEEDBACK_INTERVAL_MS) {
  //   lastFeedbackTime = now;
  //   long leftCount = leftEncoderCount;
  //   long rightCount = rightEncoderCount;
  //   // Send counts as text (left right newline)
  //   Serial.print(leftCount);
  //   Serial.print(" ");
  //   Serial.println(rightCount);
  // }
}
