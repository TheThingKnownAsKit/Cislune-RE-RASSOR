// ──────────────────────────────────────────────────────────────
// Minimal USB-Serial “alive” sketch for Teensy 4.1
//  • Compile with USB Type = Serial (or define USB_SERIAL via PlatformIO)
//  • Enumerates as VID 16C0, PID 0483  →  /dev/ttyACM0 on Linux
// ──────────────────────────────────────────────────────────────
#include <Arduino.h> 
const int LED_PIN = 13;          // Teensy 4.x on-board LED

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Start CDC-ACM port at 115 200 baud (speed is advisory on USB)
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Give host up to 3 s to enumerate (optional)
  }

  Serial.println("Teensy alive — USB Serial OK");
}

void loop()
{
  static uint32_t t0 = 0;
  if (millis() - t0 >= 1000) {        // 1 Hz blink
    t0 = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial.println("Heartbeat");
  }
}
