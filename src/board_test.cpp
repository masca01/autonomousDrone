#include <Arduino.h>

// Optional external LED: GPIO 4 -> 330 ohm resistor -> anode; cathode -> GND.
// This does not control the board's built-in RGB LED.
constexpr uint8_t kLedPin = 4;
constexpr uint32_t kBlinkIntervalMs = 1000;
uint32_t lastChangeMs = 0;
bool ledOn = false;

void setup() {
  Serial.begin(115200);
  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, LOW);
  Serial.println("ESP32 board test starting");
}

void loop() {
  const uint32_t now = millis();
  if (now - lastChangeMs >= kBlinkIntervalMs) {
    lastChangeMs = now;
    ledOn = !ledOn;
    digitalWrite(kLedPin, ledOn ? HIGH : LOW);
    Serial.print("ESP32 running | GPIO 4 LED: ");
    Serial.println(ledOn ? "ON" : "OFF");
  }
}
