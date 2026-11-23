#include <Arduino.h>

// Set to 1 to test incoming commands on the ESP32 without connecting actuators.
#ifndef ENABLE_COMMAND_BENCH
#define ENABLE_COMMAND_BENCH 0
#endif

#if ENABLE_COMMAND_BENCH
#if !defined(ARDUINO_ESP32_DEV)
#error "The command bench uses the ESP32 Dev Module target."
#endif
#include "bench_commands.h"
BenchCommands benchCommands(Serial);
#endif

#if defined(ARDUINO_ESP32_DEV)
constexpr uint8_t TRIG_PIN = 25;
constexpr uint8_t ECHO_PIN = 26;
#elif defined(ARDUINO_ARCH_ESP32)
#error "Select ESP32 Dev Module for the classic ESP32 DevKit pin mapping used by this sketch."
#else
constexpr uint8_t TRIG_PIN = 7;
constexpr uint8_t ECHO_PIN = 8;
#endif
constexpr unsigned long SAMPLE_INTERVAL_MS = 100;
constexpr unsigned long ECHO_TIMEOUT_US = 30000;

unsigned long lastTriggerMs = 0;
unsigned long sampleId = 0;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
#if ENABLE_COMMAND_BENCH
  benchCommands.poll();
#endif
  const unsigned long now = millis();
  if (now - lastTriggerMs < SAMPLE_INTERVAL_MS) {
    delay(1);
    return;
  }
  lastTriggerMs = now;

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  const unsigned long measuredAtMs = millis();
  const float distanceCm = echoUs / 58.0f;
  const bool valid = distanceCm >= 2.0f && distanceCm <= 400.0f;

  Serial.print(F("{\"type\":\"distance\",\"sample_id\":"));
  Serial.print(++sampleId);
  Serial.print(F(",\"distance_cm\":"));
  if (valid) {
    Serial.print(distanceCm, 2);
    Serial.print(F(",\"valid\":true"));
  } else {
    Serial.print(F("null,\"valid\":false"));
  }
  Serial.print(F(",\"age_ms\":"));
  Serial.print(millis() - measuredAtMs);
  Serial.println(F("}"));
}
