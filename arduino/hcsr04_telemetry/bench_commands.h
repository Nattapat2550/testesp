#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstring>

// This receiver simulates actuator state for a serial bench test.
class BenchCommands {
 public:
  explicit BenchCommands(Stream& serial) : serial_(serial) {}

  void poll() {
    const unsigned long now = millis();
    if ((moving_ || armSeq_ != 0) && now - lastCommandMs_ >= WATCHDOG_MS) {
      stop();
      sessionReady_ = false;
      serial_.println(F("{\"type\":\"bench\",\"command\":\"STOP\",\"reason\":\"watchdog\"}"));
    }
    if (armSeq_ != 0 && now - armStartedMs_ >= ARM_DURATION_MS) {
      serial_.print(F("{\"type\":\"done\",\"seq\":"));
      serial_.print(armSeq_);
      serial_.println(F(",\"success\":true}"));
      armSeq_ = 0;
    }

    for (size_t remaining = sizeof(line_); remaining > 0 && serial_.available() > 0; --remaining) {
      const char next = static_cast<char>(serial_.read());
      if (next == '\n') {
        if (!discardLine_ && length_ > 0) {
          line_[length_] = '\0';
          handleLine();
        }
        length_ = 0;
        discardLine_ = false;
      } else if (!discardLine_) {
        if (length_ == sizeof(line_) - 1) {
          discardLine_ = true;
        } else {
          line_[length_++] = next;
        }
      }
    }
  }

 private:
  static constexpr unsigned long WATCHDOG_MS = 500;
  static constexpr unsigned long ARM_DURATION_MS = 250;

  Stream& serial_;
  char line_[256] = {};
  size_t length_ = 0;
  bool discardLine_ = false;
  bool sessionReady_ = false;
  bool moving_ = false;
  unsigned long lastSeq_ = 0;
  unsigned long lastCommandMs_ = 0;
  unsigned long armSeq_ = 0;
  unsigned long armStartedMs_ = 0;

  void stop() {
    moving_ = false;
    armSeq_ = 0;
  }

  static bool equal(const char* left, const char* right) {
    return std::strcmp(left, right) == 0;
  }

  void handleLine() {
    JsonDocument message;
    if (deserializeJson(message, line_, DeserializationOption::NestingLimit(2)) ||
        !equal(message["type"] | "", "cmd") || !message["seq"].is<unsigned long>()) {
      return;
    }
    const unsigned long seq = message["seq"].as<unsigned long>();
    const char* command = message["command"] | "";
    const float linear = message["linear"].as<float>();
    const float angular = message["angular"].as<float>();
    const bool accepted = seq > 0 && message["linear"].is<float>() && message["angular"].is<float>() &&
                          linear >= -1.0f && linear <= 1.0f && angular >= -1.0f && angular <= 1.0f &&
                          applyCommand(seq, command, linear, angular, millis());
    serial_.print(F("{\"type\":\"ack\",\"seq\":"));
    serial_.print(seq);
    serial_.println(accepted ? F(",\"accepted\":true}") : F(",\"accepted\":false}"));
    if (accepted && !equal(command, "HEARTBEAT")) {
      serial_.print(F("{\"type\":\"bench\",\"command\":\""));
      serial_.print(command);
      serial_.print(F("\",\"linear\":"));
      serial_.print(linear, 3);
      serial_.print(F(",\"angular\":"));
      serial_.print(angular, 3);
      serial_.println(F("}"));
    }
  }

  bool applyCommand(unsigned long seq, const char* command, float linear, float angular, unsigned long now) {
    if (equal(command, "STOP")) {
      stop();
      if (seq == 1) {
        sessionReady_ = true;
        lastSeq_ = seq;
      } else if (sessionReady_ && seq > lastSeq_) {
        lastSeq_ = seq;
      }
      return true;
    }
    if (!sessionReady_ || seq <= lastSeq_) {
      return false;
    }

    const bool baseCommand = (equal(command, "MOVE") && angular == 0.0f) ||
                             (equal(command, "TURN") && linear == 0.0f) || equal(command, "VELOCITY");
    const bool armCommand = equal(command, "LOWER") || equal(command, "GRAB") ||
                            equal(command, "LIFT") || equal(command, "RELEASE");
    if (baseCommand && armSeq_ == 0) {
      moving_ = linear != 0.0f || angular != 0.0f;
    } else if (armCommand && !moving_ && armSeq_ == 0) {
      armSeq_ = seq;
      armStartedMs_ = now;
    } else if (!equal(command, "HEARTBEAT")) {
      return false;
    }
    lastSeq_ = seq;
    lastCommandMs_ = now;
    return true;
  }
};
