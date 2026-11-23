#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstring>

struct TargetPosition {
  float cx = 0.0f;
  float cy = 0.0f;
  bool hasPickupPoint = false;
  float pickupCx = 0.0f;
  float pickupCy = 0.0f;
};

class TargetReceiver {
 public:
  explicit TargetReceiver(Stream& input) : input_(input) {}

  void poll() {
    for (size_t remaining = sizeof(line_); remaining > 0 && input_.available() > 0; --remaining) {
      const char next = static_cast<char>(input_.read());
      if (next == '\n') {
        if (!discardLine_ && length_ > 0) {
          line_[length_] = '\0';
          acceptLine(line_);
        }
        discardLine_ = false;
        length_ = 0;
      } else if (!discardLine_) {
        if (length_ == sizeof(line_) - 1) {
          discardLine_ = true;
        } else {
          line_[length_++] = next;
        }
      }
    }
  }

  bool latest(TargetPosition& position, unsigned long timeoutMs = 500) {
    if (!found_ || millis() - receivedAtMs_ >= timeoutMs) {
      found_ = false;
      return false;
    }
    position = position_;
    return true;
  }

  void acceptLine(const char* line) {
    JsonDocument message;
    if (deserializeJson(message, line) || std::strcmp(message["type"] | "", "target") != 0) {
      return;
    }

    found_ = false;
    if (!message["found"].is<bool>() || !message["found"].as<bool>() ||
        !message["cx"].is<float>() || !message["cy"].is<float>()) {
      return;
    }
    TargetPosition position;
    position.cx = message["cx"].as<float>();
    position.cy = message["cy"].as<float>();
    if (!validCoordinate(position.cx) || !validCoordinate(position.cy)) {
      return;
    }
    if (message["pickup_cx"].is<float>() && message["pickup_cy"].is<float>()) {
      const float pickupCx = message["pickup_cx"].as<float>();
      const float pickupCy = message["pickup_cy"].as<float>();
      if (validCoordinate(pickupCx) && validCoordinate(pickupCy)) {
        position.hasPickupPoint = true;
        position.pickupCx = pickupCx;
        position.pickupCy = pickupCy;
      }
    }
    position_ = position;
    receivedAtMs_ = millis();
    found_ = true;
  }

 private:
  static bool validCoordinate(float value) {
    return value >= -1.0f && value <= 1.0f;
  }

  Stream& input_;
  char line_[512];
  size_t length_ = 0;
  bool discardLine_ = false;
  bool found_ = false;
  TargetPosition position_;
  unsigned long receivedAtMs_ = 0;
};
