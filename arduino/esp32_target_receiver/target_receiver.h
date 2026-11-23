#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstring>

struct TargetPosition {
  float cx = 0.0f;
  float cy = 0.0f;
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
    const float cx = message["cx"].as<float>();
    const float cy = message["cy"].as<float>();
    if (cx >= -1.0f && cx <= 1.0f && cy >= -1.0f && cy <= 1.0f) {
      position_.cx = cx;
      position_.cy = cy;
      receivedAtMs_ = millis();
      found_ = true;
    }
  }

 private:
  Stream& input_;
  char line_[256];
  size_t length_ = 0;
  bool discardLine_ = false;
  bool found_ = false;
  TargetPosition position_;
  unsigned long receivedAtMs_ = 0;
};
