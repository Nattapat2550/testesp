#include <Arduino.h>
#include <ArduinoJson.h>
#include <cstring>

// Replace these example values with the coordinates in your pickup-point.json.
constexpr float PICKUP_CX = 0.10f;
constexpr float PICKUP_CY = 0.40f;
constexpr float POSITION_TOLERANCE = 0.03f;
constexpr unsigned long TARGET_TIMEOUT_MS = 500;

struct Target {
  bool found = false;
  float cx = 0.0f;
  float cy = 0.0f;
  unsigned long receivedAtMs = 0;
};

Target target;
char lineBuffer[256];
size_t lineLength = 0;
bool discardLine = false;
const char* lastDecision = "";

void acceptTarget(const char* line) {
  JsonDocument message;
  if (deserializeJson(message, line) || std::strcmp(message["type"] | "", "target") != 0) {
    return;
  }

  target.found = false;
  if (!message["found"].is<bool>() || !message["found"].as<bool>() ||
      !message["cx"].is<float>() || !message["cy"].is<float>()) {
    return;
  }
  const float cx = message["cx"].as<float>();
  const float cy = message["cy"].as<float>();
  if (!(cx >= -1.0f && cx <= 1.0f && cy >= -1.0f && cy <= 1.0f)) {
    return;
  }

  target.cx = cx;
  target.cy = cy;
  target.receivedAtMs = millis();
  target.found = true;
}

void readSerialTargets() {
  for (size_t remaining = sizeof(lineBuffer); remaining > 0 && Serial.available() > 0; --remaining) {
    const char next = static_cast<char>(Serial.read());
    if (next == '\n') {
      if (!discardLine && lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        acceptTarget(lineBuffer);
      }
      discardLine = false;
      lineLength = 0;
    } else if (!discardLine) {
      if (lineLength == sizeof(lineBuffer) - 1) {
        discardLine = true;
      } else {
        lineBuffer[lineLength++] = next;
      }
    }
  }
}

const char* decideMovement(unsigned long now) {
  if (!target.found || now - target.receivedAtMs >= TARGET_TIMEOUT_MS) {
    return "STOP";
  }
  const float errorX = target.cx - PICKUP_CX;
  const float errorY = target.cy - PICKUP_CY;

  // This example assumes closer floor targets appear lower in the image.
  if (errorY > POSITION_TOLERANCE) {
    return "STOP_TOO_CLOSE";
  }
  if (errorX > POSITION_TOLERANCE) {
    return "TURN_RIGHT";
  }
  if (errorX < -POSITION_TOLERANCE) {
    return "TURN_LEFT";
  }
  if (errorY < -POSITION_TOLERANCE) {
    return "APPROACH";
  }
  return "READY_TO_PICK";
}

void setup() {
  Serial.begin(115200);
}

void loop() {
  readSerialTargets();
  const char* decision = decideMovement(millis());
  if (std::strcmp(decision, lastDecision) != 0) {
    Serial.println(decision);
    lastDecision = decision;
  }
  delay(1);
}
