#include "target_receiver.h"
#include <cstring>

// Replace these example values with the coordinates in your pickup-point.json.
constexpr float PICKUP_CX = 0.10f;
constexpr float PICKUP_CY = 0.40f;
constexpr float POSITION_TOLERANCE = 0.03f;

TargetReceiver receiver(Serial);
const char* lastDecision = "";

const char* decideMovement() {
  TargetPosition target;
  if (!receiver.latest(target)) {
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
  receiver.poll();
  const char* decision = decideMovement();
  if (std::strcmp(decision, lastDecision) != 0) {
    Serial.println(decision);
    lastDecision = decision;
  }
  delay(1);
}
