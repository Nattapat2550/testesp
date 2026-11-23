#include "target_receiver.h"
#include <cstring>

constexpr float POSITION_TOLERANCE = 0.03f;

TargetReceiver receiver(Serial);
const char* lastDecision = "";

const char* decideMovement() {
  TargetPosition target;
  if (!receiver.latest(target)) {
    return "STOP";
  }
  if (!target.hasPickupPoint) {
    return "WAIT_FOR_PICKUP_POINT";
  }
  const float errorX = target.cx - target.pickupCx;
  const float errorY = target.cy - target.pickupCy;

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
