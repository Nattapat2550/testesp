#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include "target_receiver.h"

class WiFiTargetReceiver {
 public:
  static constexpr uint16_t PORT = 8765;

  void begin(const char* ssid, const char* password) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
  }

  void poll() {
    const IPAddress address = WiFi.status() == WL_CONNECTED ? WiFi.localIP() : IPAddress();
    if (address != listeningAddress_) {
      udp_.stop();
      receiver_.reset();
      listeningAddress_ = IPAddress();
      if (address != IPAddress() && udp_.begin(address, PORT)) {
        listeningAddress_ = address;
      }
    }
    if (listeningAddress_ == IPAddress()) {
      return;
    }

    // Bound the receive work so the main loop can keep servicing the motor watchdog.
    for (int remaining = 4; remaining > 0; --remaining) {
      const int size = udp_.parsePacket();
      if (size <= 0) {
        break;
      }
      char packet[512];
      const int received = udp_.read(packet, sizeof(packet) - 1);
      if (received == size) {
        packet[received] = '\0';
        receiver_.acceptLine(packet);
      }
      while (udp_.available() > 0 && udp_.read(packet, sizeof(packet)) > 0) {
        // Discard the rest of an oversized datagram.
      }
    }
  }

  IPAddress listeningAddress() const { return listeningAddress_; }

  bool latest(TargetPosition& position, unsigned long timeoutMs = 500) {
    return receiver_.latest(position, timeoutMs);
  }

 private:
  WiFiUDP udp_;
  IPAddress listeningAddress_;
  TargetReceiver receiver_;
};
