#pragma once
// Improv Wi-Fi over serial (https://www.improv-wifi.com/serial/): lets the Lickety Split flasher page
// (ESP Web Tools) set the Wi-Fi network right after flashing, over the same USB cable.
#include <Arduino.h>
#include <WiFi.h>
#include <functional>

namespace improv {

// connect(ssid, pass) must try to join and return true once connected.
struct Hooks {
  std::function<bool(const String &, const String &)> connect;
  std::function<String()> url;          // e.g. "http://192.168.1.50/" ("" when not connected)
  const char *firmware, *version, *chip, *name;
};

void begin(const Hooks &h);
void loop();   // call often: reads Serial

}  // namespace improv
