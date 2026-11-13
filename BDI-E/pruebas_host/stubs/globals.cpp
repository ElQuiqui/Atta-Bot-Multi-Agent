#include "Arduino.h"
#include "WiFi.h"
#include "Wire.h"
#include "ArduinoOTA.h"
#include "FastLED.h"
HardwareSerial Serial; EspClass ESP; WiFiClass WiFi; TwoWire Wire; ArduinoOTAClass ArduinoOTA; CFastLED FastLED;

unsigned long g_ms=0;
#include <deque>
#include <string>
#include <vector>
std::deque<std::string> g_in; std::vector<std::string> g_out;
