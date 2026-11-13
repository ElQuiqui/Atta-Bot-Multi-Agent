#pragma once
#include "Arduino.h"
enum { WL_IDLE_STATUS, WL_NO_SSID_AVAIL, WL_SCAN_COMPLETED, WL_CONNECTED, WL_CONNECT_FAILED, WL_CONNECTION_LOST, WL_DISCONNECTED };
enum { WIFI_OFF, WIFI_STA };
struct WiFiClass { int status(){return 3;} void mode(int){} void begin(const char*,const char*){} void setHostname(const char*){} const char* getHostname(){return "";} String macAddress(){return String();} IPAddress localIP(){return IPAddress();} IPAddress gatewayIP(){return IPAddress();} IPAddress subnetMask(){return IPAddress();} IPAddress dnsIP(){return IPAddress();} void disconnect(){} int RSSI(){return 0;} bool setSleep(bool){return true;} };
extern WiFiClass WiFi;
