#pragma once
#include "Arduino.h"
struct CRGB { uint8_t r,g,b; CRGB(){} CRGB(uint8_t a,uint8_t b_,uint8_t c):r(a),g(b_),b(c){} static const CRGB Black; };
struct WS2812{}; struct GRB{};
struct CFastLED { template<class C,int P,class O> void addLeds(CRGB*,int){} void setBrightness(uint8_t){} void setMaxRefreshRate(int){} void show(){} };
extern CFastLED FastLED;
