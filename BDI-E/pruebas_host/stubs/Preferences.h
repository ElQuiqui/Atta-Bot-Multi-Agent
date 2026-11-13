#pragma once
#include "Arduino.h"
struct Preferences { bool begin(const char*,bool){return true;} void end(){} int getInt(const char*,int d){return d;} void putInt(const char*,int){} float getFloat(const char*,float d){return d;} void putFloat(const char*,float){} uint16_t getUShort(const char*,uint16_t d){return d;} void putUShort(const char*,uint16_t){} };
