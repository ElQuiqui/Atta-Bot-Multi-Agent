#pragma once
#include "Arduino.h"
struct TwoWire { void begin(){} void setClock(long){} void beginTransmission(int){} int endTransmission(){return 1;} };
extern TwoWire Wire;
