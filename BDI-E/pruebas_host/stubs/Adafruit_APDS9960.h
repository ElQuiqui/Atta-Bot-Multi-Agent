#pragma once
#include "Arduino.h"
typedef enum { APDS9960_AGAIN_1X, APDS9960_AGAIN_4X, APDS9960_AGAIN_16X, APDS9960_AGAIN_64X } apds9960AGain_t;
// Proximidad y color controlables desde la prueba (por omisión, nada delante).
inline uint8_t  g_apdsProx = 0;
inline uint16_t g_apdsR = 0, g_apdsG = 0, g_apdsB = 0, g_apdsC = 0;
struct Adafruit_APDS9960 { bool begin(){return true;} void enableColor(bool){} void enableProximity(bool){} uint8_t readProximity(){return g_apdsProx;} bool colorDataReady(){return true;} void getColorData(uint16_t*r,uint16_t*g,uint16_t*b,uint16_t*c){ *r=g_apdsR; *g=g_apdsG; *b=g_apdsB; *c=g_apdsC; } void setADCGain(apds9960AGain_t){} void setADCIntegrationTime(uint16_t){} };
