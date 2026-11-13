#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdarg>
#include <string>
#include <algorithm>
using std::isfinite;
#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define FALLING 2
#define IRAM_ATTR
#define HEX 16
inline long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
typedef uint8_t byte;
template<class T, class L, class H> auto constrain(T x, L l, H h) -> decltype(x+l+h) { return x<l?l:(x>h?h:x); }
using std::min; using std::max; using std::abs;
extern unsigned long g_ms; inline unsigned long millis(){return g_ms;} inline unsigned long micros(){static unsigned long k=0; return g_ms*1000UL + (++k);}
inline void delay(unsigned long){} inline void delayMicroseconds(unsigned){}
inline long random(long a){return a?rand()%a:0;} inline long random(long a,long b){return a+random(b-a);}
inline void randomSeed(unsigned long){}
inline int digitalRead(int){return 0;} inline void digitalWrite(int,int){} inline void pinMode(int,int){}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int, void(*)(), int){}
inline void noInterrupts(){} inline void interrupts(){}
class String {
  std::string s;
public:
  String(){} String(const char*c):s(c?c:""){} String(const std::string&x):s(x){}
  String(uint32_t v, int base){char b[20]; snprintf(b,20,base==16?"%x":"%u",v); s=b;}
  explicit String(int v){s=std::to_string(v);} explicit String(float v){s=std::to_string(v);}
  const char* c_str() const {return s.c_str();}
  unsigned length() const {return s.size();}
  int indexOf(char c, unsigned from=0) const {auto p=s.find(c,from); return p==std::string::npos?-1:(int)p;}
  String substring(unsigned a) const {return a>=s.size()?String():String(s.substr(a));}
  String substring(unsigned a, unsigned b) const {return a>=s.size()?String():String(s.substr(a,b-a));}
  long toInt() const {return atol(s.c_str());} float toFloat() const {return atof(s.c_str());}
  void trim(){}
  bool operator==(const String&o) const {return s==o.s;} bool operator!=(const String&o) const {return s!=o.s;}
  bool operator==(const char*o) const {return s==o;} bool operator!=(const char*o) const {return s!=o;}
  bool operator<(const String&o) const {return s<o.s;}
  String operator+(const String&o) const {return String(s+o.s);}
  friend String operator+(const char*a,const String&b){return String(std::string(a)+b.s);}
};
struct HardwareSerial { void begin(long){} template<class T> void print(T){} template<class T> void println(T){} void println(){} int printf(const char*,...){return 0;} int available(){return 0;} String readString(){return String();} };
extern HardwareSerial Serial;
struct EspClass { uint64_t getEfuseMac(){return 0;} void restart(){} const char* getSdkVersion(){return "";} uint32_t getPsramSize(){return 0;} uint32_t getFreeHeap(){return 0;} uint32_t getFlashChipSize(){return 0;} uint32_t getCpuFreqMHz(){return 240;} uint8_t getChipRevision(){return 0;} const char* getChipModel(){return "";} uint8_t getChipCores(){return 2;} };
extern EspClass ESP;
enum { LEDC_AUTO_CLK, LEDC_USE_APB_CLK };
inline bool ledcSetClockSource(int){return true;} inline bool ledcAttach(int,int,int){return true;}
inline void ledcWrite(int,uint32_t){} inline uint32_t ledcReadFreq(int){return 0;}
class IPAddress { uint32_t a=0; public: IPAddress(){} IPAddress(int x,int y,int z,int w):a((x<<24)|(y<<16)|(z<<8)|w){} bool fromString(const String&){return true;} String toString() const {return String();} bool operator==(const IPAddress&o) const {return a==o.a;} bool operator!=(const IPAddress&o) const {return a!=o.a;} };
