#pragma once
#include "Arduino.h"
#include <deque>
#include <string>
#include <vector>
extern std::deque<std::string> g_in; extern std::vector<std::string> g_out;
struct WiFiUDP { std::string cur, pend; void begin(unsigned){}
 int parsePacket(){ if(g_in.empty()) return 0; cur=g_in.front(); g_in.pop_front(); return cur.size(); }
 int read(char*b,int n){ int k=std::min<int>(n,cur.size()); memcpy(b,cur.data(),k); return k; }
 IPAddress remoteIP(){return IPAddress(192,168,1,10);} void beginPacket(IPAddress,unsigned){pend.clear();}
 size_t write(const uint8_t*d,size_t n){pend.append((const char*)d,n); return n;} void endPacket(){g_out.push_back(pend);} };
