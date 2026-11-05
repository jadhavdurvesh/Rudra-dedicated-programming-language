#pragma once
#include <map>
#include "Arduino.h"

struct I2cDev { std::vector<uint8_t> data; bool fail = false; };
extern std::map<int, I2cDev> g_i2c;   // key: (addr << 9) | (reg & 0x1FF), reg -1 -> 0x1FF; fail flag lives on reg key 0x1FE

struct TwoWire {
  int addr = 0, pending_reg = -1;
  std::vector<uint8_t> tx;
  std::deque<uint8_t> rxq;
  static int key(int a, int r) { return (a << 9) | (r < 0 ? 0x1FF : (r & 0x1FF)); }
  bool failing() { return g_i2c[key(addr, 0x1FE)].fail; }
  void begin() {}
  void beginTransmission(uint8_t a) { addr = a; tx.clear(); }
  size_t write(uint8_t b) { tx.push_back(b); return 1; }
  uint8_t endTransmission(bool stop = true) {
    if (failing()) return 2;
    if (stop) {
      char hex[4];
      std::string h;
      for (uint8_t b : tx) { snprintf(hex, sizeof hex, "%02x", b); h += hex; }
      rec("I " + std::to_string(g_now) + " " + std::to_string(addr) + " " + h);
    } else {
      pending_reg = tx.empty() ? -1 : tx[0];
    }
    return 0;
  }
  uint8_t requestFrom(uint8_t a, uint8_t n) {
    addr = a;
    rxq.clear();
    if (failing()) return 0;
    auto it = g_i2c.find(key(a, pending_reg));
    if (it == g_i2c.end() || it->second.data.size() < n) return 0;
    for (int i = 0; i < n; i++) rxq.push_back(it->second.data[i]);
    return n;
  }
  int read() { if (rxq.empty()) return -1; uint8_t b = rxq.front(); rxq.pop_front(); return b; }
};
extern TwoWire Wire;
