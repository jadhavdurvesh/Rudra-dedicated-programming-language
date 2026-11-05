#pragma once
#include "Arduino.h"

#define NEO_RGB 0x06
#define NEO_GRB 0x52
#define NEO_KHZ800 0x0000

class Adafruit_NeoPixel {
 public:
  Adafruit_NeoPixel(uint16_t n, uint8_t pin, uint32_t type) : px_(n, 0), pin_(pin), type_(type) { (void)pin_; (void)type_; }
  void begin() {}
  void setPixelColor(uint16_t i, uint32_t c) { if (i < px_.size()) px_[i] = c; }
  void show() {
    std::string s = "N " + std::to_string(g_now);
    for (uint32_t c : px_) s += " " + std::to_string(c);
    rec(s);
  }
 private:
  std::vector<uint32_t> px_;
  uint8_t pin_;
  uint32_t type_;
};
