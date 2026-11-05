#pragma once
#include "Arduino.h"

class Servo {
 public:
  void attach(uint8_t pin) { pin_ = pin; }
  void write(int angle) { rec("S " + std::to_string(g_now) + " " + std::to_string(pin_) + " " + std::to_string(angle)); }
 private:
  uint8_t pin_ = 0;
};
