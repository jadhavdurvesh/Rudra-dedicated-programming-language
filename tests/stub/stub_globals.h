// Include exactly once per test program (defines the globals the stub headers declare).
#pragma once
#include "Arduino.h"
#include "Wire.h"

uint32_t g_now = 0;
int g_pin_in[64] = {0};
int g_analog_in[64] = {0};
long g_pulse[64] = {0};
std::vector<std::string> g_events;
std::string g_tx;
std::vector<std::string> g_txlog;
std::string g_txcur;
std::deque<char> g_rx;
SerialClass Serial;
std::map<int, I2cDev> g_i2c;
TwoWire Wire;
