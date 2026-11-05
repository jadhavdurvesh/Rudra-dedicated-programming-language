// Runs a generated sketch against the fake Arduino core.
// stdin:  RUN <end_ms>
//         <ms> PIN <pin> <0|1>             digital input level
//         <ms> ANALOG <pin> <value>        analog input value
//         <ms> I2C <addr> <reg> <hex>      register contents (reg -1 = none)
//         <ms> I2CFAIL <addr> <0|1>        device stops / starts answering
//         <ms> RX <text...>                line from the master
// stdout: one hardware event per line (see Arduino.h), then ---TX--- and the serial output.
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include "stub_globals.h"

#include SKETCH

struct Ev { long ms; std::string kind; std::string rest; };

static std::vector<uint8_t> from_hex(const std::string &h) {
  std::vector<uint8_t> v;
  for (size_t i = 0; i + 1 < h.size(); i += 2) v.push_back((uint8_t)strtol(h.substr(i, 2).c_str(), NULL, 16));
  return v;
}

int main() {
  std::string line;
  long end = 0;
  std::vector<Ev> evs;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    if (line.rfind("RUN ", 0) == 0) { end = atol(line.c_str() + 4); continue; }
    std::istringstream ss(line);
    Ev e;
    ss >> e.ms >> e.kind;
    std::getline(ss, e.rest);
    if (!e.rest.empty() && e.rest[0] == ' ') e.rest.erase(0, 1);
    evs.push_back(e);
  }
  // the very first events (t = 0) must be in place before setup() runs
  for (auto &e : evs) {
    if (e.ms != 0) continue;
    if (e.kind == "I2C") { int a, r; char h[64]; sscanf(e.rest.c_str(), "%d %d %63s", &a, &r, h); g_i2c[TwoWire::key(a, r)].data = from_hex(h); }
    else if (e.kind == "PIN") { int p, v; sscanf(e.rest.c_str(), "%d %d", &p, &v); g_pin_in[p] = v; }
    else if (e.kind == "ANALOG") { int p, v; sscanf(e.rest.c_str(), "%d %d", &p, &v); g_analog_in[p] = v; }
    else if (e.kind == "ECHO") { int p; long v; sscanf(e.rest.c_str(), "%d %ld", &p, &v); g_pulse[p] = v; }
  }
  g_now = 0;
  setup();
  for (long t = 0; t <= end; t++) {
    g_now = (uint32_t)t;
    for (auto &e : evs) {
      if (e.ms != t) continue;
      if (e.kind == "PIN") { int p, v; sscanf(e.rest.c_str(), "%d %d", &p, &v); g_pin_in[p] = v; }
      else if (e.kind == "ANALOG") { int p, v; sscanf(e.rest.c_str(), "%d %d", &p, &v); g_analog_in[p] = v; }
      else if (e.kind == "ECHO") { int p; long v; sscanf(e.rest.c_str(), "%d %ld", &p, &v); g_pulse[p] = v; }
      else if (e.kind == "I2C") { int a, r; char h[64]; sscanf(e.rest.c_str(), "%d %d %63s", &a, &r, h); g_i2c[TwoWire::key(a, r)].data = from_hex(h); }
      else if (e.kind == "I2CFAIL") { int a, f; sscanf(e.rest.c_str(), "%d %d", &a, &f); g_i2c[TwoWire::key(a, 0x1FE)].fail = f != 0; }
      else if (e.kind == "RX") { for (char c : e.rest) g_rx.push_back(c); g_rx.push_back('\n'); }
    }
    loop();
  }
  for (auto &s : g_events) printf("%s\n", s.c_str());
  printf("---TX---\n");
  for (auto &s : g_txlog) printf("%s\n", s.c_str());
  return 0;
}
