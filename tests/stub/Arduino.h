// Fake Arduino core for testing generated code on a PC. Records every hardware effect as a text event.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <deque>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
static const uint8_t A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18, A5 = 19, A6 = 20, A7 = 21;
#define F(x) x

extern uint32_t g_now;
extern int g_pin_in[64];
extern int g_analog_in[64];
extern std::vector<std::string> g_events;   // in call order: "W t pin v" "S t pin angle" "I t addr hex" "N t c0 c1 ..."
extern std::string g_tx;
extern std::vector<std::string> g_txlog;   // "t line" per completed serial line
extern std::string g_txcur;
extern std::deque<char> g_rx;

inline void rec(const std::string &s) { g_events.push_back(s); }

inline unsigned long millis() { return g_now; }
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t p, uint8_t v) { rec("W " + std::to_string(g_now) + " " + std::to_string(p) + " " + std::to_string(v)); }
inline void analogWrite(uint8_t p, int v) { rec("W " + std::to_string(g_now) + " " + std::to_string(p) + " " + std::to_string(v)); }
inline int digitalRead(uint8_t p) { return g_pin_in[p]; }
inline int analogRead(uint8_t p) { return g_analog_in[p]; }
inline void delayMicroseconds(unsigned int) {}
extern long g_pulse[64];                       // echo pulse length in microseconds per pin (0 = timeout)
inline unsigned long pulseIn(uint8_t p, uint8_t, unsigned long) { return (unsigned long)g_pulse[p]; }
inline void tone(uint8_t p, unsigned int f) { rec("T " + std::to_string(g_now) + " " + std::to_string(p) + " " + std::to_string(f)); }
inline void noTone(uint8_t p) { rec("T " + std::to_string(g_now) + " " + std::to_string(p) + " 0"); }

struct Print {
  virtual ~Print() {}
  virtual void print(const char *s) = 0;
  virtual void print(long v) = 0;
  virtual void println(const char *s) { print(s); print("\n"); }
  void println() { print("\n"); }
  virtual void println(long v) { print(v); print("\n"); }
};

struct SerialClass : Print {
  using Print::println;
  void begin(long) {}
  void flush() {}
  int available() { return (int)g_rx.size(); }
  int read() { if (g_rx.empty()) return -1; char c = g_rx.front(); g_rx.pop_front(); return (unsigned char)c; }
  void print(const char *s) override {
    g_tx += s;
    for (const char *c = s; *c; c++) {
      if (*c == '\n') { g_txlog.push_back(std::to_string(g_now) + " " + g_txcur); g_txcur.clear(); }
      else g_txcur += *c;
    }
  }
  void print(long v) override { g_tx += std::to_string(v); g_txcur += std::to_string(v); }
  void println(const char *s) override { Print::println(s); }
  void println(long v) override { Print::println(v); }
};
extern SerialClass Serial;

void setup();
void loop();
