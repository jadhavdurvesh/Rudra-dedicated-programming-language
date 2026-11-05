// Compiles the generated master header of an addressed node and prints what it sends.
#include <stdio.h>
#include "stub_globals.h"
#include LINKH

int main() {
  turret_ping(Serial);
  turret_AIM(Serial, 120);
  turret_WHERE(Serial);
  printf("%s", g_tx.c_str());
  char l[] = "@3 AT 120";
  turret_msg m;
  bool ok = turret_parse(l, &m);
  printf("%d %ld %s %d %ld\n", ok, m.addr, m.name, m.has_value, m.value);
  char l2[] = "PONG";
  turret_parse(l2, &m);
  printf("%ld %s\n", m.addr, m.name);
  printf("%d\n", TURRET_ADDRESS);
  return 0;
}
