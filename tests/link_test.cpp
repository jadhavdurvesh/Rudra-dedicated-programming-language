// Compiles the generated master-side header against the fake Arduino core and exercises it.
#include <stdio.h>
#include "stub_globals.h"
#include LINKH

int main() {
  scout_ping(Serial);
  scout_PAN(Serial, 45);
  scout_COLOR(Serial, 1, 2, 3);
  scout_SWEEP(Serial);
  scout_who(Serial);
  scout_status(Serial);
  printf("%s", g_tx.c_str());

  char l1[] = "TEMP -250";
  char l2[] = "  BTN  ";
  char l3[] = "PONG";
  char l4[] = "   ";
  scout_msg m;
  bool ok1 = scout_parse(l1, &m);
  printf("%d %s %d %ld\n", ok1, m.name, m.has_value, m.value);
  bool ok2 = scout_parse(l2, &m);
  printf("%d %s %d\n", ok2, m.name, m.has_value);
  bool ok3 = scout_parse(l3, &m);
  printf("%d %d\n", ok3, strcmp(m.name, SCOUT_MSG_PONG) == 0);
  printf("%d\n", scout_parse(l4, &m));
  printf("%d %d\n", SCOUT_BAUD, SCOUT_KEEPALIVE_MS);
  printf("%ld\n", SCOUT_PROTO_HASH);
  return 0;
}
