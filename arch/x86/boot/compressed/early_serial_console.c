#include "misc.h"

unsigned int (*serial_in)(unsigned long addr, int offset);
void (*serial_out)(unsigned long addr, int offset, int value);

unsigned long early_serial_base;

#include "../early_serial_console.c"
