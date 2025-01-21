/*
.---------------------.
|                     |
|                     |
|        _  ._  _     |
|    /_// ///_ /_/    |
|                     |
|                     |
'---------------------'
*/

#include "bsp.h"
#include "tasklist.h"

int main(void) {

  board_init();
  bootup_system();

  // Should not reach here
  return 0;
}
