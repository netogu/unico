#include "unilib.h"
#include <assert.h>
#include <stdio.h>

void test_ulib_fifo_init() {
  ULIB_fifo_t fifo;
  const int bufferSize = 32 * sizeof(int);
  char buffer[bufferSize];
  int result = ULIB_fifo_init(&fifo, buffer, 32, sizeof(int));
  assert(result == 0);
  assert(32 == ULIB_fifo_capacity(&fifo));
  assert(0 == ULIB_fifo_count(&fifo));
  printf("test_ulib_fifo_init passed \n");
}
