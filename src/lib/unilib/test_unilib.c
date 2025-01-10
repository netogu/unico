#include "unilib.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void test_ul_fifo_init() {
  ul_fifo_t fifo;
  char buffer[32 * sizeof(int)];
  int result = ul_fifo_init(&fifo, buffer, 32, sizeof(int));
  assert(result == 0);
  assert(32 == ul_fifo_capacity(&fifo));
  assert(0 == ul_fifo_count(&fifo));
  printf("test_ulib_fifo_init passed \n");
}

void test_ul_fifo_put() {
  ul_fifo_t fifo;
  char buffer[32 * sizeof(char)];
  int result = ul_fifo_init(&fifo, buffer, 32, sizeof(char));

  char test_data[32] = "FIFO Test Data 1234567890\n";
  char read_data[32] = "";

  for (size_t i = 0; i < strlen(test_data + 1); i++) {
    ul_fifo_put(&fifo, &test_data[i]);
  }

  for (size_t i = 0; i < strlen(test_data + 1); i++) {
    ul_fifo_get(&fifo, &read_data[i]);
  }

  assert(strcmp(test_data, read_data) == 0);
  assert() printf("test_ul_fifo_put_get passed\n");
}
