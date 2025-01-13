#include "../unilib.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define PRINT_COLOR_RED "\033[0;31m"
#define PRINT_COLOR_GREEN "\033[0;32m"
#define PRINT_COLOR_YELLOW "\033[0;33m"
#define PRINT_COLOR_BLUE "\033[0;34m"
#define PRINT_COLOR_PURPLE "\033[0;35m"

#define printf_color(color, fmt, ...)                                          \
  do {                                                                         \
    printf(color);                                                             \
    printf(fmt, ##__VA_ARGS__);                                                \
    printf("\033[0m");                                                         \
  } while (0)

void test_ul_fifo_init() {
  printf_color(PRINT_COLOR_PURPLE, "\nRunning test_ul_fifo_init\n");
  ul_fifo_t fifo;
  char buffer[32 * sizeof(int)];
  int result = ul_fifo_init(&fifo, buffer, 32, sizeof(int));
  assert(result == 0);
  assert(32 == ul_fifo_capacity(&fifo));
  assert(0 == ul_fifo_count(&fifo));
  // assert(strcmp(test_data, read_data) == 0);
  printf_color(PRINT_COLOR_GREEN, "test_ul_fifo_init passed\n");
  // assert(strcmp(test_data, read_data) == 0);
}

void test_ul_fifo_enqueue_dequeue() {
  printf_color(PRINT_COLOR_PURPLE, "\nRunning test_ul_fifo_enqueue_dequeue\n");
  ul_fifo_t fifo;
  char buffer[128 * sizeof(char)];
  int result = ul_fifo_init(&fifo, buffer, 128, sizeof(char));

  char test_data[] = "0123456789:;<=>?@"
                     "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz{|}~";
  size_t test_data_len = strlen(test_data) + 1;

  printf("Test Data: %s\n", test_data);
  printf("strlen : %ld\n", test_data_len);

  char read_data[128] = {};

  for (size_t i = 0; i < test_data_len; i++) {
    ul_fifo_enqueue(&fifo, &test_data[i]);
  }

  printf("FIFO count: %ld/%ld\n", ul_fifo_count(&fifo),
         ul_fifo_capacity(&fifo));

  assert(ul_fifo_count(&fifo) == test_data_len);

  size_t fifo_count = ul_fifo_count(&fifo);
  for (size_t i = 0; i < fifo_count; i++) {
    ul_fifo_dequeue(&fifo, &read_data[i]);
  }

  printf("Read Data: %s\n", read_data);
  printf("FIFO count: %ld/%ld\n", ul_fifo_count(&fifo),
         ul_fifo_capacity(&fifo));

  assert(ul_fifo_count(&fifo) == 0);

  assert(strcmp(test_data, read_data) == 0);
  printf_color(PRINT_COLOR_GREEN, "test_ul_fifo_put_get passed\n");
}

void test_ul_fifo_overflow() {
  printf_color(PRINT_COLOR_PURPLE, "\nRunning test_ul_fifo_overflow\n");
  ul_fifo_t fifo;
  char buffer[50 * sizeof(char)];
  int result = ul_fifo_init(&fifo, buffer, 50, sizeof(char));
  char test_data[] = "0123456789:;<=>?@"

                     "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz{|}~";
  size_t test_data_len = strlen(test_data) + 1;

  printf("Test Data: %s\n", test_data);
  printf("strlen : %ld\n", test_data_len);

  char read_data[128] = {};

  for (size_t i = 0; i < test_data_len; i++) {
    ul_fifo_enqueue(&fifo, &test_data[i]);
  }

  printf("FIFO count: %ld/%ld\n", ul_fifo_count(&fifo),
         ul_fifo_capacity(&fifo));

  assert(ul_fifo_count(&fifo) ==
         50); // Must not exceed or overflow max fifo size

  size_t fifo_count = ul_fifo_count(&fifo);
  for (size_t i = 0; i < fifo_count; i++) {
    ul_fifo_dequeue(&fifo, &read_data[i]);
  }

  printf("Read Data: %s\n", read_data);
  printf("FIFO count: %ld/%ld\n", ul_fifo_count(&fifo),
         ul_fifo_capacity(&fifo));

  assert(ul_fifo_count(&fifo) == 0);

  // assert(strcmp(test_data, read_data) == 0);
  printf_color(PRINT_COLOR_GREEN, "test_ul_fifo_overflow passed\n");
}

int main(void) {
  printf("Starting Tests: unilib\n");
  test_ul_fifo_init();
  test_ul_fifo_enqueue_dequeue();
  test_ul_fifo_overflow();
  printf("Tests completed\n");
}
