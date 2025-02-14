#include "../uclib.h"
#include <stdio.h>

int main(int argc, char *argv[]) {
  float num = 1.9876543210;
  char str[12];
  for (int i = 0; i < 9; i++) {
    uclib_ftoa(num, str, i);
    printf("%s\n", str);
  }

  num = -2.3252323424;
  for (int i = 0; i < 9; i++) {
    uclib_ftoa(num, str, i);
    printf("%s\n", str);
  }
  return 0;
}
