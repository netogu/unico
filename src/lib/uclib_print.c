#include <math.h>
#include <stdio.h>

void uclib_ftoa(float f32, char *s, int decimal_places) {
  if (s == NULL || decimal_places < 0)
    return;

  // Handle negative numbers
  if (f32 < 0) {
    *s++ = '-';
    f32 = -f32; // Convert to positive
  } else {
    *s++ = ' ';
  }

  // Round the number correctly
  float rounding_factor = powf(10.0f, decimal_places);
  f32 = roundf(f32 * rounding_factor) / rounding_factor;

  // Extract integer and fractional parts
  int int_part = (int)f32;
  float frac_part = f32 - int_part;

  // Convert integer part to string
  if (decimal_places == 0) {
    sprintf(s, "%d", int_part); // No decimal point if places = 0
    return;
  }

  // Convert fractional part with correct rounding
  int frac_int = (int)(frac_part * rounding_factor);

  // Print formatted string
  sprintf(s, "%d.%0*d", int_part, decimal_places, frac_int);
}
