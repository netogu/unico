#include "sensor_ntc.h"
#include "hal.h"
#include <math.h>

static inline float sensor_ntc_r_to_celsius(sensor_ntc_t *self,
                                            float ntc_resistance) {
  float temperature_c;
  temperature_c =
      1 / (logf(ntc_resistance / self->rth_25C) / self->B + 1 / 298.15f) -
      273.15f;
  return temperature_c;
}

float sensor_ntc_read_f32(sensor_ntc_t *self, float ntc_volts) {
  if (!self) {
    // Error
    return 0;
  }
  float ntc_resistance;
  if (self->config == NTC_CONFIG_BOT) {
    ntc_resistance = self->rs / (self->vs / ntc_volts - 1.0f) - 2 * self->rcmc;
  } else {
    ntc_resistance =
        (self->rs + 2 * self->rcmc) * (self->vs / ntc_volts - 1.0f);
  }

  float temperature_c = sensor_ntc_r_to_celsius(self, ntc_resistance);

  return temperature_c;
}
