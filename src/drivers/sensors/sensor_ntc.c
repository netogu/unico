#include "sensor_ntc.h"
#include "hal.h"
#include <math.h>

// class NTC : public TemperatureSensor {
//  public:
//     NTC(volatile uint32_t& adc, float B = 3435, bool top_side = false, float
//     adc_scaling = 3.3) :
//         adc_(adc), B_(B), top_side_(top_side), adc_scaling_(adc_scaling) {}
//     float read() {
//         float voltage = adc_scaling_/4096*adc_;
//         // bottom side 10k/10k
//         // r is thermistor
//         // v = 3.3*r/(r10 + r)
//         // v/3.3/r = 1/(r10+r)
//         // 3.3*r/v = r10 + r
//         // r(3.3/v - 1) = r10
//         // r = r10/(3.3/v - 1)
//
//         // top side 10k/10k
//         // v = 3.3*r10/(r + r10)
//         // r + r10 = 3.3*r10/v
//         // r = 3.3*r10/v - r10
//         // r = r10*(3.3/v - 1)
//         float resistance;
//         if (top_side_) {
//             resistance = 10000*(3.3/voltage-1);
//         } else {
//             resistance = 10000/(3.3/voltage-1);
//         }
//         temperature_ = 1/(std::log(resistance/10000)/B_ + 1/298.15) - 273.15;
//         return get_temperature();
//     }
//     float get_temperature() const { return temperature_; }
//  private:
//     volatile uint32_t& adc_;
//     float B_;
//     bool top_side_;
//     float adc_scaling_;
//     float temperature_ = 0;
// };
static inline float sensor_ntc_r_to_celsius(sensor_ntc_t *self,
                                            float ntc_resistance) {
  float temperature_c;
  temperature_c =
      1 / (logf(ntc_resistance / self->rth_25C) / self->B + 1 / 298.15f) -
      273.15f;
  return temperature_c;
}

float sensor_ntc_read_f32(sensor_ntc_t *self) {
  if (!self) {
    // Error
    return 0;
  }
  float ntc_resistance;
  float ntc_volts = hal_analog_read_f32(self->analog_input);

  if (self->config == NTC_CONFIG_BOT) {
    ntc_resistance = self->rs / (self->vs / ntc_volts - 1.0f) - 2 * self->rcmc;
  } else {
    ntc_resistance =
        (self->rs + 2 * self->rcmc) * (self->vs / ntc_volts - 1.0f);
  }

  float temperature_c = sensor_ntc_r_to_celsius(self, ntc_resistance);

  return temperature_c;
}
