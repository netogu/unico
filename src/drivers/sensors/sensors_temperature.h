#ifndef __SENSORS_TEMPERATURE__
#define __SENSORS_TEMPERATURE__

typedef struct {
  enum {
    NTC_CONFIG_TOP,
    NTC_CONFIG_BOT,
  } config;

  float resistance_25C;
  float resistance_bias;
  float voltage_bias;

} sensor_ntc_t;

float sensor_temperature_ntc_read_f32(sensor_ntc_t *self, float ntc_volts);

#endif
