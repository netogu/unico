#ifndef __SENSOR_NTC_H
#define __SENSOR_NTC_H

#include "hal.h"

typedef struct {
  enum {
    NTC_CONFIG_TOP,
    NTC_CONFIG_BOT,
  } config;

  float rth_25C;
  float rs;
  float rcmc; // if using R common mode choke
  float vs;
  float B;

} sensor_ntc_t;

float sensor_ntc_read_f32(sensor_ntc_t *self, float ntc_volts);

#endif //__SENSOR_NTC_H
