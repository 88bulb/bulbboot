#ifndef __TEMP_SENSOR_H
#define __TEMP_SENSOR_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/temp_sensor.h"

extern temp_sensor_config_t tsens_config;
extern uint8_t temp;

void temp_sensor_init();

#endif
