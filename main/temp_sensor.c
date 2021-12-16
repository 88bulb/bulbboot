#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/temp_sensor.h"
#include "bulbboot.h" // for TAG
#include "temp_sensor.h"

uint8_t temp = 0;

static void temp_sensor_timer_callback(TimerHandle_t timer) {
    float tsens_out;
    temp_sensor_read_celsius(&tsens_out);
    if (tsens_out <= 0) {
        temp = 0;
    } else {
        temp = (uint8_t)tsens_out;
    }
    ESP_LOGI(TAG, "temperature %u (degree)", temp);
}

void temp_sensor_init() {
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor_get_config(&temp_sensor);
    temp_sensor.dac_offset = TSENS_DAC_L0; /* 50-125 degree, error < 3C */
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();

    TimerHandle_t timer =
        xTimerCreate("temp_timer", 10 * 1000 / portTICK_PERIOD_MS, pdTRUE, 0,
                     &temp_sensor_timer_callback);
    // first sensing
    temp_sensor_timer_callback(timer);
    xTimerStart(timer, 0);
}
