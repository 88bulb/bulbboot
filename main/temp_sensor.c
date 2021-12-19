#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/temp_sensor.h"
#include "esp_efuse_rtc_calib.h"
#include "bulbboot.h" // for TAG
#include "temp_sensor.h"

/* used for read out config */
temp_sensor_config_t tsens_config = TSENS_CONFIG_DEFAULT();

/* raw value read out of registers */
uint32_t temp_raw = 0;

/* celsius degree in float */
float temp_celsius = 0;

/* celcius degree in uint8_t */
uint8_t temp = 0;

/* Set wait cycle time(8MHz) from power up to reset enable. */
#define TSENS_XPD_WAIT_DEFAULT 0xFF
#define TSENS_ADC_FACTOR (0.4386)
#define TSENS_DAC_FACTOR (27.88)
#define TSENS_SYS_OFFSET (20.52)

typedef struct {
    int index;
    int offset;
    int set_val;
    int range_min;
    int range_max;
    int error_max;
} tsens_dac_offset_t;

static const tsens_dac_offset_t entries[TSENS_DAC_MAX] = {
    /*     DAC     Offset reg_val  min  max  error */
    {TSENS_DAC_L0, -2, 5, 50, 125, 3}, {TSENS_DAC_L1, -1, 7, 20, 100, 2},
    {TSENS_DAC_L2, 0, 15, -10, 80, 1}, {TSENS_DAC_L3, 1, 11, -30, 50, 2},
    {TSENS_DAC_L4, 2, 10, -40, 20, 3},
};

static float s_deltaT = NAN;

static void read_delta_t_from_efuse(void) {
    uint32_t version = esp_efuse_rtc_calib_get_ver();
    if (version == 1) {
        // fetch calibration value for temp sensor from eFuse
        s_deltaT = esp_efuse_rtc_calib_get_cal_temp(version);
    } else {
        // no value to fetch, use 0.
        s_deltaT = 0;
    }
    ESP_LOGI(TAG, "s_deltaT = %f", s_deltaT);
}

static float parse_temp_sensor_raw_value(uint32_t raw, const int offset) {
    // NAN suggests that the value is not initialized
    if (isnan(s_deltaT)) {
        read_delta_t_from_efuse();
    }
    float celsius = TSENS_ADC_FACTOR * (float)raw - TSENS_DAC_FACTOR * offset -
                    TSENS_SYS_OFFSET - s_deltaT / 10.0;
    return celsius;
}

static void tsense(void *params) {

    /* initial delay before first read */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    while (1) {

        /* This function returns only ESP_OK
         * tsens_config.dac_offset is the index (TSENS_DAC_Lx)
         */
        temp_sensor_get_config(&tsens_config);

        /* This function return only ESP_OK too.
         * APB_SARADC.apb_tsens_ctrl.tsens_out
         */
        temp_sensor_read_raw(&temp_raw);

        /*
         * Pick entry according to index (misnamed as dac_offset)
         */
        const tsens_dac_offset_t *entry = &entries[tsens_config.dac_offset];

        /*
         * Calculate celsius degree
         */
        temp_celsius = parse_temp_sensor_raw_value(temp_raw, entry->offset);

        /*
         * Convert to int value
         */
        if (temp_celsius <= 1) {
            temp = 1;
        } else {
            temp = (uint8_t)temp_celsius;
        }

        ESP_LOGI(TAG, "temp: %u (raw), %f (float), %u (uint8), dac_offset: %d",
                 temp_raw, temp_celsius, temp, tsens_config.dac_offset);

        if (temp_celsius < entry->range_min) {
            ESP_LOGI(TAG, "temp below range_min %d", entry->range_min);
            if (tsens_config.dac_offset < TSENS_DAC_L4) {
                temp_sensor_stop();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                tsens_config.dac_offset += 1;
                ESP_LOGI(TAG, "reconfigure tsens with dac_offst: %d",
                         tsens_config.dac_offset);
                temp_sensor_set_config(tsens_config);
                temp_sensor_start();
            }
        } else if (temp_celsius > entry->range_max) {
            ESP_LOGI(TAG, "temp above range_max %d", entry->range_max);
            if (tsens_config.dac_offset > TSENS_DAC_L0) {
                temp_sensor_stop();
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                tsens_config.dac_offset -= 1;
                temp_sensor_set_config(tsens_config);
                ESP_LOGI(TAG, "reconfigure tsens with dac_offst: %d",
                         tsens_config.dac_offset);
            }
            temp_sensor_start();
        }

        vTaskDelay(15 * 1000 / portTICK_PERIOD_MS);
    }
}

void temp_sensor_init() {
    // temp_sensor_get_config(&tsens_config);
    tsens_config.dac_offset = TSENS_DAC_DEFAULT;
    temp_sensor_set_config(tsens_config);
    temp_sensor_start();
    ESP_LOGI(TAG, "Temperature sensor started");

    xTaskCreate(&tsense, "tsense", 2048, NULL, 4, NULL);
}
