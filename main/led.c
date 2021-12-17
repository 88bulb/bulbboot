#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs_flash.h"

#include "bulbboot.h"
#include "led.h"
#include "wifi.h"
#include "temp_sensor.h"

uint8_t aging_minutes = 0;
uint8_t highest_temp = 0;
uint8_t target_brightness = 0;
uint8_t actual_brightness = 0;
int8_t color_temp = 0;
bool led_illuminating = false;

static nvs_handle_t nvs;
static TickType_t initial_fade_start = 0;
static TickType_t initial_fade_duration_ms = 1000;
static bool found;
static wifi_ap_record_t ap;

static esp_err_t update_aging_minutes(uint8_t minutes) {
    aging_minutes = minutes;
    return nvs_set_u8(nvs, "aging_minutes", minutes);
}

static void five_color_set_duty(uint32_t r, uint32_t g, uint32_t b, uint32_t c,
                                uint32_t w) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, c);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

static void five_color_fade(uint32_t r, uint32_t g, uint32_t b, uint32_t c,
                            uint32_t w, uint32_t duration) {
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, r, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, g, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, b, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, c, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, LEDC_FADE_NO_WAIT);
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, w, duration);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, LEDC_FADE_NO_WAIT);
}

static void wait_fade_finish() {
    TickType_t current = xTaskGetTickCount();

    ESP_LOGI(TAG, "wait_fade_finish, current tick: %d", current);

    TickType_t elapse = current - initial_fade_start;
    TickType_t duration = initial_fade_duration_ms / portTICK_PERIOD_MS;
    if (elapse < duration) {
        ESP_LOGI(TAG, "wait %d ticks (%d milliseconds) for fade finish",
                 (duration - elapse), (duration - elapse) * portTICK_PERIOD_MS);
        vTaskDelay(duration - elapse);
    } else {
        ESP_LOGI(TAG, "fade already finished");
    }
}

static void aging_test1() {
    wait_fade_finish();

    if (aging_minutes) {
        ESP_LOGI(TAG, "continuing aging test");
    } else {
        ESP_LOGI(TAG, "starting aging test");
    }

    if (aging_minutes == 0) {
        ESP_LOGI(TAG, "cycling led colors for 2 minutes");
        /* five seconds per cycle, 24 cycles = 2 minutes, right? */
        for (int i = 0; i < 24; i++) {
            five_color_set_duty(255, 0, 0, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 255, 0, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 255, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 192, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 0, 192);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    if (aging_minutes) {
        ESP_LOGI(TAG, "cycling all leds 5 times before continuing");
        for (int i = 0; i < 5; i++) {
            five_color_set_duty(255, 0, 0, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 255, 0, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 255, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 192, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 0, 192);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    if (aging_minutes < 20) {
        ESP_LOGI(TAG, "aging cold white led at full brightness");
        five_color_set_duty(0, 0, 0, 255, 0);
        do {
            ESP_LOGI(TAG, "%u minutes left", 20 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            update_aging_minutes(aging_minutes + 1);
        } while (aging_minutes < 20);
        ESP_LOGI(TAG, "cold white led aging finished");
    }

    if (aging_minutes < 40) {
        ESP_LOGI(TAG, "aging warm white led at full brightness");
        five_color_set_duty(0, 0, 0, 0, 255);
        do {
            ESP_LOGI(TAG, "%u minutes left", 40 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            update_aging_minutes(aging_minutes + 1);
        } while (aging_minutes < 40);
        ESP_LOGI(TAG, "warm white led aging finished");
    }

    if (aging_minutes < 50) {
        ESP_LOGI(TAG, "aging color leds at full brightness");
        five_color_set_duty(192, 192, 192, 0, 0);
        do {
            ESP_LOGI(TAG, "%u minutes left", 50 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            update_aging_minutes(aging_minutes + 1);
        } while (aging_minutes < 50);
        ESP_LOGI(TAG, "color led aging finished");
    }

    // finished, low intensity green light
    five_color_set_duty(0, 40, 0, 0, 0);
    ESP_LOGI(TAG, "all aging test finished, green on");
    vTaskDelay(portMAX_DELAY);
}

static void aging_test2() {
    wait_fade_finish();
    ESP_LOGI(TAG, "breathing all leds endlessly");
    five_color_fade(0, 0, 0, 0, 0, 1000);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (1) {
        five_color_fade(255, 0, 0, 0, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 255, 0, 0, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 0, 255, 0, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 0, 0, 192, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 0, 0, 0, 192, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void blink(uint8_t cold, uint8_t warm, int quarter) {
    five_color_fade(0, 0, 0, 0, 0, quarter - 10);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    five_color_fade(0, 0, 0, cold, warm, quarter - 10);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
}

static void illuminate(void *params) {
    ESP_LOGI(TAG, "illuminate");
    TickType_t wait = 60 * 1000 / portTICK_PERIOD_MS;
    led_illuminating = true;
    wait_fade_finish();
    while (1) {
        if (temp <= highest_temp) {
            if (actual_brightness < target_brightness) {
                actual_brightness++;
                five_color_set_duty(0, 0, 0, actual_brightness,
                                    actual_brightness);
            }
        } else if (temp > highest_temp) {
            if (actual_brightness > 0)
                actual_brightness--;
        }

        uint8_t cold, warm;
        if ((int)actual_brightness + (int)color_temp > 255) {
            cold = 255;
        } else if ((int)actual_brightness + (int)color_temp < 0) {
            cold = 0;
        } else {
            cold = actual_brightness + color_temp;
        }

        if ((int)actual_brightness - (int)color_temp > 255) {
            warm = 255;
        } else if ((int)actual_brightness - (int)color_temp < 0) {
            warm = 0;
        } else {
            warm = actual_brightness - color_temp;
        }

        five_color_set_duty(0, 0, 0, cold, warm);
        xEventGroupWaitBits(ev, BLINK, pdFALSE, pdFALSE, wait);
        if (xEventGroupGetBits(ev) & BLINK) {
            blink(cold, warm, 100);
            blink(cold, warm, 100);
            blink(cold, warm, 100);
            blink(cold, warm, 100);
            xEventGroupClearBits(ev, BLINK);
        }
    }
}

static void low_light() {
    ESP_LOGI(TAG, "low light");
    wait_fade_finish();
    while (1) {
        five_color_set_duty(0, 0, 0, 32, 32);
        xEventGroupWaitBits(ev, BLINK, pdFALSE, pdFALSE, wait);
        if (xEventGroupGetBits(ev) & BLINK) {
            blink(32, 32, 100);
            blink(32, 32, 100);
            blink(32, 32, 100);
            blink(32, 32, 100);
            xEventGroupClearBits(ev, BLINK);
        }
    }
}

void led_init() {
    esp_err_t err;
    /* initialize led */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = 5000,                     // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_1,           // timer index
        .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t lc[5] = {
        {.channel = LEDC_CHANNEL_0,
         .duty = 0,
         .gpio_num = 3,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_1,
         .duty = 0,
         .gpio_num = 0,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_2,
         .duty = 0,
         .gpio_num = 1,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_3,
         .duty = 0,
         .gpio_num = 19,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_4,
         .duty = 0,
         .gpio_num = 18,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
    };

    for (int ch = 0; ch < 5; ch++) {
        ledc_channel_config(&lc[ch]);
    }

    ledc_fade_func_install(0);
    five_color_set_duty(0, 0, 0, 0xff, 0);

    ESP_ERROR_CHECK(nvs_open("nvs", NVS_READWRITE, &nvs));
    err = nvs_get_u8(nvs, "aging_minutes", &aging_minutes);
    if (err != ESP_OK)
        aging_minutes = 0;

    ESP_LOGI(TAG, "aging minutes: %u", aging_minutes);

    err = nvs_get_u8(nvs, "highest_temp", &highest_temp);
    if (err != ESP_OK) {
        highest_temp = DEFAULT_HIGHEST_TEMP;
    } else if (highest_temp > ABSOLUTE_HIGHEST_TEMP) {
        highest_temp = ABSOLUTE_HIGHEST_TEMP;
    }

    ESP_LOGI(TAG, "highest temp: %u", highest_temp);

    err = nvs_get_u8(nvs, "target_brightness", &target_brightness);
    if (err != ESP_OK) {
        target_brightness = DEFAULT_BRIGHTNESS;
    } else if (target_brightness > ABSOLUTE_HIGHEST_BRIGHTNESS) {
        target_brightness = ABSOLUTE_HIGHEST_BRIGHTNESS;
    }

    ESP_LOGI(TAG, "target brightness: %u", target_brightness);

    actual_brightness = target_brightness;

    err = nvs_get_i8(nvs, "color_temp", &color_temp);
    if (err != ESP_OK) {
        color_temp = 0;
    } else if (color_temp < -16) {
        color_temp = -16;
    } else if (color_temp > 16) {
        color_temp = 16;
    }

    ESP_LOGI(TAG, "color temp: %d", color_temp);

    initial_fade_start = xTaskGetTickCount();
    uint8_t cold, warm;
    if ((int)actual_brightness + (int)color_temp > 255) {
        cold = 255;
    } else if ((int)actual_brightness + (int)color_temp < 0) {
        cold = 0;
    } else {
        cold = actual_brightness + color_temp;
    }

    if ((int)actual_brightness - (int)color_temp > 255) {
        warm = 255;
    } else if ((int)actual_brightness - (int)color_temp < 0) {
        warm = 0;
    } else {
        warm = actual_brightness - color_temp;
    }

    ESP_LOGI(TAG, "initial_fade_start: %d", initial_fade_start);
    ESP_LOGI(TAG, "portTICK_PERIOD_MS: %d", portTICK_PERIOD_MS);

    five_color_fade(0, 0, 0, cold, warm, initial_fade_duration_ms - 10);
}

void led_run() {
    if (aging_minutes < 50) {
        wifi_scan("tuya_mdev_test1", true, &found, &ap);
        if (found) {
            if (0 == strcmp((char *)ap.ssid, "tuya_mdev_test1")) {
                ESP_LOGI(TAG, "found tuya_mdev_test1 ap");
                xTaskCreate(&aging_test1, "aging_test1", 4096, NULL, 5, NULL);
            } else if (0 == strcmp((char *)ap.ssid, "skip_tuya_mdev_test1")) {
                ESP_LOGI(TAG, "found skip_tuya_mdev_test1 ap");
                ESP_LOGI(TAG, "aging_minutes set to 0xee (238)");
                update_aging_minutes(0xee);
                xTaskCreate(&illuminate, "illuminate", 4096, NULL, 5, NULL);
            }
        } else {
            ESP_LOGI(TAG, "tuya_mdev_test1 not found");
            xTaskCreate(&low_light, "low_light", 4096, NULL, 5, NULL);
        }
    } else if (aging_minutes == 50) {
        wifi_scan("tuya_mdev_test2", true, &found, &ap);
        if (found) {
            if (0 == strcmp((char *)ap.ssid, "tuya_mdev_test2")) {
                ESP_LOGI(TAG, "found tuya_mdev_test2");
                xTaskCreate(&aging_test2, "aging_test2", 4096, NULL, 5, NULL);
            } else if (0 == strcmp((char *)ap.ssid, "skip_tuya_mdev_test2")) {
                ESP_LOGI(TAG, "found skip_tuya_mdev_test2");
                ESP_LOGI(TAG, "aging_minutes set to 0xff (255)");
                update_aging_minutes(0xff);
                xTaskCreate(&illuminate, "illuminate", 4096, NULL, 5, NULL);
            }
        } else {
            ESP_LOGI(TAG, "tuya_mdev_test2 not found");
            xTaskCreate(&illuminate, "illuminate", 4096, NULL, 5, NULL);
        }
    } else {
        xTaskCreate(&illuminate, "illuminate", 4096, NULL, 5, NULL);
    }
}
