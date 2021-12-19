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
uint8_t color_temp = 0;
uint8_t cold_white_brightness = 0;
uint8_t warm_white_brightness = 0;

bool led_illuminating = false;

static nvs_handle_t nvs;
static TickType_t initial_fade_start = 0;
static bool found;
static wifi_ap_record_t ap;

static esp_err_t update_aging_minutes(uint8_t minutes) {
    aging_minutes = minutes;
    return nvs_set_u8(nvs, "aging_minutes", minutes);
}

static void update_white_brightness() {
    cold_white_brightness = actual_brightness;
    warm_white_brightness =
        (uint8_t)((int)actual_brightness * (int)color_temp / 100);
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

static void wait_initial_fade() {
    TickType_t current = xTaskGetTickCount();

    ESP_LOGI(TAG, "wait_initial_fade, current tick: %d", current);

    TickType_t elapse = current - initial_fade_start;
    TickType_t duration = FADE_DURATION_MS / portTICK_PERIOD_MS;
    if (elapse < duration) {
        ESP_LOGI(TAG, "wait %d ticks (%d milliseconds) for fade finish",
                 (duration - elapse), (duration - elapse) * portTICK_PERIOD_MS);
        vTaskDelay(duration - elapse);
    } else {
        ESP_LOGI(TAG, "fade already finished");
    }
}

static void aging_test1() {
    wait_initial_fade();

    if (aging_minutes) {
        ESP_LOGI(TAG, "continuing aging test");
    } else {
        ESP_LOGI(TAG, "starting aging test");
    }

    if (aging_minutes == 0) {
        ESP_LOGI(TAG, "cycling led colors for 2 minutes");
        /* five seconds per cycle, 24 cycles = 2 minutes, right? */
        for (int i = 0; i < 24; i++) {
            five_color_set_duty(TESTING_COLOR_BRIGHTNESS, 0, 0, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, TESTING_COLOR_BRIGHTNESS, 0, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, TESTING_COLOR_BRIGHTNESS, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, TESTING_WHITE_BRIGHTNESS, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 0, TESTING_WHITE_BRIGHTNESS);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    if (aging_minutes) {
        ESP_LOGI(TAG, "cycling all leds 5 times before continuing");
        for (int i = 0; i < 5; i++) {
            five_color_set_duty(TESTING_COLOR_BRIGHTNESS, 0, 0, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, TESTING_COLOR_BRIGHTNESS, 0, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, TESTING_COLOR_BRIGHTNESS, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, TESTING_WHITE_BRIGHTNESS, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 0, TESTING_WHITE_BRIGHTNESS);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    if (aging_minutes < 20) {
        ESP_LOGI(TAG, "aging cold white led at full brightness");
        five_color_set_duty(0, 0, 0, TESTING_WHITE_BRIGHTNESS, 0);
        do {
            ESP_LOGI(TAG, "%u minutes left", 20 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            update_aging_minutes(aging_minutes + 1);
        } while (aging_minutes < 20);
        ESP_LOGI(TAG, "cold white led aging finished");
    }

    if (aging_minutes < 40) {
        ESP_LOGI(TAG, "aging warm white led at full brightness");
        five_color_set_duty(0, 0, 0, 0, TESTING_WHITE_BRIGHTNESS - 16);
        do {
            ESP_LOGI(TAG, "%u minutes left", 40 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            update_aging_minutes(aging_minutes + 1);
        } while (aging_minutes < 40);
        ESP_LOGI(TAG, "warm white led aging finished");
    }

    if (aging_minutes < 50) {
        ESP_LOGI(TAG, "aging color leds at full brightness");
        five_color_set_duty(TESTING_COLOR_BRIGHTNESS, TESTING_COLOR_BRIGHTNESS,
                            TESTING_COLOR_BRIGHTNESS, 0, 0);
        do {
            ESP_LOGI(TAG, "%u minutes left", 50 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            update_aging_minutes(aging_minutes + 1);
        } while (aging_minutes < 50);
        ESP_LOGI(TAG, "color led aging finished");
    }

    // finished, low intensity green light
    five_color_set_duty(0, LOW_LIGHT_BRIGHTNESS, 0, 0, 0);
    ESP_LOGI(TAG, "all aging test finished, green on");
    vTaskDelay(portMAX_DELAY);
}

static void aging_test2() {
    wait_initial_fade();
    ESP_LOGI(TAG, "breathing all leds endlessly");
    five_color_fade(0, 0, 0, 0, 0, FADE_DURATION_MS);
    vTaskDelay(2 * FADE_DURATION_MS / portTICK_PERIOD_MS);
    while (1) {
        five_color_fade(TESTING_COLOR_BRIGHTNESS, 0, 0, 0, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, TESTING_COLOR_BRIGHTNESS, 0, 0, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 0, TESTING_COLOR_BRIGHTNESS, 0, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 0, 0, TESTING_WHITE_BRIGHTNESS, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 0, 0, 0, TESTING_WHITE_BRIGHTNESS, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void blink(int quarter) {
    five_color_fade(0, 0, 0, 0, 0, quarter - 10);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    five_color_fade(0, 0, 0, cold_white_brightness, warm_white_brightness,
                    quarter - 10);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
}

static void illuminate(void *params) {
    TickType_t sec = 1000 / portTICK_PERIOD_MS;
    TickType_t wait;

    ESP_LOGI(TAG, "illumination mode");
    wait_initial_fade();

    xEventGroupSetBits(ev, BOOTABLE);
    led_illuminating = true;
    while (1) {
        if (temp > ABSOLUTE_HIGHEST_TEMP) {
            actual_brightness = LOWEST_LIGHT_BRIGHTNESS;
        } else if (temp <= highest_temp &&
                   actual_brightness < target_brightness) {
            actual_brightness++;
        } else if (temp > highest_temp &&
                   actual_brightness > LOWEST_LIGHT_BRIGHTNESS) {
            actual_brightness--;
        }

        update_white_brightness();
        five_color_set_duty(0, 0, 0, cold_white_brightness,
                            warm_white_brightness);

        /* this algorithm should not be related to
         * ABSOLUTE_HIGHEST_TEMP or ALLOWED_HIGHEST_TEMP
         */
        if (temp > highest_temp + 6 || temp < highest_temp - 6) {
            wait = 1 * sec;
        } else if (temp > highest_temp + 4 || temp < highest_temp - 4) {
            wait = 4 * sec;
        } else if (temp > highest_temp + 2 || temp < highest_temp - 2) {
            wait = 16 * sec;
        } else {
            wait = 64 * sec;
        }

        xEventGroupWaitBits(ev, BLINK, pdFALSE, pdFALSE, wait);

        if (xEventGroupGetBits(ev) & BLINK) {
            for (int i = 0; i < BLINK_TIMES; i++) {
                blink(100);
            }
            xEventGroupClearBits(ev, BLINK);
        }
    }
}

static void low_light_illuminate() {
    ESP_LOGI(TAG, "low light illuminating");
    wait_initial_fade();

    actual_brightness = LOW_LIGHT_BRIGHTNESS;
    update_white_brightness();
    five_color_fade(0, 0, 0, cold_white_brightness, warm_white_brightness,
                    FADE_DURATION_MS - 10);
    vTaskDelay(FADE_DURATION_MS / portTICK_PERIOD_MS);

    while (1) {
        xEventGroupWaitBits(ev, BLINK, pdFALSE, pdFALSE, portMAX_DELAY);
        if (xEventGroupGetBits(ev) & BLINK) {
            for (int i = 0; i < BLINK_TIMES; i++) {
                blink(100);
            }
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
    } else if (highest_temp > ALLOWED_HIGHEST_TEMP) {
        highest_temp = ALLOWED_HIGHEST_TEMP;
    }

    ESP_LOGI(TAG, "highest temp: %u", highest_temp);

    err = nvs_get_u8(nvs, "target_brightness", &target_brightness);
    if (err != ESP_OK) {
        target_brightness = DEFAULT_BRIGHTNESS;
    } else if (target_brightness > ABSOLUTE_HIGHEST_BRIGHTNESS) {
        target_brightness = ABSOLUTE_HIGHEST_BRIGHTNESS;
    }

    ESP_LOGI(TAG, "target brightness: %u", target_brightness);

    err = nvs_get_u8(nvs, "color_temp", &color_temp);
    if (err != ESP_OK) {
        color_temp = DEFAULT_COLOR_TEMP;
    } else if (color_temp > 100) {
        color_temp = 100;
    }

    ESP_LOGI(TAG, "color temp: %u%%, (ratio of warm white / cold white)",
             color_temp, color_temp);

    initial_fade_start = xTaskGetTickCount();

    ESP_LOGI(TAG, "initial_fade_start: %d", initial_fade_start);
    ESP_LOGI(TAG, "portTICK_PERIOD_MS: %d", portTICK_PERIOD_MS);

    actual_brightness = target_brightness;
    update_white_brightness();
    five_color_fade(0, 0, 0, cold_white_brightness, warm_white_brightness,
                    FADE_DURATION_MS - 10);
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
            xTaskCreate(&low_light_illuminate, "low_light", 4096, NULL, 5,
                        NULL);
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
