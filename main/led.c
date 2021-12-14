#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs_flash.h"

#include "bulbboot.h"

bool led_illuminating = false;
uint8_t highest_temp = 0;
uint8_t target_brightness = 0;
uint8_t actual_brightness = 0;

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

    err = nvs_get_u8(nvs, "highest_temp", &highest_temp);
    if (err != ESP_OK) {
        highest_temp = DEFAULT_HIGHEST_TEMP;
    } else if (highest_temp > ABSOLUTE_HIGHEST_TEMP) {
        highest_temp = ABSOLUTE_HIGHEST_TEMP;
    }

    err = nvs_get_u8(nvs, "target_brightness", &target_brightness);
    if (err != ESP_OK) {
        target_brightness = DEFAULT_BRIGHTNESS;
    } else if (target_brightness > ABSOLUTE_HIGHEST_BRIGHTNESS) {
        target_brightness = ABSOLUTE_HIGHEST_BRIGHTNESS;
    }

    actual_brightness = target_brightness;

    five_color_fade(0, 0, 0, actual_brightness, actual_brightness, 1000);
}

void aging_test1() {
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
            aging_minutes++;
            write_aging_minutes(aging_minutes);
        } while (aging_minutes < 20);
        ESP_LOGI(TAG, "cold white led aging finished");
    }

    if (aging_minutes < 40) {
        ESP_LOGI(TAG, "aging warm white led at full brightness");
        five_color_set_duty(0, 0, 0, 0, 255);
        do {
            ESP_LOGI(TAG, "%u minutes left", 40 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            aging_minutes++;
            write_aging_minutes(aging_minutes);
        } while (aging_minutes < 40);
        ESP_LOGI(TAG, "warm white led aging finished");
    }

    if (aging_minutes < 50) {
        ESP_LOGI(TAG, "aging color leds at full brightness");
        five_color_set_duty(192, 192, 192, 0, 0);
        do {
            ESP_LOGI(TAG, "%u minutes left", 50 - aging_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            aging_minutes++;
            write_aging_minutes(aging_minutes);
        } while (aging_minutes < 50);
        ESP_LOGI(TAG, "color led aging finished");
    }

    // finished, low intensity green light
    five_color_set_duty(0, 40, 0, 0, 0);
    ESP_LOGI(TAG, "all aging test finished, green on");
}

void aging_test2() {
    ESP_LOGI(TAG, "breathing all leds endlessly");
    five_color_set_duty(0, 0, 0, 0, 0);
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

void blink(uint8_t brightness, int quarter) {
    five_color_fade(0, 0, 0, 0, 0, quarter - 10);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    five_color_fade(0, 0, 0, brightness, brightness, quarter - 10);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
    vTaskDelay(quarter / portTICK_PERIOD_MS);
}

void led_illuminate(void *params) {

    const int short_wait = 1000 / portTICK_PERIOD_MS;
    const int long_wait = 4000 / portTICK_PERIOD_MS;

    while (1) {
        int wait;
        if (temp <= highest_temp) {
            if (actual_brightness < target_brightness) {
                actual_brightness++;
                five_color_set_duty(0, 0, 0, actual_brightness,
                                    actual_brightness);
            }
            wait = long_wait;
        } else if (temp > highest_temp) {
            actual_brightness -= 1;
            wait = short_wait;
        }

        xEventGroupWaitBits(ev, BLINK, pdFALSE, pdFALSE, wait);

        if (xEventGroupGetBits(ev) & BLINK) {
            xEventGroupClearBits(ev, BLINK);

            blink(actual_brightness, 200);
            blink(actual_brightness, 200);
            blink(actual_brightness, 200);
            blink(actual_brightness, 200);
            blink(actual_brightness, 200);
        }
    }
}
