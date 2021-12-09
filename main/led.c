#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "bulbboot.h"

static const char *TAG = "bulbboot";

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

static void four_color_set_duty(uint32_t r, uint32_t g, uint32_t b,
                                uint32_t white) {
    five_color_set_duty(r, g, b, white / 2, white / 2);
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
         .gpio_num = 18,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
        {.channel = LEDC_CHANNEL_4,
         .duty = 0,
         .gpio_num = 19,
         .speed_mode = LEDC_LOW_SPEED_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_TIMER_1},
    };

    for (int ch = 0; ch < 5; ch++) {
        ledc_channel_config(&lc[ch]);
    }

    ledc_fade_func_install(0);
    five_color_set_duty(0, 0, 0, 128, 128);
}

void aging_test1(uint8_t aged_minutes) {
    if (aged_minutes) {
        ESP_LOGI(TAG, "continuing aging test");
    } else {
        ESP_LOGI(TAG, "starting aging test");
    }

    if (aged_minutes == 0) {
        ESP_LOGI(TAG, "cycling led colors for 2 minutes");
        /* five seconds per cycle, 24 cycles = 2 minutes, right? */
        for (int i = 0; i < 24; i++) {
            five_color_set_duty(255, 0, 0, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 255, 0, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 255, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 255, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 0, 255);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    if (aged_minutes) {
        ESP_LOGI(TAG, "cycling all leds 5 times before continuing");
        for (int i = 0; i < 5; i++) {
            five_color_set_duty(255, 0, 0, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 255, 0, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 255, 0, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 255, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            five_color_set_duty(0, 0, 0, 0, 255);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    if (aged_minutes < 20) {
        ESP_LOGI(TAG, "aging cold white led at full brightness");
        five_color_set_duty(0, 0, 0, 255, 0);
        do {
            ESP_LOGI(TAG, "%u minutes left", 20 - aged_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            aged_minutes++;
            write_aging_minutes(aged_minutes);
        } while (aged_minutes < 20);
        ESP_LOGI(TAG, "cold white led aging finished");
    }

    if (aged_minutes < 40) {
        ESP_LOGI(TAG, "aging warm white led at full brightness");
        five_color_set_duty(0, 0, 0, 0, 255);
        do {
            ESP_LOGI(TAG, "%u minutes left", 40 - aged_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            aged_minutes++;
            write_aging_minutes(aged_minutes);
        } while (aged_minutes < 40);
        ESP_LOGI(TAG, "warm white led aging finished");
    }

    if (aged_minutes < 50) {
        ESP_LOGI(TAG, "aging color leds at full brightness");
        five_color_set_duty(255, 255, 255, 0, 0);
        do {
            ESP_LOGI(TAG, "%u minutes left", 50 - aged_minutes);
            vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
            aged_minutes++;
            write_aging_minutes(aged_minutes);
        } while (aged_minutes < 50);
        ESP_LOGI(TAG, "color led aging finished");
    }

    // finished, low intensity green light
    five_color_set_duty(0, 64, 0, 0, 0);
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

        five_color_fade(0, 0, 0, 255, 0, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        five_color_fade(0, 0, 0, 0, 255, 1000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        five_color_fade(0, 0, 0, 0, 0, 990);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
