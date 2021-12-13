#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "bulbboot.h"
#include "version.h"

uint8_t version[4] = {0xff, 0xff, 0xff, 0xff};

uint8_t hex2u8(char high, char low) {
    uint8_t h, l;
    if (high >= '0' && high <= '9') {
        h = high - '0';
    } else if (high >= 'a' && high <= 'f') {
        h = high - 'a';
    } else if (high >= 'A' && high <= 'F') {
        h = high - 'A';
    } else {
        return 0xff;
    }

    if (low >= '0' && low <= '9') {
        l = low - '0';
    } else if (low >= 'a' && low <= 'f') {
        l = low - 'a';
    } else if (low >= 'A' && low <= 'F') {
        l = low - 'A';
    } else {
        return 0xff;
    }

    return h * 16 + l;
}

void ver_init() {
    const esp_app_desc_t *app_desc = esp_ota_get_app_description();
    ESP_LOGI(TAG, "app name: %s", app_desc->project_name);
    ESP_LOGI(TAG, "app version: %s", app_desc->version);
    ESP_LOGI(TAG, "idf version: %s", app_desc->idf_ver);
    ESP_LOGI(TAG, "compile date: %s", app_desc->date);
    ESP_LOGI(TAG, "compile time: %s", app_desc->time);

    const char *s = app_desc->version;
    if (strlen(s) != 8) {
        ESP_LOGI(TAG, "invalid app version string");
        return;
    }

    for (int i = 0; i < 8; i++) {
        if (s[i] >= '0' && s[i] <= '9')
            continue;
        if (s[i] >= 'a' && s[i] <= 'f')
            continue;
        if (s[i] >= 'A' && s[i] <= 'F')
            continue;
        ESP_LOGI(TAG, "invalid app version string");
        return;
    }

    for (int i = 0; i < 4; i++) {
        version[i] = hex2u8(s[i * 2], s[i * 2 + 1]);
    }
}
