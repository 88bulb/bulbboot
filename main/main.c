#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_partition.h"
#include "esp_app_format.h"
#include "esp_image_format.h"
#include "esp_wifi.h"
#include "bootloader_common.h"

#include "bulbboot.h"

#define PORT (6016)

static const char *TAG = "bulbboot";

char ssid_token[4] = {0};
char ssid_token_str[16] = {0};
uint8_t sha88[11] = {0};
char sha88_str[32] = {0};

last_will_t last_will_reason;
esp_err_t last_will_error;

static StaticEventGroup_t eg_data;
EventGroupHandle_t ev;

static nvs_handle_t nvs;

static uint8_t read_aging_minutes() {
    esp_err_t err;
    uint8_t minutes;
    err = nvs_get_u8(nvs, "aging_minutes", &minutes);
    if (err == ESP_OK) {
        return minutes;
    } else {
        return 0;
    }
}

esp_err_t write_aging_minutes(uint8_t minutes) {
    return nvs_set_u8(nvs, "aging_minutes", minutes);
}

static void nvs_init() {
    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    nvs_open("nvs", NVS_READWRITE, &nvs);
}

static void last_will(last_will_t reason, esp_err_t err) {
    last_will_reason = reason;
    last_will_error = err;
    xEventGroupSetBits(ev, LAST_WILL);
    vTaskDelay(portMAX_DELAY);
}

static esp_netif_t *wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    return netif;
}

static void wifi_scan(char *token, bool stop_after_scan, bool *found,
                      wifi_ap_record_t *ap) {
    wifi_ap_record_t ap_record[20] = {};
    uint16_t ap_record_max = 20;
    uint16_t ap_num = 0;

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_record_max, ap_record));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));

    *found = false;
    for (uint16_t i = 0; i < ap_num; i++) {
        char *ssid_str = (char *)ap_record[i].ssid;
        if (strstr(ssid_str, token)) {
            *found = true;
            if (ap)
                memcpy(ap, &ap_record[i], sizeof(wifi_ap_record_t));
            break;
        }
    }

    if (stop_after_scan) {
        ESP_ERROR_CHECK(esp_wifi_stop());
    }
}

/* got ip handler */
static void got_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    xEventGroupSetBits(ev, STA_GOT_IP);
}

/* (freertos) main task */
void app_main(void) {
    esp_err_t err;
    esp_netif_t *sta_netif = NULL;
    esp_netif_ip_info_t ip_info;
    bool found;
    wifi_ap_record_t ap;

    ev = xEventGroupCreateStatic(&eg_data);

    nvs_init();
    led_init();
    wifi_init();

    uint8_t age = read_aging_minutes();
    ESP_LOGI(TAG, "aged time in minutes: %u", age);

    if (age < 50) {
        wifi_scan("tuya_mdev_test1", true, &found, &ap);
        if (found) {
            if (0 == strcmp((char *)ap.ssid, "tuya_mdev_test1")) {
                ESP_LOGI(TAG, "found tuya_mdev_test1 ap, start aging test");
                aging_test1(age);
                vTaskDelay(portMAX_DELAY);
            } else if (0 == strcmp((char *)ap.ssid, "skip_tuya_mdev_test1")) {
                ESP_LOGI(TAG, "forcefully skip tuya_mdev_test1, aging time set "
                              "to 0xee minutes");
                write_aging_minutes(nvs, 0xee);
            }
        } else {
            ESP_LOGI(TAG, "tuya_mdev_test1 not found");
            // ble broadcasting
            xTaskCreate(&ble_adv_scan, "ble_adv_scan", 4096, &age, 6, NULL);
            vTaskDelay(portMAX_DELAY);
        }
    } else if (age == 50) {
        wifi_scan("tuya_mdev_test2", true, &found, &ap);
        if (found) {
            if (0 == strcmp((char *)ap.ssid, "tuya_mdev_test2")) {
                ESP_LOGI(TAG, "found tuya_mdev_test2, blinking all colors");
                xTaskCreate(&ble_adv_scan, "ble_adv_scan", 4096, &age, 6, NULL);
                // this function should loop forever according to definition
                aging_test2();
            }
        }
    }

    /* create ble task */
    xTaskCreate(&ble_adv_scan, "ble_adv_scan", 4096, &age, 6, NULL);
    xEventGroupWaitBits(ev, BOOT_SIGNALLED, pdFALSE, pdFALSE, portMAX_DELAY);

    /* retrieve ota1 partition */
    esp_partition_t *ota1 = (esp_partition_t *)esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_OTA(1), NULL);
    LAST_WILL_COND(!ota1, OTA_PARTITION_NOT_FOUND);
    ESP_LOGI(TAG, "ota_1 partition address: %u, size: %u", ota1->address,
             ota1->size);

    /* verify current image in ota1 partition */
    esp_partition_pos_t ota1_pos = {
        .offset = ota1->address,
        .size = ota1->size,
    };
    esp_image_metadata_t meta;
    err = esp_image_verify(ESP_IMAGE_VERIFY, &ota1_pos, &meta);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_image_verify ota1 failed, %s", esp_err_to_name(err));
    } else if (meta.image.hash_appended) {
        ESP_LOGI(TAG,
                 "ota1 image start addr: %u, size: %u, with sha256 "
                 "digest:",
                 meta.start_addr, meta.image_len);
        ESP_LOG_BUFFER_HEX(TAG, meta.image_digest, sizeof(meta.image_digest));
        ESP_LOG_BUFFER_HEX(TAG, sha88, 11);

        /* if digest matches, direct boot */
        bool match = true;
        for (int i = 0; i < 11; i++) {
            if (meta.image_digest[i] != sha88[i]) {
                match = false;
                break;
            }
        }
        if (match)
            goto sleepboot;
    }

    /* find wifi access point given in boot signal */
    wifi_scan((char *)ssid_token_str, false, &found, &ap);
    LAST_WILL_COND(!found, OTA_AP_NOT_FOUND);

    /* connect to ap */
    wifi_config_t wifi_config = {0};
    wifi_config.sta.pmf_cfg.capable = true;
    memcpy(wifi_config.sta.ssid, ap.ssid, sizeof(wifi_config.sta.ssid));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip, NULL, (void *)&found));
    ESP_ERROR_CHECK(esp_wifi_connect());

    /* wait at most 30 seconds */
    xEventGroupWaitBits(ev, STA_GOT_IP, pdFALSE, pdFALSE,
                        30 * 1000 / portTICK_PERIOD_MS);
    LAST_WILL_COND(!(xEventGroupGetBits(ev) & STA_GOT_IP), OTA_NETWORK_TIMEOUT);

    /* get ip address */
    ESP_ERROR_CHECK(esp_netif_get_ip_info(sta_netif, &ip_info));

    /* connect to server (gateway) */
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = ip_info.gw.addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    LAST_WILL_COND(sock < 0, LWIP_SOCKET);
    LAST_WILL_COND(
        (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr))),
        OTA_LWIP_CONNECT);
    ESP_LOGI(TAG, "server connected");

    /* send command BULBBOOT */
    const char bulbboot[] = "BULBBOOT\n";
    LAST_WILL_COND((0 > send(sock, bulbboot, strlen(bulbboot), 0)),
                   OTA_SEND_BULBBOOT);

    /* read first 4 bytes, which are firmware size in little endian order */
    uint8_t rx_buffer[128];
    int len = read(sock, rx_buffer, 4);
    LAST_WILL_COND(len < 4, OTA_READ_SIZE);
    int fw_size = rx_buffer[0] + rx_buffer[1] * 256 + rx_buffer[2] * 256 * 256 +
                  rx_buffer[3] * 256 * 256 * 256;
    ESP_LOGI(TAG, "firmware size: %d", fw_size);
    LAST_WILL_COND(fw_size > ota1->size, OTA_BAD_FIRMWARE_SIZE);

    /* erase flash */
    int erase_size = (fw_size % 4096) ? (fw_size / 4096 + 1) * 4096 : fw_size;
    LAST_WILL_CHECK(esp_partition_erase_range(ota1, 0, erase_size),
                    OTA_PARTITION_ERASE_RANGE);
    ESP_LOGI(TAG, "%d bytes erased in ota1 partition", erase_size);

    /* receive data and write to flash */
    int ota1_written = 0;
    while ((len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0))) {
        LAST_WILL_COND(len < 0, OTA_RECV_FILE);
        LAST_WILL_COND(len + ota1_written > fw_size, OTA_FIRMWARE_OVERSIZE);
        LAST_WILL_CHECK(esp_partition_write(ota1, ota1_written, rx_buffer, len),
                        OTA_ESP_PARTITION_WRITE);
        ota1_written += len;
    }
    LAST_WILL_COND(ota1_written < fw_size, OTA_FIRMWARE_UNDERSIZE);

sleepboot:
    bootloader_common_update_rtc_retain_mem(&ota1_pos, true);
    esp_deep_sleep(1000);
}
