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
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "bootloader_common.h"

#define PORT (6016)

#define BOOT_REQUESTED (1 << 1)
#define STA_GOT_IP (1 << 2)
#define LAST_WILL (1 << 3)

static char ssid_token[4] = {0};
static char ssid_token_str[16] = {0};
static uint8_t sha88[11] = {0};
static char sha88_str[32] = {0};

static const char *TAG = "rootbulb";

static EventGroupHandle_t eg_handle;
static StaticEventGroup_t eg_data;

/**
 * b01bca57 0f  xx:xx:xx:xx:xx:xx   xx:xx:xx:xx
 * 4 bytes  1   6-byte address      4-byte ssid token (15 bytes)
 * 11 bytes (sha88)
 */
static void handle_mfr_data(uint8_t *bda, uint8_t *data, size_t data_len) {
    if (data_len != 26 || data[0] != 0xb0 || data[1] != 0x1b ||
        data[2] != 0xca || data[3] != 0x57 || data[4] != 0x0f)
        return;

    // little endian in air packet
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BT);
    if (data[5] != mac[5] || data[6] != mac[4] || data[7] != mac[3] ||
        data[8] != mac[2] || data[9] != mac[1] || data[10] != mac[0])
        return;

    const char hex_char[16] = "0123456789abcdef";
    for (int i = 0; i < 4; i++) {
        uint8_t n = data[11 + i];
        ssid_token[i] = n;
        ssid_token_str[2 * i] = hex_char[n / 16];
        ssid_token_str[2 * i + 1] = hex_char[n % 16];
    }

    ESP_LOGI(TAG, "ssid token: %s", ssid_token_str);

    for (int i = 0; i < sizeof(sha88); i++) {
        uint8_t n = data[15 + i];
        sha88[i] = n;
        sha88_str[2 * i] = hex_char[n / 16];
        sha88_str[2 * i + 1] = hex_char[n % 16];
    }

    ESP_LOGI(TAG, "sha88 in hex string: %s", sha88_str);
    ESP_LOGI(TAG, "boot request received");

    ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
        esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x0C00, // 1920ms
            .adv_int_max = 0x1000, // 2560ms
            .adv_type = ADV_TYPE_NONCONN_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    } break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
        ESP_LOGI(TAG, "ble adv started");
        esp_ble_scan_params_t scan_params = {
            .scan_type = BLE_SCAN_TYPE_PASSIVE,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
            .scan_interval = 0x50,
            .scan_window = 0x50,
            .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};
        ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
    } break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: 
        ESP_LOGI(TAG, "ble adv stopped");
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0));
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "ble scan started");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            uint8_t mfr_data_len;
            uint8_t *mfr_data = esp_ble_resolve_adv_data(
                scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &mfr_data_len);
            handle_mfr_data(scan_result->scan_rst.bda, mfr_data, mfr_data_len);
        } break;
        default:
            break;
        }
    } break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "ble scan stopped");
        xEventGroupSetBits(eg_handle, BOOT_REQUESTED);
        break;
    default:
        ESP_LOGI(TAG, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

void ble_adv_scan (void *pvParameters) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    uint8_t adv_mfr_data[5] = {0xb0, 0x1b, 0xca, 0x57, 0x00};
    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = false,
        .include_txpower = false,
        .min_interval = 0x0006,
        .max_interval = 0x0010,
        .appearance = 0x00,
        .manufacturer_len = 5,
        .p_manufacturer_data = adv_mfr_data,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    xEventGroupWaitBits(eg_handle, LAST_WILL, pdFALSE, pdFALSE, portMAX_DELAY);
}

static esp_netif_t* init_wifi() {
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

static void esp_wifi_cb(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
    xEventGroupSetBits(eg_handle, STA_GOT_IP);
}

static int connect_to_server(uint32_t addr, int port) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) return -1;
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr))) {
        close(sock);
        return -1;
    }
    return sock;
}

void app_main(void) {
    esp_err_t err;
    esp_netif_t *sta_netif = NULL;
    bool found;
    wifi_ap_record_t ap;
    esp_event_handler_instance_t instance_got_ip;
    esp_netif_ip_info_t ip_info;

    eg_handle = xEventGroupCreateStatic(&eg_data);

    /* init nvs flash */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    uint8_t tuya_test = 0;
    uint16_t tuya_aged_time = 0;

    nvs_handle_t nvs_handle;
    nvs_open("storage", NVS_READWRITE, &nvs_handle);
    nvs_get_u8(nvs_handle, "tuya_test", &tuya_test);
    nvs_get_u16(nvs_handle, "tuya_aged_time", &tuya_aged_time);

    sta_netif = init_wifi();

    if (tuya_test == 0) { 
        // before aging (only once)
        wifi_scan("tuya_mdev_test1", true, &found, &ap);
        if (found) {
            // do test1 router found 2 minutes
            // then 50 minutes aging (aging time persistent)
            // and 5 cycles flashing before restoring aging if power
            // interrupted
        } else {
            // do test1 router not found
        }
    } else if (tuya_test == 1) {
        // after aging (many times, but could be disabled)
        wifi_scan("tuya_mdev_test2", true, &found, &ap);
        if (found) {
        } else {
        }
    } 

    xTaskCreate(&ble_adv_scan, "ble_adv_scan", 4096, NULL, 6, NULL);

    xEventGroupWaitBits(eg_handle, BOOT_REQUESTED, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    esp_partition_t *ota1 = (esp_partition_t *)esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_OTA(1), NULL);
    if (!ota1) {
        ESP_LOGE(TAG, "no partition of type app and subtype ota_1 found");
        // TODO
    } else {
        ESP_LOGI(TAG, "ota_1 partition found, address: %u, size: %u, label %s",
                 ota1->address, ota1->size, ota1->label);
    }

    /* defined in esp_image_format.h */
    esp_partition_pos_t ota1_pos = {
        .offset = ota1->address,
        .size = ota1->size,
    };

    /* defined in esp_image_format.h */
    esp_image_metadata_t meta;
    err = esp_image_verify(ESP_IMAGE_VERIFY, &ota1_pos, &meta);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_image_verify ota1 failed, %s", esp_err_to_name(err));
    } else if (meta.image.hash_appended) {
        /* metadata.image is of type esp_image_header_t, which is defined in
         * esp_app_format.h */
        ESP_LOGI(TAG,
                 "ota1 image metadata, start addr: %u, size: %u, with sha256 "
                 "digest:",
                 meta.start_addr, meta.image_len);
        ESP_LOG_BUFFER_HEX(TAG, meta.image_digest, sizeof(meta.image_digest));
        ESP_LOG_BUFFER_HEX(TAG, sha88, 11);

        bool match = true;
        for (int i = 0; i < 11; i++) {
            if (meta.image_digest[i] != sha88[i]) {
                match = false;
                break;
            }
        }

        if (match) {
            bootloader_common_update_rtc_retain_mem(&ota1_pos, false);
            esp_deep_sleep(1000);
        }
    }

    wifi_scan((char*)ssid_token_str, false, &found, &ap);
    if (!found) {
        ESP_LOGE(TAG, "no ap ssid contains %s", ssid_token_str);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &esp_wifi_cb, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    wifi_config.sta.pmf_cfg.capable = true;
    memcpy(wifi_config.sta.ssid, ap.ssid, sizeof(wifi_config.sta.ssid));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());

    xEventGroupWaitBits(eg_handle, STA_GOT_IP, pdFALSE, pdFALSE,
                        30 * 1000 / portTICK_PERIOD_MS);
    if (!(xEventGroupGetBits(eg_handle) & STA_GOT_IP))
        esp_restart();

    ESP_ERROR_CHECK(esp_netif_get_ip_info(sta_netif, &ip_info));

    int sock = connect_to_server(ip_info.gw.addr, PORT);
    if (sock < 0) {
        ESP_LOGE(TAG, "failed to connect to server");
        esp_restart(); // TODO
    } else {
        ESP_LOGI(TAG, "successfully connected");
    }

    uint8_t rx_buffer[128];
    const char payload[] = "BULBBOOT\n";

    err = send(sock, payload, strlen(payload), 0);
    if (err < 0) {
        // TODO
    }

    int len = read(sock, rx_buffer, 4);
    if (len < 4) {
        // TODO
    }

    ESP_LOGI(TAG, "read first 4 bytes, len: %d", len);

    int fw_size = rx_buffer[0] + rx_buffer[1] * 256 + rx_buffer[2] * 256 * 256 +
                  rx_buffer[3] * 256 * 256 * 256;

    ESP_LOGI(TAG, "read first 4 bytes, firmware size: %d", fw_size);

    if (fw_size > ota1->size) {
        close(sock);
        // TODO
    }

    int erase_size = fw_size / 4096 * 4096;
    if (fw_size % 4096) {
        erase_size += 4096;
    }

    ESP_LOGI(TAG, "erasing first %d bytes in ota1 partition", erase_size);

    err = esp_partition_erase_range(ota1, 0, erase_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase failed", errno);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_restart(); // TODO
        return;
    }

    ESP_LOGI(TAG, "erase done");

    int ota1_written = 0;
    while (1) {
        int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len == 0) {
            ESP_LOGI(TAG, "recv returns 0, finished.");
            break;
        }

        if (len < 0) {
            ESP_LOGE(TAG, "error occured during recv, errno %d", errno);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }

        if (len + ota1_written > ota1->size) {
            ESP_LOGE(TAG, "error, receivd file larger than partition size");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }

        err = esp_partition_write(ota1, ota1_written, rx_buffer, len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "error writing partition, %s", esp_err_to_name(err));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        }

        ota1_written += len;
    }

    ESP_LOGI(TAG, "ota_written %d", ota1_written);

    bootloader_common_update_rtc_retain_mem(&ota1_pos, true);
    esp_deep_sleep(1000);
}
