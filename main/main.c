#include <stdio.h>
#include <string.h>
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

#define ADV_DATA_SET_COMPLETE (1 << 0)
#define ADV_START_COMPLETE (1 << 1)
#define SCAN_PARAM_SET_COMPLETE (1 << 2)
#define SCAN_START_COMPLETE (1 << 3)
#define SCAN_STOP_COMPLETE (1 << 4)
#define BOOT_REQUESTED (1 << 5)
#define WIFI_CONNECTED (1 << 6)
#define WIFI_DISCONNECTED (1 << 7)

static char hex_char[16] = "0123456789abcdef";
static char ssid_token[4] = {0};
static char ssid_token_str[16] = {0};
static uint8_t sha88[11] = {0};
static char sha88_str[32] = {0};

static const char *TAG = "rootbulb";

static EventGroupHandle_t eg_handle;
static StaticEventGroup_t eg_data;

static uint8_t bmac[6] = {0};

#define AP_RECORD_MAX (20)

esp_netif_t *sta_netif = NULL;
static uint16_t ap_record_max = AP_RECORD_MAX;
wifi_ap_record_t ap_record[AP_RECORD_MAX] = {};
static uint16_t ap_num = 0;

esp_event_handler_instance_t instance_any_id;
esp_event_handler_instance_t instance_got_ip;

/**
 * scanning and wifi are mutual exclusive
 */
static const esp_ble_scan_params_t scan_params_default = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x50,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE};

uint8_t adv_mfr_data[5] = {0xb0, 0x1b, 0xca, 0x57, 0x00};

static const esp_ble_adv_data_t adv_data_default = {
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

static const esp_ble_adv_params_t adv_params_default = {
    .adv_int_min = 0x0C00, // 1920ms
    .adv_int_max = 0x1000, // 2560ms
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static void until(const EventBits_t bits) {
    xEventGroupWaitBits(eg_handle, bits, pdFALSE, pdFALSE, portMAX_DELAY);
}

/**
 * b01bca57 0f  xx:xx:xx:xx:xx:xx   xx:xx:xx:xx
 * 4 bytes  1   6-byte address      4-byte ssid token (15 bytes)
 * 11 bytes (sha88)
 */
static void handle_mfr_data(uint8_t *bda, uint8_t *data, size_t data_len) {
    if (data_len != 26 || data[0] != 0xb0 || data[1] != 0x1b ||
        data[2] != 0xca || data[3] != 0x57 || data[4] != 0x0f ||
        data[5] != bmac[0] || data[6] != bmac[1] || data[7] != bmac[2] ||
        data[8] != bmac[3] || data[9] != bmac[4] || data[10] != bmac[5])
        return;

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

    xEventGroupSetBits(eg_handle, BOOT_REQUESTED);
}

static void esp_wifi_cb(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "connect to AP failed");
        xEventGroupSetBits(eg_handle, WIFI_DISCONNECTED);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(eg_handle, WIFI_CONNECTED);
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
    uint8_t *mfr_data;
    uint8_t mfr_data_len;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, SCAN_PARAM_SET_COMPLETE);
        break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, SCAN_START_COMPLETE);
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            mfr_data = esp_ble_resolve_adv_data(
                scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &mfr_data_len);
            handle_mfr_data(scan_result->scan_rst.bda, mfr_data, mfr_data_len);
        } break;
        default:
            break;
        }
    } break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, SCAN_STOP_COMPLETE);
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, ADV_DATA_SET_COMPLETE);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, ADV_START_COMPLETE);
        break;
    default:
        ESP_LOGI(TAG, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

static bool is_factory_test_disabled() {
    esp_err_t err;
    nvs_handle_t handle;
    size_t length;

    err = nvs_open("storage", NVS_READONLY, &handle);
    if (err != ESP_OK)
        return false;
    err = nvs_get_str(handle, "disable_factory_test", NULL, &length);
    nvs_close(handle);

    // it doesn't matter what the string is, as long as the key exists
    return (err == ESP_OK);
}

static void do_wifi_scan(void) {
    static bool initialized = false;
    if (!initialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        sta_netif = esp_netif_create_default_wifi_sta();
        assert(sta_netif);
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        initialized = true;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    memset(ap_record, 0, sizeof(ap_record));
    ap_record_max = AP_RECORD_MAX;
    ap_num = 0;
    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_record_max, ap_record));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
    ESP_ERROR_CHECK(esp_wifi_stop());
}

/**
 *
 */
void app_main(void) {
    esp_err_t err;
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

    esp_partition_t *ota1 = (esp_partition_t *)esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_OTA(1), NULL);
    if (!ota1) {
        ESP_LOGE(TAG, "no partition of type app and subtype ota_1 found");
        vTaskDelay(portMAX_DELAY);
    }

    ESP_LOGI(TAG, "ota_1 partition found, address: %u, size: %u, label %s",
             ota1->address, ota1->size, ota1->label);

    // prepare mac
    esp_read_mac(bmac, ESP_MAC_BT);

    /* this function may never return */
    if (!is_factory_test_disabled()) {
        do_wifi_scan();

        ESP_LOGI(TAG, "wifi scan for factory test finished");
        for (uint16_t i = 0; i < ap_num; i++) {
            char *ssid_str = (char *)ap_record[i].ssid;
            ESP_LOGI(TAG, "found ssid: %s", ssid_str);
        }

        // TODO
        ESP_LOGI(TAG, "move on...");
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    esp_ble_adv_data_t adv_data = adv_data_default;
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));

    ESP_LOGI(TAG, "configuring adv data");

    until(ADV_DATA_SET_COMPLETE);

    esp_ble_adv_params_t adv_params = adv_params_default;
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    until(ADV_START_COMPLETE);
    ESP_LOGI(TAG, "adv start complete");

    esp_ble_scan_params_t scan_params = scan_params_default;
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
    until(SCAN_PARAM_SET_COMPLETE);

    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0));
    until(SCAN_START_COMPLETE);
    ESP_LOGI(TAG, "ble scan started");

    until(BOOT_REQUESTED);
    ESP_LOGI(TAG, "boot request received");

    // may generate ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT
    ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
    do_wifi_scan();

    // find ssid (if not found, reboot)
    wifi_ap_record_t *ap = NULL;
    for (uint16_t i = 0; i < ap_num; i++) {
        char *ssid_str = (char *)ap_record[i].ssid;
        if (strstr(ssid_str, ssid_token_str) &&
            ap_record[i].authmode == WIFI_AUTH_OPEN) {
            ap = &ap_record[i];
            break;
        }
    }

    if (ap == NULL) {
        ESP_LOGE(TAG, "no ap ssid contains %s", ssid_token_str);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
        return;
    }

    wifi_config_t wifi_config = {0};
    wifi_config.sta.pmf_cfg.capable = true;
    memcpy(wifi_config.sta.ssid, ap->ssid, sizeof(wifi_config.sta.ssid));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &esp_wifi_cb, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &esp_wifi_cb, NULL, &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    until(WIFI_CONNECTED);

    ESP_ERROR_CHECK(esp_netif_get_ip_info(sta_netif, &ip_info));

    char rx_buffer[128];
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = ip_info.gw.addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d, restarting...",
                 errno);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_restart();
        return;
    }

    // stackoverflow.com how-to-convert-string-to-ip-address-and-vice-versa
    char ip_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &(dest_addr.sin_addr), ip_str, INET_ADDRSTRLEN);
    ESP_LOGI(TAG, "socket created, connecting to %s:%d", ip_str, PORT);

    err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_restart();
        return;
    }

    ESP_LOGI(TAG, "successfully connected");

    const char payload[] = "ota\n";
    err = send(sock, payload, strlen(payload), 0);
    if (err < 0) {
        // TODO
    }

    ESP_LOGI(TAG, "erasing ota1 partition");

    err = esp_partition_erase_range(ota1, 0, ota1->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "erase failed", errno);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        esp_restart();
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

    /* defined in esp_image_format.h */
    esp_image_metadata_t meta;
    esp_partition_pos_t ota1_pos = {
        .offset = ota1->address,
        .size = ota1->size,
    };

    ESP_ERROR_CHECK(esp_image_verify(ESP_IMAGE_VERIFY, &ota1_pos, &meta));
    if (!meta.image.hash_appended) {
        ESP_LOGE(TAG, "no sha256 appended at the end of image");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    }

    bootloader_common_update_rtc_retain_mem(&ota1_pos, false);
    esp_deep_sleep(1000);
}
