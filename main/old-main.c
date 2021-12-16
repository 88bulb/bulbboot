#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_app_format.h"
#include "esp_image_format.h"

#include "esp_wifi.h"
#include "esp_http_client.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#define ADV_DATA_SET_COMPLETE (1 << 0)
#define ADV_START_COMPLETE (1 << 1)
#define ADV_STOP_COMPLETE (1 << 2)
#define ADV_TASK_INITIALIZED (1 << 3)
#define SCAN_PARAM_SET_COMPLETE (1 << 4)
#define SCAN_START_COMPLETE (1 << 5)
#define MADDR_ALLOCATED (1 << 6)
#define MADDR_DEALLOCATED (1 << 7)
#define BULBBOOT_REQUESTED (1 << 8)
#define BULBBOOT_READY (1 << 9)

#define HTTP_BUFFER_SIZE (16384)
static char http_buffer[HTTP_BUFFER_SIZE] = {};

static const char *TAG = "rootbulb";

static EventGroupHandle_t eg_handle;
static StaticEventGroup_t eg_data;

static uint8_t ble_mac_addr[6] = {};

static uint8_t mcast_addr[16] = {};
static uint8_t mcast_addr_len = 0; // total len, group id + bits

static char ssid_fingerprint[16] = {};
static uint8_t half_digest[16] = {};

#define AP_RECORD_MAX (20)

static uint16_t ap_record_max = AP_RECORD_MAX;
wifi_ap_record_t ap_record[AP_RECORD_MAX] = {};
static uint16_t ap_num = 0;

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

static const esp_ble_adv_data_t adv_data_default = {
    .set_scan_rsp = false,
    .include_name = false,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
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

uint8_t outgoing_mfr_data[32];

static bool is_bulbboot_msg(uint8_t *data, size_t data_len) {
    uint8_t *ptr;
    if (data_len != 26)
        return false;

    // TODO

    return false;
}

static bool is_bulbcast_msg(uint8_t *data, size_t data_len) {
    return data_len >= 5 && data[0] == 0xb0 && data[1] == 0x1b &&
           data[2] == 0xca && data[3] == 0x57;
}

static bool is_me(uint8_t *data, size_t data_len) {
    if (data_len < 6)
        return false;
    for (int i = 0; i < 6; i++) {
        if (data[i] != ble_mac_addr[i])
            return false;
    }
    return true;
}

static bool includes_me(uint8_t *data, size_t data_len) {
    if (mcast_addr_len == 0)
        return false;
    if (data_len < mcast_addr_len)
        return false;
    if (mcast_addr[0] != data[0] || mcast_addr[1] != data[1] ||
        mcast_addr[2] != data[2] || mcast_addr[3] != data[3])
        return false;
    for (int i = 4; i < mcast_addr_len; i++) {
        if (mcast_addr[i] & data[i])
            return true;
    }
    return false;
}

static void do_led(uint8_t red, uint8_t green, uint8_t blue, uint8_t white,
                   uint16_t transition_time) {
    ESP_LOGI(TAG,
             "update led, r:%u, g:%u, b:%u, w:%u, "
             "transition time (in 100ms): %u",
             red, green, blue, white, transition_time);
}

static void handle_mfr_data(uint8_t *bda, uint8_t *data, size_t data_len) {
    uint8_t addr_len, cmd;

    if (is_bulbboot_msg(data, data_len)) {
        // TODO
        return;
    }

    if (!is_bulbcast_msg(data, data_len)) {
        return;
    }

    // TODO debug level
    ESP_LOG_BUFFER_HEX(TAG, data, data_len);

    cmd = data[4];
    data = &data[5];
    data_len -= 5;

    switch (cmd) {
    case 0x10:
        break;
    case 0x20:
        if (is_me(data, data_len) && data_len == 12) {
            data = &data[6];
            data_len -= 6;
            if (data_len >= 6) {
                do_led(data[0], data[1], data[2], data[3],
                       data[4] + data[5] * 256);
            }
        }
        break;
    case 0x32:
    case 0x33:
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
    case 0x38: {
        addr_len = (0x0f & cmd) + 4;
        if (data_len > addr_len &&
            is_me(&data[addr_len], data_len - addr_len)) {
            for (int i = 0; i < addr_len; i++) {
                mcast_addr[i] = data[i];
            }
            mcast_addr_len = addr_len;
            ESP_LOGI(TAG, "multicast addr allocated");
            xEventGroupSetBits(eg_handle, MADDR_ALLOCATED);
        }
    } break;
    case 0x40:
        if (includes_me(data, data_len)) {
            data = &data[mcast_addr_len];
            data_len -= mcast_addr_len;
            if (data_len >= 6) {
                do_led(data[0], data[1], data[2], data[3],
                       data[4] + data[5] * 256);
            }
        }
        break;
    case 0x50:
        if (includes_me(data, data_len)) {
            esp_restart();
        }
        break;
    default:
        break;
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

            if (mfr_data_len > 0) {
                handle_mfr_data(scan_result->scan_rst.bda, mfr_data,
                                mfr_data_len);
            }
        } break;
        default:
            break;
        }
    } break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, ADV_DATA_SET_COMPLETE);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, ADV_START_COMPLETE);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, ADV_STOP_COMPLETE);
        break;
    default:
        ESP_LOGI(TAG, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

void advertising(void *pvParams) {
    esp_ble_adv_data_t adv_data = adv_data_default;
    uint8_t mfr_data[32] = {0xb0, 0x1b, 0xca, 0x57};
    adv_data.p_manufacturer_data = mfr_data;

    mfr_data[4] = 0x10;
    adv_data.manufacturer_len = 5;
    xEventGroupClearBits(eg_handle, ADV_DATA_SET_COMPLETE);
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    xEventGroupWaitBits(eg_handle, ADV_DATA_SET_COMPLETE, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    xEventGroupClearBits(eg_handle, ADV_START_COMPLETE);
    esp_ble_adv_params_t adv_params = adv_params_default;
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    xEventGroupWaitBits(eg_handle, ADV_START_COMPLETE, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    xEventGroupSetBits(eg_handle, ADV_TASK_INITIALIZED);
    xEventGroupWaitBits(eg_handle, MADDR_ALLOCATED | BULBBOOT_REQUESTED,
                        pdFALSE, pdFALSE, portMAX_DELAY);

    if (xEventGroupGetBits(eg_handle) & MADDR_ALLOCATED) {

        xEventGroupClearBits(eg_handle, ADV_STOP_COMPLETE);
        ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
        xEventGroupWaitBits(eg_handle, ADV_STOP_COMPLETE, pdFALSE, pdFALSE,
                            portMAX_DELAY);

        ESP_LOGI(TAG, "adv stop complete");

        mfr_data[4] = 0x11;
        for (int i = 0; i < mcast_addr_len; i++) {
            mfr_data[5 + i] = mcast_addr[i];
        }
        adv_data.manufacturer_len = 5 + mcast_addr_len;

        xEventGroupClearBits(eg_handle, ADV_DATA_SET_COMPLETE);
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
        xEventGroupWaitBits(eg_handle, ADV_DATA_SET_COMPLETE, pdFALSE, pdFALSE,
                            portMAX_DELAY);

        xEventGroupClearBits(eg_handle, ADV_START_COMPLETE);
        esp_ble_adv_params_t adv_params = adv_params_default;
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
        xEventGroupWaitBits(eg_handle, ADV_START_COMPLETE, pdFALSE, pdFALSE,
                            portMAX_DELAY);

        ESP_LOGI(TAG, "adv start complete");
    } else {
        xEventGroupWaitBits(eg_handle, BULBBOOT_READY, pdFALSE, pdFALSE,
                            portMAX_DELAY);
        // TODO sleep boot
    }

    vTaskDelay(portMAX_DELAY);
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

static void init_wifi() {
    static bool initialized = false;
    if (initialized)
        return;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
}

static void start_wifi() {
    init_wifi();
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void stop_wifi() { ESP_ERROR_CHECK(esp_wifi_stop()); }

static void do_wifi_scan() {
    memset(ap_record, 0, sizeof(ap_record));
    ap_record_max = AP_RECORD_MAX;
    ap_num = 0;
    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_record_max, ap_record));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));
}

static void connect_to_ap() {}

static esp_err_t update_ota1(const esp_partition_t *ota1, const char *url) {
    esp_err_t err;
    int ota1_written = 0;

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 5 * 1000,
        .keep_alive_enable = true,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    err = esp_http_client_open(client, 0);
    if (err)
        return err;

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length <= 0) {
        goto cleanup;
    }

    if (content_length > ota1->size) {
        goto cleanup;
    }

    int erase_size = content_length / 4096 * 4096;
    if (content_length % 4096) {
        erase_size += 4096;
    }

    err = esp_partition_erase_range(ota1, 0, erase_size);
    if (err != ESP_OK) {
        goto cleanup;
    }

    while (1) {
        int data_read =
            esp_http_client_read(client, http_buffer, HTTP_BUFFER_SIZE);

        if (data_read == 0) {
            // esp_http_client_read clear errno before invoking
            // esp_transport_read, the latter may set errno
            if (errno || !esp_http_client_is_complete_data_received(client)) {
                goto cleanup;
            }
            return ESP_OK;
        }

        if (data_read < 0) {
            goto cleanup;
        }

        if (data_read + ota1_written > ota1->size) {
            goto cleanup;
        }

        err = esp_partition_write(ota1, ota1_written, http_buffer, data_read);
        if (err != ESP_OK) {
            goto cleanup;
        }

        ota1_written += data_read;
    }

cleanup:
    esp_http_client_cleanup(client);
    return ESP_FAIL;
}

/**
 *
 * recipe
 * 1. (tear down ble scan) bring up wifi;
 * 2. search ap with ssid containing given hex string (group id);
 * 3. connect to ap
 * 4. connect to
 */
static void ota_task(void *pvParameter) {
    esp_err_t err;

    start_wifi();
    do_wifi_scan();

    // TODO find ap meets requirement
    // 1. containing some predefined hex string
    // 2. open wifi

    connect_to_ap();

    // Got IP
    // calculate url http://xx.xx.xx.xx/file/hash
    update_ota1(NULL, NULL);

    return;
}

void app_main(void) {

    ota_task(NULL);

    // init (a)synchronizer
    eg_handle = xEventGroupCreateStatic(&eg_data);

    // prepare mac
    esp_read_mac(&ble_mac_addr[0], ESP_MAC_BT);

    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* this function may never return */
    if (!is_factory_test_disabled()) {
        start_wifi();
        do_wifi_scan();
        stop_wifi();
        // TODO
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    // ESP_ERR_NOT_SUPPORTED
    // ESP_ERROR_CHECK(esp_bt_sleep_disable());

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    ESP_LOGI(TAG, "start adv task...");

    xTaskCreate(&advertising, "advertising", 4096, NULL, 3, NULL);
    xEventGroupWaitBits(eg_handle, ADV_TASK_INITIALIZED, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    ESP_LOGI(TAG, "adv task initialized");

    esp_ble_scan_params_t scan_params = scan_params_default;
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));
    xEventGroupWaitBits(eg_handle, SCAN_PARAM_SET_COMPLETE, pdFALSE, pdFALSE,
                        portMAX_DELAY);
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0));
    xEventGroupWaitBits(eg_handle, SCAN_START_COMPLETE, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    ESP_LOGI(TAG, "ble scan started");

    xEventGroupWaitBits(eg_handle, MADDR_ALLOCATED | BULBBOOT_REQUESTED,
                        pdFALSE, pdFALSE, portMAX_DELAY);
    if (xEventGroupGetBits(eg_handle) & MADDR_ALLOCATED) {
        xEventGroupWaitBits(eg_handle, MADDR_DEALLOCATED, pdFALSE, pdFALSE,
                            portMAX_DELAY);
    } else {
        ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());

        const esp_partition_t *ota1 = esp_partition_find_first(
            ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_OTA(1), NULL);
        if (!ota1) {
            ESP_LOGE(TAG, "no partition of type app and subtype ota_1 found");
            // goto restart;

            // TODO
            esp_restart();
        }

        ESP_LOGI(TAG, "ota_1 partition found, address: %u, size: %u, label %s",
                 ota1->address, ota1->size, ota1->label);

        const esp_partition_pos_t ota1_pos = {
            .offset = ota1->address,
            .size = ota1->size,
        };

        /* defined in esp_image_format.h */
        esp_image_metadata_t meta;
        err = esp_image_verify(ESP_IMAGE_VERIFY, &ota1_pos, &meta);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_image_verify ota1 failed, %s",
                     esp_err_to_name(err));
            // TODO
            esp_restart();
        }

        if (esp_image_get_metadata(&ota1_pos, &meta) != ESP_OK) {
            // ESP_ERR_IMAGE_INVALID
            ESP_LOGE(TAG, "invalid image metadata");

            // TODO
            esp_restart();
            // goto restart;
        }

        /* metadata.image is of type esp_image_header_t, which is defined in
         * esp_app_format.h */
        if (!meta.image.hash_appended) {
            ESP_LOGE(TAG, "no sha256 appended at the end of image");
            xEventGroupWaitBits(eg_handle, BULBBOOT_READY, pdFALSE, pdFALSE,
                                portMAX_DELAY);
            // TODO sleep boot
        }
    }
}
