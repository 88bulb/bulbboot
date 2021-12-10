#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_log.h"

#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#include "bulbboot.h"

static const char *TAG = "bulbboot";

/**
 * - b0:1b:b0:07 (magic, 4 bytes)
 * - 7c:df:a1:61:ec:72 (ble mac address, big-endian, 6 bytes)
 * - xx:xx:xx:xx:xx:xx:xx:xx:xx:xx (sha80, 10 bytes)
 * - xx:xx:xx:xx:xx:xx (the first 3 bytes are used as ssid token,
 *                      all six bytes are stored in custom field in rtc mem)
 */
static void handle_mfr_data(uint8_t *bda, uint8_t *data, size_t data_len) {
    /* size */
    if (data_len != 26)
        return;

    /* magic must match */
    if (data[0] != 0xb0 || data[1] != 0x1b || data[2] != 0xb0 ||
        data[3] != 0x07)
        return;

    /* (big-endian) ble mac must match */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    if (data[4] != mac[0] || data[5] != mac[1] || data[6] != mac[2] ||
        data[7] != mac[3] || data[8] != mac[4] || data[9] != mac[5])
        return;

    ESP_LOGI(TAG, "bulbboot packet received");

    /* extract sha80 */
    sha80[0] = data[10];
    sha80[1] = data[11];
    sha80[2] = data[12];
    sha80[3] = data[13];
    sha80[4] = data[14];
    sha80[5] = data[15];
    sha80[6] = data[16];
    sha80[7] = data[17];
    sha80[8] = data[18];
    sha80[9] = data[19];

    /* last six bytes are boot parameters */
    boot_params[0] = data[20];
    boot_params[1] = data[21];
    boot_params[2] = data[22];
    boot_params[3] = data[23];
    boot_params[4] = data[24];
    boot_params[5] = data[25];

    /* generate hex string of sha80 */
    const char hex_char[16] = "0123456789abcdef";
    int i;
    for (i = 0; i < sizeof(sha80); i++) {
        sha80_hex[2 * i] = hex_char[sha80[i] / 16];
        sha80_hex[2 * i + 1] = hex_char[sha80[i] % 16];
    }
    sha80_hex[2 * i] = '\0';
    ESP_LOGI(TAG, "sha80 in hex string: %s", sha80_hex);

    /* the hex string of the first three bytes of boot_params
      are used as ssid token */
    ssid_token[0] = hex_char[boot_params[0] / 16];
    ssid_token[1] = hex_char[boot_params[0] % 16];
    ssid_token[2] = hex_char[boot_params[1] / 16];
    ssid_token[3] = hex_char[boot_params[1] % 16];
    ssid_token[4] = hex_char[boot_params[2] / 16];
    ssid_token[5] = hex_char[boot_params[2] % 16];
    ssid_token[6] = '\n';
    ESP_LOGI(TAG, "ssid token: %s", ssid_token);

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

        if (xEventGroupGetBits(ev) & LAST_WILL) {
            adv_params.adv_int_min = 0x0040; // 40ms
            adv_params.adv_int_max = 0x0080; // 80ms
        }
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    } break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
        ESP_LOGI(TAG, "ble adv started");
        if (xEventGroupGetBits(ev) & LAST_WILL) {
            xEventGroupSetBits(ev, LAST_WILL_ADV_START_COMPLETE);
            return;
        }
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
        xEventGroupSetBits(ev, ADV_STOP_COMPLETE);
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
        xEventGroupSetBits(ev, BOOT_SIGNALLED);
        break;
    default:
        ESP_LOGI(TAG, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

void ble_adv_scan(void *pvParameters) {
    uint8_t *age = (uint8_t *)pvParameters;
    uint8_t adv_mfr_data[32] = {0xb0, 0x1b, 0xca, 0x57, 0x00, 0x02, 0x00, 0x00};
    adv_mfr_data[7] = *age;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));

    esp_ble_adv_data_t adv_data = {
        .set_scan_rsp = false,
        .include_name = false,
        .include_txpower = false,
        .min_interval = 0x0000,
        .max_interval = 0x0000,
        .appearance = 0x00,
        .manufacturer_len = 8,
        .p_manufacturer_data = adv_mfr_data,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 0,
        .p_service_uuid = NULL,
        .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    };
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    xEventGroupWaitBits(ev, LAST_WILL, pdFALSE, pdFALSE, portMAX_DELAY);

    ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
    xEventGroupWaitBits(ev, ADV_STOP_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);

    // last will advertisement
    adv_data.manufacturer_len = 10;
    adv_mfr_data[5] = 0x04; // length
    adv_mfr_data[6] = 0x01; // type, last will
    adv_mfr_data[7] = last_will_reason;
    adv_mfr_data[8] = last_will_error;
    adv_mfr_data[9] = last_will_errno;
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    xEventGroupWaitBits(ev, LAST_WILL_ADV_START_COMPLETE, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    vTaskDelay(400 / portTICK_PERIOD_MS);
    esp_restart();
}
