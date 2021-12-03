#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

#define ADV_DATA_SET_COMPLETE (1 << 0)
#define ADV_START_COMPLETE (1 << 1)
#define ADV_STOP_COMPLETE (1 << 2)

static const *TAG = "rootbulb"

static EventGroupHandle_t eg_handle;
static StaticEventGroup_t eg_data;

/**
 * scanning and wifi are mutual exclusive
 */
static const esp_ble_scan_params_t scan_params_default = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x50,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

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
    .adv_int_min = 0x20,
    .adv_int_max = 0x28,
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

uint8_t outgoing_mfr_data[32];

/**
 *
 */
static void esp_gap_cb(esp_gap_ble_cb_event_t event,
                       esp_ble_gap_cb_param_t *param) {
    uint8_t *mfr_data;
    uint8_t mfr_data_len;

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        ESP_LOGI(tag, "starting ble scan (permanently)");
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
            mfr_data = esp_ble_resolve_adv_data(
                scan_result->scan_rst.ble_adv,
                ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &mfr_data_len);

            if (mfr_data_len > 0) {
                char *bda_str = u8_to_hex(bda_buf, sizeof(bda_buf),
                                          scan_result->scan_rst.bda, 6);
                char *mfr_str =
                    u8_to_hex(mfr_buf, sizeof(mfr_buf), mfr_data, mfr_data_len);

                printf("BTRECV ADDR: %s, MFR: %s\n", bda_str, mfr_str);
            }
        } break;
        default:
            break;
        }
    } break;
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(tag, "ble scan started");
        break;
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, BIT_0);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, BIT_1);
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        xEventGroupSetBits(eg_handle, BIT_2);
        break;
    default:
        ESP_LOGI(tag, "unhandled ble event %d in esp_gap_cb()", event);
    }
}

void app_main(void)
{
    // init (a)synchronizer
    eg_handle = xEventGroupCreateStatic(&eg_data);
    xEventGroupSetBits(ADV_STOP_COMPLETE);

    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bt controller init error: %s",
                 esp_err_to_name(err));
        return;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OR) {
        ESP_LOGE(TAG, "bt controller enable error: %s", esp_err_to_name(err));
        return;
    } 

    err = esp_bt_sleep_disable();
    if (err != ESP_OR) {
        ESP_LOGE(TAG, "bt sleep disable error: %s", esp_err_to_name(err));
        return;
    } 

    err = esp_bluedroid_init();
    if (err != ESP_OR) {
        ESP_LOGE(TAG, "bluedroid init error: %s", esp_err_to_name(err));
        return;
    } 

    err = esp_bluedroid_enable();
    if (err != ESP_OR) {
        ESP_LOGE(TAG, "bluedroid enable error: %s", esp_err_to_name(err));
        return;
    } 

    err = esp_ble_gap_register_callback(esp_gap_cb);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "ble gap register callback error: %s",
                 esp_err_to_name(err));
        return;
    }

    esp_ble_scan_params_t scan_params = scan_params_default; 
    err = esp_ble_gap_set_scan_params(&scan_params); 
    if (err != ESP_OK) {
        ESP_LOGE(tag, "ble gap set scan params error: %s",
                 esp_err_to_name(err));
        return;
    }

    esp_ble_adv_data_t = adv_data = adv_data_default; 
    adv_data.p_manufacturer_data = &outgoing_mfr_data[0];
    adv_data.manufacturer_len = // TODO 

    err = esp_ble_gap_config_adv_data(&adv_data);
    if (err) {
        ESP_LOGE(TAG, "esp_ble_gap_config_adv_data error: %s",
                 esp_error_to_name(err));
    }
    

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
