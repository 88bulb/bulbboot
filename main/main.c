#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

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

#define BOOT_SIGNALLED (1 << 1)
#define STA_GOT_IP (1 << 2)
#define LAST_WILL (1 << 3)
#define ADV_STOP_COMPLETE (1 << 4)
#define LAST_WILL_ADV_START_COMPLETE (1 << 5)

typedef enum {
    OTA_PARTITION_NOT_FOUND = 0,
    OTA_AP_NOT_FOUND = 1,
    OTA_NETWORK_TIMEOUT = 2,
    OTA_LWIP_SOCKET,
    OTA_LWIP_CONNECT,
    OTA_SEND_BULBBOOT,
    OTA_READ_SIZE,
    OTA_BAD_FIRMWARE_SIZE,
    OTA_PARTITION_ERASE_RANGE,
    OTA_RECV_FILE,
    OTA_FIRMWARE_UNDERSIZE,
    OTA_FIRMWARE_OVERSIZE,
    OTA_ESP_PARTITION_WRITE,
} last_will_t;

static last_will_t last_will_reason;
static esp_err_t last_will_error;

#define LAST_WILL_CHECK(x, reason)                                             \
    do {                                                                       \
        esp_err_t __err = (x);                                                 \
        if (__err != ESP_OK) {                                                 \
            last_will((reason), __err);                                        \
        }                                                                      \
    } while (0)

#define LAST_WILL_COND(x, reason)                                              \
    do {                                                                       \
        if ((x)) {                                                             \
            last_will((reason), ESP_OK);                                       \
        }                                                                      \
    } while (0)

static char ssid_token[4] = {0};
static char ssid_token_str[16] = {0};
static uint8_t sha88[11] = {0};
static char sha88_str[32] = {0};

static const char *TAG = "rootbulb";

static EventGroupHandle_t ev;
static StaticEventGroup_t eg_data;

const ledc_channel_config_t lc[5] = {
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

static void last_will(last_will_t reason, esp_err_t err) {
    last_will_reason = reason;
    last_will_error = err;
    xEventGroupSetBits(ev, LAST_WILL);
    vTaskDelay(portMAX_DELAY);
}

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
    uint8_t mac[6];
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
    uint8_t adv_mfr_data[32] = {0xb0, 0x1b, 0xca, 0x57, 0x00};

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
    xEventGroupWaitBits(ev, LAST_WILL, pdFALSE, pdFALSE, portMAX_DELAY);

    ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
    xEventGroupWaitBits(ev, ADV_STOP_COMPLETE, pdFALSE, pdFALSE, portMAX_DELAY);

    // last will advertisement
    adv_mfr_data[4] = 0x08;
    adv_mfr_data[5] = last_will_reason;
    adv_mfr_data[6] = last_will_error;
    adv_mfr_data[7] = (errno);
    adv_data.manufacturer_len = 8;
    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
    xEventGroupWaitBits(ev, LAST_WILL_ADV_START_COMPLETE, pdFALSE, pdFALSE,
                        portMAX_DELAY);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    esp_restart();
}

static esp_netif_t *init_wifi() {
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
    bool found;
    esp_err_t err;
    esp_netif_t *sta_netif = NULL;
    wifi_ap_record_t ap;
    esp_netif_ip_info_t ip_info;

    ev = xEventGroupCreateStatic(&eg_data);

    /* initialize led */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .freq_hz = 5000,                     // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_1,           // timer index
        .clk_cfg = LEDC_AUTO_CLK,            // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);
    for (int ch = 0; ch < 5; ch++) {
        ledc_channel_config(&lc[ch]);
    }
    ledc_fade_func_install(0);

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

    /* create ble task */
    xTaskCreate(&ble_adv_scan, "ble_adv_scan", 4096, NULL, 6, NULL);
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
