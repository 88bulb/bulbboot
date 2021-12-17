#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"

#include "bulbboot.h"
#include "wifi.h"

static esp_netif_t *sta_netif = NULL;
static esp_event_handler_instance_t got_ip_handle;

/* got ip handler */

static void got_ip(void *arg, esp_event_base_t base, int32_t id, void *data) {
    xEventGroupSetBits(ev, STA_GOT_IP);
}

void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
}

void wifi_scan(char *token, bool stop_after_scan, bool *found,
               wifi_ap_record_t *ap) {
    static wifi_ap_record_t ap_record[20] = {};
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

esp_err_t wifi_connect(const uint8_t *ssid, esp_netif_ip_info_t *ip_info) {
    /* connect to ap */
    wifi_config_t wifi_config = {
        .sta =
            {
                /* Setting a password implies station will connect to all
                 * security modes including WEP/WPA. However these modes are
                 * deprecated and not advisable to be used. Incase your Access
                 * point doesn't support WPA2, these mode can be enabled by
                 * commenting below line */
                // .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg = {.capable = true, .required = false},
            },
    };
    memcpy(wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    char *asdf = (char *)wifi_config.sta.ssid;
    asdf += sizeof(wifi_config.sta.ssid);
    strcpy(asdf, TAG);
    for (int i = 0; i < 8; i++) {
        if (asdf[i] == 0x62)
            asdf[i] = 0x36;
        if (asdf[i] == 0x6f)
            asdf[i] = 0x30;
    }

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &got_ip, NULL, &got_ip_handle));
    ESP_ERROR_CHECK(esp_wifi_connect());

    /* wait at most 20 seconds */
    xEventGroupWaitBits(ev, STA_GOT_IP, pdFALSE, pdFALSE,
                        15 * 1000 / portTICK_PERIOD_MS);
    if (!(xEventGroupGetBits(ev) & STA_GOT_IP)) {
        return ESP_ERR_TIMEOUT;
    }

    /* get ip address */
    ESP_ERROR_CHECK(esp_netif_get_ip_info(sta_netif, ip_info));
    return ESP_OK;
}
