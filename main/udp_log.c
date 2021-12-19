#include <string.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "esp_log.h"
#include "wifi.h"

#define UDP_LOGGING_MAX_PAYLOAD_LEN (2048)

static int udp_log_fd = 0;
static struct sockaddr_in server_addr = {0};
static uint8_t buf[UDP_LOGGING_MAX_PAYLOAD_LEN] = {0};

static int udp_logging_vprintf(const char *str, va_list l) {
    int len = vsprintf((char *)buf, str, l);
    sendto(udp_log_fd, buf, len, 0, (struct sockaddr *)&server_addr,
           sizeof(server_addr));
    return len;
}

int udp_logging_init(const char *ssid_token, unsigned long port) {
    esp_err_t err;
    bool found = false;
    wifi_ap_record_t ap = {0};

    wifi_scan(ssid_token, false, &found, &ap);
    if (!found) {
        return -1;
    }

    const uint8_t ssid[] = "TP-LINK_a5a5a5a5";
    esp_netif_ip_info_t ip_info;
    err = wifi_connect(ap.ssid, &ip_info);
    if (err != ESP_OK) {
        ESP_LOGE("UDP_LOGGING", "failed to connect to ap %s", (char *)ssid);
        return -1;
    }

    ESP_LOGI("UDP_LOGGING", "initializing udp logging...");
    if ((udp_log_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0) {
        ESP_LOGE("UDP_LOGGING", "Cannot open socket!");
        return -1;
    }

    server_addr.sin_addr.s_addr = ip_info.gw.addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    struct timeval send_timeout = {1, 0};
    err = setsockopt(udp_log_fd, SOL_SOCKET, SO_SNDTIMEO,
                     (const char *)&send_timeout, sizeof(send_timeout));
    if (err < 0) {
        ESP_LOGE("UDP_LOGGING", "Failed to set SO_SNDTIMEO. Error %d", err);
        return -1;
    }

    int broadcast = 1;
    err = setsockopt(udp_log_fd, SOL_SOCKET, SO_BROADCAST, &broadcast,
                     sizeof(broadcast));
    if (err < 0) {
        ESP_LOGE("UDP_LOGGING", "Failed to set SO_BROADCAST. Error %d", err);
        return -1;
    }

    esp_log_set_vprintf(&udp_logging_vprintf);
    return 0;
}
