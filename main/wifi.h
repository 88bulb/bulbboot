#ifndef __WIFI_H
#define __WIFI_H

#include "esp_err.h"
#include "esp_netif.h"

void wifi_init();
void wifi_scan(char *token, bool stop_after_scan, bool *found,
               wifi_ap_record_t *ap);
esp_err_t wifi_connect(const uint8_t *ssid, esp_netif_ip_info_t *ip_info);

#endif
