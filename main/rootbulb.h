#ifndef _ROOT_BULB_H
#define _ROOT_BULB_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"

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

extern last_will_t last_will_reason;
extern esp_err_t last_will_error;

extern EventGroupHandle_t ev;

extern char ssid_token[4];
extern char ssid_token_str[16];
extern uint8_t sha88[11];
extern char sha88_str[32];

/* freertos task */
void ble_adv_scan(void *pvParameters);

/* freertos tasks */
void aging_test1(uint16_t aged_minutes);
void aging_test2();

void led_init();

/* read and write tuya aged time (in minutes) */
uint16_t read_aging_minutes();
esp_err_t write_aging_minutes(uint16_t minutes);

#endif
