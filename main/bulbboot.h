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

#define ADT_DEVICE_INFO     (0x00)
#define ADT_AGING_TIME      (0x01)
#define ADT_TEMP            (0x02)
#define ADT_DYN_BRIGHTNESS  (0x03)

#define HARDWARE_ID         (0x00)     // bulb
#define SOFTWARE_ID         (0x00)     // bulbboot

#define BOOT_SIGNALLED (1 << 1)
#define STA_GOT_IP (1 << 2)
#define LAST_WILL (1 << 3)
#define ADV_STOP_COMPLETE (1 << 4)
#define LAST_WILL_ADV_START_COMPLETE (1 << 5)

#define TAG ("bulbboot")

typedef enum {
    OTA_PARTITION_NOT_FOUND = 0,
    OTA_AP_NOT_FOUND = 1,
    OTA_NETWORK_TIMEOUT = 2,
    OTA_LWIP_SOCKET = 3,
    OTA_LWIP_CONNECT = 4,
    OTA_SEND_BULBBOOT = 5,
    OTA_READ_SIZE = 6,
    OTA_BAD_FIRMWARE_SIZE = 7,
    OTA_PARTITION_ERASE_RANGE = 8,
    OTA_RECV_FILE = 9,
    OTA_FIRMWARE_UNDERSIZE = 10,
    OTA_FIRMWARE_OVERSIZE = 11,
    OTA_ESP_PARTITION_WRITE = 12,
} last_will_t;

extern last_will_t last_will_reason;
extern esp_err_t last_will_error;
extern int last_will_errno;

extern EventGroupHandle_t ev;

/* sha80 in bulbboot packet */
extern uint8_t sha80[10];
/* last six bytes in bulbboot packet */
extern uint8_t boot_params[6];
/* sha80 in hex string */
extern char sha80_hex[21];
/* the hex string of first 3 bytes in
   boot params are used as ssid token */
extern char ssid_token[7];

/* initialize led in pwm mode */
void led_init();

/* read and write tuya aged time (in minutes) */

extern uint8_t aging_minutes;
esp_err_t write_aging_minutes(uint8_t minutes);

/* temperature */
extern uint8_t temp;

/* these functions run in main task */
void aging_test1();
void aging_test2();

/* freertos task */
void ble_adv_scan(void *pvParameters);

#endif
