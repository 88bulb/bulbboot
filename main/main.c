#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
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
#include "bootloader_common.h"

#include "esp_ota_ops.h"

#include "version.h"

#include "bulbboot.h"
#include "led.h"
#include "wifi.h"
#include "temp_sensor.h"
#include "ble_adv_scan.h"

#define PORT (6016)

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

uint8_t sha80[10] = {0};
uint8_t boot_params[6] = {0};
char sha80_hex[21] = {0};
char ssid_token[7] = {0};

last_will_t last_will_reason;
esp_err_t last_will_error;
int last_will_errno;

static StaticEventGroup_t eg_data;
EventGroupHandle_t ev;
static bool found;
static wifi_ap_record_t ap;

static void nvs_init() {
    /* init nvs flash */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void last_will(last_will_t reason, esp_err_t err) {
    last_will_reason = reason;
    last_will_error = err;
    last_will_errno = errno;

    ESP_LOGE(TAG, "last will reason %d, %d, %d", last_will_reason,
             last_will_error, last_will_errno);

    xEventGroupSetBits(ev, LAST_WILL);
    vTaskDelay(portMAX_DELAY);
}

/* storing boot_params into rtc_mem custom field and fast boot to given ota
 * partition */
static void sleep_boot(esp_partition_pos_t *ota1_pos) {
    /**
     * typedef struct {
     *     esp_partition_pos_t partition;   // Partition of application which
     *                                      // worked before goes to the deep
     *                                      // sleep.
     *     uint16_t reboot_counter;         // Reboot counter. Reset only when
     *                                      // power is off.
     *     uint16_t reserve;                //  Reserve
     * #ifdef CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC
     *     // Reserved for custom purpose
     *     uint8_t custom[CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC_SIZE];
     * #endif
     *     uint32_t crc;                    // Check sum crc32
     * } rtc_retain_mem_t;
     */
    rtc_retain_mem_t *rtc_mem = bootloader_common_get_rtc_retain_mem();
    for (int i = 0; i < sizeof(boot_params); i++) {
        rtc_mem->custom[0] = boot_params[0];
        rtc_mem->custom[1] = boot_params[1];
        rtc_mem->custom[2] = boot_params[2];
        rtc_mem->custom[3] = boot_params[3];
        rtc_mem->custom[4] = boot_params[4];
        rtc_mem->custom[5] = boot_params[5];
    }

    /* reboot_counter must be set to false, otherwise
       the crc will be checked. And because we have modified
       custom field, the check is going to fail and the rtc mem is reset.
       the custom field is cleared. */
    bootloader_common_update_rtc_retain_mem(ota1_pos, false);
    esp_deep_sleep(1000);
}

static void print_partitions() {
    ESP_LOGI(TAG, "Iterating through partitions...");
    esp_partition_iterator_t it;
    for (it = esp_partition_find(ESP_PARTITION_TYPE_APP,
                                 ESP_PARTITION_SUBTYPE_ANY, NULL);
         it != NULL; it = esp_partition_next(it)) {
        const esp_partition_t *part = esp_partition_get(it);
        ESP_LOGI(TAG, "found app partition '%s' at offset 0x%x with size 0x%x",
                 part->label, part->address, part->size);
    }
    esp_partition_iterator_release(it);
    for (it = esp_partition_find(ESP_PARTITION_TYPE_DATA,
                                 ESP_PARTITION_SUBTYPE_ANY, NULL);
         it != NULL; it = esp_partition_next(it)) {
        const esp_partition_t *part = esp_partition_get(it);
        ESP_LOGI(TAG, "found data partition '%s' at offset 0x%x with size 0x%x",
                 part->label, part->address, part->size);
    }
    esp_partition_iterator_release(it);
}

static void boot_ota1() {
    esp_err_t err;

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
        ESP_LOG_BUFFER_HEX(TAG, sha80, sizeof(sha80));

        /* if digest matches, direct boot */
        bool match = true;
        for (int i = 0; i < sizeof(sha80); i++) {
            if (meta.image_digest[i] != sha80[i]) {
                match = false;
                break;
            }
        }
        if (match)
            sleep_boot(&ota1_pos);
    }

    /* find wifi access point given in boot signal */
    wifi_scan((char *)ssid_token, false, &found, &ap);
    LAST_WILL_COND(!found, OTA_AP_NOT_FOUND);

    esp_netif_ip_info_t ip_info = {};
    LAST_WILL_CHECK(wifi_connect(ap.ssid, &ip_info), OTA_NETWORK_TIMEOUT);

    /* connect to server (gateway) */
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = ip_info.gw.addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    LAST_WILL_COND(sock < 0, LWIP_SOCKET);
    LAST_WILL_COND(
        /* lwip_connect returns 0 for OK, -1 for Failure */
        (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr))),
        OTA_LWIP_CONNECT);
    ESP_LOGI(TAG, "server connected");

    /* GET <SHA80> */
    char req[32] = {0};
    strcat(strcat(strcat(req, "GET "), sha80_hex), "\n");
    printf("req line: %s", req);
    LAST_WILL_COND((0 > send(sock, req, strlen(req), 0)), OTA_SEND_BULBBOOT);

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

    sleep_boot(&ota1_pos);
}

void app_main(void) {
    ev = xEventGroupCreateStatic(&eg_data);
    nvs_init();
    led_init();
    print_partitions();
    ver_init();
    temp_sensor_init();
    wifi_init();
    led_run();

    xTaskCreate(&ble_adv_scan, "ble_adv_scan", 4096, NULL, 6, NULL);

    bool bootable = false;
    bool boot_signalled = false;

    while (1) {
        xEventGroupWaitBits(ev, BOOTABLE | BOOT_SIGNALLED, pdFALSE, pdFALSE,
                            portMAX_DELAY);
        EventBits_t bits = xEventGroupGetBits(ev);
        if (bits & BOOTABLE) {
            xEventGroupClearBits(ev, BOOTABLE);
            bootable = true;
        }

        if (bits & BOOT_SIGNALLED) {
            xEventGroupClearBits(ev, BOOT_SIGNALLED);
            boot_signalled = true;
        }

        if (bootable && boot_signalled)
            break;
    };

    boot_ota1();
}
