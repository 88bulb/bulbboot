#ifndef __LED_H
#define __LED_H

/* When temperature approaching this point, duty cycle is decreased faster.
 * When temperature reaching this point, the actual_brightness will be set to
 * the lowest possible value.
 */
#define ABSOLUTE_HIGHEST_TEMP (125)

/* The upper bound of highest_temp.  */
#define ALLOWED_HIGHEST_TEMP (120)

/* If no highest_temp is set by nvs, use this value */
#define DEFAULT_HIGHEST_TEMP (115)

/* When dynamic brightness is enabled, the finally stabilized brightness is not
 * influenced by initial setting. However, too high an initial value will
 * cause a long period of fluctuation of both temperature and brightness. So we
 * limit the initial setting by this value. It is the upper bound of initial
 * setting of brightness.
 */
#define ABSOLUTE_HIGHEST_BRIGHTNESS (192)

/* Default (cold white) brightness if no value stored in nvs. This value is
 * suitable for DEFAULT_HIGHEST_TEMP set to 112 (0x70).
 */
#define DEFAULT_BRIGHTNESS (115)

/* Default color temperature if no nvs value provided. 20% warm:code ratio. */
#define DEFAULT_COLOR_TEMP (20)

/* Brightness used for cold white and warm white during aging test. */
#define TESTING_WHITE_BRIGHTNESS (208)

/* Brightness used for RGB color during aging test. */
#define TESTING_COLOR_BRIGHTNESS (255)

#define BLINK_TIMES (5)
#define FADE_DURATION_MS (1000)
#define LOW_LIGHT_BRIGHTNESS (40)
#define LOWEST_LIGHT_BRIGHTNESS (10)

/* Aging time in minutes
 * - <50, aging test unfinished
 * - 50, aging test finished, inintial wifi scan for tuya_mdev_test2
 * - 0xee, aging test unfinished, initial wifi scan skipped.
 * - 0xff, aging test finished, initial wifi scan skipped.
 */
extern uint8_t aging_minutes;

/* The illuminating brightness (duty cycle) is decreased when temperature
 * reaching this point */
extern uint8_t highest_temp;

/* color_temp is defined as the warm white fraction of total brightness.
 * For example, 0 means all brightness is provided by cold white. 255 means
 * all brightness is provided by warm white. and 127 means 50:50.
 */
extern uint8_t color_temp;

/* The targeted brightness if temperature allowed. */
extern uint8_t target_brightness;

/* actual_brightness is the average value of cold white and warm white */
extern uint8_t actual_brightness;

/* The cold white brightness, used by boot */
extern uint8_t cold_white_brightness;

/* The warm white brightness, used by boot */
extern uint8_t warm_white_brightness;

/* Set to true when illumiate task starts. BLE module uses this value to
 * determine whether brightness and color temperature is going to be
 * broadcasted.
 */
extern bool led_illuminating;

/* Initialize led controller, temperature settings, brightness, as
 * well as color temperature.
 */
void led_init();

/* Choose appropriate task to run according to aging_minutes and possibly wifi
 * scan result. This function returns, that is, it won't stuck itself with
 * vTaskDelay.
 */
void led_run();

#endif
