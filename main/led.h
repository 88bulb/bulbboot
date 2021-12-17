#ifndef __LED_H
#define __LED_H

#define ABSOLUTE_HIGHEST_TEMP (115)
#define DEFAULT_HIGHEST_TEMP (105)
#define ABSOLUTE_HIGHEST_BRIGHTNESS (128)
#define DEFAULT_BRIGHTNESS (80)

extern uint8_t aging_minutes;
extern uint8_t highest_temp;
extern int8_t color_temp;
extern uint8_t target_brightness;
extern uint8_t actual_brightness;
extern bool led_illuminating;

/* initialize led in pwm mode */
void led_init();
void led_run();

#endif
