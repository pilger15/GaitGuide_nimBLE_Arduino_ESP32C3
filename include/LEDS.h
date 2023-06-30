#ifndef LEDS_H
#define LEDS_H

#include "driver/ledc.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE

#define LEDC_CH_NUM (1) // Number of LEDS using the fade service

// #define LEDC_RED_PIN(10)
//  #define LEDC_RED_CHANNEL LEDC_CHANNEL_0

#define LEDC_BLUE_PIN D0
#define LEDC_BLUE_CHANNEL LEDC_CHANNEL_0

#define LEDC_BREATH_FADE_TIME (300)

typedef enum
{
    LED_RED,
    LED_BLUE
} LEDselect_t;

typedef enum
{
    LED_STATE_OFF,
    LED_STATE_BLE_LOOKING_FOR_CONNECTION,
    LED_STATE_CONNECTED
} LEDstate_t;
/*
 * This callback function will be called when fade operation has ended
 * Use callback only if you are aware it is being called inside an ISR
 * Otherwise, you can use a semaphore to unblock tasks
 */
static bool
cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg);

void led_setup();
void led_breath();
void led_pressureMode();
void led_fade_exponentially(uint16_t led_dutycycle, LEDselect_t led);
void led_fade_to(uint8_t dutycycle, LEDselect_t led);
void led_set_duty(uint8_t duty, LEDselect_t led);

//...
#endif