/** Gait guide
 *
 * DRV2605-Driver interaction
 * Currently all drivers use the same I2C bus (only one I2 peripheral available).
 * Since there are multiple drivers present for each side (medial, lateral, two each)
 * and all devices have the same address, the bus can not be used  for reading actions.
 *
 * To control lateral and medial derivers independently, the external trigger level is used.
 *
 *  Created: 2023
 *      Author: JMartus
 */

/*
#TODO ----------------------------------------------
- fix bluetoth resetting device?
*/

#include "Adafruit_DRV2605.h"
#include <Arduino.h>
#include "NimBLEDevice.h"
#include <SPI.h>
#include <Wire.h>
#include <GaitGuide.h>
#include <DRV2605_util.h>
#include <BLE_HANDLER.h>
#include <LEDS.h>
#include <ADC_util.h>

#define I2C_SDA D5
#define I2C_SCL D6
#define I2C_FREQ 20000UL // 500000UL
#define TRIGGER_MEDIAL D3
#define TRIGGER_LATERAL D4
#define ENABLE_MEDIAL D2
#define ENABLE_LATERAL D1
// TwoWire I2C = TwoWire(0);

DRV2605_UTIL drv;
bool is_timescale_1ms = false;

static const std::string deviceName = "GaitGuide_DEV";

uint8_t effect_list_index = 0;
uint8_t effect_list[] = {
    DRV_EFF_STRONG_CLK_100,
    // DRV_EFF_STRONG_CLK_1,
    // DRV_EFF_SHARP_CLK_100,
    // DRV_EFF_SHARP_TICK_1,
    // DRV_EFF_SOFT_BUMP_100,
    // DRV_EFF_PULSING_STRONG_1,
    DRV_EFF_PULSING_SHARP_1,
    DRV_EFF_STRONG_BUZZ_100,
    // DRV_EFF_BUZZ_1,
    DRV_EFF_LONG_BUZZ,
    DRV_EFF_SMOOTH_HUM_1,
};
uint8_t effect_list_size = sizeof(effect_list) / sizeof(effect_list[0]);
uint8_t effect = 0;
uint8_t effect_duration = 100;

uint8_t rtp_index = 0;
uint8_t rtp[] = {0x01, 0x20, 0x40, 0x60, 0x7F, 0xCC, 0xFF};
uint8_t rtp_size = sizeof(rtp) / sizeof(rtp[0]);
uint8_t rtp_amp = 0x00;

uint16_t targetLED = 1400;
uint16_t margin = 100;
uint16_t bound = 500;

auto &gaitGuide = GaitGuide::getInstance();
gaitGuide_stimMode_t stimMode = gaitGuide.stimMode();

void ble_setup();
void haptuation();
void findPressure();
// void led_setup();

void setup()
{

    log_i("Initialising GaitGuide");
    // Wire.setClock();
    Wire.setPins(I2C_SDA, I2C_SCL);
    // I2C.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    delay(1);
    drv.setTrigger(TRIGGER_MEDIAL, TRIGGER_LATERAL);
    drv.setEnable(ENABLE_MEDIAL, ENABLE_LATERAL);
    drv.begin();
    drv.enable();
    drv.init();

    drv.disable();

    ble_setup(deviceName);
    led_setup();
    configure_adc(ADC_WIDTH_BIT_12, ADC_ATTEN_DB_11);
    ;
    log_d("STARTING DEMO MODE - Fix for production!");
    gaitGuide.stimMode(GAITGUIDE_USERMODE_DEMO00);
    gaitGuide.newEvent(GAITGUIDE_EVENT_STIM);
    delay(5000);
}
void loop()
{
    switch (gaitGuide.currentState())
    {
    case GAITGUIDE_STATE_STARTUP:
        // Handle Start-up state
        // if this state is reached the startup phase is completed
        log_i("Initialisation complete");
        gaitGuide.newEvent(GAITGUIDE_EVENT_DONE);
        break;

    case GAITGUIDE_STATE_LFC:
        log_d("Entering LFC_State");
        // Handle Looking-for-connection state
        // #TODO: implement breathing and advertising as well as sleeping
        break;

    case GAITGUIDE_STATE_LOW_POWER:
        log_d("Entering Low Power State");
        // Handle Low-power state
        // #TODO: implement breathing and advertising as well as sleeping
        break;

    case GAITGUIDE_STATE_IDLE:
        log_d("Entering IDLE_State");
        // Handle Idle state
        // time for an event to occur, in case an event occurs dont do idle  tasks
        // #TODO implementiere entsprechenden timer

        // #TODO update Battery

        break;

    case GAITGUIDE_STATE_STIM:
        log_d("Entering STIM_State");
        // Handle Stimulation state
        haptuation();
        break;

    case GAITGUIDE_STATE_PRESSURE_SENSING:
        // Handle Pressure-finding state
        findPressure();
        break;

    case GAITGUIDE_STATE_PRESSURE_SETTING:
        // Handle Pressure-setting state
        // When a pressure value != 0 has been sent, use this to set the pressure, otherwise measure and set the current pressure as targetPressure        */
        gaitGuide.setTargetPressure(multisample_read(16));
        break;
    case GAITGUIDE_STATE_CHECK_PRESSURE:
        // #TODO implementiere entsprechenden timer nud event
        gaitGuide.set_currentPressureLevel(multisample_read(16)); // multisample_read(16));
        break;
    default:
        // Handle error state
        break;
    }
    // indicate that all events have been processed - free the event handler
    gaitGuide.newEvent(GAITGUIDE_EVENT_NONE);
}

void findPressure()
{
    // #TODO SET up pressure finding mode
    ESP_LOGI("+++", "USERPRESSURE");

    uint16_t voltage = 0;
    uint16_t led_dutycycle = 0;

    // this event indicates pressure mode is entered for the first time
    if (gaitGuide.currentEvent() == GAITGUIDE_EVENT_FIND_PRESSURE)
    {
        led_pressureMode();
    }

    voltage = multisample_read(16);
    gaitGuide.set_currentPressureLevel(voltage); // multisample_read(16));
    ble_advertisePressure(voltage);
    if (voltage > (gaitGuide.getTargetPressure() + margin))
    {

        if (voltage > (gaitGuide.getTargetPressure() + bound))
        {
            led_dutycycle = 255;
        }
        else
        {
            // map voltage to 0-255
            led_dutycycle = (voltage - (gaitGuide.getTargetPressure() + margin)) * 255 / (bound - margin);

            // fadeTo = (255.0 / (pow(R, ((bound - margin) / 255)))) * pow(R, (voltage - (targetLED + margin)));
        }
        ESP_LOGI("+++", "RED  TARGET:%d VALUE: %d -- FADE:%d", gaitGuide.getTargetPressure(), voltage, led_dutycycle);
        led_fade_exponentially(led_dutycycle, LED_RED);
        led_fade_to(0, LED_BLUE);
    }
    else if (voltage < (gaitGuide.getTargetPressure() - margin))
    {
        led_dutycycle = ((gaitGuide.getTargetPressure() - margin) - voltage) * 255 / (bound - margin);

        if (voltage < (gaitGuide.getTargetPressure() - bound))
        {
            led_dutycycle = 255;
        }
        led_fade_exponentially(led_dutycycle, LED_BLUE);
        led_fade_to(0, LED_RED);
        ESP_LOGI("---", "BLUE|| TARGET:%d VALUE: %d -- FADE:%d", gaitGuide.getTargetPressure(), voltage, led_dutycycle);
    }
    else
    {
        led_fade_to(0, LED_RED);
        led_fade_to(0, LED_BLUE);
    }

    //  delay(100);
}
void haptuation()
{
    log_i("starting stimulation");
    switch (gaitGuide.stimMode())
    {
    case GAITGUIDE_USERMODE_AMPLITUDE:
        if (gaitGuide.goMedial)
        {
            drv.enableMedial();
        }
        else
        {
            drv.enableLateral();
        }
        drv.setRealtimeValue(gaitGuide.amp());
        delay(gaitGuide.duration());
        drv.setRealtimeValue(0x00);
        drv.disable();
        //  ESP_LOGD(TAG_DRV, "SetRealtimeValue = %d for %dms", gaitGuide.amp(), gaitGuide.duration());
        break;

    case GAITGUIDE_USERMODE_EFFECT:
        if (gaitGuide.goMedial)
        {
            drv.enableMedial();
            drv.startMedial();
        }
        else
        {

            drv.enableLateral();
            drv.startLateral();
        }
        delay(gaitGuide.duration());
        drv.stop();
        drv.disable();
        //   ESP_LOGD(TAG_DRV, "Effect #%d: %s for %dms", gaitGuide.effect(0), drv_effect_string_map[gaitGuide.effect(0)], gaitGuide.duration());
        break;

    case GAITGUIDE_USERMODE_PRESSURE:
        // #TODO remove and clean-up usermodes
        break;
    case GAITGUIDE_USERMODE_DEMO00:

        //** ROB MODE **/

        drv.enable();
        drv.setMode(DRV2605_MODE_EXTTRIGLVL);

        effect = DRV_EFF_STRONG_BUZZ_100;
        for (uint8_t i = 0; i < 8; i++)
        {
            drv.setWaveform(i, effect);
        }
        drv.disable();
        while (gaitGuide.stimMode() == GAITGUIDE_USERMODE_DEMO00)
        {
            drv.enableMedial();
            // #TODO: move to drv
            log_d("USERDEMO\n---------\n");

            drv.startMedial();
            // digitalWrite(TRIGGER_LATERAL, HIGH);
            delay(150);
            // stop
            // drv.writeRegister8(DRV2605_REG_GO, 0);
            drv.stopMedial();
            drv.disableMedial();
            // digitalWrite(TRIGGER_LATERAL, LOW);
            delay(10000);
        }

        ESP_LOGD(TAG_DRV, "SetRealtimeValue = %d for %dms", gaitGuide.amp(), gaitGuide.duration());
        break;
    default:
        ESP_LOGD(TAG_DRV, "Undefined Usermode");
        break;
    }
}

/*
     digitalWrite(TRIGGER_MEDIAL, HIGH);
 digitalWrite(TRIGGER_LATERAL, HIGH);
 delay(150);
 // stop
 // drv.writeRegister8(DRV2605_REG_GO, 0);
 digitalWrite(TRIGGER_MEDIAL, LOW);
 digitalWrite(TRIGGER_LATERAL, LOW);
 delay(1000);

 digitalWrite(TRIGGER_MEDIAL, HIGH);
 // digitalWrite(TRIGGER_LATERAL, HIGH);
 delay(150);
 // stop
 // drv.writeRegister8(DRV2605_REG_GO, 0);
 digitalWrite(TRIGGER_MEDIAL, LOW);
 // digitalWrite(TRIGGER_LATERAL, LOW);
 delay(1000);


     switch (stimMode)
     {
     case demo_idle:
         break;
     case demo_individual_effects:
         drv.setMode(DRV2605_MODE_INTTRIG);
         drv.setWaveform(0, effect); // play effect
         drv.setWaveform(1, 0);      // end waveform

         // play the effect!
         ESP_LOGI(TAG_DRV, "Effect #%d: %s for %dms", effect, drv_effect_string_map[effect], effect_duration);
         drv.go();
         delay(effect_duration);
         // stop
         drv.writeRegister8(DRV2605_REG_GO, 0);
         if (effect_list_index >= effect_list_size)
         {
             ESP_LOGI(TAG_DRV, "-------");
             // stimMode = demo_repeated;
             effect_list_index = 0;
         }
         break;
     case demo_effects_loop:
         drv.setMode(DRV2605_MODE_INTTRIG);
         effect = effect_list[effect_list_index];
         drv.setWaveform(0, effect); // play effect
         drv.setWaveform(1, 0);      // end waveform

         // play the effect!
         ESP_LOGI(TAG_DRV, "Effect #%d: %s for 100ms", effect, drv_effect_string_map[effect]);
         drv.go();
         delay(100);
         // stop
         drv.writeRegister8(DRV2605_REG_GO, 0);
         effect_list_index++;
         if (effect_list_index >= effect_list_size)
         {
             ESP_LOGI(TAG_DRV, "-------");
             // stimMode = demo_repeated;
             effect_list_index = 0;
         }
         break;

     case demo_repeated:
         drv.setMode(DRV2605_MODE_INTTRIG);
         effect = effect_list[effect_list_index];

         for (uint8_t i = 0; i < 8; i++)
         {
             drv.setWaveform(i, effect); // wobble effects
         }
         for (uint8_t i = 20; i <= 100; i += 20)
         {
             ESP_LOGI(TAG_DRV, "EffectArray #%d: %s for %dms", effect, drv_effect_string_map[effect], i);
             drv.go();
             delay(i);
             // stop
             drv.writeRegister8(DRV2605_REG_GO, 0);
             delay(1000);
         }

         effect_list_index++;
         if (effect_list_index >= effect_list_size)
         {
             ESP_LOGI(TAG_DRV, "-------");
             // stimMode = demo_rtp_amp;
             // drv.writeRegister8(DRV2605_REG_MODE, DRV2605_MODE_REALTIME);
             //  unsigned

             effect_list_index = 0;
         }

         break;

     case demo_rtp_amp:
         drv.setMode(DRV2605_MODE_REALTIME);
         rtp_amp = rtp[rtp_index++];
         ESP_LOGI(TAG_DRV, "SetRealtimeValue = %d for 100ms", rtp_amp);
         drv.setRealtimeValue(rtp_amp);
         delay(100);
         drv.setRealtimeValue(0x00);
         if (rtp_index >= rtp_size)
         {
             ESP_LOGI(TAG_DRV, "-------");
             // stimMode = demo_rtp_time;
             //  drv.writeRegister8(DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);
             rtp_index = 0;
         }

         break;

     case demo_rtp_time:
         drv.setMode(DRV2605_MODE_REALTIME);
         for (uint8_t i = 10; i <= 100; i += 10)
         {
             ESP_LOGI(TAG_DRV, "SetRealtimeValue = 127 for %dms", i);
             drv.setRealtimeValue(127);
             delay(i);
             drv.setRealtimeValue(0x00);
             delay(1000);
         }
         ESP_LOGI(TAG_DRV, "-------");
         // stimMode = demo_individual_effects;
         // drv.writeRegister8(DRV2605_REG_MODE, DRV2605_MODE_INTTRIG);

         break;
     case demo_shifting:
         drv.setMode(DRV2605_MODE_EXTTRIGLVL);
         effect = DRV_EFF_STRONG_BUZZ_100;
         for (uint8_t i = 0; i < 8; i++)
         {
             drv.setWaveform(i, effect); // wobble effects
         }
         for (uint8_t i = 25; i <= 100; i += 25)
         {
             // play the effect!
             ESP_LOGI(TAG_DRV, "TT Effect #%d: %s for %dms", effect, drv_effect_string_map[effect], i);
             // drv.go();

             digitalWrite(drv_go, HIGH);
             delay(i);
             // stop
             // drv.writeRegister8(DRV2605_REG_GO, 0);
             digitalWrite(drv_go, LOW);
             delay(500);
             digitalWrite(drv_lateral_go, HIGH);
             delay(i);
             // stop
             // drv.writeRegister8(DRV2605_REG_GO, 0);
             digitalWrite(drv_lateral_go, LOW);
             delay(500);
         }

         break;

     default:
         ESP_LOGI(TAG_DRV, "DEMO not found");
         break;
     }

     delay(1000);
 }
 void doDemo(uint8_t *value)
 {
     stimMode = (DRV_demo_t)value[0];
     switch (stimMode)
     {
     case demo_idle:
         break;
     case demo_individual_effects:
         effect = value[1];
         effect_duration = value[2];
         break;
     case demo_effects_loop:
         ESP_LOGI(TAG_BLE, "demo_individual_effects");
         break;
     case demo_repeated:
         ESP_LOGI(TAG_BLE, "demo_repeated");
         break;
     case demo_rtp_amp:
         ESP_LOGI(TAG_BLE, "demo_rtp_amp");
         break;
     case demo_rtp_time:
         ESP_LOGI(TAG_BLE, "demo_rtp_time");
         break;
     case demo_shifting:
         ESP_LOGI(TAG_BLE, "demo_shifting");
         break;
     default:
         ESP_LOGI(TAG_BLE, "Invalid DEMO");
         break;
     }
     */
