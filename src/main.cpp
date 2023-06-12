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
- implement different SPI transactions
- make DMA transactions for buffer
- implement USB-Jtag
-  stream Fifo data
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

#include <IIS3DWB.h>

#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SDA D5
#define I2C_SCL D6
#define I2C_FREQ 20000UL // 500000UL
#define TRIGGER_MEDIAL D3
#define TRIGGER_LATERAL D4
#define ENABLE_MEDIAL D2
#define ENABLE_LATERAL D1
// TwoWire I2C = TwoWire(0);

#define SPI_CS_LATERAL GPIO_NUM_5
#define SPI_CS_MEDIAL GPIO_NUM_6
#define SPI_CLK GPIO_NUM_8
#define SPI_MISO GPIO_NUM_9
#define SPI_MOSI GPIO_NUM_6
#define SPI_FREQUENCY 10000000 // 10 Mhz max -> may need to be reduced if so check tCS->CLK

#define ACC_WORD 7
#define BUFFER_WORDS 512
#define WATERMARK (BUFFER_WORDS / 2)
#define BUFFER_BYTES (BUFFER_WORDS * ACC_WORD)

#define ACC_SCALE ACCEL_2G
#define ACC_AXIS Z_AXIS_ONLY

uint8_t medial_acc_buffer[BUFFER_BYTES];
iis3dwb_device_t acc_medial = {
    .IDx = 0,
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS,
    .data_buffer = medial_acc_buffer};

uint8_t lateral_acc_buffer[BUFFER_BYTES];
iis3dwb_device_t acc_lateral{
    .IDx = 1,
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS,
    .data_buffer = lateral_acc_buffer};

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

void spi_init(void);
void accelerometer_init(iis3dwb_device_t *device, gpio_num_t cs_pin);
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

void accelerometer_init(iis3dwb_device_t *device, gpio_num_t cs_pin)
{
    // SPI device configuration
    spi_device_interface_config_t dev_cfg_lateral = {
        .command_bits = 1, ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
        .address_bits = 7, ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
        .dummy_bits = 0,   ///< Amount of dummy bits to insert between address and data phase
        .mode = 0,
        .cs_ena_pretrans = 1, //
        .cs_ena_posttrans = 1,
        .clock_speed_hz = SPI_FREQUENCY,
        .spics_io_num = cs_pin,
        // .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device->spi_config, &device->spi_handle));

    ESP_ERROR_CHECK(spi_device_acquire_bus(device->spi_handle, portMAX_DELAY));

    // set to sleep
    iis3dwb_sleep(device);
    // iis3dwb_ODR_on_int1(cs_pin); #TODO
    iis3dwb_reset_set(device);
    /* Set XL Batch Data Rate */
    iis3dwb_fifo_xl_batch_set(device, IIS3DWB_XL_BATCHED_AT_26k7Hz);

    /*  Set Temperature Batch Data Rate */
    iis3dwb_fifo_temp_batch_set(device, IIS3DWB_TEMP_NOT_BATCHED);

    /* Set  FIFO Bypass Mode */
    iis3dwb_fifo_mode_set(device, IIS3DWB_BYPASS_MODE);

    /* Set  FIFO Watermark */
    iis3dwb_fifo_watermark_set(device, WATERMARK); // Total Bytes =  Number of words * FIFO_WORD (7 or 6) + 1 to account for occasional temperature datum

    /* Set default acceleration scale */
    iis3dwb_xl_full_scale_set(device, ACCEL_2G);

    iis3dwb_axis_sel_set(device, Z_AXIS_ONLY);

    /*	Configure filtering chain(No aux interface)
     *	Accelerometer - LPF1 + LPF2 path
     */
    iis3dwb_xl_hp_path_on_out_set(device, IIS3DWB_LP_6k3Hz);
    // iis3dwb_xl_filter_lp2_set(handle, PROPERTY_ENABLE);
    iis3dwb_wake(device);

    /* Wait stable output */
    if (iis3dwb_who_am_i(device) != IIS3DWB_WHO_AM_I_EXPECTED)
    {
        log_e("Device %d was not found", device->IDx);
    };

    spi_device_release_bus(device->spi_handle);
}

void spi_init(void)
{

    // gpio_set_direction(GPIO_NUM_10, GPIO_MODE_OUTPUT); // conversion

    // SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    iis3dwb_init(&acc_medial, SPI_CS_MEDIAL);
    iis3dwb_init(&acc_lateral, SPI_CS_LATERAL);
    delay(BOOT_TIME);
}