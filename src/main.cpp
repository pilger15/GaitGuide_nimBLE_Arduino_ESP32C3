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
#include "esp_system.h"

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

#include "hal/usb_serial_jtag_ll.h"
#include "driver/usb_serial_jtag.h"

#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SDA D4
#define I2C_SCL D5
#define I2C_FREQ 20000UL // 500000UL
#define TRIGGER_MEDIAL D0
#define TRIGGER_LATERAL D1
#define ENABLE_MEDIAL D3
#define ENABLE_LATERAL D2
// TwoWire I2C = TwoWire(0);

#define SPI_CS_LATERAL D7
#define SPI_CS_MEDIAL D6
#define SPI_CLK D8
#define SPI_MISO D10
#define SPI_MOSI D9
#define SPI_FREQUENCY (10 * 1000 * 1000) // 10 Mhz max -> may need to be reduced if so check tCS->CLK

#define ACC_WORD 7
#define ACC_BUFFER_WORDS 512
#define ACC_BUFFER_HEADER_SZ 4
#define ACC_BUFFER_OVERHEAD (ACC_BUFFER_HEADER_SZ)
#define ACC_WATERMARK (ACC_BUFFER_WORDS / 2)
#define ACC_BUFFER_BYTES (ACC_BUFFER_WORDS * ACC_WORD)

#define ACC_SCALE ACCEL_2G
#define ACC_AXIS Z_AXIS_ONLY

uint8_t *acc_medial_buffer; //[ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD];
iis3dwb_device_t acc_medial = {
    .IDx = 0,
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS};

uint8_t *acc_lateral_buffer; //[ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD];
iis3dwb_device_t acc_lateral{
    .IDx = 1,
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS};

spi_transaction_t spi_read_fifo = {
    //.flags = SPI_DEVICE_HALFDUPLEX,
    .cmd = SPI_READ_CMD,
    .addr = IIS3DWB_FIFO_DATA_OUT_TAG};

#define USB_NUM_BUFFER 2
#define USB_BUFFER_SCALE 4
#define USB_BUFFER_RX 128
#define USB_BUFFER_TX ((ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD) * USB_BUFFER_SCALE)
// uint8_t usb_buffer_tx[USB_NUM_BUFFER][USB_BUFFER_TX];
uint8_t usb_buffer_rx[USB_BUFFER_RX];

DRV2605_UTIL drv;
bool is_timescale_1ms = false;

unsigned long time_us = 0;

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

esp_timer_handle_t stimulation_timer;
esp_timer_handle_t acceleration_timer;

bool streaming = false;
auto &gaitGuide = GaitGuide::getInstance();
gaitGuide_stimMode_t stimMode = gaitGuide.stimMode();

void timer_setup();
void ble_setup();
void stimulation();
void findPressure();
void accelerometer_setup(void);
void accelerometer_config(iis3dwb_device_t *device, gpio_num_t cs_pin);
void get_acc_fifo();
void usb_init();
// void led_setup();
uint16_t cnt = 0;
static void acceleration_timer_callback(void *arg)
{
    // get_acc_fifo();
    if (streaming)
    {
        digitalWrite(D0, HIGH);
    }
    else
    {
        digitalWrite(D0, LOW);
    }
    streaming = !streaming;
    gaitGuide.newEvent(GAITGUIDE_EVENT_ACC);
}
static void stimulation_timer_callback(void *arg)
{
    drv.setRealtimeValue(0x00);
    drv.disable();
    gaitGuide.goLateral = false;
    gaitGuide.goMedial = false;
    log_d("\n--------------STOPPED-------------------\n\n");
}

int is_plugged_usb(void)
{
    uint32_t *aa = (uint32_t *)USB_SERIAL_JTAG_FRAM_NUM_REG;
    uint32_t first = *aa;
    vTaskDelay(pdMS_TO_TICKS(10));
    return (int)(*aa - first);
}

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
    usb_init();

    ble_setup(deviceName);
    accelerometer_setup();
    timer_setup();

    pinMode(D0, OUTPUT);

    log_d("\n ------->%d", sizeof(iis3dwb_OUT_WORD_t));
    // led_setup();
    // configure_adc(ADC_WIDTH_BIT_12, ADC_ATTEN_DB_11);
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
        //   log_d("Entering LFC_State");
        // Handle Looking-for-connection state
        // #TODO: implement breathing and advertising as well as sleeping
        gaitGuide.newEvent(GAITGUIDE_EVENT_BT_CONNECT);
        break;

    case GAITGUIDE_STATE_LOW_POWER:
        //   log_d("Entering Low Power State");
        // Handle Low-power state
        // #TODO: implement breathing and advertising as well as sleeping
        break;

    case GAITGUIDE_STATE_IDLE:
        if (usb_serial_jtag_read_bytes(usb_buffer_rx, USB_BUFFER_RX, 1 / portTICK_PERIOD_MS))
        {
            if (!streaming)
            {
                iis3dwb_fifo_mode_set(&acc_lateral, IIS3DWB_STREAM_MODE);
                iis3dwb_fifo_mode_set(&acc_medial, IIS3DWB_STREAM_MODE);
                ESP_ERROR_CHECK(esp_timer_start_periodic(acceleration_timer, 10 * 1000)); // time in us
                streaming = true;
            }
            else
            {
                ESP_ERROR_CHECK(esp_timer_stop(acceleration_timer));
                streaming = false;
            }
        }
        // log_d("Entering IDLE_State");
        // Handle Idle state
        // time for an event to occur, in case an event occurs dont do idle  tasks
        // #TODO implementiere entsprechenden timer

        // #TODO update Battery

        break;

    case GAITGUIDE_STATE_STIM:
        // log_d("Entering STIM_State");
        // Handle Stimulation state
        stimulation();
        gaitGuide.newEvent(GAITGUIDE_EVENT_DONE);
        break;

    case GAITGUIDE_STATE_ACC:
        // toggles acc measurement
        get_acc_fifo();
        gaitGuide.newEvent(GAITGUIDE_EVENT_DONE);
        break;

    case GAITGUIDE_STATE_STOP_ACC:
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
    gaitGuide.nextEvent();
}

void findPressure()
{
    // #TODO SET up pressure finding mode
    ESP_LOGI("+++", "USERPRESSURE");

    uint16_t voltage = 0;
    uint16_t led_dutycycle = 0;

    // this event indicates pressure mode is entered for the first time
    if (gaitGuide.currentEvent() == GAITGUIDE_EVENT_ACC)
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
void stimulation()
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
        ESP_ERROR_CHECK(esp_timer_start_once(stimulation_timer, gaitGuide.duration() * 1000)); // time in us
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

void accelerometer_config(iis3dwb_device_t *device, gpio_num_t cs_pin)
{

    // SPI device configuration

    spi_device_interface_config_t dev_cfg = {
        .command_bits = 1, ///< Default amount of bits in command phase (0-16), used when ``SPI_TRANS_VARIABLE_CMD`` is not used, otherwise ignored.
        .address_bits = 7, ///< Default amount of bits in address phase (0-64), used when ``SPI_TRANS_VARIABLE_ADDR`` is not used, otherwise ignored.
        .dummy_bits = 0,   ///< Amount of dummy bits to insert between address and data phase
        .mode = 0,
        .cs_ena_pretrans = 0, //
        .cs_ena_posttrans = 0,
        .clock_speed_hz = SPI_FREQUENCY,
        .spics_io_num = cs_pin,
        // .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
    };
    device->spi_config = dev_cfg;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &device->spi_config, &device->spi_handle));
    ESP_ERROR_CHECK(spi_device_acquire_bus(device->spi_handle, portMAX_DELAY));
    // set to sleep

    iis3dwb_sleep(device);

    delay(IIS3DWB_SHUTDOWN_TIME);
    // iis3dwb_ODR_on_int1(cs_pin); #TODO
    iis3dwb_reset_set(device);
    /* Set XL Batch Data Rate */
    iis3dwb_fifo_xl_batch_set(device, IIS3DWB_XL_BATCHED_AT_26k7Hz);

    /*  Set Temperature Batch Data Rate */
    iis3dwb_fifo_temp_batch_set(device, IIS3DWB_TEMP_NOT_BATCHED);

    /* Set  FIFO Bypass Mode */
    iis3dwb_fifo_mode_set(device, IIS3DWB_BYPASS_MODE);

    /* Set  FIFO Watermark */
    iis3dwb_fifo_watermark_set(device, ACC_WATERMARK); // Total Bytes =  Number of words * FIFO_WORD (7 or 6) + 1 to account for occasional temperature datum

    /* Set default acceleration scale */
    iis3dwb_xl_full_scale_set(device, ACCEL_2G);

    /*Significantly improves noise density if only one axis is chosen*/
    iis3dwb_axis_sel_set(device, Z_AXIS_ONLY);

    /*Register address automatically incremented*/
    iis3dwb_auto_increment_set(device, 1);

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
    }
    else
    {
        log_i("Device %d was found", device->IDx);
    }
    spi_device_release_bus(device->spi_handle);
}

void accelerometer_setup(void)
{

    // setup buffers
    acc_medial_buffer = (uint8_t *)heap_caps_malloc(ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD, MALLOC_CAP_DMA);
    acc_medial.data_buffer = &acc_medial_buffer[ACC_BUFFER_HEADER_SZ];
    acc_medial_buffer[0] = 'D';
    acc_medial_buffer[1] = 'A';
    acc_medial_buffer[2] = 'T';
    acc_medial_buffer[3] = acc_medial.IDx;

    // setup buffers
    acc_lateral_buffer = (uint8_t *)heap_caps_malloc(ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD, MALLOC_CAP_DMA);
    acc_lateral.data_buffer = &acc_lateral_buffer[ACC_BUFFER_HEADER_SZ];

    acc_lateral_buffer[0] = 'D';
    acc_lateral_buffer[1] = 'A';
    acc_lateral_buffer[2] = 'T';
    acc_lateral_buffer[3] = acc_lateral.IDx;

    // SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ACC_BUFFER_BYTES,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    log_i("init ACCs");
    accelerometer_config(&acc_medial, (gpio_num_t)SPI_CS_MEDIAL);
    accelerometer_config(&acc_lateral, (gpio_num_t)SPI_CS_LATERAL);
    delay(IIS3DWB_BOOT_TIME);
}

iis3dwb_OUT_WORD_t WORDS[ACC_BUFFER_WORDS + ACC_BUFFER_OVERHEAD];
typedef union
{
    uint8_t uint8[ACC_BUFFER_WORDS * sizeof(uint16_t)];
    uint16_t uint16[ACC_BUFFER_WORDS];
} sendUSB_t;
sendUSB_t sendUSB;

void get_acc_fifo()
{
    log_i("\n -->tick #%d -- time %dus", cnt++, time_us);
    // # TODO consolidate weird buffer arrangement
    char terminator = '\n';
    char header[] = {
        'D',
        'A',
        'T'};
    int send0 = 0, send1 = 0;

    iis3dwb_fifo_status_t status0, status1;
    uint16_t num_words0 = 0,
             num_words1 = 0;

    // for future reference - queue tansacion and read out last queue  to allow simulaion request in the middle
    if (false) // queue transmission on first request
    {
        /*
        status0 = iis3dwb_fifo_status_get(&acc_medial);
        num_words0 = (uint16_t)(status0.fifo_status.fifo_status2.status.fifo_status2.diff_fifo << 8) | status0.fifo_status.fifo_status1.data;
        spi_read_fifo.length = num_words0 * ACC_WORD * 8;
        spi_read_fifo.rxlength = num_words0 * ACC_WORD * 8;
        spi_device_queue_trans(acc_medial.spi_handle, &spi_read_fifo, portMAX_DELAY);
        if (status0.fifo_status.fifo_status2.status.fifo_status2.fifo_ovr_ia)
        {
            log_e("\e[1;31mFIFO medial OVER-RUN\e[0;37m");
        }
        status0 = iis3dwb_fifo_status_get(&acc_medial);
        num_words0 = (uint16_t)(status0.fifo_status.fifo_status2.status.fifo_status2.diff_fifo << 8) | status0.fifo_status.fifo_status1.data;
        spi_device_queue_trans(acc_lateral.spi_handle, &spi_read_fifo, portMAX_DELAY);
        if (status0.fifo_status.fifo_status2.status.fifo_status2.fifo_ovr_ia)
        {
            log_e("\e[1;31mFIFO lateral OVER-RUN\e[0;37m");
        }
        */
    }
    else // read queued transmissions and queue next
    {
        if (iis3dwb_who_am_i(&acc_medial) == IIS3DWB_WHO_AM_I_EXPECTED)
        {
            ESP_ERROR_CHECK(spi_device_acquire_bus(acc_medial.spi_handle, portMAX_DELAY));
            // status0 = iis3dwb_fifo_status_get(&acc_medial);
            num_words0 = iis3dwb_fifo_data_level_get(&acc_medial);
            ; //(uint16_t)(status0.fifo_status.status2.status2.diff_fifo << 8) | status0.fifo_status.status1.fifo_status1.diff_fifo;
            iis3dwb_fifo_batch_get(&acc_medial, num_words0);
            spi_device_release_bus(acc_medial.spi_handle);
        }
        else
        {
            log_e("Device %d was not found", acc_medial.IDx);
        }

        if (iis3dwb_who_am_i(&acc_lateral) == IIS3DWB_WHO_AM_I_EXPECTED)
        {
            ESP_ERROR_CHECK(spi_device_acquire_bus(acc_lateral.spi_handle, portMAX_DELAY));
            num_words1 = iis3dwb_fifo_data_level_get(&acc_lateral);
            // status1 = iis3dwb_fifo_status_get(&acc_lateral);
            // num_words1 = (uint16_t)(status1.fifo_status.status2.status2.diff_fifo << 8) | status1.data;
            iis3dwb_fifo_batch_get(&acc_lateral, num_words1);
            spi_device_release_bus(acc_lateral.spi_handle);
        }
        else
        {
            log_e("Device %d was not found", acc_lateral.IDx);
        }

        if (num_words0)
        {
            // memcpy(&sendUSB, acc_medial_buffer, ACC_BUFFER_OVERHEAD);
            memcpy(WORDS, acc_medial.data_buffer, num_words0);
            for (int i = 0; i < num_words0; i++)
            {
                sendUSB.uint16[i] = WORDS[i].OUT_WORD.OUT_A.OUT_A.OUT_A_Z;
            }
            usb_serial_jtag_write_bytes(&header, 3, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&acc_medial.IDx, 1, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&num_words0, 2, pdMS_TO_TICKS(1));

            send0 = usb_serial_jtag_write_bytes(&sendUSB, num_words0 * sizeof(uint16_t), pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&terminator, 1, pdMS_TO_TICKS(1));
        }
        if (num_words1)
        {
            memcpy(WORDS, acc_lateral.data_buffer, num_words1);
            for (int i = 0; i < num_words1; i++)
            {
                sendUSB.uint16[i] = WORDS[i].OUT_WORD.OUT_A.OUT_A.OUT_A_Z;
            }
            usb_serial_jtag_write_bytes(&header, 3, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&acc_lateral.IDx, 1, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&num_words1, 2, pdMS_TO_TICKS(1));

            send1 = usb_serial_jtag_write_bytes(&sendUSB, num_words1 * sizeof(uint16_t), pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&terminator, 1, pdMS_TO_TICKS(1));
        }
        /*log_d("\n\e[1;35mtime for Fifo %d us\n[DEVICE0] %d words retrived & %d bytes send\n[DEVICE1] %d words retrived & %d bytes send\e[0;37m\n ",
              time_us,
              num_words0, send0,
              num_words1, send1);*/
    }
}
void timer_setup()
{
    const esp_timer_create_args_t acceleration_timer_args = {
        .callback = &acceleration_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "acceleration_timer"};

    const esp_timer_create_args_t stimulation_timer_args = {
        .callback = &stimulation_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "stimulation_timer"};
    ESP_ERROR_CHECK(esp_timer_create(&acceleration_timer_args, &acceleration_timer));
    ESP_ERROR_CHECK(esp_timer_create(&stimulation_timer_args, &stimulation_timer));

    // #debug
}

void usb_init()
{
    /* Configure USB-CDC */
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = USB_BUFFER_TX,
        .rx_buffer_size = USB_BUFFER_RX};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));

    /* Configure a bridge buffer for the incoming data */
    memset(usb_buffer_rx, 0x00, sizeof(usb_buffer_rx) * sizeof(uint8_t));
    // memset(usb_buffer_tx, 0x00, sizeof(usb_buffer_tx) * sizeof(uint8_t));
    ESP_LOGI("USB", "USB initialised");
    /*while (true)
    {
        if (send)
        {
            usb_serial_jtag_write_bytes(&buffer_espnow_rx[0][send], ESP_NOW_MAX_DATA_LEN, 1 / portTICK_PERIOD_MS);
            send--;
        }
        else if (usb_serial_jtag_read_bytes(buffer_usb_rx, USB_MESSAGE_LEN, 1 / portTICK_PERIOD_MS))
        {
            // len = usb_serial_jtag_ll_read_rxfifo(buffer1, ADC_BUFFER_LEN * sizeof(uint8_t));
            REG_WRITE(GPIO_OUT_W1TS_REG, BIT2); // LOW
            msg = buffer_usb_rx[0];
            esp_err_t ret = esp_now_send(peer->peer_addr, buffer_usb_rx, 1);
            if (ret != ESP_OK)
            {
                ESP_LOGE("ESP-NOW", "Error sending data: %s", esp_err_to_name(ret));
            }
            REG_WRITE(GPIO_OUT_W1TC_REG, BIT2); // LOW
        }
        else
        {
            vTaskDelay(1);
        }

} */
}