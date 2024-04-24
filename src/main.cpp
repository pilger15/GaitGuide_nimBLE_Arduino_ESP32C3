/** Gait guide
 *
 * DRV2605-Driver interaction
 * Currently all drivers use the same I2C bus (only one I2 peripheral available).
 * Since there are multiple drivers present for each side (Right, Left, two each)
 * and all devices have the same address, the bus can not be used  for reading actions.
 *
 *
 *
 *
 *  Created: 2023
 *      Author: JMartus
 */

/*

*/
#include <Arduino.h>
#include "esp_system.h"

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "Adafruit_DRV2605.h"

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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// USE_TESTBENCH

// #define USE_TESTBENCH // comment if using the testbench setup

#ifdef USE_TESTBENCH
// TESTBENCH SETUP
#define LED_RED D6
#define I2C_SDA D2       // D4
#define I2C_SCL D3       // D5
#define I2C_FREQ 20000UL // 500000UL
#define TRIGGER_Right D1 // D0
#define TRIGGER_Left D1  // D1
#define ENABLE_Right D5  // D3
#define ENABLE_Left D5   // D2

#define SPI_CS_Left D10 // D7
#define SPI_CS_Right D4 // LED_RED
#define SPI_CLK D9      // D8
#define SPI_MISO D7     // D10
#define SPI_MOSI D8     // D9

#else
// REGULAR GAITGUIDE
#define LED_RED D1
#define I2C_SDA D4
#define I2C_SCL D5
#define I2C_FREQ 20000UL // 500000UL

#define ENABLE_Right D3
#define ENABLE_Left D2

#define SPI_CS_Left D7
#define SPI_CS_Right D6
#define SPI_CLK D8
#define SPI_MISO D10
#define SPI_MOSI D9

#endif

SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048

#define SPI_FREQUENCY (20 * 1000 * 1000) // may need to be reduced if so check tCS->CLK

#define ACC_WORD 7
#define ACC_BUFFER_WORDS 512
#define ACC_BUFFER_HEADER_SZ 6
#define ACC_BUFFER_OVERHEAD (ACC_BUFFER_HEADER_SZ)
#define ACC_WATERMARK (ACC_BUFFER_WORDS / 2)
#define ACC_BUFFER_BYTES (ACC_BUFFER_WORDS * ACC_WORD)

#define ACC_SCALE ACCEL_16G
#define ACC_AXIS Z_AXIS_ONLY

uint8_t *acc_Right_buffer; //[ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD];
iis3dwb_device_t acc_Right = {
    .IDx = (uint8_t)'0',
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS,
    .data_buffer = acc_Right_buffer};

uint8_t *acc_Left_buffer; //[ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD];
iis3dwb_device_t acc_Left{
    .IDx = (uint8_t)'1',
    .full_scale = ACC_SCALE,
    .xl_axis = ACC_AXIS,
    .data_buffer = acc_Left_buffer};

#define USB_NUM_BUFFER 2
#define USB_BUFFER_SCALE 4
#define USB_BUFFER_RX 128
#define USB_BUFFER_TX ((ACC_BUFFER_BYTES + ACC_BUFFER_OVERHEAD) * USB_BUFFER_SCALE)
// uint8_t usb_buffer_tx[USB_NUM_BUFFER][USB_BUFFER_TX];
uint8_t usb_buffer_rx[USB_BUFFER_RX];

bool is_timescale_1ms = false;

volatile unsigned long recording_timestamp_us = 0;

static const std::string deviceName = "GaitGuide";

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

bool is_recording = false;
auto &drv = DRV2605_UTIL::getInstance();
auto &gaitGuide = GaitGuide::getInstance();
gaitGuide_stimMode_t stimMode = gaitGuide.stimMode();

void ble_setup();
void accelerometer_setup(void);
void accelerometer_config(iis3dwb_device_t *device, gpio_num_t cs_pin);
void usb_init();
void led_setup();
uint16_t cnt = 0;

static iis3dwb_OUT_WORD_t WORDS[ACC_BUFFER_WORDS];
typedef union
{
    uint8_t uint8[ACC_BUFFER_WORDS * sizeof(uint16_t)];
    int16_t int16[ACC_BUFFER_WORDS];
} sendUSB_t;
sendUSB_t sendUSB;

void IRAM_ATTR get_acc_fifo_task(void *pvParameters)
{
    // # TODO consolidate weird buffer arrangement
    char terminator[] = {'\n'};
    char header[] = {
        'D',
        'A',
        'T'};
    char timestamp_header[8] = {'A', 'C', 'C', '_', 'T', 'I', 'M', 'E'};

    int send0 = 0, send1 = 0;

    spi_transaction_t *trans_result;

    spi_transaction_t spi_read_fifo1 = {
        //.flags = SPI_DEVICE_HALFDUPLEX,
        .cmd = SPI_READ_CMD,
        .addr = IIS3DWB_FIFO_DATA_OUT_TAG,
        .rx_buffer = acc_Right_buffer};

    spi_transaction_t spi_read_fifo2 = {
        //.flags = SPI_DEVICE_HALFDUPLEX,
        .cmd = SPI_READ_CMD,
        .addr = IIS3DWB_FIFO_DATA_OUT_TAG,
        .rx_buffer = acc_Left_buffer};

    iis3dwb_fifo_status_t status0, status1;
    uint16_t num_words0 = 0,
             num_words1 = 0;
    recording_timestamp_us = micros();
    iis3dwb_fifo_mode_set(&acc_Left, IIS3DWB_STREAM_MODE);
    iis3dwb_fifo_mode_set(&acc_Right, IIS3DWB_STREAM_MODE);
    vTaskDelay(pdMS_TO_TICKS(6));
    // for future reference - queue transacion and read out last queue  to allow stimulation request in the middle

    while (is_recording)
    {

        if (iis3dwb_who_am_i(&acc_Right) == IIS3DWB_WHO_AM_I_EXPECTED)
        {
            // ESP_ERROR_CHECK(spi_device_acquire_bus(acc_Right.spi_handle, portMAX_DELAY));
            status0 = iis3dwb_fifo_status_get(&acc_Right);
            num_words0 = (uint16_t)status0.data & 0x03FF;

            if (status0.fifo_status.status2.status2.fifo_ovr_ia)
            {
                log_e("\n\e[1;31mFIFO Right OVERRUN\e[0;38m\n");
                digitalWrite(LED_RED, HIGH);
            }
            //(uint16_t)(status0.fifo_status.status2.status2.diff_fifo << 8) | status0.fifo_status.status1.fifo_status1.diff_fifo;
            // iis3dwb_fifo_batch_get(&acc_Right, num_words0);
            // spi_device_release_bus(acc_Right.spi_handle);
        }
        else
        {
            log_e("Device %d was not found", acc_Right.IDx);
            digitalWrite(LED_RED, HIGH);
        }

        if (iis3dwb_who_am_i(&acc_Left) == IIS3DWB_WHO_AM_I_EXPECTED)
        {
            status1 = iis3dwb_fifo_status_get(&acc_Left);
            num_words1 = (uint16_t)status1.data & 0x03FF;

            if (status1.fifo_status.status2.status2.fifo_ovr_ia)
            {
                log_e("\n\e[1;31mFIFO Left OVERRUN\e[0;38m\n");
                digitalWrite(LED_RED, HIGH);
            }
        }
        else
        {
            log_e("Device %d was not found", acc_Left.IDx);
            digitalWrite(LED_RED, HIGH);
        }

        if (num_words0)
        {
            spi_read_fifo1.length = num_words0 * 8 * ACC_WORD;
            spi_read_fifo1.rxlength = num_words0 * 8 * ACC_WORD;
            ESP_ERROR_CHECK(spi_device_queue_trans(acc_Right.spi_handle, &spi_read_fifo1, pdMS_TO_TICKS(15)));
        }
        if (num_words1)
        {
            spi_read_fifo2.length = num_words1 * 8 * ACC_WORD;
            spi_read_fifo2.rxlength = num_words1 * 8 * ACC_WORD;
            ESP_ERROR_CHECK(spi_device_queue_trans(acc_Left.spi_handle, &spi_read_fifo2, pdMS_TO_TICKS(15)));
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        if (num_words0)
        {

            spi_device_get_trans_result(acc_Right.spi_handle, &trans_result, pdMS_TO_TICKS(2));
            //  memcpy(&sendUSB, acc_Right_buffer, ACC_BUFFER_OVERHEAD);log_e("before");
            memcpy(WORDS, acc_Right_buffer, num_words0 * ACC_WORD);
            for (int i = 0; i < num_words0; i++)
            {
                sendUSB.int16[i] = WORDS[i].OUT_WORD.OUT_A.OUT_A.OUT_A_Z;
            }

            usb_serial_jtag_write_bytes(&header, 3, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&acc_Right.IDx, 1, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&num_words0, 2, pdMS_TO_TICKS(1));

            send0 = usb_serial_jtag_write_bytes(&sendUSB, num_words0 * sizeof(uint16_t), pdMS_TO_TICKS(1));
        }
        if (num_words1)
        {

            spi_device_get_trans_result(acc_Left.spi_handle, &trans_result, pdMS_TO_TICKS(2));
            memcpy(WORDS, acc_Left_buffer, num_words1 * ACC_WORD);
            for (int i = 0; i < num_words1; i++)
            {
                sendUSB.int16[i] = WORDS[i].OUT_WORD.OUT_A.OUT_A.OUT_A_Z;
            }
            usb_serial_jtag_write_bytes(&header, 3, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&acc_Left.IDx, 1, pdMS_TO_TICKS(1));
            usb_serial_jtag_write_bytes(&num_words1, 2, pdMS_TO_TICKS(1));

            send1 = usb_serial_jtag_write_bytes(&sendUSB, num_words1 * sizeof(uint16_t), pdMS_TO_TICKS(1));
        }

        taskYIELD();
    }
    iis3dwb_fifo_mode_set(&acc_Left, IIS3DWB_BYPASS_MODE);
    iis3dwb_fifo_mode_set(&acc_Right, IIS3DWB_BYPASS_MODE);
    vTaskDelete(NULL);
}

int is_plugged_usb(void)
{
    uint32_t *serialFrame = (uint32_t *)USB_SERIAL_JTAG_FRAM_NUM_REG;
    uint32_t firstSerFrame = *serialFrame;
    vTaskDelay(pdMS_TO_TICKS(10));
    return (int)(*serialFrame - firstSerFrame);
}
void usbTask(void *pvParameters)
{
    while (1)
    {
        if (usb_serial_jtag_read_bytes(usb_buffer_rx, USB_BUFFER_RX, 1 / portTICK_PERIOD_MS))
        {

            if (!is_recording)
            {

                is_recording = true;
                xTaskCreate(get_acc_fifo_task, "ACC TASK", 2048 * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
                digitalWrite(LED_RED, LOW);
            }
            else
            {
                is_recording = false;
            }
        }
        taskYIELD();
    }
    vTaskDelete(NULL);
}
void stimTask(void *pvParameters)
{
    unsigned long stim_timestamp_us = 0;
    char stim_time_header[8] = {'S', 'T', 'I', 'M', 'T', 'I', 'M', 'E'};
    // #TODO use MUTEX
    while (1)
    {
        if (gaitGuide.goLeft || gaitGuide.goRight)
        {
            if (is_recording)
            {
                stim_timestamp_us = micros() - recording_timestamp_us;
#ifdef USE_TESTBENCH // measure-JIG
                pinMode(TRIGGER_Right, OUTPUT);
                digitalWrite(TRIGGER_Right, HIGH);
#endif // USE_TESTBENCH
            }

            drv.stimulate(gaitGuide.goLeft, gaitGuide.goRight);
            vTaskDelay(pdTICKS_TO_MS(gaitGuide.duration() + 10)); // wait for stimulation to end + a few extra milliseconds

            if (is_recording)
            {
#ifdef USE_TESTBENCH // measure-JIG
                pinMode(TRIGGER_Right, OUTPUT);
                digitalWrite(TRIGGER_Right, LOW);
#endif // USE_TESTBENCH
                usb_serial_jtag_write_bytes(stim_time_header, sizeof(stim_time_header), pdMS_TO_TICKS(1));
                usb_serial_jtag_write_bytes(&stim_timestamp_us, sizeof(long), pdMS_TO_TICKS(1));
            }
        }

        taskYIELD();
    }
    vTaskDelete(NULL);
}
void setup()
{
    log_i("Initialising GaitGuide");
    Wire.setPins(I2C_SDA, I2C_SCL);
    delay(1);

    // Init LRA-Drivers
    // drv.setTrigger(TRIGGER_Right, TRIGGER_Left);
    drv.setEnablePins(ENABLE_Right, ENABLE_Left);
    drv.begin(&Wire);
    drv.enable();
    drv.init();

    drv.disable();

    // Init lipo battery gauge
    lipo.begin();
    if (!lipo.begin(Wire)) // Connect to the MAX17048
    {
        log_w("Battery gauge not detected.");
    }
    else
    {
        lipo.quickStart();
        gaitGuide.setBatteryGauge(&lipo);
    }

    usb_init();

    ble_setup(deviceName);
    led_setup();

    accelerometer_setup();

    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, LOW);

#ifdef USE_TESTBENCH // measure-JIG
    pinMode(TRIGGER_Right, OUTPUT);
    digitalWrite(TRIGGER_Right, LOW);
#endif // USE_TESTBENCH

    led_breath();
    xTaskCreate(usbTask, "USB_TASK", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(
        stimTask,
        "TaskRunnerTask",
        configMINIMAL_STACK_SIZE * 4,
        NULL, // Pass the 'this' pointer to the task as the task parameter
        tskIDLE_PRIORITY + 1,
        NULL);
    log_i("Initialisation Complete");
}

void loop()
{
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
        .queue_size = 2,
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
    iis3dwb_xl_full_scale_set(device, device->full_scale);

    /*Significantly improves noise density if only one axis is chosen*/
    iis3dwb_axis_sel_set(device, device->xl_axis);

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
        // digitalWrite(D1, HIGH);
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
    acc_Right_buffer = (uint8_t *)heap_caps_malloc(ACC_BUFFER_BYTES, MALLOC_CAP_DMA);

    // setup buffers
    acc_Left_buffer = (uint8_t *)heap_caps_malloc(ACC_BUFFER_BYTES, MALLOC_CAP_DMA);

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
    accelerometer_config(&acc_Right, (gpio_num_t)SPI_CS_Right);
    accelerometer_config(&acc_Left, (gpio_num_t)SPI_CS_Left);
    delay(IIS3DWB_BOOT_TIME);
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