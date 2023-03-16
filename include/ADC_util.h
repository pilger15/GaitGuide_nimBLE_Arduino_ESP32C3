#ifndef ADC_UTIL_H
#define ADC_UTIL_H

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/adc.h"

#define TIMES 256

static const float _ADC_READ_MAX = 4095;
static const float _ADC_REF_VOLTAGE = 1100;
static const uint32_t _sample_rate = 620;

static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = 256,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ret = adc_digi_initialize(&adc_dma_config);
    assert(ret == ESP_OK);

    adc_digi_pattern_config_t adc_pattern[10] = {0};

    // Do not set the sampling frequency out of the range between `SOC_ADC_SAMPLE_FREQ_THRES_LOW` and `SOC_ADC_SAMPLE_FREQ_THRES_HIGH`
    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = 0,
        .conv_limit_num = 250,
        .sample_freq_hz = _sample_rate,
    };

    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        uint8_t unit = ((channel[i] >> 3) & 0x1);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ret = adc_digi_controller_configure(&dig_cfg);
    assert(ret == ESP_OK);
}

static bool check_valid_data(const adc_digi_output_data_t *data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2)
        return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
        return false;

    return true;
}

static void continuous_read(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

    uint16_t adc1_chan_mask = BIT(2);                      // BIT(0) | BIT(1);
    uint16_t adc2_chan_mask = 0;                           // BIT(0);
    adc_channel_t channel = (adc_channel_t)ADC1_CHANNEL_2; //[3] = {ADC1_CHANNEL_0, ADC1_CHANNEL_1, (ADC2_CHANNEL_0 | 1 << 3)};

    continuous_adc_init(adc1_chan_mask, adc2_chan_mask, &channel, sizeof(channel) / sizeof(adc_channel_t));
    adc_digi_start();

    int n = 20;
    while (n--)
    {
        ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
        for (int i = 0; i < ret_num; i += 4)
        {
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
            if (check_valid_data(p))
            {
                printf("ADC%d_CH%d: %x\n", p->type2.unit + 1, p->type2.channel, p->type2.data);
            }
            else
            {
                printf("Invalid data [%d_%d_%x]\n", p->type2.unit + 1, p->type2.channel, p->type2.data);
            }
        }
        // If you see task WDT in this task, it means the conversion is too fast for the task to handle
    }

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);
}

static uint16_t single_read()
{
    uint16_t adc1_reading = {0xcc};

    adc1_reading = adc1_get_raw(ADC1_CHANNEL_2);

    return adc1_reading;
}
static void configure_adc(adc_bits_width_t width, adc_atten_t atten)
{
    adc1_config_width(width);
    adc1_config_channel_atten(ADC1_CHANNEL_2, atten);
}
static uint32_t multisample_read(uint8_t num_samples)
{

    uint32_t adc_reading = 0xc00;
    for (int i = 0; i < num_samples; i++)
    {
        adc_reading += adc1_get_raw(ADC1_CHANNEL_2);
    }
    adc_reading /= num_samples;
    return adc_reading;
}

static float convert_to_V(uint16_t adc_raw)
{

    float voltage = (adc_raw / (float)_ADC_READ_MAX) * _ADC_REF_VOLTAGE;

    return voltage;
}

#endif // DRV2605_UTIL_H