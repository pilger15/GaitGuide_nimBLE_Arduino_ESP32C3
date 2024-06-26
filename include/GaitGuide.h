/**
 * @file GaitGuide.h
 *
 * @brief Header file for the GaitGuide singleton class.
 *
 * The GaitGuide class provides a singleton interface for managing the GaitGuide device.
 * This class ensures that there is only one instance of the class that can be accessed
 * by any part of the program, making it a singleton. The GaitGuide class provides methods
 * for accessing and modifying the state of the GaitGuide device.
 *
 * @author [Julian Martus]
 * @date [31.01.23]
 */
#ifndef GAITGUIDE_MANAGER_H
#define GAITGUIDE_MANAGER_H

#include <Arduino.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "nvs_flash.h"
#include "nvs.h"

typedef enum
{
    // GaitGuide User-Modes
    GAITGUIDE_USERMODE_AMPLITUDE, ///< Amplitude mode: control amplitude (default mode)
    GAITGUIDE_USERMODE_EFFECT,    ///< Effect mode: effect chain up to 8 effects
    GAITGUIDE_USERMODE_DEMO00,    ///< User demo 00
    GAITGUIDE_USERMODE_DEMO01,    ///< User demo 01
    GAITGUIDE_USERMODE_DEMO02,    ///< User demo 02
    GAITGUIDE_USERMODE_PRESSURE,  ///< Pressure mode: used for attaching the device with a repeatable pressure settung
    GAITGUIDE_USERMODE_DIAGNOS,   ///< DO NOT USE Diagnostics mode. Due to overlaying the i2C bus diagnostics cann
    GAITGUIDE_USERMODE_AUTOCAL    ///< DO NOT USE Auto calibration mode
} gaitGuide_stimMode_t;

typedef enum
{
    drv_right,
    drv_left
} DRV_pos_t;

class GaitGuide
{
public:
    static GaitGuide &getInstance();

    SFE_MAX1704X *m_lipo;
    void setBatteryGauge(SFE_MAX1704X *lipo)
    {
        m_lipo = lipo;
    }

    bool goRight = false;
    bool goLeft = false;

    uint8_t deviceId() const;
    void deviceId(uint8_t value);

    std::string getFirmwareVersion() const;
    void setFirmwareVersion(std::string newVersion);

    uint8_t batteryLevel();
    void batteryLevel(uint8_t batteryLevel);

    gaitGuide_stimMode_t stimMode() const;
    void stimMode(gaitGuide_stimMode_t value);

    bool timescale();
    void timescale(bool is_timescale_5ms);

    void setAmplitude(const uint8_t data);
    void setDuration(const uint16_t data, bool driver_side);

    uint16_t duration(const uint8_t pos = 0)
    {
        return m_duration[pos];
    }

    uint8_t amp(const uint8_t pos = 0)
    {
        return m_amplitude[pos];
    }

    uint8_t effect(const uint8_t pos = 0)
    {
        return m_effects[pos];
    }

    // callbacks
    typedef std::function<void(bool)> StimulationCallback;

private:
    GaitGuide();
    GaitGuide(const GaitGuide &) = delete;
    GaitGuide &operator=(const GaitGuide &) = delete;
    uint8_t _deviceId;
    std::string _firmwareVersion = "2";

    void initialiseNVS()
    {
        // Initialize NVS
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            err = nvs_flash_init();
        }
        ESP_ERROR_CHECK(err);
    };

    uint8_t m_amplitude[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t m_duration[8] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
    uint8_t m_effects[8] = {0x0010, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // effect 1000ms Alert
    uint8_t m_error = 0;

    uint8_t m_batteryLevel = 0;
    uint8_t m_subjectID = 0;

    bool m_is_timescale_5ms = false;

    uint8_t m_packet_seq = 0; // for

    gaitGuide_stimMode_t _stimMode = gaitGuide_stimMode_t::GAITGUIDE_USERMODE_AMPLITUDE;

    const char *m_nvs_namespace = "gaitguide_NVS";
};

#endif
