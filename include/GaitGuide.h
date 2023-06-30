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
#include <DRV2605_util.h>
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
    GAITGUIDE_STATE_STARTUP,       ///< Start-up state
    GAITGUIDE_STATE_LFC,           ///< Looking-for-connection state
    GAITGUIDE_STATE_LOW_POWER,     ///< Low-power state
    GAITGUIDE_STATE_IDLE,          ///< Idle state
    GAITGUIDE_STATE_STIM,          ///< Stimulation state
    GAITGUIDE_STATE_ACC,           ///< Pressure-finding state
    GAITGUIDE_STATE_STOP_ACC,      ///< Pressure-setting state
    GAITGUIDE_STATE_CHECK_PRESSURE ///< Check if currentPressure is within bounds of targetPressure
} gaitGuide_state_t;

typedef enum
{
    GAITGUIDE_EVENT_NONE,  ///< Default event: No Event
    GAITGUIDE_EVENT_RESET, ///< Reset event: sets device into Start-up state

    GAITGUIDE_EVENT_DONE,          ///< Done event: signals the completion of a task
    GAITGUIDE_EVENT_SLEEP,         ///< Sets the device into low power mode
    GAITGUIDE_EVENT_WAKEUP,        ///< Wake the device up from low power mode
    GAITGUIDE_EVENT_BT_CONNECT,    ///< Bluetooth connection event
    GAITGUIDE_EVENT_BT_DISCONNECT, ///< Bluetooth disconnection event
    GAITGUIDE_EVENT_ACC,           ///< Start measuring acceleration
    GAITGUIDE_EVENT_STOP_ACC,      ///< Stop measuring acceleration
    GAITGUIDE_EVENT_STIM,          ///< Stimulation event
    GAITGUIDE_EVENT_STOP_STIM      ///< stop Stimulation event
} gaitGuide_event_t;

typedef enum
{
    drv_medial,
    drv_lateral
} DRV_pos_t;

class GaitGuide
{
public:
    static GaitGuide &getInstance();

    gaitGuide_state_t currentState();
    gaitGuide_event_t currentEvent();
    uint8_t newEvent(gaitGuide_event_t event);
    uint8_t nextEvent();

    bool goMedial = false;
    bool goLateral = false;

    uint8_t deviceId() const;
    void deviceId(uint8_t value);

    std::string getFirmwareVersion() const;
    void setFirmwareVersion(std::string newVersion);

    uint8_t batteryLevel() const;
    void batteryLevel(uint8_t batteryLevel);

    uint8_t subjectId() const;
    void subjectId(uint8_t subjectId);

    uint16_t currentPressureLevel() const;
    void set_currentPressureLevel(uint16_t currentPrepressureLevelssureLevel);

    void setTargetPressure();
    void setTargetPressure(uint8_t subjectId);
    uint16_t getTargetPressure();
    uint16_t getTargetPressure(uint8_t subjectId);

    gaitGuide_stimMode_t stimMode() const;
    void stimMode(gaitGuide_stimMode_t value);

    bool timescale();
    void timescale(bool is_timescale_5ms);

    void setAmplitude(const uint8_t data);
    void setDuration(const uint8_t data, bool driver_side);
    /**
     * @brief default stimulation using Amplitude Real Time Mode this is a "make work" solution
     *
     */
    void stimulateDefault(bool driver_side);

    uint8_t duration(const uint8_t pos = 0)
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

    DRV2605_UTIL *_drv_handle;

    // callbacks
    typedef std::function<void(uint16_t)> CurrentPressureChangedCallback;
    // Set up a callback function to be called when m_currentPressure is changed
    void onCurrentPressureChanged(const CurrentPressureChangedCallback &callback)
    {
        m_currentPressureChangedCallback = callback;
    }

private:
    GaitGuide();
    GaitGuide(const GaitGuide &) = delete;
    GaitGuide &operator=(const GaitGuide &) = delete;
    uint8_t _deviceId;
    std::string _firmwareVersion;

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
    void nvs_save_targetPressure(uint8_t subject_id, uint16_t pressure_value);
    void nvs_save_targetPressure(const char *subject_key, uint16_t pressure_value);
    uint16_t nvs_read_targetPressure(uint8_t subject_id);
    uint16_t nvs_read_targetPressure(const char *subject_key);
    uint16_t nvs_read_targetPressure_backup(uint8_t subject_id);

    uint8_t m_amplitude[8] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t m_duration[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t m_effects[8] = {14, 14, 14, 14, 14, 14, 14, 14}; // effect strong buzz
    uint8_t m_error = 0;

    uint8_t m_batteryLevel = 0;
    uint8_t m_subjectID = 0;
    uint16_t m_currentPressure = 0;
    uint16_t m_targetPressure = 0;
    bool m_is_timescale_5ms = false;

    uint8_t m_packet_seq = 0; // for

    gaitGuide_stimMode_t _stimMode = gaitGuide_stimMode_t::GAITGUIDE_USERMODE_AMPLITUDE;
    gaitGuide_state_t m_currentState = GAITGUIDE_STATE_STARTUP;
    gaitGuide_event_t m_currentEvent = GAITGUIDE_EVENT_NONE;

    const char *m_nvs_namespace = "gaitguide_NVS";

    // callbacks
    CurrentPressureChangedCallback m_currentPressureChangedCallback;
};

#endif
