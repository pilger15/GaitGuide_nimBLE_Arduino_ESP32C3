#include "GaitGuide.h"

GaitGuide::GaitGuide()
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
}

GaitGuide &GaitGuide::getInstance()
{
    static GaitGuide instance;
    return instance;
}

uint8_t GaitGuide::deviceId() const
{
    return _deviceId;
}

void GaitGuide::deviceId(uint8_t value)
{
    _deviceId = value;
}

std::string GaitGuide::getFirmwareVersion() const
{
    return _firmwareVersion;
}
void GaitGuide::setFirmwareVersion(std::string newVersion)
{
    _firmwareVersion = newVersion;
}

uint8_t GaitGuide::batteryLevel() const
{
    return m_batteryLevel;
}

void GaitGuide::batteryLevel(uint8_t batteryLevel)
{
    m_batteryLevel = batteryLevel;
}

// #TODO implement get backup of subjectIDs pressure

gaitGuide_stimMode_t GaitGuide::stimMode() const
{
    return _stimMode;
}

void GaitGuide::stimMode(gaitGuide_stimMode_t value)
{
    _stimMode = value;
}

bool GaitGuide::timescale()
{
    return m_is_timescale_5ms;
}

void GaitGuide::timescale(bool is_timescale_5ms)
{
    m_is_timescale_5ms = is_timescale_5ms;
}

void GaitGuide::setAmplitude(const uint8_t data)
{
    m_amplitude[0] = data;
    /*
    uint8_t driver_side;
    uint8_t packet_seq = data[2] & 0x7F; // #Todo include packet_seq to ensure no data has been missing
    if (packet_seq != m_packet_seq++)
    {
        // #TODO errorhandling;
        // make packet_seq skip to new value
        m_packet_seq = packet_seq;
    }
    driver_side = data[2]; // Right or Left

    if (driver_side == drv_Right)
    {
        goLeft = false;
        goRight = true;
    }
    else
    {
        goLeft = true;
        goRight = false;
    }
    m_amplitude[0] = data[1]; // amplitude can be 0-127. effects are within that range as well bit 7 will control the side (Right/Left)
    */
    // m_duration[0] = data[0];
}
void GaitGuide::setDuration(const uint8_t data, bool driver_side)
{

    m_duration[0] = data;
    if (driver_side == drv_right)
    {
        goLeft = false;
        goRight = true;
    }
    else
    {
        goLeft = true;
        goRight = false;
    }

    if (m_stimulationCallback)
    {
        m_stimulationCallback(driver_side);
    }
}
