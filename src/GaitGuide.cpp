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

gaitGuide_state_t GaitGuide::currentState()
{
    return m_currentState;
}
gaitGuide_event_t GaitGuide::currentEvent()
{
    return m_currentEvent;
}

uint8_t GaitGuide::newEvent(gaitGuide_event_t event)
{
    uint8_t ret = 0;
    noInterrupts();
    log_d("NEW EVENT: %d, current State: %d", (uint8_t)event, m_currentState);
    if (m_currentEvent != GAITGUIDE_EVENT_NONE && event != GAITGUIDE_EVENT_NONE)
    {
        ESP_LOGE("EVENT", "Previous event is not yet done");
        ret = 1;
    }
    else
    {
        m_currentEvent = event;
        log_d("Current EVENT: %d, current State: %d", m_currentEvent, m_currentState);
        if (m_currentEvent == GAITGUIDE_EVENT_RESET)
        {
            log_i("restarting device...");
            ESP.restart();
        }
        else if (m_currentEvent != GAITGUIDE_EVENT_NONE)
        {

            switch (m_currentState)
            {
            case GAITGUIDE_STATE_STARTUP:
                // Handle Start-up state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    ESP_LOGI("EVENT", "ON_STARTUP_DONE: Switching from STARTUP to LOOKING for connection");
                    m_currentState = GAITGUIDE_STATE_LFC;
                    break;
                }

                break;

            case GAITGUIDE_STATE_LFC:
                // Handle Looking-for-connection state        if (m_currentEvent ==GAITGUIDE_EVENT_DONE)
                if (m_currentEvent == GAITGUIDE_EVENT_SLEEP)
                {
                    ESP_LOGI("EVENT", "ON_SLEEP: Switching from Looking for connection to LOW Power Mode");
                    // #TODO Implement sleep state;
                    m_currentState = GAITGUIDE_STATE_LOW_POWER;
                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_BT_CONNECT)
                {
                    ESP_LOGI("EVENT", "BT_CONNECT: Switching from Looking for connection to IDLE");
                    m_currentState = GAITGUIDE_STATE_LOW_POWER;

                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_SLEEP)
                {
                    ESP_LOGI("EVENT", "ON_SLEEP: Switching from Looking for connection to LOW Power Mode");
                    m_currentState = GAITGUIDE_STATE_LOW_POWER;

                    break;
                }
                break;

            case GAITGUIDE_STATE_LOW_POWER:
                // Handle Low-power state
                if (m_currentEvent == GAITGUIDE_EVENT_WAKEUP)
                {
                    ESP_LOGI("EVENT", "ON_WAKEUP: Switching from LOW Power Mode to looking for connection");
                    m_currentState = GAITGUIDE_STATE_LFC;

                    break;
                }
                break;

            case GAITGUIDE_STATE_IDLE:
                // Handle Idle state
                if (m_currentEvent == GAITGUIDE_EVENT_FIND_PRESSURE)
                {
                    ESP_LOGI("EVENT", "FIND_PRESSURE: Switching from IDLE Mode to STATE_PRESSURE_SENSING");
                    m_currentState = GAITGUIDE_STATE_PRESSURE_SENSING;

                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_SET_PRESSURE)
                {
                    ESP_LOGI("EVENT", "SET_PRESSURE: Switching from IDLE Mode to STATE_PRESSURE_SETTING");
                    m_currentState = GAITGUIDE_STATE_PRESSURE_SETTING;

                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_STIM)
                {
                    ESP_LOGI("EVENT", "FIND_PRESSURE: Switching from IDLE Mode to STIM_MODE");
                    m_currentState = GAITGUIDE_STATE_STIM;

                    break;
                }
                break;

            case GAITGUIDE_STATE_STIM:
                // Handle Stimulation state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    ESP_LOGI("EVENT", "Stimulation Done: Switching to IDLE Mode");
                    m_currentState = GAITGUIDE_STATE_IDLE;

                    break;
                }
                break;

            case GAITGUIDE_STATE_PRESSURE_SENSING:
                // Handle Pressure-finding state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    ESP_LOGI("EVENT", "PRESSURE_SENSING Done: Switching to IDLE Mode");
                    m_currentState = GAITGUIDE_STATE_IDLE;

                    break;
                }
                break;

            case GAITGUIDE_STATE_PRESSURE_SETTING:
                // Handle Pressure-setting state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    ESP_LOGI("EVENT", "PRESSURE_SETTING Done: Switching to IDLE Mode");
                    m_currentState = GAITGUIDE_STATE_IDLE;

                    break;
                }
                break;

            default:
                // Handle error state
                ESP_LOGE("STATE", "Error unknown state");
                ret = 1;
                break;
            }
        }
    }
    interrupts();
    return ret;
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

uint8_t GaitGuide::subjectId() const
{
    return m_subjectID;
}
void GaitGuide::subjectId(uint8_t subjectId)
{
    m_subjectID = subjectId;
}

uint16_t GaitGuide::currentPressureLevel() const
{
    return m_currentPressure;
}

void GaitGuide::set_currentPressureLevel(uint16_t pressureLevel)
{
    m_currentPressure = pressureLevel;
    if (m_currentPressureChangedCallback)
    {
        m_currentPressureChangedCallback(m_currentPressure);
    }
}

// sets the target pressure to the current pressure; #TODO implement patient based
void GaitGuide::setTargetPressure()
{
    setTargetPressure(m_subjectID);
}
void GaitGuide::setTargetPressure(uint8_t subjectID)
{
    m_targetPressure = m_currentPressure;
    nvs_save_targetPressure(subjectID, m_targetPressure);
    // # TODO implement targetPressure callback
}
uint16_t GaitGuide::getTargetPressure()
{
    return getTargetPressure(m_subjectID);
}

uint16_t GaitGuide::getTargetPressure(uint8_t subjectID)
{
    // m_targetPressure = nvs_read_targetPressure(subjectID);
    return m_targetPressure;
}

// #TODO implement getbackup

gaitGuide_stimMode_t GaitGuide::stimMode() const
{
    return _userMode;
}

void GaitGuide::stimMode(gaitGuide_stimMode_t value)
{
    _userMode = value;
}

bool GaitGuide::timescale()
{
    return m_is_timescale_5ms;
}

void GaitGuide::timescale(bool is_timescale_5ms)
{
    m_is_timescale_5ms = is_timescale_5ms;
}

void GaitGuide::setCommand(const uint8_t *data)
{
    uint8_t driver_side;
    uint8_t packet_seq = data[2] & 0x7F; // #Todo include packet_seq to ensure no data has been missing
    if (packet_seq != m_packet_seq++)
    {
        // #TODO errorhandling;
        // make packet_seq skip to new value
        m_packet_seq = packet_seq;
    }
    driver_side = data[2]; // medial or lateral

    if (driver_side == drv_medial)
    {
        goLateral = false;
        goMedial = true;
    }
    else
    {
        goLateral = true;
        goMedial = false;
    }
    m_amplitude[0] = data[1]; // amplitude can be 0-127. effects are within that range as well bit 7 will control the side (medial/lateral)

    m_duration[0] = data[0];
}
//------------ private functions ---------------

// saves the target pressure and backs it up
void GaitGuide::nvs_save_targetPressure(uint8_t subject_id, uint16_t pressure_value)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;

    // Open NVS namespace for storing pressure values
    err = nvs_open(m_nvs_namespace, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        printf("Error opening NVS namespace: %s\n", esp_err_to_name(err));
        return;
    }

    // Read the old pressure value for this subject and save it in a backup nvs
    uint16_t old_pressure_value;
    char backup_key[10];
    sprintf(backup_key, "backup_%d", subject_id);
    err = nvs_get_u16(nvs_handle, backup_key, &old_pressure_value);
    if (err == ESP_OK)
    {
        // Backup the old pressure value
        err = nvs_set_u16(nvs_handle, backup_key, old_pressure_value);
        if (err != ESP_OK)
        {
            ESP_LOGE("NVS", "Error backing up old pressure value: %s\n", esp_err_to_name(err));
        }
    }

    // Save the new pressure value
    char key[3];
    sprintf(key, "%d", subject_id);
    err = nvs_set_u16(nvs_handle, key, pressure_value);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error saving pressure value: %s\n", esp_err_to_name(err));
    }

    // Commit the changes to NVS
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error committing changes to NVS: %s\n", esp_err_to_name(err));
    }

    // Close the NVS handle
    nvs_close(nvs_handle);
}
// Function to read the pressure value for a subject
uint16_t GaitGuide::nvs_read_targetPressure(uint8_t subject_id)
{
    uint16_t pressure_value = 0;
    char key[3];
    sprintf(key, "%d", subject_id);
    nvs_read_targetPressure(key);

    pressure_value = nvs_read_targetPressure(key);

    return pressure_value;
}
// Function to read the pressure value for a subject
uint16_t GaitGuide::nvs_read_targetPressure(const char *subject_key)
{
    esp_err_t err;
    nvs_handle_t nvs_handle;
    uint16_t pressure_value = 0;
    // Open NVS namespace for storing pressure values
    err = nvs_open(m_nvs_namespace, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK)
    {
        switch (err)
        {
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE("NVS", "The value is not initialized yet!\n");
            break;
        default:
            ESP_LOGE("NVS", "Error (%s) reading!\n", esp_err_to_name(err));
        }
        return 0;
    }

    // Read the pressure value for this subject
    err = nvs_get_u16(nvs_handle, subject_key, &pressure_value);
    if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "Error reading pressure value: %s\n", esp_err_to_name(err));
    }

    // Close the NVS handle
    nvs_close(nvs_handle);

    return pressure_value;
}
// Function to read the pressure value for a subject
uint16_t GaitGuide::nvs_read_targetPressure_backup(uint8_t subject_id)
{
    uint16_t pressure_value = 0;
    char key[3];
    sprintf(key, "backup_%d", subject_id);
    nvs_read_targetPressure(key);

    pressure_value = nvs_read_targetPressure(key);

    return pressure_value;
}