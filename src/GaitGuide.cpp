#include "GaitGuide.h"

portMUX_TYPE eventQueueMux = portMUX_INITIALIZER_UNLOCKED;
#define EVENT_QUEUE_LEN 16
gaitGuide_event_t eventQueue[EVENT_QUEUE_LEN];
uint8_t eventQueue_idx = 0;
uint8_t eventQueue_sel = 0;

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

/**
 * @brief Adds a new event to the event queue.
 *
 * This function adds the specified event to the event queue. If the event is a high priority reset event,
 * it restarts the device. The event is added to the queue, taking into account the circular nature of the queue.
 * If the event queue is full, an error is logged, and the function returns 1.
 *
 * @param event The event to be added.
 * @return Returns 0 on success or 1 if the event queue is full.
 */
uint8_t GaitGuide::newEvent(gaitGuide_event_t event)
{
    // High priority restart
    if (event == GAITGUIDE_EVENT_RESET)
    {
        // Restart the device
        ESP.restart();
    }

    // Add event to the queue
    portENTER_CRITICAL_ISR(&eventQueueMux);
    eventQueue[eventQueue_idx++] = event;
    eventQueue_idx %= EVENT_QUEUE_LEN;
    if (eventQueue_idx == eventQueue_sel)
    {
        // Event queue overflow occurred
        log_e("Event-queue Overflow");
        return 1;
    }
    portEXIT_CRITICAL_ISR(&eventQueueMux);

    return 0;
}

uint8_t GaitGuide::nextEvent()
{
    uint8_t ret = 0;
    if (eventQueue_idx != eventQueue_sel) // non handled events exist
    {
        m_currentEvent = eventQueue[eventQueue_sel++]; // get event from queue
        eventQueue_sel %= EVENT_QUEUE_LEN;
        if (m_currentState != GAITGUIDE_STATE_IDLE && m_currentEvent != GAITGUIDE_EVENT_DONE)
        {
            log_e("AcTION EVENT IN NON IDLE STATE\nCURRENT STATE %d, EVENT %d", m_currentState, m_currentEvent);
        }

        if (m_currentEvent != GAITGUIDE_EVENT_NONE)
        {

            switch (m_currentState)
            {
            case GAITGUIDE_STATE_STARTUP:
                // Handle Start-up state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    //    ESP_LOGI("EVENT", "ON_STARTUP_DONE: Switching from STARTUP to LOOKING for connection");
                    m_currentState = GAITGUIDE_STATE_LFC;
                    break;
                }

                break;

            case GAITGUIDE_STATE_LFC:
                // Handle Looking-for-connection state        if (m_currentEvent ==GAITGUIDE_EVENT_DONE)
                if (m_currentEvent == GAITGUIDE_EVENT_SLEEP)
                {
                    //      ESP_LOGI("EVENT", "ON_SLEEP: Switching from Looking for connection to LOW Power Mode");
                    // #TODO Implement sleep state;
                    m_currentState = GAITGUIDE_STATE_LOW_POWER;
                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_BT_CONNECT)
                {
                    //   ESP_LOGI("EVENT", "BT_CONNECT: Switching from Looking for connection to IDLE");
                    m_currentState = GAITGUIDE_STATE_IDLE;

                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_SLEEP)
                {
                    //    ESP_LOGI("EVENT", "ON_SLEEP: Switching from Looking for connection to LOW Power Mode");
                    m_currentState = GAITGUIDE_STATE_LOW_POWER;

                    break;
                }
                break;

            case GAITGUIDE_STATE_LOW_POWER:
                // Handle Low-power state
                if (m_currentEvent == GAITGUIDE_EVENT_WAKEUP)
                {
                    //   ESP_LOGI("EVENT", "ON_WAKEUP: Switching from LOW Power Mode to looking for connection");
                    m_currentState = GAITGUIDE_STATE_LFC;

                    break;
                }
                break;

            case GAITGUIDE_STATE_IDLE:
                // Handle Idle state
                if (m_currentEvent == GAITGUIDE_EVENT_ACC)
                {
                    //   ESP_LOGI("EVENT", "FIND_PRESSURE: Switching from IDLE Mode to STATE_PRESSURE_SENSING");
                    m_currentState = GAITGUIDE_STATE_ACC;

                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_STOP_ACC)
                {
                    //  ESP_LOGI("EVENT", "SET_PRESSURE: Switching from IDLE Mode to STATE_PRESSURE_SETTING");
                    m_currentState = GAITGUIDE_STATE_STOP_ACC;

                    break;
                }
                if (m_currentEvent == GAITGUIDE_EVENT_STIM)
                {
                    //  ESP_LOGI("EVENT", "FIND_PRESSURE: Switching from IDLE Mode to STIM_MODE");
                    m_currentState = GAITGUIDE_STATE_STIM;

                    break;
                }
                break;

            case GAITGUIDE_STATE_STIM:
                // Handle Stimulation state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    //    ESP_LOGI("EVENT", "Stimulation Done: Switching to IDLE Mode");
                    m_currentState = GAITGUIDE_STATE_IDLE;

                    break;
                }
                break;

            case GAITGUIDE_STATE_ACC:
                // Handle Pressure-finding state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    //    ESP_LOGI("EVENT", "PRESSURE_SENSING Done: Switching to IDLE Mode");
                    m_currentState = GAITGUIDE_STATE_IDLE;

                    break;
                }
                break;

            case GAITGUIDE_STATE_STOP_ACC:
                // Handle Pressure-setting state
                if (m_currentEvent == GAITGUIDE_EVENT_DONE)
                {
                    //   ESP_LOGI("EVENT", "PRESSURE_SETTING Done: Switching to IDLE Mode");
                    m_currentState = GAITGUIDE_STATE_IDLE;

                    break;
                }
                break;

            default:
                // Handle error state
                //  ESP_LOGE("STATE", "Error unknown state");
                ret = 1;
                break;
            }
        }
    }
    m_currentEvent = GAITGUIDE_EVENT_NONE;
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