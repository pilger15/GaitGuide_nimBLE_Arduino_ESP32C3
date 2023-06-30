#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <GaitGuide.h>
#include <LEDS.h>

typedef enum
{
    BLE_SERVER_UUID = 0xF000,
    BLE_DEVICE_INFO_SERVICE_UUID = 0x180A,
    BLE_LRA_CONTROL_SERVICE_UUID = 0x1111,
    BLE_LRA_DIAG_SERVICE_UUID = 0x2220
} ServiceUUID_t;

typedef enum
{
    // DEVICE_INFO_SERVICE
    BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID = 0x2A19,
    // BLE_TX_POWER_LEVEL_CHARACTERISTIC_UUID = 0x2A07,
    //  BLE_FIRMWARE_REVISION_CHARACTERISTIC_UUID = 0x2A26,

    // DEVICE_CONTROL_SERVICE
    BLE_AMPLITUDE_CHARACTERISTIC_UUID = 0x1112,
    BLE_DURATION_MED_CHARACTERISTIC_UUID = 0x1113,
    BLE_DURATION_LAT_CHARACTERISTIC_UUID = 0x1114,

    BLE_PRESSURE_CHARACTERISTIC_UUID = 0x2225 // warning this is not implemented correctly
    /*     BLE_LAT_CONNECTED_CHARACTERISTIC_UUID = 0x1112,
     BLE_LAT_RUNNING_CHARACTERISTIC_UUID = 0x1113,
     BLE_CONTROL_CHARACTERISTIC_UUID = 0x1112,
     BLE_STIM_MODE_CHARACTERISTIC_UUID = 0x1114,
     BLE_EFFECTS_CHARACTERISTIC_UUID = 0x1113,
     BLE_DURATION_CHARACTERISTIC_UUID = 0x1114,
     BLE_STIM_COMMAND_CHARACTERISTIC_UUID = 0x1115,

     // LRA_DIAG_SERVICE
     BLE_MED_CONNECTED_CHARACTERISTIC_UUID = 0x2221,
     BLE_MED_RUNNING_CHARACTERISTIC_UUID = 0x2222,
     BLE_LAT_CONNECTED_CHARACTERISTIC_UUID = 0x2223,
     BLE_LAT_RUNNING_CHARACTERISTIC_UUID = 0x2224,

     */
} CharacteristicUUID_t;

// not used but may be relevant for future use cases
typedef struct __attribute__((packed))
{
    uint8_t BLE_COMMAND_DURATION : 8;
    uint8_t BLE_COMMAND_CONTROL_VAL : 8;
    uint8_t BLE_COMMAND_SEQUENCE : 7;
    bool BLE_COMMAND_POS : 1; // Can be 0: medial or 1: lateral
} BLE_command_t;

static const char *TAG_BLE = "BLE-DEV";

/* Handler class for server events */
class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
    {
        led_pressureMode();
        led_fade_to(127, LED_BLUE);
        ESP_LOGI(TAG_BLE, "Client connected: %s\n", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    };

    void onDisconnect(NimBLEServer *pServer)
    {
        led_breath();
        ESP_LOGI(TAG_BLE, "Client disconnected\n");
        // if still advertising we won't sleep yet.
        if (!pServer->getAdvertising()->isAdvertising())
        {
            // ESP_LOGI(TAG_BLE, "Sleeping for %u seconds\n", 5);
            // esp_deep_sleep_start();
        }
    };
};

/** Handler class for characteristic actions */
class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
    GaitGuide &gaitGuide = GaitGuide::getInstance();
    void onRead(NimBLECharacteristic *pCharacteristic)
    {

        ESP_LOGI(TAG_BLE, "%s: onRead(), value: %s", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().c_str());
    };

    void onWrite(NimBLECharacteristic *pCharacteristic)
    {
        const uint8_t *data;

        ESP_LOGI(TAG_BLE, "[%s] onWrite(), value: %s", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().c_str());
        switch (pCharacteristic->getUUID().getNative()->u16.value)
        {
        case BLE_AMPLITUDE_CHARACTERISTIC_UUID:
            ESP_LOGD(TAG_BLE, "[%s] Amplitude set to %d", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue()[0]);
            if (gaitGuide.goLateral || gaitGuide.goMedial)
            {
                ESP_LOGD(TAG_DRV, "COMMAND IGNORED: Device still running");
            }
            else
            {
                gaitGuide.setAmplitude(pCharacteristic->getValue().data()[0]);
            }

            break;

        case BLE_DURATION_MED_CHARACTERISTIC_UUID:
            ESP_LOGD(TAG_BLE, "[%s] Started MED: %dms", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue()[0]);

            if (gaitGuide.goLateral || gaitGuide.goMedial)
            {
                ESP_LOGD(TAG_DRV, "COMMAND IGNORED: Device still running");
            }
            else
            {
                gaitGuide.setDuration(pCharacteristic->getValue().data()[0], drv_medial);
            }

            break;
        case BLE_DURATION_LAT_CHARACTERISTIC_UUID:
            ESP_LOGD(TAG_BLE, "[%s] Started LAT: %dms", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue()[0]);

            if (gaitGuide.goLateral || gaitGuide.goMedial)
            {
                ESP_LOGD(TAG_DRV, "COMMAND IGNORED: Device still running");
            }
            else
            {
                gaitGuide.setDuration(pCharacteristic->getValue().data()[0], drv_lateral);
            }

            break;
        default:

            ESP_LOGD(TAG_BLE, "Undefined BLE-Characteristic");
            break;
        }
        /*
    case BLE_CONTROL_CHARACTERISTIC_UUID:
        data = pCharacteristic->getValue().data();
        if ((gaitGuide_event_t)data[0])
        {
            ESP_LOGD(TAG_BLE, "Control-Event: #%s", pCharacteristic->getValue().c_str());
            gaitGuide.newEvent((gaitGuide_event_t)data[0]);
        }
        else
        {
            ESP_LOGE(TAG_BLE, "Control-Event not allowed: #%s", pCharacteristic->getValue().c_str());
        }
        break;
    case BLE_STIM_MODE_CHARACTERISTIC_UUID:
        ESP_LOGD(TAG_BLE, "Changing Usermode: %s", pCharacteristic->getValue().c_str());
        data = pCharacteristic->getValue().data();
        gaitGuide.stimMode((gaitGuide_stimMode_t)data[0]);
        break;
    case BLE_STIM_COMMAND_CHARACTERISTIC_UUID:
        ESP_LOGD(TAG_BLE, "%s: LAT_COMMAND", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().c_str());

        if (gaitGuide.goLateral || gaitGuide.goMedial)
        {
            ESP_LOGD(TAG_DRV, "COMMAND IGNORED: Device still running");
        }
        else
        {
            gaitGuide.setCommand(pCharacteristic->getValue().data());
        }

        break;
    case BLE_PRESSURE_CHARACTERISTIC_UUID:
        gaitGuide.setTargetPressure();
        pCharacteristic->setValue(gaitGuide.getTargetPressure());
        break;


        case BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: BATTERY_LEVEL NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());
            break;
        case BLE_TX_POWER_LEVEL_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: TX_POWER_LEVEL NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

            break;
        case BLE_FIRMWARE_REVISION_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: FIRMWARE_REVISION NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

            break;
        case BLE_LAT_CONNECTED_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: LAT_CONNECTED NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

            break;
        case BLE_LAT_RUNNING_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: LAT_RUNNING_CHARACTERISTIC NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

            break;
        case BLE_STIM_MODE_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: LAT_MODE NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

            break;
        case BLE_EFFECTS_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: LAT_EFFECT NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

            break;
        case BLE_DURATION_CHARACTERISTIC_UUID:
            ESP_LOGI(TAG_BLE, "%s: LAT_DURATION NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

            break;
        */

        /*
    case BLE_MED_CONNECTED_CHARACTERISTIC_UUID:
        ESP_LOGI(TAG_BLE, "%s: MED_CONNECTED NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

        break;
    case BLE_MED_RUNNING_CHARACTERISTIC_UUID:
        ESP_LOGI(TAG_BLE, "%s: MED_RUNNING_CHARACTERISTIC NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

        break;
    case BLE_MED_MODE_CHARACTERISTIC_UUID:
        ESP_LOGI(TAG_BLE, "%s: MED_MODE NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

        break;
    case BLE_MED_EFFECT_CHARACTERISTIC_UUID:
        ESP_LOGI(TAG_BLE, "%s: MED_EFFECT NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

        break;
    case BLE_MED_DURATION_CHARACTERISTIC_UUID:
        ESP_LOGI(TAG_BLE, "%s: MED_DURATION NOT IMPLEMENTED!", pCharacteristic->getUUID().toString().c_str());

        break;*/
    };
    /** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */
    void onNotify(NimBLECharacteristic *pCharacteristic)
    {
        ESP_LOGI(TAG_BLE, "Sending notification to clients");
    };

    /** The status returned in status is defined in NimBLECharacteristic.h.
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic *pCharacteristic, Status status, int code)
    {
        String str = ("Notification/Indication status code: ");
        str += status;
        str += ", return code: ";
        str += code;
        str += ", ";
        str += NimBLEUtils::returnCodeToString(code);
        ESP_LOGI(TAG_BLE, "%s", str);
    };

    void onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue)
    {
        String str = "Client ID: ";
        str += desc->conn_handle;
        str += " Address: ";
        str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
        if (subValue == 0)
        {
            str += " Unsubscribed to ";
        }
        else if (subValue == 1)
        {
            str += " Subscribed to notfications for ";
        }
        else if (subValue == 2)
        {
            str += " Subscribed to indications for ";
        }
        else if (subValue == 3)
        {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID()).c_str();
        ESP_LOGI(TAG_BLE, "%s", str);
    };
};

static CharacteristicCallbacks chrCallbacks;
extern DRV2605_UTIL drv;

void ble_setup(const std::string &deviceName);
void ble_advertisePressure(uint16_t pressure);
/*
#define BLE_SERVER_UUID "F000"

// Device Info service
#define BLE_DEVICE_INFO_SERVICE_UUID "180A"
#define BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID "2A19"
#define BLE_TX_POWER_LEVEL_CHARACTERISTIC_UUID "2A07"
#define BLE_FIRMWARE_REVISION_CHARACTERISTIC_UUID "2A26"

// Device Info service
NimBLEUUID ble_device_info_service_uuid = NimBLEUUID(BLE_DEVICE_INFO_SERVICE_UUID);
NimBLEUUID ble_battery_level_characteristic_uuid = NimBLEUUID(BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID);
NimBLEUUID ble_tx_power_level_characteristic_uuid = NimBLEUUID(BLE_TX_POWER_LEVEL_CHARACTERISTIC_UUID);
NimBLEUUID ble_firmware_revision_characteristic_uuid = NimBLEUUID(BLE_FIRMWARE_REVISION_CHARACTERISTIC_UUID);



// DRV-Lateral service
#define BLE_LRA_CONTROL_SERVICE_UUID "F0001110-0451-4000-B000-000000000000"
#define BLE_LAT_CONNECTED_CHARACTERISTIC_UUID "F0001111-0451-4000-B000-000000000000"
#define BLE_LAT_RUNNING_CHARACTERISTIC_UUID "F0001112-0451-4000-B000-000000000000"
#define BLE_MODE_CHARACTERISTIC_UUID "F0001113-0451-4000-B000-000000000000"
#define BLE_EFFECTS_CHARACTERISTIC_UUID "F0001114-0451-4000-B000-000000000000"
#define BLE_DURATION_CHARACTERISTIC_UUID "F0001115-0451-4000-B000-000000000000"
#define BLE_COMMAND_CHARACTERISTIC_UUID "F0001116-0451-4000-B000-000000000000"

NimBLEUUID ble_lat_service_uuid = NimBLEUUID(BLE_LRA_CONTROL_SERVICE_UUID);
NimBLEUUID ble_lat_connected_characteristic_uuid = NimBLEUUID(BLE_LAT_CONNECTED_CHARACTERISTIC_UUID);
NimBLEUUID ble_lat_running_characteristic_uuid = NimBLEUUID(BLE_LAT_RUNNING_CHARACTERISTIC_UUID);
NimBLEUUID ble_lat_mode_characteristic_uuid = NimBLEUUID(BLE_STIM_MODE_CHARACTERISTIC_UUID);
NimBLEUUID ble_lat_effect_characteristic_uuid = NimBLEUUID(BLE_EFFECTS_CHARACTERISTIC_UUID);
NimBLEUUID ble_lat_duration_characteristic_uuid = NimBLEUUID(BLE_DURATION_CHARACTERISTIC_UUID);
NimBLEUUID ble_lat_command_characteristic_uuid = NimBLEUUID(BLE_COMMAND_CHARACTERISTIC_UUID);

// DRV-medial service
#define BLE_LRA_DIAG_SERVICE_UUID "F0002220-0451-4000-B000-000000000000"
#define BLE_MED_CONNECTED_CHARACTERISTIC_UUID "F0002221-0451-4000-B000-000000000000"
#define BLE_MED_RUNNING_CHARACTERISTIC_UUID "F0002222-0451-4000-B000-000000000000"
#define BLE_MED_MODE_CHARACTERISTIC_UUID "F0002223-0451-4000-B000-000000000000"
#define BLE_MED_EFFECT_CHARACTERISTIC_UUID "F0002224-0451-4000-B000-000000000000"
#define BLE_MED_DURATION_CHARACTERISTIC_UUID "F0002225-0451-4000-B000-000000000000"
#define BLE_MED_COMMAND_CHARACTERISTIC_UUID "F0002226-0451-4000-B000-000000000000"

NimBLEUUID ble_med_service_uuid = NimBLEUUID(BLE_LRA_DIAG_SERVICE_UUID);
NimBLEUUID ble_med_connected_characteristic_uuid = NimBLEUUID(BLE_MED_CONNECTED_CHARACTERISTIC_UUID);
NimBLEUUID ble_med_running_characteristic_uuid = NimBLEUUID(BLE_MED_RUNNING_CHARACTERISTIC_UUID);
NimBLEUUID ble_med_mode_characteristic_uuid = NimBLEUUID(BLE_MED_MODE_CHARACTERISTIC_UUID);
NimBLEUUID ble_med_effect_characteristic_uuid = NimBLEUUID(BLE_MED_EFFECT_CHARACTERISTIC_UUID);
NimBLEUUID ble_med_duration_characteristic_uuid = NimBLEUUID(BLE_MED_DURATION_CHARACTERISTIC_UUID);
NimBLEUUID ble_med_command_characteristic_uuid = NimBLEUUID(BLE_MED_COMMAND_CHARACTERISTIC_UUID);
*/

/*
// UUIDs for the services and characteristics
#define DEVICE_INFO_SERVICE_UUID      "0000180a-0000-1000-8000-00805f9b34fb"
#define BATTERY_LEVEL_UUID             "00002a19-0000-1000-8000-00805f9b34fb"
#define TX_POWER_LEVEL_UUID            "00002a07-0000-1000-8000-00805f9b34fb"
#define FIRMWARE_REVISION_UUID         "00002a26-0000-1000-8000-00805f9b34fb"
*/
#endif // BLE_HANDLER_H
