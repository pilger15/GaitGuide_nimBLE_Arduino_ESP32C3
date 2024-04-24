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
    BLE_BATTERY_SERVICE_UUID = 0x180F,
    BLE_LRA_CONTROL_SERVICE_UUID = 0x1111,
    BLE_LRA_DIAG_SERVICE_UUID = 0x2220
} ServiceUUID_t;

typedef enum
{
    // BATTERY_SERVICE
    BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID = 0x2A19,
    // DEVICE_INFO_SERVICE
    // BLE_TX_POWER_LEVEL_CHARACTERISTIC_UUID = 0x2A07,
    BLE_FIRMWARE_REVISION_CHARACTERISTIC_UUID = 0x2A26,

    // DEVICE_CONTROL_SERVICE
    BLE_AMPLITUDE_CHARACTERISTIC_UUID = 0x1112,

    BLE_DURATION_LEFT_CHARACTERISTIC_UUID = 0x1113,
    BLE_DURATION_RIGHT_CHARACTERISTIC_UUID = 0x1114,
    /*
        BLE_PRESSURE_CHARACTERISTIC_UUID = 0x2225 // warning this is not implemented correctly
           BLE_LAT_CONNECTED_CHARACTERISTIC_UUID = 0x1112,
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
    bool BLE_COMMAND_POS : 1; // Can be 0: Right or 1: Left
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
    uint8_t incrementer = 0;
    GaitGuide &gaitGuide = GaitGuide::getInstance();
    void onRead(NimBLECharacteristic *pCharacteristic)
    {
        switch (pCharacteristic->getUUID().getNative()->u16.value)
        {
        case BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID:
            incrementer++;
            pCharacteristic->setValue(gaitGuide.batteryLevel());
            ESP_LOGD(TAG_BLE, , "[%s] Battery level %d", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue()[0] + 48);
            break;
        default:

            ESP_LOGD(TAG_BLE, "Undefined BLE-Characteristic");
            break;
        }
    }

    void
    onWrite(NimBLECharacteristic *pCharacteristic)
    {
        const uint8_t *data;

        ESP_LOGI(TAG_BLE, "[%s] onWrite(), value: %s", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue().c_str());
        switch (pCharacteristic->getUUID().getNative()->u16.value)
        {
        case BLE_AMPLITUDE_CHARACTERISTIC_UUID:
            ESP_LOGD(TAG_BLE, "[%s] Amplitude set to %d", pCharacteristic->getUUID().toString().c_str(), pCharacteristic->getValue()[0]);
            if (gaitGuide.goLeft || gaitGuide.goRight)
            {
                ESP_LOGD("", "COMMAND IGNORED: Device still running");
            }
            else
            {
                gaitGuide.setAmplitude(pCharacteristic->getValue().data()[0]);
            }

            break;

        case BLE_DURATION_RIGHT_CHARACTERISTIC_UUID:
            ESP_LOGD(TAG_BLE, "[%s] Command RIGHT: %dms", pCharacteristic->getUUID().toString().c_str(), ((uint16_t)pCharacteristic->getValue().data()[1] << 8 | pCharacteristic->getValue().data()[0]));

            if (gaitGuide.goLeft || gaitGuide.goRight)
            {
                ESP_LOGD("", "COMMAND IGNORED: Device still running");
            }
            else
            {
                uint16_t duration = ((uint16_t)pCharacteristic->getValue().data()[1] << 8 | pCharacteristic->getValue().data()[0]);
                gaitGuide.setDuration(duration, drv_right);
            }

            break;
        case BLE_DURATION_LEFT_CHARACTERISTIC_UUID:
            ESP_LOGD(TAG_BLE, "[%s] Command LEFT: %dms", pCharacteristic->getUUID().toString().c_str(), ((uint16_t)pCharacteristic->getValue().data()[1] << 8 | pCharacteristic->getValue().data()[0]));

            if (gaitGuide.goLeft || gaitGuide.goRight)
            {
                ESP_LOGD("", "COMMAND IGNORED: Device still running");
            }
            else
            {
                uint16_t duration = ((uint16_t)pCharacteristic->getValue().data()[1] << 8 | pCharacteristic->getValue().data()[0]);
                gaitGuide.setDuration(duration, drv_left);
            }

            break;
        default:

            ESP_LOGD(TAG_BLE, "Undefined BLE-Characteristic");
            break;
        }
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

void ble_setup(const std::string &deviceName);

#endif // BLE_HANDLER_H
