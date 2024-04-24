#include <BLE_HANDLER.h>

void ble_setup(const std::string &deviceName)
{

    // Serial.begin(115200);
    // Serial.println("Starting GaitGuide BLE - Server");
    NimBLEDevice::init(deviceName);
    //  NimBLEDevice::
#ifdef ESP_PLATFORM
    // NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif

    NimBLEDevice::setSecurityAuth(true, true, true);
    //  NimBLEDevice::setSecurityPasskey(123456);
    //  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // deactivates passkey -> Just works
    NimBLEServer *pServer = NimBLEDevice::createServer();

    NimBLEService *controlService = pServer->createService((uint16_t)BLE_LRA_CONTROL_SERVICE_UUID);
    NimBLEService *batteryService = pServer->createService((uint16_t)BLE_BATTERY_SERVICE_UUID);

    // NimBLEService *diagService = pServer->createService((uint16_t)BLE_LRA_DIAG_SERVICE_UUID);
    pServer->setCallbacks(new ServerCallbacks);
    // NimBLECharacteristic *pNonSecureCharacteristic = pService->createCharacteristic("1234", NIMBLE_PROPERTY::READ );
    // NimBLECharacteristic *pSecureCharacteristic = pService->createCharacteristic("1235", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN);

    NimBLECharacteristic *BLE_Amplitude = controlService->createCharacteristic((uint16_t)BLE_AMPLITUDE_CHARACTERISTIC_UUID,
                                                                               NIMBLE_PROPERTY::READ |
                                                                                   NIMBLE_PROPERTY::WRITE,
                                                                               //  NIMBLE_PROPERTY::READ_ENC |
                                                                               //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                               1);
    BLE_Amplitude->setCallbacks(&chrCallbacks);
    BLE_Amplitude->setValue((uint8_t)0x00);
    NimBLECharacteristic *BLE_Duration_MED = controlService->createCharacteristic((uint16_t)BLE_DURATION_RIGHT_CHARACTERISTIC_UUID,
                                                                                  NIMBLE_PROPERTY::READ |
                                                                                      NIMBLE_PROPERTY::WRITE,
                                                                                  //  NIMBLE_PROPERTY::READ_ENC |
                                                                                  //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                                  2);
    BLE_Duration_MED->setCallbacks(&chrCallbacks);
    BLE_Duration_MED->setValue((uint16_t)0x0000);

    NimBLECharacteristic *BLE_Duration_LAT = controlService->createCharacteristic((uint16_t)BLE_DURATION_LEFT_CHARACTERISTIC_UUID,
                                                                                  NIMBLE_PROPERTY::READ |
                                                                                      NIMBLE_PROPERTY::WRITE,
                                                                                  //  NIMBLE_PROPERTY::READ_ENC |
                                                                                  //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                                  2);
    BLE_Duration_LAT->setCallbacks(&chrCallbacks);
    BLE_Duration_LAT->setValue((uint16_t)0x0000);

    // Battery sertvice
    NimBLECharacteristic *BLE_Battery_Level = batteryService->createCharacteristic((uint16_t)BLE_BATTERY_LEVEL_CHARACTERISTIC_UUID,
                                                                                   NIMBLE_PROPERTY::READ,
                                                                                   1);
    BLE_Battery_Level->setCallbacks(&chrCallbacks);
    BLE_Battery_Level->setValue((uint8_t)0xFF);

    /*
      READ         =  BLE_GATT_CHR_F_READ,
      READ_ENC     =  BLE_GATT_CHR_F_READ_ENC,
      READ_AUTHEN  =  BLE_GATT_CHR_F_READ_AUTHEN,
      READ_AUTHOR  =  BLE_GATT_CHR_F_READ_AUTHOR,
      WRITE        =  BLE_GATT_CHR_F_WRITE,
      WRITE_NR     =  BLE_GATT_CHR_F_WRITE_NO_RSP,
      WRITE_ENC    =  BLE_GATT_CHR_F_WRITE_ENC,
      WRITE_AUTHEN =  BLE_GATT_CHR_F_WRITE_AUTHEN,
      WRITE_AUTHOR =  BLE_GATT_CHR_F_WRITE_AUTHOR,
      BROADCAST    =  BLE_GATT_CHR_F_BROADCAST,
      NOTIFY       =  BLE_GATT_CHR_F_NOTIFY,
      INDICATE     =  BLE_GATT_CHR_F_INDICATE*/
    controlService->start();
    batteryService->start();
    // diagService->start();
    //  pNonSecureCharacteristic->setValue("Hello Non Secure BLE");
    //  pSecureCharacteristic->setValue("Hello Secure BLE");
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID((uint16_t)BLE_LRA_CONTROL_SERVICE_UUID);
    pAdvertising->addServiceUUID((uint16_t)BLE_BATTERY_SERVICE_UUID);
    pAdvertising->start();
    // register GaitGuide callbacks
    auto &gaitGuide = GaitGuide::getInstance();
    // gaitGuide.onCurrentPressureChanged([](uint16_t pressure)
    //                                   { ble_advertisePressure(pressure); });
    // esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000);
}