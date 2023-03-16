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
    NimBLEService *diagService = pServer->createService((uint16_t)BLE_LRA_DIAG_SERVICE_UUID);
    pServer->setCallbacks(new ServerCallbacks);
    // NimBLECharacteristic *pNonSecureCharacteristic = pService->createCharacteristic("1234", NIMBLE_PROPERTY::READ );
    // NimBLECharacteristic *pSecureCharacteristic = pService->createCharacteristic("1235", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN);

    NimBLECharacteristic *BLE_Mode = controlService->createCharacteristic((uint16_t)BLE_MODE_CHARACTERISTIC_UUID,
                                                                          NIMBLE_PROPERTY::READ |
                                                                              NIMBLE_PROPERTY::WRITE,
                                                                          // NIMBLE_PROPERTY::READ_ENC |
                                                                          // NIMBLE_PROPERTY::WRITE_ENC,
                                                                          4);
    BLE_Mode->setCallbacks(&chrCallbacks);
    BLE_Mode->setValue(0x0000);
    NimBLECharacteristic *BLE_Effects = controlService->createCharacteristic((uint16_t)BLE_EFFECTS_CHARACTERISTIC_UUID,
                                                                             NIMBLE_PROPERTY::READ |
                                                                                 NIMBLE_PROPERTY::WRITE,
                                                                             // NIMBLE_PROPERTY::READ_ENC |
                                                                             // NIMBLE_PROPERTY::WRITE_ENC,
                                                                             4);
    BLE_Effects->setCallbacks(&chrCallbacks);
    BLE_Effects->setValue(0x0000);
    NimBLECharacteristic *BLE_Command = controlService->createCharacteristic((uint16_t)BLE_COMMAND_CHARACTERISTIC_UUID,
                                                                             NIMBLE_PROPERTY::READ |
                                                                                 NIMBLE_PROPERTY::WRITE,
                                                                             //  NIMBLE_PROPERTY::READ_ENC |
                                                                             //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                             4);
    BLE_Command->setCallbacks(&chrCallbacks);
    BLE_Command->setValue(0x0000);
    NimBLECharacteristic *BLE_MED_CON = diagService->createCharacteristic((uint16_t)BLE_MED_CONNECTED_CHARACTERISTIC_UUID,
                                                                          NIMBLE_PROPERTY::READ |
                                                                              NIMBLE_PROPERTY::WRITE,
                                                                          // NIMBLE_PROPERTY::READ_ENC |
                                                                          // NIMBLE_PROPERTY::WRITE_ENC,
                                                                          4);

    BLE_Mode->setCallbacks(&chrCallbacks);
    BLE_Mode->setValue(0x0000);
    NimBLECharacteristic *BLE_Pressure = diagService->createCharacteristic((uint16_t)BLE_PRESSURE_CHARACTERISTIC_UUID,
                                                                           NIMBLE_PROPERTY::READ |
                                                                               NIMBLE_PROPERTY::WRITE,
                                                                           // NIMBLE_PROPERTY::READ_ENC |
                                                                           // NIMBLE_PROPERTY::WRITE_ENC,
                                                                           4);

    BLE_Pressure->setCallbacks(&chrCallbacks);
    BLE_Pressure->setValue(0x0000);
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
    diagService->start();
    // pNonSecureCharacteristic->setValue("Hello Non Secure BLE");
    // pSecureCharacteristic->setValue("Hello Secure BLE");
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID((uint16_t)BLE_LRA_CONTROL_SERVICE_UUID); // TODO
    pAdvertising->addServiceUUID((uint16_t)BLE_LRA_DIAG_SERVICE_UUID);    // TODO
    pAdvertising->start();

// register GaitGuide callbacks
auto &gaitGuide = GaitGuide::getInstance();
gaitGuide.onCurrentPressureChanged([](uint16_t pressure) {
    ble_advertisePressure(pressure);
});
    



    // esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000);
}

void ble_advertisePressure(uint16_t pressure)
{
    NimBLEServer *pserver = NimBLEDevice::getServer();
    NimBLEService *pService = pserver->getServiceByUUID((uint16_t)BLE_LRA_DIAG_SERVICE_UUID);
    NimBLECharacteristic *pCharacteristic = pService->getCharacteristic((uint16_t)BLE_PRESSURE_CHARACTERISTIC_UUID);
    pCharacteristic->setValue(pressure);
}