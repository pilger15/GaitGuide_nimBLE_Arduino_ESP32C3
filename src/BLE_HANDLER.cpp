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
    BLE_Amplitude->setValue(0x00);
    NimBLECharacteristic *BLE_Duration_MED = controlService->createCharacteristic((uint16_t)BLE_DURATION_MED_CHARACTERISTIC_UUID,
                                                                                  NIMBLE_PROPERTY::READ |
                                                                                      NIMBLE_PROPERTY::WRITE,
                                                                                  //  NIMBLE_PROPERTY::READ_ENC |
                                                                                  //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                                  1);
    BLE_Duration_MED->setCallbacks(&chrCallbacks);
    BLE_Duration_MED->setValue(0x00);

    NimBLECharacteristic *BLE_Duration_LAT = controlService->createCharacteristic((uint16_t)BLE_DURATION_LAT_CHARACTERISTIC_UUID,
                                                                                  NIMBLE_PROPERTY::READ |
                                                                                      NIMBLE_PROPERTY::WRITE,
                                                                                  //  NIMBLE_PROPERTY::READ_ENC |
                                                                                  //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                                  1);
    BLE_Duration_LAT->setCallbacks(&chrCallbacks);
    BLE_Duration_LAT->setValue(0x00);

    /*NimBLECharacteristic *BLE_Mode = controlService->createCharacteristic((uint16_t)BLE_STIM_MODE_CHARACTERISTIC_UUID,
                                                                          NIMBLE_PROPERTY::READ |
                                                                              NIMBLE_PROPERTY::WRITE,
                                                                          // NIMBLE_PROPERTY::READ_ENC |
                                                                          // NIMBLE_PROPERTY::WRITE_ENC,
                                                                          4);
    BLE_Mode->setCallbacks(&chrCallbacks);
    BLE_Mode->setValue(0x0000);
    NimBLECharacteristic *BLE_Control = controlService->createCharacteristic((uint16_t)BLE_CONTROL_CHARACTERISTIC_UUID,
                                                                             NIMBLE_PROPERTY::READ |
                                                                                 NIMBLE_PROPERTY::WRITE,
                                                                             // NIMBLE_PROPERTY::READ_ENC |
                                                                             // NIMBLE_PROPERTY::WRITE_ENC,
                                                                             1);
    BLE_Control->setCallbacks(&chrCallbacks);
    BLE_Control->setValue(0x0);
    NimBLECharacteristic *BLE_Command = controlService->createCharacteristic((uint16_t)BLE_STIM_COMMAND_CHARACTERISTIC_UUID,
                                                                             NIMBLE_PROPERTY::READ |
                                                                                 NIMBLE_PROPERTY::WRITE,
                                                                             //  NIMBLE_PROPERTY::READ_ENC |
                                                                             //  NIMBLE_PROPERTY::WRITE_ENC,
                                                                             1);
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
    */
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
    // diagService->start();
    //  pNonSecureCharacteristic->setValue("Hello Non Secure BLE");
    //  pSecureCharacteristic->setValue("Hello Secure BLE");
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID((uint16_t)BLE_LRA_CONTROL_SERVICE_UUID); // TODO
    // pAdvertising->addServiceUUID((uint16_t)BLE_LRA_DIAG_SERVICE_UUID);    // TODO
    pAdvertising->start();
    // register GaitGuide callbacks
    auto &gaitGuide = GaitGuide::getInstance();
    // gaitGuide.onCurrentPressureChanged([](uint16_t pressure)
    //                                    { ble_advertisePressure(pressure); });
    // esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000);
}
/// @brief NOT IMPLEMENTED
/// @param pressure
void ble_advertisePressure(uint16_t pressure)
{
    NimBLEServer *pserver = NimBLEDevice::getServer();
    NimBLEService *pService = pserver->getServiceByUUID((uint16_t)BLE_LRA_DIAG_SERVICE_UUID);
    NimBLECharacteristic *pCharacteristic = pService->getCharacteristic((uint16_t)BLE_PRESSURE_CHARACTERISTIC_UUID);
    pCharacteristic->setValue(pressure);
}