#include "controllers/tcode.hpp"

#ifdef BLUETOOTH_USED

void TCodeController::taskBLEConnect(void *pvParameters){
  UBaseType_t uxHighWaterMark;

  ESP_LOGI("", "Initializing BLE Server...");
  BLEDevice::init("OSSM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new TCodeServerCallback(this));
  
  infoService = pServer->createService(BLEUUID((uint16_t) 0x180a));
  BLE2904* softwareVersionDescriptor = new BLE2904();
  softwareVersionDescriptor->setFormat(BLE2904::FORMAT_UINT8);
  softwareVersionDescriptor->setNamespace(1);
  softwareVersionDescriptor->setUnit(0x27ad);

  softwareVersionCharacteristic = infoService->createCharacteristic((uint16_t) 0x2a28, BLECharacteristic::PROPERTY_READ);
  softwareVersionCharacteristic->addDescriptor(softwareVersionDescriptor);
  softwareVersionCharacteristic->addDescriptor(new BLE2902());
  softwareVersionCharacteristic->setValue(FIRMWARE_VERSION);
  infoService->start();
  
  pService = pServer->createService(SERVICE_UUID);
  controlCharacteristic = pService->createCharacteristic(
                                         CONTROL_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  controlCharacteristic->addDescriptor(new BLE2902());
  controlCharacteristic->setValue("");
  controlCharacteristic->setCallbacks(new TCodeCommandCharacteristicCallback(this));
  
  settingsCharacteristic = pService->createCharacteristic(
                                         SETTINGS_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  settingsCharacteristic->addDescriptor(new BLE2902());
  settingsCharacteristic->setValue("");
  settingsCharacteristic->setCallbacks(new TCodeSettingsCharacteristicCallback(this));

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  // TODO - Figure this out - updateSettingsCharacteristic();
  // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  // LogDebugFormatted("Ble Free Stack size %ld \n", static_cast<long int>(uxHighWaterMark));
  vTaskDelete(NULL);
}

#endif