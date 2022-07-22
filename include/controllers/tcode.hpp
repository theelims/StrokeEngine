#pragma once

#include "sdkconfig.h"

#ifdef BLUETOOTH_USED

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLE2902.h"
#include "BLE2904.h"

#include "controller.hpp"
#include "engines/stroke.hpp"

const char* FIRMWARE_VERSION = "v1.1";
#define SERVICE_UUID                "e556ec25-6a2d-436f-a43d-82eab88dcefd"

#define CONTROL_CHARACTERISTIC_UUID "c4bee434-ae8f-4e67-a741-0607141f185b"
// WRITE
// T-Code messages in the format:
// ex. L199I100 = move linear actuator to the 99% position over 100ms
// ex. L10I100 = move linear actuator to the 0% position over 100ms
// DSTOP = stop
// DENABLE = enable motor (non-standard T-Code command)
// DDISABLE = disable motor (non-standard T-Code command)

#define SETTINGS_CHARACTERISTIC_UUID "fe9a02ab-2713-40ef-a677-1716f2c03bad"
// WRITE
// Preferences in the format:
// minSpeed:200 = set minimum speed of half-stroke to 200ms (used by XToys client)
// maxSpeed:2000 = set maximum speed of half-stroke to 2000ms (used by XToys client)
// maxOut:0 = set the position the stepper should stop at when moving in the one direction
// maxIn:1000 = set the position the stepper should stop at when moving in the other direction
// READ
// Returns all preference values in the format:
// minSpeed:200,maxSpeed:2000,maxOut:0,maxIn:1000

class TCodeSettingsCharacteristicCallback;
class TCodeCommandCharacteristicCallback;
class TCodeServerCallback;

class TCodeController : public Controller {
  public:
    TCodeController();
    void loop();
    void processCommand(String msg);
  
  protected:
    BLEServer *pServer;
    BLEService *pService;
    BLECharacteristic *controlCharacteristic;
    BLECharacteristic *settingsCharacteristic;
    BLEService *infoService;
    BLECharacteristic *softwareVersionCharacteristic;

    TCodeSettingsCharacteristicCallback* settingsCallback;
    TCodeCommandCharacteristicCallback* commandCallback;
    TCodeServerCallback* serverCallback;

    void taskBLEConnect(void *pvParameters);
};

class TCodeSettingsCharacteristicCallback : public BLECharacteristicCallbacks {
  public:
    TCodeSettingsCharacteristicCallback(TCodeController* controller) : controller(controller) {};
    void onWrite(BLECharacteristic *characteristic);
  
  private:
    TCodeController* controller;
};

class TCodeCommandCharacteristicCallback : public BLECharacteristicCallbacks {
  public:
    TCodeCommandCharacteristicCallback(TCodeController* controller) : controller(controller) {};
    void onWrite(BLECharacteristic *characteristic);
  
  private:
    TCodeController* controller;
};

class TCodeServerCallback : public BLEServerCallbacks {
  public:
    TCodeServerCallback(TCodeController* controller) : controller(controller) {};
    void onConnect(BLEServer* pServerm, esp_ble_gatts_cb_param_t *param);
    void onDisconnect(BLEServer* pServer);
  
  private:
    TCodeController* controller;
};

#endif