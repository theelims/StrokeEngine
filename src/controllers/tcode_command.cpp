#include "controllers/tcode.hpp"

#ifdef BLUETOOTH_USED

void TCodeCommandCharacteristicCallback::onWrite(BLECharacteristic *characteristic) {
  std::string msg = characteristic->getValue();
}

#endif