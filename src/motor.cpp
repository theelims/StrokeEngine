#include "motor.hpp"
/*
void MotorInterface::ready() { 
  if (this->state == MotorState::INACTIVE) {
    if (this->checkCanReady()) {
      this->state = MotorState::READY;
    }
  } else {
    ESP_LOGE("motor", "Attempted to ready motor while not in INACTIVE state! state: %s", this->getStateString());
  }
}
void MotorInterface::deactivate() {}

void MotorInterface::energize() {
  if (this->state == MotorState::READY) {

  } else {
    ESP_LOGE("motor", "Attempted to energize motor while not in READY state! state: %s", this->getStateString());
  }
}
void MotorInterface::deenergize() {
  if (this->state == MotorState::ACTIVE) { // TODO - Can be de-energize from ERROR state? Or is that already de-energized?

  } else {
    ESP_LOGE("motor", "Attempted to de-energize motor while not in ACTIVE/ERROR state! state: %s", this->getStateString());
  }
}

bool MotorInterface::checkCanReady() {
  if (this->geometry == NULL) {
    ESP_LOGE("motor", "Attempted to ready motor, but setMachineGeometry has not been called!",);
    return;
  }
  
  if (this->maxSpeed > 0) {
    ESP_LOGE("motor", "Attempted to ready motor, but setMaxSpeed has not been called!",);
    return;
  }
  
  if (this->maxAcceleration > 0) {
    ESP_LOGE("motor", "Attempted to ready motor, but setMaxAcceleration has not been called!",);
    return;
  }
}*/