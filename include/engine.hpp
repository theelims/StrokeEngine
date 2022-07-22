#pragma once

#include <motor.hpp>

class Engine {
  public:
    void attachMotor(MotorInterface *motor) {
      this->motor = motor;
      ESP_LOGI("Engine", "Attached Motor succesfully to Engine!");
    }
    void detachMotor() {
      this->stopMotion();
      this->motor = NULL;
    }

    bool isActive() { return this->active; }
  protected:
    bool active = false;
    MotorInterface *motor;

    void startMotion() {
      if (_taskStrokingHandle == NULL) {
        // Create Stroke Task
        xTaskCreatePinnedToCore(
          this->_strokingImpl,    // Function that should be called
          "Stroking",             // Name of the task (for debugging)
          4096,                   // Stack size (bytes)
          this,                   // Pass reference to this class instance
          24,                     // Pretty high task priority
          &_taskStrokingHandle,   // Task handle
          1                       // Pin to application core
        ); 
      } else {
        // Resume task, if it already exists
        vTaskResume(_taskStrokingHandle);
      }
      this->active = true;
    }

    void stopMotion() {
      if (_taskStrokingHandle != NULL) {
        vTaskSuspend(_taskStrokingHandle);
      }
      this->active = false;
      this->motor->stopMotion();
    }

    static void _strokingImpl(void* _this) { static_cast<Engine*>(_this)->_stroking(); }
    virtual void _stroking();
    TaskHandle_t _taskStrokingHandle = NULL;
};