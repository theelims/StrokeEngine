#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "pattern.h"

typedef enum {
  INIT = (1 << 0),
  REQUEST_EXIT = (1 << 1),
  REQUEST_NEXT = (1 << 2),
  MOTION_COMPLETED = (1 << 3)
} MotionTaskEvent;

class MotorInterface;
class MotionTaskInterface {
  friend class MotorInterface;
  protected:
    MotorInterface *motor;
    
    uint16_t _motion_task_event_bitfield = 0;
    static void _motion_task_impl(void* _this) { static_cast<MotionTaskInterface*>(_this)->_motion_task(); }
    void _motion_task();
    TaskHandle_t _motionTaskHandle = NULL;

    virtual void motion_task_init();
    virtual void motion_task_event(MotionTaskEvent event);
    virtual void motion_task_exit();

    void startMotionTask() {
      if (this->_motionTaskHandle == NULL) {
        // Create Stroke Task
        xTaskCreatePinnedToCore(
          this->_motion_task_impl,    // Function that should be called
          "Motion Task",             // Name of the task (for debugging)
          4096,                   // Stack size (bytes)
          this,                   // Pass reference to this class instance
          24,                     // Pretty high task priority
          &this->_motionTaskHandle,   // Task handle
          1                       // Pin to application core
        );

        this->notifyMotionTask(MotionTaskEvent::INIT);
      }
    };

    void notifyMotionTask(MotionTaskEvent event) {
      if (this->_motionTaskHandle != NULL) {
        this->_motion_task_event_bitfield |= static_cast<uint16_t>(event);
        xTaskNotifyGive(this->_motionTaskHandle);
      }
    };

    void exitMotionTask() {
      this->notifyMotionTask(MotionTaskEvent::REQUEST_EXIT);
    }
};