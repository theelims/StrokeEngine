#include "MotionTask.hpp"
#include "motor.hpp"

void MotionTaskInterface::_motion_task() {
  try {
    ESP_LOGI("Motion Task", "Starting...");
    this->motor->lockTask(this);

    // Stop current move, should one be pending (moveToMax or moveToMin)
    if (this->motor->hasStatusFlag(MOTOR_FLAG_MOTION_ACTIVE)) {
      this->motor->motionInterrupt();
    }

    this->_motion_task_event_bitfield |= static_cast<uint16_t>(MotionTaskEvent::INIT);
    this->motion_task_init();

    while(1) { // infinite loop
        if (!this->motor->isInState(MotorState::ACTIVE)) {
          ESP_LOGI("Motion Task", "Motor is no longer active! Detaching Motion Task");
          this->motor->unlockTask();
          this->motion_task_exit();
          vTaskDelete(NULL);
        }

        // Delay 10ms while waiting for an event from the Motor
        xTaskNotifyWait(0, 0, NULL, 10 / portTICK_PERIOD_MS);
        if (this->_motion_task_event_bitfield && static_cast<uint16_t>(MotionTaskEvent::INIT)) {
          this->motion_task_event(MotionTaskEvent::INIT);
          this->_motion_task_event_bitfield &= ~(static_cast<uint16_t>(MotionTaskEvent::INIT));
        } else if (this->_motion_task_event_bitfield && static_cast<uint16_t>(MotionTaskEvent::REQUEST_EXIT)) {
          ESP_LOGI("Motion Task", "Exit Requested...");
          this->motor->unlockTask();
          this->motion_task_exit();
          vTaskDelete(NULL);
        } else if (this->_motion_task_event_bitfield && static_cast<uint16_t>(MotionTaskEvent::REQUEST_NEXT)) {
          this->motion_task_event(MotionTaskEvent::REQUEST_NEXT);
          this->_motion_task_event_bitfield &= ~(static_cast<uint16_t>(MotionTaskEvent::REQUEST_NEXT));
        } else if (this->_motion_task_event_bitfield && static_cast<uint16_t>(MotionTaskEvent::MOTION_COMPLETED)) {
          this->motion_task_event(MotionTaskEvent::MOTION_COMPLETED);
          this->_motion_task_event_bitfield &= ~(static_cast<uint16_t>(MotionTaskEvent::MOTION_COMPLETED));
        }
    }
  } catch (std::exception ex) {
    ESP_LOGE("Motion Task", "Exception caused early exit! %s", ex.what());
    this->motor->unlockTask();
    this->motion_task_exit();
    vTaskDelete(NULL);
  }
};