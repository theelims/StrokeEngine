#pragma once

#include "Arduino.h"
//#include <exception.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//#include "freertos/semphr.h"

/**
 *  Motor
 *   
 *  position = mm
 *  speed = mm/s
 *  acceleration = mm/s^2
 */


//class MotorException extends std::exception { }

class MotorInterface {
  public:

    // Control
    virtual void enable();
    virtual void disable();
    virtual bool isEnabled() { return _enabled; } 
    virtual void home();
    virtual bool isHomed() { return _homed; }
    virtual bool isActive() { return (_enabled && _homed); }

/*     SemaphoreHandle_t claimMotorControl() {
      if (taskSemaphore == null) {
        taskSemaphore = xSemaphoreCreateBinary();
      }

      if( xSemaphoreTake( taskSemaphore, 100 / portTICK_PERIOD_MS ) != pdTRUE ) {
        throw new MotorBusyError("Unable to attach a new Motion Task, as one is already active!");
      }

      motionTask = xGetCurrentTaskHandle();
    }
    void releaseMotorControl(SemaphoreHandle_t semaphore) {
      if (semaphore != taskSemaphore) {
        throw new MotorGenericError("Attempted to release Semaphore using invalid handle!");
      }

      xSemaphoreGive(taskSemaphore);
      motionTask = null;
    } */
     
    // Safety Bounds
    virtual void setMachineGeometry(float start, float end, float keepout) {
      _start = start;
      _end = end;
      _keepout = keepout;
      _maxPosition = abs(start - end) - (keepout * 2);
    };

    // Max Position cannot be set directly, but is computed from Machine Limits
    float getMaxPosition() { return _maxPosition; }

    virtual void setMaxSpeed(float speed) { _maxSpeed = speed; }
    float getMaxSpeed() { return _maxSpeed; }

    virtual void setMaxAcceleration(float acceleration) { _maxAcceleration = acceleration; }
    float getMaxAcceleration() { return _maxAcceleration; }

    // Motion
    void goToPos(float position, float speed, float acceleration) {
      // TODO - If a motion task is provided, ensure the caller is the motion task (Mutex?)
      // Ensure in ACTIVE and valid movement state
      if (!_enabled || !_homed) {
        //throw new MotorInvalidStateError("Unable to command motion while motor is not ENABLED or HOMED!");
      }

/*       // Take Semaphore for movement
      if( xSemaphoreTake( movementSemaphore, 1000 / portTICK_PERIOD_MS ) != pdTRUE ) {
        throw new MotorInMotionError("Unable to acquire Motor Movement Semaphore within 1000ms. Motor currently within movement still!");
      } */

      // Apply bounds and protections
      float safePosition = constrain(position, 0, _maxPosition);
      float safeSpeed = constrain(speed, 0, _maxSpeed);
      float safeAcceleration = constrain(acceleration, 0, _maxAcceleration);
      
      if (safePosition != position) {
        ESP_LOGW("motor", "Clipped position to fit within bounds! %05.1f was clipped to %05.1f", position, safePosition);
      }

      if (safeSpeed != speed) {
        ESP_LOGW("motor", "Clipped speed to fit within bounds! %05.1f was clipped to %05.1f", speed, safeSpeed);
      }

      if (safeAcceleration != acceleration) {
        ESP_LOGW("motor", "Clipped acceleration to fit within bounds! %05.1f was clipped to %05.1f", acceleration, safeAcceleration);
      }

      // TODO - Add logging based on tags. Allows user to filter out these without filtering other debug messages
      ESP_LOGD("motor", "Going to position %05.1f mm @ %05.1f m/s, %05.1f m/s^2", safePosition, safeSpeed, safeAcceleration);
      _unsafeGoToPos(safePosition, safeSpeed, safeAcceleration);
    }

    virtual void stopMotion();
    virtual bool motionCompleted();

  protected:
    bool _enabled = false;
    bool _homed = false;

    //TaskHandle_t motionTask; // Task which will provide movement commands to Motor
    SemaphoreHandle_t taskSemaphore; // Prevent more than one task being interfaced to a motor at a time
    SemaphoreHandle_t movementSemaphore; // Prevent additional movement commands while in motion

    virtual void _unsafeGoToPos(float position, float speed, float acceleration);

    float _start;
    float _end;
    float _keepout;
    float _maxPosition;
    float _maxSpeed;
    float _maxAcceleration;

};
