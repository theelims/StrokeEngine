#pragma once

#include "Arduino.h"
#include <exception.h>
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

typedef struct {
  float start;
  float end;
  float keepout; /*  Soft endstop preventing hard crashes in mm. Will be 
                  *  subtracted twice from physicalTravel. Should be 
                  *  sufficiently to completely drive clear from 
                  *  homing switch */
} MachineGeometry;

#define MOVEMENT_TASK_FLAG_EXIT (1 << 0)
#define MOVEMENT_TASK_FLAG_NEXT (1 << 1)

// TODO - Change flags into a FreeRTOS Event Group which allows better synchronizing on Flag State
// TODO - Clean up Flags, as only ENABLED and HOMED are crucial. Move the rest on level lower (derived class only?)
#define MOTOR_FLAG_ENABLED (1 << 0)
#define MOTOR_FLAG_HOMED (1 << 1)

// Not relevant for basic stepper
#define MOTOR_FLAG_WARNING (1 << 2)
#define MOTOR_FLAG_ERROR (1 << 3)
#define MOTOR_FLAG_FATAL (1 << 4)

#define MOTOR_FLAG_AT_TARGET (1 << 5)
#define MOTOR_FLAG_MOTION_ACTIVE (1 << 6)

#define MOTOR_FLAG_OVERHEATING (1 << 7)
#define MOTOR_FLAG_OVERFORCE (1 << 8)

class MotorException extends std::exception { }

class MotorInterface {
  public:
    virtual void enable() { addStatusFlag(MOTOR_FLAG_ENABLED); }
    virtual void disable() { removeStatusFlag(MOTOR_FLAG_ENABLED); }

    SemaphoreHandle_t claimMotorControl() {
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
    }
     
    // Safety Bounds
    void setMachineGeometry(MachineGeometry geometry) {
      geometry = geometry;
      maxPosition = abs(geometry.start - geometry.end) - (geometry.keepout * 2);
    };

    MachineGeometry getMachineGeometry() { geometry; };

    // Max Position cannot be set directly, but is computed from Machine Limits
    float getMaxPosition() { return maxPosition; }

    void setMaxSpeed(float speed) { maxSpeed = speed; }
    float getMaxSpeed() { return maxSpeed; }

    void setMaxAcceleration(float acceleration) { maxAcceleration = acceleration; }
    float getMaxAcceleration() { return maxAcceleration; }

    // Motion
    virtual void goToHome();
    void goToPos(float position, float speed, float acceleration) {
      // TODO - If a motion task is provided, ensure the caller is the motion task (Mutex?)
      // Ensure in ACTIVE and valid movement state
      if (!isInState(MotorState::ACTIVE)) {
        throw new MotorInvalidStateError("Unable to command motion while motor is not in ACTIVE state!");
      }

      // Take Semaphore for movement
      if( xSemaphoreTake( movementSemaphore, 1000 / portTICK_PERIOD_MS ) != pdTRUE ) {
        throw new MotorInMotionError("Unable to acquire Motor Movement Semaphore within 1000ms. Motor currently within movement still!");
      }

      // Apply bounds and protections
      float safePosition = constrain(position, 0, maxPosition);
      float safeSpeed = constrain(speed, 0, maxSpeed);
      float safeAcceleration = constrain(acceleration, 0, maxAcceleration);
      
      if (safePosition != position) {
        ESP_LOGW("motor", "Clipped position to fit within bounds! %05.1f was clipped to %05.1f", position, safePosition);
      }

      if (safeSpeed != speed) {
        ESP_LOGW("motor", "Clipped speed to fit within bounds! %05.1f was clipped to %05.1f", speed, safeSpeed);
      }

      if (safeAcceleration != acceleration) {
        ESP_LOGW("motor", "Clipped acceleration to fit within bounds! %05.1f was clipped to %05.1f", acceleration, safeAcceleration);
      }

      // Acceleration cannot be lowered, only increased, unless the current motion command has finished executing
      // This prevents putting the system into a state where the acceleration is too low to come to a stop before a crash occurs
      if (!motionCompleted() && safeAcceleration > currentAcceleration) {
        ESP_LOGW("motor", "Clipped acceleration to prevent a crash! %05.1f was clipped to %05.1f", safeAcceleration, currentAcceleration);
        safeAcceleration = currentAcceleration;
      }

      currentAcceleration = safeAcceleration;

      // TODO - Add logging based on tags. Allows user to filter out these without filtering other debug messages
      ESP_LOGD("motor", "Going to position %05.1f mm @ %05.1f m/s, %05.1f m/s^2", safePosition, safeSpeed, safeAcceleration);
      unsafeGoToPos(safePosition, safeSpeed, safeAcceleration);
    }

    virtual void stopMotion();

    virtual void motionCompleted();

    uint32_t getStatus() { return status; }
    bool hasStatusFlag(uint32_t flag) { return (status & flag) > 0; }

  protected:
    uint32_t status = 0;

    //TaskHandle_t motionTask; // Task which will provide movement commands to Motor
    SemaphoreHandle_t taskSemaphore; // Prevent more than one task being interfaced to a motor at a time
    SemaphoreHandle_t movementSemaphore; // Prevent additional movement commands while in motion

    void addStatusFlag(uint32_t flag) { status |= flag; }
    void removeStatusFlag(uint32_t flag) { status &= ~flag; }

    MachineGeometry geometry;
    float maxPosition;
    float maxSpeed;
    float maxAcceleration;

    float currentAcceleration = 0;

    virtual void unsafeGoToPos(float position, float speed, float acceleration);

};
