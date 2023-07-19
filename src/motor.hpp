#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
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

#define DEBUG_CLIPPING              // Show debug messages when motions violating the machine 
                                    // physics are commanded

typedef struct {
  float start;
  float end;
  float keepout; /*  Soft endstop preventing hard crashes in mm. Will be 
                  *  subtracted twice from physicalTravel. Should be 
                  *  sufficiently to completley drive clear from 
                  *  homing switch */
} MachineGeometry;

#define MOTOR_FLAG_ENABLED (1 << 0)

#define MOTOR_FLAG_WARNING (1 << 1)
#define MOTOR_FLAG_ERROR (1 << 2)
#define MOTOR_FLAG_FATAL (1 << 3)

#define MOTOR_FLAG_AT_TARGET (1 << 4)
#define MOTOR_FLAG_MOTION_ACTIVE (1 << 5)
#define MOTOR_FLAG_HOMED (1 << 6)

typedef enum {
  INACTIVE = 0,
  ACTIVE = 1,
  HOMING = 2, 
  ERROR = 3
} MotorState;

class MotorInterface {
  public:
    void enable() { this->addStatusFlag(MOTOR_FLAG_ENABLED); }
    // TODO - virtual void disable();
     
    // Safety Bounds
    void setMachineGeometry(MachineGeometry geometry) {
      this->geometry = geometry;
      this->maxPosition = abs(geometry.start - geometry.end) - (geometry.keepout * 2);
    };
    MachineGeometry getMachineGeometry() { this->geometry; };

    // Max Position cannot be set directly, but is computed from Machine Limits
    float getMaxPosition() { return this->maxPosition; }

    void setMaxSpeed(float speed) { this->maxSpeed = speed; }
    float getMaxSpeed() { return this->maxSpeed; }

    void setMaxAcceleration(float acceleration) { this->maxAcceleration = acceleration; }
    float getMaxAcceleration() { return this->maxAcceleration; }

    // Motion
    virtual void goToHome();
    void goToPos(float position, float speed, float acceleration) {
      // Map Bounded Coordinate Space into Machine Coordinate Space

      // Apply bounds and protections
      float safePosition = constrain(
        position, 
        0, 
        this->maxPosition
      );
      float safeSpeed = constrain(speed, 0, this->maxSpeed);
      float safeAcceleration = constrain(acceleration, 1, this->maxAcceleration);
      
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
      if (!this->isMotionCompleted() && safeAcceleration > this->currentAcceleration) {
        ESP_LOGW("motor", "Clipped acceleration to prevent a crash! %05.1f was clipped to %05.1f", safeAcceleration, this->currentAcceleration);
        safeAcceleration = this->currentAcceleration;
      }

      this->currentAcceleration = safeAcceleration;

      // TODO - Add logging based on tags. Allows user to filter out these without filtering other debug messages
      ESP_LOGD("motor", "Going to position %05.1f mm @ %05.1f mm/s, %05.1f mm/s^2", safePosition, safeSpeed, safeAcceleration);
      this->unsafeGoToPos(safePosition, safeSpeed, safeAcceleration);
    }
    virtual void stopMotion();
    bool isMotionCompleted() { return this->hasStatusFlag(MOTOR_FLAG_AT_TARGET) && !this->hasStatusFlag(MOTOR_FLAG_MOTION_ACTIVE); }

    // Status / State flags
    MotorState getState() { return this->state; }
    const char* getStateString() {
      if (this->state == MotorState::INACTIVE) { return "INACTIVE"; }
      if (this->state == MotorState::ACTIVE) { return "ACTIVE"; }
      if (this->state == MotorState::HOMING) { return "HOMING"; }
      if (this->state == MotorState::ERROR) { return "ERROR"; }

      return "UNKNOWN";
    }
    uint32_t getStatus() { return this->status; }
    bool isInState(MotorState state) { return this->state == state; }
    bool hasStatusFlag(uint32_t flag) { return (this->status & flag) > 0; }

  protected:
    MotorState state = MotorState::INACTIVE;
    uint32_t status = 0;

    void addStatusFlag(uint32_t flag) { this->status |= flag; }
    void removeStatusFlag(uint32_t flag) { this->status &= ~flag; }

    MachineGeometry geometry;
    float maxPosition;
    float maxSpeed;
    float maxAcceleration;
    float currentAcceleration = 0;

    virtual void unsafeGoToPos(float position, float speed, float acceleration);
    float mapSafePosition(float position) {
      float safeStart = this->geometry.start;
      float safeEnd = this->geometry.end;
      float keepout = this->geometry.keepout;

      if (safeStart > safeEnd) {
        safeStart -= keepout;
        safeEnd += keepout;
      } else {
        safeStart += keepout;
        safeEnd -= keepout;
      }

      // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
      float in_min = 0; float in_max = this->maxPosition;
      float out_min = safeStart; float out_max = safeEnd;

      return (position - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif