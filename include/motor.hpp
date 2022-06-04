#ifndef MOTOR_H
#define MOTOR_H

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

// If start > end, indicates an inversion of direction
typedef struct {
  float start;
  float end;
  float keepout; /*  Soft endstop preventing hard crashes in mm. Will be 
                  *  subtracted twice from physicalTravel. Should be 
                  *  sufficiently to completley drive clear from 
                  *  homing switch */
} motionBounds;

#define MOTOR_FLAG_ENABLED (1 << 0)

#define MOTOR_FLAG_WARNING (1 << 1)
#define MOTOR_FLAG_ERROR (1 << 2)
#define MOTOR_FLAG_FATAL (1 << 3)

#define MOTOR_FLAG_AT_TARGET (1 << 4)
#define MOTOR_FLAG_MOTION_ACTIVE (1 << 5)
#define MOTOR_FLAG_HOMED (1 << 6)

#define MOTOR_FLAG_OVERHEATING (1 << 7)
#define MOTOR_FLAG_OVERFORCE (1 << 8)

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
    void setMaxSpeed(float speed) { this->maxSpeed = speed; }
    float getMaxSpeed() { return this->maxSpeed; }

    void setMaxAcceleration(float acceleration) { this->maxAcceleration = acceleration; }
    float getMaxAcceleration() { return this->maxAcceleration; }

    void setBounds(motionBounds bounds) { this->bounds = bounds; }
    motionBounds getBounds() { return this->bounds; }

    // Motion
    virtual void goToHome();
    void goToPos(float position, float speed, float acceleration) {
      // TODO - Build protections here - Compute basic trajectory and see if a crash happens
      // Should be able to cut and re-compute a position/speed/acceleration that won't crash then
      this->unsafeGoToPos(position, speed, acceleration);
    }
    virtual void stopMotion();
    bool isMotionCompleted() { return this->hasStatusFlag(MOTOR_FLAG_AT_TARGET) && !this->hasStatusFlag(MOTOR_FLAG_MOTION_ACTIVE); }

    // Status / State flags
    MotorState getState() { return this->state; }
    char* getStateString() {
      if (this->state == MotorState::INACTIVE) { return "INACTIVE"; }
      if (this->state == MotorState::ACTIVE) { return "ACTIVE"; }
      if (this->state == MotorState::HOMING) { return "HOMING"; }
      if (this->state == MotorState::ERROR) { return "ERROR"; }
    }
    uint32_t getStatus() { return this->status; }
    bool isInState(MotorState state) { return this->state == state; }
    bool hasStatusFlag(uint32_t flag) { return (this->status & flag) > 0; }

  protected:
    MotorState state = MotorState::INACTIVE;
    uint32_t status = 0;

    void addStatusFlag(uint32_t flag) { this->status |= flag; }
    void removeStatusFlag(uint32_t flag) { this->status &= ~flag; }

    motionBounds bounds;
    float maxSpeed;
    float maxAcceleration;

    virtual void unsafeGoToPos(float position, float speed, float acceleration);
};

#endif