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

typedef struct {
  float start;
  float end;
  float keepout; /*  Soft endstop preventing hard crashes in mm. Will be 
                  *  subtracted twice from physicalTravel. Should be 
                  *  sufficiently to completley drive clear from 
                  *  homing switch */

  float length; // Bounded Space size
} MachineGeometry;

typedef struct {
  float start;
  float end;

  float length;
} EnvelopeGeometry;

enum class MotorState {
  // Low-level Power - Control Logic enabled, high-power can be enabled
  UNPOWERED = 0,

  // High-level Power - Only holding torque on motor, but the coils are being supplied current
  STOPPED = 1, 

  // Machine is in active motion and accepting motion commands
  HOMING = 2,
  RUNNING = 3, 

  // Error condition. Driver fault, voltage missing, communication lost, etc
  FAULT = 4
};

// Machine Space [geometry.start to geometry.end] 
//  - The physical extents of travel for the machine itself
// Bounded Space [0 to geometry.length] 
//  - The safe space within which a pattern can direct the motor to move
// Pattern Space [0 to envelope.length] 
//  - Each pattern runs in a subset of the BoundedSpace. 
//  - This allows StrokeEngine to translate the pattern around at-will whenever Depth is changed.
typedef float MachinePosition;
typedef float BoundedPosition;
typedef float EnvelopePosition;

class MotorInterface {
  public:
    // Motor Commands
    virtual void powerUp() {
      if (state == MotorState::UNPOWERED) {
        state = MotorState::STOPPED;
      ESP_LOGI("test", "STOPPED");
      }
    }

    virtual void powerDown() {
      state = MotorState::UNPOWERED;
      ESP_LOGI("test", "UNPOWERED");
    }

    // Allow motion commands. Motor will start moving. 
    // Transitions from STOPPED to RUNNING
    virtual void allowMotion() {
      if (state == MotorState::STOPPED) {
        state = MotorState::RUNNING;
      ESP_LOGI("test", "RUNNING");
      }
    }

    // Disallow motion commands, start homing sequence. 
    // Transitions from RUNNING to HOMING. Routine will run, then transition to RUNNING
    virtual void startHoming() {
      if (state == MotorState::RUNNING) {
        state = MotorState::HOMING;
      ESP_LOGI("test", "HOMING");
      }

      // Implementation will need to start homing task which will switch back to RUNNING state
    }

    // Disallow motion commands
    virtual void stopMotion() {
      if (state == MotorState::RUNNING || state == MotorState::HOMING) {
        state = MotorState::STOPPED;
      ESP_LOGI("test", "STOPPED");
      }
    }

    // Motor State
    virtual bool isUnpowered() {
      return state == MotorState::UNPOWERED;
    }
    virtual bool isStopped() {
      return state == MotorState::STOPPED;
    }
    virtual bool isRunning() {
      return state == MotorState::RUNNING;
    }
    virtual bool isHoming() {
      return state == MotorState::HOMING;
    }
    virtual bool hasFault() {
      return state == MotorState::FAULT;
    }

    // Fault Handling
    void getFault() { return; }
    void clearFault() { return; }

    // Motor Flags
    virtual bool isInMotion() { return false; }
    virtual bool isMotionCompleted() { return false; }
    
    // Safety Bounds
    void setMachineGeometry(MachineGeometry _geometry) {
      geometry = {
        .start = _geometry.start,
        .end = _geometry.end,
        .keepout = _geometry.keepout,
        .length = abs(_geometry.start - _geometry.end) - (_geometry.keepout * 2)
      };
    };
    void setMachineGeometry(float travel, float keepout = 5.0) {
      geometry = {
        .start = 0,
        .end = travel,
        .keepout = keepout,
        .length = travel
      };
    }
    MachineGeometry getMachineGeometry() { geometry; };

    void setMaxSpeed(float speed) { maxSpeed = speed; }
    float getMaxSpeed() { return maxSpeed; }

    void setMaxAcceleration(float acceleration) { maxAcceleration = acceleration; }
    float getMaxAcceleration() { return maxAcceleration; }

    // Motion
    void goToPosition(BoundedPosition position, float speed, float acceleration) {
      // Map Bounded Coordinate Space into Machine Coordinate Space

      // Apply bounds and protections
      float safePosition = constrain(
        position, 
        0, 
        geometry.length
      );
      float safeSpeed = constrain(speed, 0, maxSpeed);
      float safeAcceleration = constrain(acceleration, 1, maxAcceleration);
      
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
      if (!isMotionCompleted() && safeAcceleration < currentAcceleration) {
        ESP_LOGW("motor", "Clipped acceleration to prevent a crash! %05.1f was clipped to %05.1f", safeAcceleration, currentAcceleration);
        safeAcceleration = currentAcceleration;
      }

      currentAcceleration = safeAcceleration;

      // TODO - Add logging based on tags. Allows user to filter out these without filtering other debug messages
      ESP_LOGD("motor", "Going to position %05.1f mm @ %05.1f mm/s, %05.1f mm/s^2", safePosition, safeSpeed, safeAcceleration);
      unsafeGoToPos(toMachinePosition(safePosition), safeSpeed, safeAcceleration);
    }

  protected:
    MotorState state = MotorState::UNPOWERED;

    MachineGeometry geometry;
    float maxSpeed;
    float maxAcceleration;
    float currentAcceleration = 0;

    virtual void unsafeGoToPos(MachinePosition position, float speed, float acceleration) {}
    MachinePosition toMachinePosition(BoundedPosition position) {
      float safeStart = geometry.start;
      float safeEnd = geometry.end;
      float keepout = geometry.keepout;

      if (safeStart > safeEnd) {
        safeStart -= keepout;
        safeEnd += keepout;
      } else {
        safeStart += keepout;
        safeEnd -= keepout;
      }

      // (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
      float in_min = 0; float in_max = geometry.length;
      float out_min = safeStart; float out_max = safeEnd;

      // Maps from 
      return (position - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif