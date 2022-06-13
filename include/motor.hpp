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

#define MOVEMENT_TASK_FLAG_EXIT (1 << 0)
#define MOVEMENT_TASK_FLAG_MOTION_COMPLETED (1 << 1)
#define MOVEMENT_TASK_FLAG_NEXT (1 << 2)

// TODO - Change flags into a FreeRTOS Event Group which allows better synchronizing on Flag State
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

typedef enum {
  // Retraction Emergency useful for Throat play, where a quick retraction to a safe position is required
  RETRACT = 0, 

  // De-energize Emergency useful for Vaginal/Anal play, where a retraction could cause pain.
  // Instead the motor itself is de-energized so no Holding Torque is experienced.
  DEENERGIZE = 1 
} MotorEmergency;

class MotorException : public std::exception {
  public:
    explicit MotorException(const char* message) : msg_(message) {}

    /** Destructor.
     * Virtual to allow for subclassing.
     */
    virtual ~MotorException() noexcept {}

    /** Returns a pointer to the (constant) error description.
     *  @return A pointer to a const char*. The underlying memory
     *          is in posession of the Exception object. Callers must
     *          not attempt to free the memory.
     */
    virtual const char* what() const noexcept {
       return msg_.c_str();
    }

protected:
    /** Error message.
     */
    String msg_;
};

class MotorBusyError : public MotorException {
  public:
    explicit MotorBusyError(const char* message) : MotorException(message) {}
};
class MotorGenericError : public MotorException {
  public:
    explicit MotorGenericError(const char* message) : MotorException(message) {}
};
class MotorInvalidStateError : public MotorException {
  public:
    explicit MotorInvalidStateError(const char* message) : MotorException(message) {}
};
class MotorInMotionError : public MotorException {
  public:
    explicit MotorInMotionError(const char* message) : MotorException(message) {}
};

class MotorInterface {
  public:
    void enable() { this->addStatusFlag(MOTOR_FLAG_ENABLED); }
    // TODO - virtual void disable();

    void lockTask() {
      if (this->taskSemaphore == NULL) {
        this->taskSemaphore = xSemaphoreCreateBinary();
      }

      if( xSemaphoreTake( this->taskSemaphore, 100 / portTICK_PERIOD_MS ) != pdTRUE ) {
        throw new MotorBusyError("Unable to attach a new Motion Task, as one is already active!");
      }

      this->motionTask = xTaskGetCurrentTaskHandle();
    }
    void unlockTask() {
      if (this->motionTask = xTaskGetCurrentTaskHandle()) {
        throw new MotorGenericError("Attempted to release Semaphore using invalid handle!");
      }

      xSemaphoreGive(this->taskSemaphore);
      this->motionTask = NULL;
    }
    
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
      this->checkTaskLock();

      // TODO - If a motion task is provided, ensure the caller is the motion task (Mutex?)
      // Ensure in ACTIVE and valid movement state
      if (!this->isInState(MotorState::ACTIVE)) {
        throw new MotorInvalidStateError("Unable to command motion while motor is not in ACTIVE state!");
      }

      // Take Semaphore for movement
      if( xSemaphoreTake( this->movementSemaphore, 1000 / portTICK_PERIOD_MS ) != pdTRUE ) {
        throw new MotorInMotionError("Unable to acquire Motor Movement Semaphore within 1000ms. Motor currently within movement still!");
      }

      // Apply bounds and protections
      float safePosition = constrain(
        position, 
        0, 
        this->maxPosition
      );
      float safeSpeed = constrain(speed, 0, this->maxSpeed);
      float safeAcceleration = constrain(acceleration, 0, this->maxAcceleration);
      
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
      ESP_LOGD("motor", "Going to position %05.1f mm @ %05.1f m/s, %05.1f m/s^2", safePosition, safeSpeed, safeAcceleration);
      this->unsafeGoToPos(safePosition, safeSpeed, safeAcceleration);
    }

    // Execute the defined Emergency behavior, disconnect Motion Task, and disable the motor
    void emergencyStop(MotorEmergency emergency) {
      switch (emergency) {
        case MotorEmergency::RETRACT:

          return;
        
        case MotorEmergency::DEENERGIZE:
        default:

          return;
      }
    }

    // Force stops all motor motion, and abandons current Motion Command, allowing a new one to be accepted
    void motionStop() {
      this->checkTaskLock();

      xSemaphoreGive(this->movementSemaphore);
    }

    // Force current Motion Command to be abandoned, and new one to be accepted.
    // Allows changing parameters mid-command and updating command
    void motionInterrupt() {
      this->checkTaskLock();

      xSemaphoreGive(this->movementSemaphore);
    }

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
    
    bool isMotionCompleted() { return this->hasStatusFlag(MOTOR_FLAG_AT_TARGET) && !this->hasStatusFlag(MOTOR_FLAG_MOTION_ACTIVE); }
    
  protected:
    MotorState state = MotorState::INACTIVE;
    uint32_t status = 0;

    TaskHandle_t motionTask; // Task which will provide movement commands to Motor
    SemaphoreHandle_t taskSemaphore; // Prevent more than one task being interfaced to a motor at a time
    SemaphoreHandle_t movementSemaphore; // Prevent additional movement commands while in motion

    void addStatusFlag(uint32_t flag) { this->status |= flag; }
    void removeStatusFlag(uint32_t flag) { this->status &= ~flag; }

    MachineGeometry geometry;
    float maxPosition;
    float maxSpeed;
    float maxAcceleration;

    // TODO - needs to be reset to 0 whenever motion comes to a stop
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
    
    // TODO - do we just enforce that the driver must call this when motion is completed?
    void motionCompleted() {
      if (this->motionTask) {
        xTaskNotifyGiveIndexed(this->motionTask, MOVEMENT_TASK_FLAG_MOTION_COMPLETED);
      }
    }

    void checkTaskLock() {
      if (this->taskSemaphore != NULL && xTaskGetCurrentTaskHandle() != this->motionTask) {
        throw new MotorGenericError("Attempted to command motion from Task that doesn't own Motion Task Semaphore!");
      }
    }
};

#endif