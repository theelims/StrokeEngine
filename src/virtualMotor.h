/**
 *   Virtual Motor Driver of StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine 
 *
 * Copyright (C) 2023 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */


#pragma once

#include "motor.h"
#include "math.h"

/**************************************************************************/
/*!
  @brief  Struct defining a speed and position tuple
*/
/**************************************************************************/
struct speedAndPosition {
    float speed;            /*> Speed in [mm/s] */
    float position;         /*> Position in [mm] */
    float acceleration;     /*> Acceleration in [mm/s²] */
};

/**************************************************************************/
/*!
  @brief  Struct defining a point of a trapezoidal motion profile
*/
/**************************************************************************/
struct trapezoidalRampPoint {
    float time;             /*> Time in [s] */
    float position;         /*> Position in [mm] */
    float speed;            /*> Speed in [mm/s] */
};

/**************************************************************************/
/*!
  @brief  The Virtual Motor inherits from MotorInterface and provides a 
  purely virtual motor. It has a trapezoidal motion planner and returns the
  speed and position of StrokeEngine in real time. The time granularity is
  configurable. The motion planner mimics the one of the FastAccelStepper-
  library, allowing in-motion updates and recalculations. The main purpose 
  for testing of StrokeEngine's safety features, new features and new patterns
  without putting real hardware at risk.
*/
/**************************************************************************/
class VirtualMotor: public MotorInterface {
  public:

    VirtualMotor() {}
    
    /**************************************************************************/
    /*!
      @brief  Initializes the virtual motor Arduino Style. It also attaches a
      callback function where the speed and position are reported on a regular 
      interval specified with timeInMs. 
      @param cbMotionPoint Callback with the signature 
      `cbMotionPoint(float now, float position, float speed)`. time is reported
      seconds since the controller has started (`millis()`), speed in [m/s] and
      position in [mm].
      @param timeInMs time interval at which speed and position should be
      reported in [ms]
    */
    /**************************************************************************/
    void begin(void(*cbMotionPoint)(float, float, float), unsigned int timeInMs) { 
        _cbMotionPoint = cbMotionPoint; 
        _timeSliceInMs = timeInMs / portTICK_PERIOD_MS;
        // Since it is virtual no homing needed
        home();

        // Set everything to defaults
        _acceleration = 0.0;
    }
    
    /**************************************************************************/
    /*!
      @brief  A virtual home function. Since a virtual driver always knows where
      it is this can be used to reset the driver to 0.0mm at 0m/s velocity.   
    */
    /**************************************************************************/
    void home() { 
        _homed = true; 
        // Safeguard thread against race condition
        if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
            // initialize ramp with 0 so that system as at home position and in stand still
            for (int i = 0; i < 5; i++) {
                _trapezoidalRamp[i].position = 0.0;
                _trapezoidalRamp[i].speed = 0.0;
                _trapezoidalRamp[i].time = 0.0;
            }
            xSemaphoreGive(_parameterMutex);
        }        
    }

    /**************************************************************************/
    /*!
      @brief  Can be used to change the update interval. 
      @param timeInMs time interval at which speed and position should be
      reported in [ms]
    */
    /**************************************************************************/
    void setTimeGranularity(unsigned int timeInMs) { _timeSliceInMs = timeInMs / portTICK_PERIOD_MS; }

    /**************************************************************************/
    /*!
      @brief  Enables the motor driver. This starts the task reporting speed and
      position at the specified intervals.
    */
    /**************************************************************************/
    void enable() { 
        _enabled = true; 

        // Reset motion to home position
        home();

        // Create / resume motion simulator task
        if (_taskMotionSimulatorHandle == NULL) {
            // Create Stroke Task
            xTaskCreatePinnedToCore(
                _motionSimulatorTaskImpl,         // Function that should be called
                "Motion Simulation",            // Name of the task (for debugging)
                4096,                           // Stack size (bytes)
                this,                           // Pass reference to this class instance
                24,                             // Pretty high task priority
                &_taskMotionSimulatorHandle,    // Task handle
                1                               // Pin to application core
            ); 
        } else {
            // Resume task, if it already exists
            vTaskResume(_taskMotionSimulatorHandle);
        }
    }

    /**************************************************************************/
    /*!
      @brief  Disables the motor driver. This stops the task reporting speed and
      position.
    */
    /**************************************************************************/
    void disable() { 
        _enabled = false; 
        // Suspend motion simulator task if it exists already
        if (_taskMotionSimulatorHandle != NULL) {
            vTaskSuspend(_taskMotionSimulatorHandle);
        }
    };

    /**************************************************************************/
    /*!
      @brief  Initiates the fastest safe breaking to stand-still stopping all
      motion without loosing position. 
    */
    /**************************************************************************/
    void stopMotion() { _unsafeGoToPosition(0.0, 0.0, _maxAcceleration); }

    /**************************************************************************/
    /*!
      @brief  Returns if a trapezoidal motion is carried out, or the machine is
      at stand-still.
      @return `true` if motion is completed, `false` if still under way
    */
    /**************************************************************************/
    bool motionCompleted() { return _motionCompleted; }

    /**************************************************************************/
    /*!
      @brief  Returns the currently used acceleration.
      @return acceleration of the motor in [mm/s²]
    */
    /**************************************************************************/
    float getAcceleration() { return _acceleration; }

    /**************************************************************************/
    /*!
      @brief  Returns the current speed the machine.
      @return speed of the motor in [mm/s]
    */
    /**************************************************************************/
    float getSpeed() { return _currentSpeedAndPosition(millis()).speed; }

    /**************************************************************************/
    /*!
      @brief  Returns the current position of the machine.
      @return position in [mm]
    */
    /**************************************************************************/
    float getPosition() { return _currentSpeedAndPosition(millis()).position; }

  protected:
    void _unsafeGoToPosition(float position, float speed, float acceleration) { _trapezoidalRampGenerator(position, speed, acceleration); }

  private:
    void(*_cbMotionPoint)(float, float, float) = NULL;
    TickType_t _timeSliceInMs = 50;
    static void _motionSimulatorTaskImpl(void* _this) { static_cast<VirtualMotor*>(_this)->_motionSimulatorTask(); }
    void _motionSimulatorTask() {
        TickType_t xLastWakeTime;
        // Initialize the xLastWakeTime variable with the current tick count.
        xLastWakeTime = xTaskGetTickCount();

        unsigned int now = millis();
        speedAndPosition currentSpeedAndPosition;

        while(true) {

            // Safeguard thread against race condition
            if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
                // Establish time stamp
                now = millis();

                // calculate current speed and position
                currentSpeedAndPosition = _currentSpeedAndPosition(now);
                
                xSemaphoreGive(_parameterMutex);
            }

            // Return results of current motion point via the callback
            _cbMotionPoint(now, currentSpeedAndPosition.position, currentSpeedAndPosition.speed);

            // Delay the task until the next tick count
            vTaskDelayUntil(&xLastWakeTime, _timeSliceInMs);
        }
    }
    TaskHandle_t _taskMotionSimulatorHandle = NULL;
    float _acceleration = 0.0;
    unsigned int _startOfProfileInMs = 0;
    bool _motionCompleted = true;
    trapezoidalRampPoint _trapezoidalRamp[5];
    SemaphoreHandle_t _parameterMutex = xSemaphoreCreateMutex();
    void _trapezoidalRampGenerator(float position, float speed, float acceleration) {

        float topSpeed = 0.0;
        float ramptime = 0.0;
        float speedSetPoint = 0.0;

        // Safeguard thread against race condition
        if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
            // Retrieve current speed and position
            unsigned int now = millis();
            speedAndPosition currentSpeedAndPosition = _currentSpeedAndPosition(now);

            // Save time as basis for later calculations
            _startOfProfileInMs = now;

            // Flag in-motion
            _motionCompleted = false;

            // store motion defining parameters
            _acceleration = acceleration;

            // The motion generator may be called while in motion and starts the ramp calculation with the current speed and position
            // In this case a trapezoidal motion always consists of these phases:
            // Now --[0]--> Deceleration --[1]--> Acceleration --[2]--> Coasting --[3]--> Deceleration to zero --[4]--> stand still / motion completed
            // Depending on the conditions certain phases have the time=0 and are effectively skipped. 

            // R A M P   P O I N T   0   - Where everything starts
            _trapezoidalRamp[0].time = 0.0;
            _trapezoidalRamp[0].position = currentSpeedAndPosition.position;
            _trapezoidalRamp[0].speed = currentSpeedAndPosition.speed;


            // R A M P   P O I N T   1   - Do we need to decelerate?
            // Calculated deceleration to stand still --> also becomes all 0 if we are already at stand still.
            _trapezoidalRamp[1].time = abs(currentSpeedAndPosition.speed) / acceleration;
            _trapezoidalRamp[1].speed = 0.0;
            if (currentSpeedAndPosition.speed < 0) {
                _trapezoidalRamp[1].position = currentSpeedAndPosition.position - 0.5 * acceleration * _trapezoidalRamp[1].time  * _trapezoidalRamp[1].time;
            } else {
                _trapezoidalRamp[1].position = currentSpeedAndPosition.position + 0.5 * acceleration * _trapezoidalRamp[1].time  * _trapezoidalRamp[1].time;
            }

            // Is a full stop requested? Then there is nothing to do after the deceleration to 0
            if (speed == 0.0) {
                for (int i = 2; i < 5; i++) {
                    _trapezoidalRamp[i].time = _trapezoidalRamp[1].time;
                    _trapezoidalRamp[i].position = _trapezoidalRamp[1].position;
                    _trapezoidalRamp[i].speed = 0.0;
                }
                // we are done, can give the mutex back and return
                xSemaphoreGive(_parameterMutex);
                return;
            }

            // Do we still travel in the same direction?
            if (signbit(position - currentSpeedAndPosition.position) == signbit(currentSpeedAndPosition.speed)) {

                // Will we overshoot? Standstill position > target position
                if (abs(position - _trapezoidalRamp[1].position) > abs(position - currentSpeedAndPosition.position)) {
                    // in that case we can decelerate to zero --> all values set correctly, already

                // will we need to slow down
                } else if (abs(currentSpeedAndPosition.speed) > speed) {
                    _trapezoidalRamp[1].time = abs(currentSpeedAndPosition.speed) - speed / acceleration;

                    // traveling backwards
                    if (currentSpeedAndPosition.speed < 0) {
                        _trapezoidalRamp[1].speed = -speed;
                        _trapezoidalRamp[1].position = currentSpeedAndPosition.position 
                            - 0.5 * acceleration * _trapezoidalRamp[1].time  * _trapezoidalRamp[1].time 
                            + currentSpeedAndPosition.speed * _trapezoidalRamp[1].time;

                    // traveling forwards 
                    } else {
                        _trapezoidalRamp[1].speed = speed;
                        _trapezoidalRamp[1].position = currentSpeedAndPosition.position 
                            + 0.5 * acceleration * _trapezoidalRamp[1].time  * _trapezoidalRamp[1].time 
                            + currentSpeedAndPosition.speed * _trapezoidalRamp[1].time;
                    }

                // then we must accelerate --> skip
                } else {
                    _trapezoidalRamp[1].time = _trapezoidalRamp[0].time;
                    _trapezoidalRamp[1].position = _trapezoidalRamp[0].position;
                    _trapezoidalRamp[1].speed = _trapezoidalRamp[0].speed;
                }
            }


            // R A M P   P O I N T   2   - Do we need to accelerate?
            // Are we at coasting speed already? --> skip
            if (abs(_trapezoidalRamp[1].speed) == speed) {
                _trapezoidalRamp[2].time = _trapezoidalRamp[1].time;
                _trapezoidalRamp[2].position = _trapezoidalRamp[1].position;
                _trapezoidalRamp[2].speed = _trapezoidalRamp[1].speed;

            // We need to accelerate to coasting speed
            } else {
                // calculate the top speed for a triangular motion with an initial speed
                topSpeed = sqrt(2 * acceleration * abs(position - _trapezoidalRamp[1].position) - _trapezoidalRamp[1].speed * _trapezoidalRamp[1].speed) 
                    / sqrt(acceleration * acceleration - 1);

                // continue with the lower speed of these two, if speed is lower we will get a trapezoidal shape, otherwise it is triangle
                speedSetPoint = min(topSpeed, speed);

                // calculate next ramp point values
                _trapezoidalRamp[2].time = _trapezoidalRamp[1].time + speedSetPoint / acceleration;

                // traveling backwards
                if (position - _trapezoidalRamp[1].position < 0) {
                    _trapezoidalRamp[2].speed = -speedSetPoint;
                    _trapezoidalRamp[2].position = _trapezoidalRamp[1].position 
                        - 0.5 * acceleration * _trapezoidalRamp[2].time  * _trapezoidalRamp[2].time 
                        + _trapezoidalRamp[1].speed * _trapezoidalRamp[2].time;

                // traveling forwards 
                } else {
                    _trapezoidalRamp[2].speed = speedSetPoint;
                    _trapezoidalRamp[2].position = _trapezoidalRamp[1].position 
                        + 0.5 * acceleration * _trapezoidalRamp[2].time  * _trapezoidalRamp[2].time 
                        + _trapezoidalRamp[1].speed * _trapezoidalRamp[2].time;
                }
            }


            // R A M P   P O I N T   3   - Coasting at constant speed
            // If speed is not reached, we can skip as we are in a triangular profile
            if (abs(_trapezoidalRamp[2].speed) < speed) {
                _trapezoidalRamp[3].time = _trapezoidalRamp[2].time;
                _trapezoidalRamp[3].position = _trapezoidalRamp[2].position;
                
            // coasting until we hit the deceleration point
            } else {
                ramptime = abs(_trapezoidalRamp[2].speed) / acceleration;
                if (_trapezoidalRamp[2].speed < 0) {
                    _trapezoidalRamp[3].position = position + 0.5 * acceleration * ramptime  * ramptime;
                } else {
                    _trapezoidalRamp[3].position = position - 0.5 * acceleration * ramptime  * ramptime;
                }
                _trapezoidalRamp[3].time = _trapezoidalRamp[2].time + abs(_trapezoidalRamp[3].position - _trapezoidalRamp[2].position) / _trapezoidalRamp[2].speed;
            }

            // speed is not affected by coasting
            _trapezoidalRamp[3].speed = _trapezoidalRamp[2].speed;


            // R A M P   P O I N T   4   - Deceleration to standstill
            _trapezoidalRamp[4].time = abs(_trapezoidalRamp[3].speed) / acceleration;
            _trapezoidalRamp[4].position = position;
            _trapezoidalRamp[4].speed = 0.0;

            xSemaphoreGive(_parameterMutex);
        }
    }
    // This function must be called from within a xSemaphoreTake(_parameterMutex) == true scope
    speedAndPosition _currentSpeedAndPosition(unsigned int timeInMs) {
        speedAndPosition result;

        // Calculate time base in Seconds
        float t = float(timeInMs - _startOfProfileInMs) * 1.0e-3;

        // Calculate return values based on ramp phase
        if (t < _trapezoidalRamp[1].time) {
            // Deceleration Phase
            if (_trapezoidalRamp[0].speed > 0) {
                result.speed = _trapezoidalRamp[0].speed - _acceleration * t;
                result.position = _trapezoidalRamp[0].position + 0.5 * _acceleration * t * t;
            } else {
                result.speed = _trapezoidalRamp[0].speed + _acceleration * t;
                result.position = _trapezoidalRamp[0].position - 0.5 * _acceleration * t * t;
            }
        } else if (t < _trapezoidalRamp[2].time) {
            // Acceleration Phase
            if (_trapezoidalRamp[2].speed > 0) {
                result.speed = _trapezoidalRamp[1].speed + _acceleration * (t - _trapezoidalRamp[1].time);
                result.position = _trapezoidalRamp[1].position - 0.5 * _acceleration * (t - _trapezoidalRamp[1].time) * (t - _trapezoidalRamp[1].time);
            } else {
                result.speed = _trapezoidalRamp[1].speed - _acceleration * (t - _trapezoidalRamp[1].time);
                result.position = _trapezoidalRamp[1].position + 0.5 * _acceleration * (t - _trapezoidalRamp[1].time) * (t - _trapezoidalRamp[1].time);
            }
        } else if (t < _trapezoidalRamp[3].time) {
            // Coasting Phase
            result.speed = _trapezoidalRamp[2].speed;
            if (_trapezoidalRamp[2].speed > 0) {
                result.position = _trapezoidalRamp[2].position + _trapezoidalRamp[2].speed * (t - _trapezoidalRamp[2].time);
            } else {
                result.position = _trapezoidalRamp[2].position - _trapezoidalRamp[2].speed * (t - _trapezoidalRamp[2].time);
            }
        } else if (t < _trapezoidalRamp[4].time) {
            // Deceleration Phase
            if (_trapezoidalRamp[3].speed > 0) {
                result.speed = _trapezoidalRamp[3].speed - _acceleration * (t - _trapezoidalRamp[3].time);
                result.position = _trapezoidalRamp[3].position + 0.5 * _acceleration * (t - _trapezoidalRamp[3].time) * (t - _trapezoidalRamp[3].time);
            } else {
                result.speed = _trapezoidalRamp[3].speed + _acceleration * (t - _trapezoidalRamp[3].time);
                result.position = _trapezoidalRamp[3].position - 0.5 * _acceleration * (t - _trapezoidalRamp[3].time) * (t - _trapezoidalRamp[3].time);
            }
        } else {
            result.speed = 0.0;
            result.position = _trapezoidalRamp[4].position;
            _motionCompleted = true;
        }

        return result;       
    }

};
