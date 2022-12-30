#pragma once


#include "motor.h"
#include "math.h"


struct speedAndPosition {
    float speed;
    float position;
};

struct trapezoidalRampPoint {
    float time;
    float position;
    float speed;
};


class VirtualSerialMotor: public MotorInterface {
  public:

    // Init
    void begin(HardwareSerial *serialPort) { 
        _serialPort = serialPort; 

        // Since it is virtual no homing needed
        home();

        // Set everything to defaults
        _acceleration = 0.0;
    }
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
    void timeGranularity(unsigned int timeInMs) { _timeSliceInMs = timeInMs / portTICK_PERIOD_MS; }

    // Control
    void enable() { 
        _enabled = true; 
        _homed = true;

        // initialize ramp with 0's so that system as at home position and in stand still
        for (int i = 0; i < 5; i++) {
            _trapezoidalRamp[i].position = 0.0;
            _trapezoidalRamp[i].speed = 0.0;
            _trapezoidalRamp[i].time = 0.0;
        }
        
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
    void disable() { 
        _enabled = false; 
        // Suspend motion simulator task if it exists already
        if (_taskMotionSimulatorHandle != NULL) {
            vTaskSuspend(_taskMotionSimulatorHandle);
        }
    };

    // Motion
    void stopMotion() { _trapezoidalRampGenerator(0.0, 0.0, _maxAcceleration); }
    bool motionCompleted() { return _motionCompleted; }

  protected:
    void _unsafeGoToPos(float position, float speed, float acceleration) { _trapezoidalRampGenerator(position, speed, acceleration); }

  private:
    HardwareSerial *_serialPort;
    TickType_t _timeSliceInMs = 50;
    static void _motionSimulatorTaskImpl(void* _this) { static_cast<VirtualSerialMotor*>(_this)->_motionSimulatorTask(); }
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

            // Print results nicely on serial port
            _serialPort->print(float(now), 3);
            _serialPort->print("\t");
            _serialPort->print(currentSpeedAndPosition.speed, 3);
            _serialPort->print("\t");
            _serialPort->println(currentSpeedAndPosition.position, 2);

            // Delay the task until the next tick count
            vTaskDelayUntil(&xLastWakeTime, _timeSliceInMs);
        }
    }
    TaskHandle_t _taskMotionSimulatorHandle = NULL;
    float _acceleration = 0.0;
    unsigned int _startOfRampInMs = 0;
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
            _startOfRampInMs = now;

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
        float t = float(timeInMs - _startOfRampInMs) * 1.0e-3;

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
