#pragma once


#include "motor.h"
#include "math.h"

enum class MotionState {
    STOP,
    ACCELERATING, 
    COASTING,
    DECELERATING
};

class VirtualSerialMotor: public MotorInterface {
  public:

    // Init
    void begin(HardwareSerial *serialPort) { 
        _serialPort = _serialPort; 

        // Since it is virtual no homing needed
        _homed = true;

        // Set everything to defaults
        _speedSetPoint = 0.0;
        _targetPosition = 0.0;
        _accelerationSetPoint = 0.0;
        _currentPosition = 0.0;
        _currentSpeed = 0.0;
        _parameterHaveChanged = false;
        _motionState = MotionState::STOP;
    }

    void home() { 
        _homed = true; 
        if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
            _currentPosition = 0.0;
            _currentSpeed = 0.0;
            xSemaphoreGive(_parameterMutex);
        }
    }
    void timeGranularity(unsigned int time) { _timeSliceInMs = time / portTICK_PERIOD_MS; }

    // Control
    void enable() { 
        _enabled = true; 
        
        // Create motion simulator task
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
        _motionState = MotionState::STOP;
        // Suspend motion simulator task if it exists already
        if (_taskMotionSimulatorHandle != NULL) {
            vTaskSuspend(_taskMotionSimulatorHandle);
        }
    };

    // Motion
    void stopMotion() {
        if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
            _accelerationSetPoint = _maxAcceleration;
            _speedSetPoint = 0.0;
            _parameterHaveChanged = true;
            xSemaphoreGive(_parameterMutex);
        }
    }
    bool motionCompleted() { return (_motionState == MotionState::STOP) ? true : false; }

  protected:
    void _unsafeGoToPos(float position, float speed, float acceleration) {
        if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
            _speedSetPoint = speed;
            _targetPosition = position;
            _accelerationSetPoint = acceleration;
            _parameterHaveChanged = true;
            xSemaphoreGive(_parameterMutex);
        }
    }

  private:
    HardwareSerial *_serialPort;
    TickType_t _timeSliceInMs = 50;
    static void _motionSimulatorTaskImpl(void* _this) { static_cast<VirtualSerialMotor*>(_this)->_motionSimulatorTask(); }
    void _motionSimulatorTask() {
        TickType_t xLastWakeTime;
        // Initialize the xLastWakeTime variable with the current tick count.
        xLastWakeTime = xTaskGetTickCount();

        unsigned int now = millis();
        float speedSetPoint = 0.0;
        float targetPosition = 0.0;
        float accelerationSetPoint = 0.0;

        while(true) {
            //Establish time stamp
            now = millis();

            // copy parameters to local scope
            if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
                
                    speedSetPoint = _speedSetPoint;
                    targetPosition = _targetPosition;
                    accelerationSetPoint = _accelerationSetPoint;

                xSemaphoreGive(_parameterMutex);
            }

            // Did we get a parametere change and are we traveling into the right direction, or must we reverse?

            // in which phase of the trapezoidal motion are we?

            // Calculate next increment based on motion phase

            // Print results nicely on serial port
            //_serialPort->printf();

            // Delay the task until the next tick count
            vTaskDelayUntil(&xLastWakeTime, _timeSliceInMs);
        }
    }
    TaskHandle_t _taskMotionSimulatorHandle = NULL;
    float _speedSetPoint = 0.0;             // unsigned speed
    float _targetPosition = 0.0;
    float _accelerationSetPoint = 0.0;
    float _currentPosition = 0.0;
    float _currentSpeed = 0.0;              // speed sign denotes travel direction
    bool _parameterHaveChanged = false;
    SemaphoreHandle_t _parameterMutex = xSemaphoreCreateMutex();
    MotionState _motionState = MotionState::STOP; 

};
