#include <Arduino.h>
#include <motor/genericStepper.h>
#include <FastAccelStepper.h>


// Init
void GenericStepperMotor::begin(motorProperties *motor) { 
    _motor = motor;

    // Setup FastAccelStepper 
    engine.init();
    stepper = engine.stepperConnectToPin(_motor->stepPin);
    if (stepper) {
    stepper->setDirectionPin(_motor->directionPin, _motor->invertDirection);
    stepper->setEnablePin(_motor->enablePin, _motor->enableActiveLow);
    stepper->setAutoEnable(false);
    stepper->disableOutputs(); 
    }
}

void GenericStepperMotor::setMachineGeometry(float start, float end, float keepout) {
    _start = start;
    _end = end;
    _keepout = keepout;
    _maxPosition = abs(start - end) - (keepout * 2);
    _minStep = 0;
    _maxStep = int(0.5 + _maxPosition * _motor->stepsPerMillimeter);
    _maxStepPerSecond = int(0.5 + _maxSpeed * _motor->stepsPerMillimeter);
    _maxStepAcceleration = int(0.5 + _maxAcceleration * _motor->stepsPerMillimeter);
}

void GenericStepperMotor::setSensoredHoming(int homePin = 0, uint8_t pinMode = INPUT_PULLDOWN, bool activeHigh = true){

} // Assumes always homing to back of machine for safety

void GenericStepperMotor::home(float speed = 5.0) {

}

void GenericStepperMotor::home(void(*callBackHoming)(bool), float speed = 5.0) {
    _callBackHoming = callBackHoming;
    home(speed);
}

void GenericStepperMotor::attachPositionFeedback(void(*cbMotionPoint)(float, float, float), unsigned int timeInMs = 50) { 
    _cbMotionPoint = cbMotionPoint; 
    _timeSliceInMs = timeInMs / portTICK_PERIOD_MS;
}

// Control
void GenericStepperMotor::enable() {
    // Create / resume motion simulator task
    if (_taskPositionFeedbackHandle == NULL) {
        // Create Stroke Task
        xTaskCreatePinnedToCore(
            _positionFeedbackTaskImpl,         // Function that should be called
            "Motion Simulation",            // Name of the task (for debugging)
            4096,                           // Stack size (bytes)
            this,                           // Pass reference to this class instance
            24,                             // Pretty high task priority
            &_taskPositionFeedbackHandle,    // Task handle
            1                               // Pin to application core
        ); 
    } else {
        // Resume task, if it already exists
        vTaskResume(_taskPositionFeedbackHandle);
    }
}
void GenericStepperMotor::disable() {

}

// Motion
void GenericStepperMotor::stopMotion() { 
    if (stepper->isRunning()) {
    // Stop servo motor as fast as legally allowed
    stepper->setAcceleration(_maxStepAcceleration);
    stepper->applySpeedAcceleration();
    stepper->stopMove();
    }
}
bool GenericStepperMotor::motionCompleted() { 
    return stepper->isRunning(); 
}

void GenericStepperMotor::_unsafeGoToPosition(float position, float speed, float acceleration) {

}

void GenericStepperMotor::_homingProcedure() {

}

void GenericStepperMotor::_positionFeedbackTask() {
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current tick count.
    xLastWakeTime = xTaskGetTickCount();

    unsigned int now = millis();

    while(true) {

        // Return results of current motion point via the callback
        _cbMotionPoint(now, 5, 10);

        // Delay the task until the next tick count
        vTaskDelayUntil(&xLastWakeTime, _timeSliceInMs);
    }
}



