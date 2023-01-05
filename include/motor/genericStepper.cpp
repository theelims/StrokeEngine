#include <Arduino.h>
#include <motor/genericStepper.h>
#include <FastAccelStepper.h>


// Init
void GenericStepperMotor::begin(motorProperties *motor) { 
    _motor = motor;

    // Setup FastAccelStepper 
    engine.init();
    _stepper = engine.stepperConnectToPin(_motor->stepPin);
    if (_stepper) {
        _stepper->setDirectionPin(_motor->directionPin, _motor->invertDirection);
        _stepper->setEnablePin(_motor->enablePin, _motor->enableActiveLow);
        _stepper->setAutoEnable(false);
        _stepper->disableOutputs(); 
    }
}

void GenericStepperMotor::setMachineGeometry(float travel, float keepout) {
    _travel = travel;
    _keepout = keepout;
    _maxPosition = travel - (keepout * 2);
    _minStep = 0;
    _maxStep = int(0.5 + _maxPosition * _motor->stepsPerMillimeter);
    _maxStepPerSecond = int(0.5 + _maxSpeed * _motor->stepsPerMillimeter);
    _maxStepAcceleration = int(0.5 + _maxAcceleration * _motor->stepsPerMillimeter);
}

// Assumes always homing to back of machine for safety
void GenericStepperMotor::setSensoredHoming(int homePin = 0, uint8_t arduinoPinMode = INPUT_PULLDOWN, bool activeLow = true){
    // set homing pin as input
    _homingPin = homePin;
    pinMode(_homingPin, arduinoPinMode);
    _homingActiveLow = activeLow;
} 

void GenericStepperMotor::home(float homePosition = 0.0, float speed = 5.0) {
    _homePosition = homePosition;
    _homingSpeed = speed * _motor->stepsPerMillimeter;

    // set homed to false so that isActive() becomes false
    _homed = false;

    // first stop current motion and suspend motion tasks
    stopMotion();

    // Quit if stepper not enabled
    if (_enabled == false) {
        //ESP_LOGW Motor not enabled.
        return;
    } 

    // Create homing task
    xTaskCreatePinnedToCore(
        this->_homingProcedureImpl,     // Function that should be called
        "Homing",                       // Name of the task (for debugging)
        2048,                           // Stack size (bytes)
        this,                           // Pass reference to this class instance
        20,                             // Pretty high task priority
        &_taskHomingHandle,             // Task handle
        1                               // Have it on application core
    ); 
}

void GenericStepperMotor::home(void(*callBackHoming)(bool), float homePosition = 0.0, float speed = 5.0) {
    _callBackHoming = callBackHoming;
    home(homePosition, speed);
}

void GenericStepperMotor::attachPositionFeedback(void(*cbMotionPoint)(float, float, float), unsigned int timeInMs = 50) { 
    _cbMotionPoint = cbMotionPoint; 
    _timeSliceInMs = timeInMs / portTICK_PERIOD_MS;
}

// Control
void GenericStepperMotor::enable() {
    // Enable stepper
    _enabled = true;
    _stepper->enableOutputs();

    // Create / resume motion feedback task
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
    // Disable stepper
    _enabled = false; 
    _stepper->disableOutputs();

    // Delete homing task should the homing sequence be running
    if (_taskHomingHandle != NULL) {
        vTaskDelete(_taskHomingHandle);
        _taskHomingHandle = NULL;
    }

    // Suspend motion feedback task if it exists already
    if (_taskPositionFeedbackHandle != NULL) {
        vTaskSuspend(_taskPositionFeedbackHandle);
    }
}

void GenericStepperMotor::stopMotion() { 

    // Delete homing task should the homing sequence be running
    if (_taskHomingHandle != NULL) {
        vTaskDelete(_taskHomingHandle);
        _taskHomingHandle = NULL;
    }

    if (_stepper->isRunning()) {
        // Stop servo motor as fast as legally allowed
        _stepper->setAcceleration(_maxStepAcceleration);
        _stepper->applySpeedAcceleration();
        _stepper->stopMove();
    }

    // Wait for servo stopped
    while (_stepper->isRunning());
}

void GenericStepperMotor::_unsafeGoToPosition(float position, float speed, float acceleration) {
    // Translate between metric and steps
    int speedInHz = int(0.5 + speed * _motor->stepsPerMillimeter);
    int stepAcceleration = int(0.5 + acceleration * _motor->stepsPerMillimeter);
    int positionInSteps = int(0.5 + position * _motor->stepsPerMillimeter);

    // write values to stepper
    _stepper->setSpeedInHz(speedInHz);
    _stepper->setAcceleration(stepAcceleration);
    _stepper->moveTo(positionInSteps);
}

bool GenericStepperMotor::_atHome() {
    return (digitalRead(_homingPin) == !_homingActiveLow) ? true : false;
}

void GenericStepperMotor::_homingProcedure() {
    // Set feedrate for homing
    _stepper->setSpeedInHz(_homingSpeed);       
    _stepper->setAcceleration(_maxStepAcceleration / 10);    

    // Check if we are already at the home position
    if (_atHome()) {
        //back off 2*keepout from switch
        _stepper->move(_motor->stepsPerMillimeter * 2 * _keepout);

        // wait for move to complete
        while (_stepper->isRunning()) {
            // Pause the task for 100ms while waiting for move to complete
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        // move back towards endstop
        _stepper->move(-_motor->stepsPerMillimeter * 4 * _keepout);

    } else {
        // Move maximum travel distance + 2*keepout towards the homing switch
        _stepper->move(-_motor->stepsPerMillimeter * (_maxPosition + 4 * _keepout));
    }

    // Poll homing switch
    while (_stepper->isRunning()) {

        // Are we at the home position?
        if (_atHome()) {

            // Set home position
            // Switch is at -KEEPOUT
            _stepper->forceStopAndNewPosition(_motor->stepsPerMillimeter * int(_homePosition - _keepout));

            // drive free of switch and set axis to lower end
            _stepper->moveTo(_minStep);

            _homed = true;

            // drive free of switch and set axis to 0
            _stepper->moveTo(0);
            

#ifdef DEBUG_TALKATIVE
        Serial.println("Homing succeeded");
#endif

            // Break loop, home was found
            break;
        }

        // Pause the task for 20ms to allow other tasks
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    
    // disable Servo if homing has not found the homing switch
    if (!_homed) {
        _stepper->disableOutputs();

#ifdef DEBUG_TALKATIVE
        Serial.println("Homing failed");
#endif

    }  

    // Call notification callback, if it was defined.
    if (_callBackHoming != NULL) {
        _callBackHoming(_homed);
    }

    // delete one-time task
    _taskHomingHandle = NULL;
    vTaskDelete(NULL);
}

void GenericStepperMotor::_positionFeedbackTask() {
    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current tick count.
    xLastWakeTime = xTaskGetTickCount();

    unsigned int now = millis();

    while(true) {
        // Return results of current motion point via the callback
        _cbMotionPoint(
            float(millis()) * 1.0e-3,
            getPosition(),
            getSpeed()            
        );

        // Delay the task until the next tick count
        vTaskDelayUntil(&xLastWakeTime, _timeSliceInMs);
    }
}



