#include <Arduino.h>
#include <StrokeEngine.h>
#include <FastAccelStepper.h>
#include <pattern.h>


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *servo = NULL;

void StrokeEngine::begin(machineGeometry *physics, motorProperties *motor) {
    // store the machine geometry and motor properties pointer
    _physics = physics;
    _motor = motor;

    // Derived Machine Geometry & Motor Limits in steps:
    _travel = (_physics->physicalTravel - (2 * _physics->keepoutBoundary));
    _minStep = 0;
    _maxStep = int(0.5 + _travel * _motor->stepsPerMillimeter);
    _maxStepPerSecond = int(0.5 + _motor->maxSpeed * _motor->stepsPerMillimeter);
    _maxStepAcceleration = int(0.5 + _motor->maxAcceleration * _motor->stepsPerMillimeter);
          
    // Initialize with default values
    _state = UNDEFINED;
    _isHomed = false;
    _patternIndex = 0;
    _index = 0;
    _depth = _maxStep; 
    _stroke = _maxStep / 3;
    _timeOfStroke = 1.0;
    _sensation = 0.0;

    // Setup FastAccelStepper 
    engine.init();
    servo = engine.stepperConnectToPin(_motor->stepPin);
    if (servo) {
        servo->setDirectionPin(_motor->directionPin, _motor->directionPin);
        servo->setEnablePin(_motor->enablePin, _motor->enableActiveLow);
        servo->setAutoEnable(false);
        servo->disableOutputs(); 
    }
    Serial.println("Servo initialized");

#ifdef DEBUG_VERBOSE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

void StrokeEngine::setSpeed(float speed) {
    // Convert FPM into seconds to complete a full stroke
    _timeOfStroke = 60.0 / speed;

    // Constrain stroke time between 10ms and 120 seconds
    _timeOfStroke = constrain(_timeOfStroke, 0.01, 120.0); 

    // Update pattern with new speed, will be used with the next stroke or on update request
    patternTable[_patternIndex]->setTimeOfStroke(_timeOfStroke);

#ifdef DEBUG_VERBOSE
    Serial.println("setTimeOfStroke: " + String(_timeOfStroke, 2));
#endif

}

float StrokeEngine::getSpeed() {
    // Convert speed into FPMs
    return 60.0 / _timeOfStroke;
}

void StrokeEngine::setDepth(float depth) {
    // Convert depth from mm into steps
    _depth = int(depth * _motor->stepsPerMillimeter);

    // Constrain depth between minStep and maxStep
    _depth = constrain(_depth, _minStep, _maxStep); 

    // Update pattern with new speed, will be used with the next stroke or on update request
    patternTable[_patternIndex]->setDepth(_depth);

#ifdef DEBUG_VERBOSE
    Serial.println("setDepth: " + String(_depth));
#endif

    // if in state SETUPDEPTH then adjust
    if (_state == SETUPDEPTH) {
        _setupDepths();
    }
}

float StrokeEngine::getDepth() {
    // Convert depth from steps into mm
    return _depth / _motor->stepsPerMillimeter;
}

void StrokeEngine::setStroke(float stroke) {
    // Convert stroke from mm into steps
    _stroke = int(stroke * _motor->stepsPerMillimeter);

    // Constrain stroke between minStep and maxStep
    _stroke = constrain(_stroke, _minStep, _maxStep); 

    // Update pattern with new speed, will be used with the next stroke or on update request
    patternTable[_patternIndex]->setStroke(_stroke);

#ifdef DEBUG_VERBOSE
    Serial.println("setStroke: " + String(_stroke));
#endif

    // if in state SETUPDEPTH then adjust
    if (_state == SETUPDEPTH) {
        _setupDepths();
    }
}

float StrokeEngine::getStroke() {
    // Convert stroke from steps into mm
    return _stroke / _motor->stepsPerMillimeter;
}

void StrokeEngine::setSensation(float sensation) {
    // Constrain sensation between -100 and 100
    _sensation = constrain(sensation, -100, 100); 

    // Update pattern with new speed, will be used with the next stroke or on update request
    patternTable[_patternIndex]->setSensation(_sensation);

#ifdef DEBUG_VERBOSE
    Serial.println("setSensation: " + String(_sensation));
#endif

    // if in state SETUPDEPTH then adjust
    if (_state == SETUPDEPTH) {
        _setupDepths();
    }
}

float StrokeEngine::getSensation() {
    return _sensation;
}

bool StrokeEngine::setPattern(int patternIndex) {
    // Check wether pattern Index is in range
    if (patternIndex < patternTableSize) {
        _patternIndex = patternIndex;

        // Inject current motion parameters into new pattern
        patternTable[_patternIndex]->setTimeOfStroke(_timeOfStroke);
        patternTable[_patternIndex]->setDepth(_depth);
        patternTable[_patternIndex]->setStroke(_stroke);
        patternTable[_patternIndex]->setSensation(_sensation);

        // Reset index counter
        _index = 0; 

#ifdef DEBUG_VERBOSE
    Serial.println("setPattern: [" + String(_patternIndex) + "] " + patternTable[_patternIndex]->getName());
    Serial.println("setTimeOfStroke: " + String(_timeOfStroke, 2));
    Serial.println("setDepth: " + String(_depth));
    Serial.println("setStroke: " + String(_stroke));
    Serial.println("setSensation: " + String(_sensation));
#endif
        return true;
    }

    // Return false on no match
#ifdef DEBUG_VERBOSE
    Serial.println("Failed to set pattern: " + String(_patternIndex));
#endif
    return false;   
}

int StrokeEngine::getPattern() {
    return _patternIndex;
}

bool StrokeEngine::applyNewSettingsNow() {
    motionParameter currentMotion;

#ifdef DEBUG_VERBOSE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

    // Only allowed when in a running state
    if (_state == PATTERN) {
        // Ask pattern for update on motion parameters
        currentMotion = patternTable[_patternIndex]->nextTarget(_index);

        // Increase deceleration if required to avoid crash
        if (servo->getAcceleration() > currentMotion.acceleration) {
#ifdef DEBUG_VERBOSE
            Serial.print("Crash avoidance! Set Acceleration from " + String(currentMotion.acceleration));
            Serial.println(" to " + String(servo->getAcceleration()));
#endif
            currentMotion.acceleration = servo->getAcceleration();
        }

        // Apply new trapezoidal motion profile to servo
        _applyMotionProfile(&currentMotion);

#ifdef DEBUG_VERBOSE
    Serial.println("Apply New Settings Now");
#endif
        // Success
        return true;
    }

#ifdef DEBUG_VERBOSE
    Serial.println("Failed to apply settings");
#endif

    // Updating not allowed
    return false;
}

bool StrokeEngine::startPattern() {
    // Only valid if state is ready
    if (_state == READY || SETUPDEPTH) {

        // Stop current move, should one be pending (moveToMax or moveToMin)
        if (servo->isRunning()) {
            // Stop servo motor as fast as legaly allowed
            servo->setAcceleration(_maxStepAcceleration);
            servo->applySpeedAcceleration();
            servo->stopMove();
        }

        // Set state to PATTERN
        _state = PATTERN;

        // Reset Stroke and Motion parameters
        _index = -1;
        patternTable[_patternIndex]->setTimeOfStroke(_timeOfStroke);
        patternTable[_patternIndex]->setDepth(_depth);
        patternTable[_patternIndex]->setStroke(_stroke);
        patternTable[_patternIndex]->setSensation(_sensation);
#ifdef DEBUG_VERBOSE
        Serial.print(" _timeOfStroke: " + String(_timeOfStroke));
        Serial.print(" | _depth: " + String(_depth));
        Serial.print(" | _stroke: " + String(_stroke));
        Serial.println(" | _sensation: " + String(_sensation));
#endif

        // Create Stroke Task
        xTaskCreate(
            this->_strokingImpl,    // Function that should be called
            "Stroking",             // Name of the task (for debugging)
            2048,                   // Stack size (bytes)
            this,                   // Pass reference to this class instance
            24,                     // Pretty high task piority
            &_taskStrokingHandle    // Task handle
        ); 

#ifdef DEBUG_VERBOSE
        Serial.println("Started motion task");
        Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

        return true;

    } else {

#ifdef DEBUG_VERBOSE
        Serial.println("Failed to start motion");
#endif
        return false;

    }
}

void StrokeEngine::stopMotion() {
    // only valid when 
    if (_state == PATTERN || SETUPDEPTH) {
        // Stop servo motor as fast as legaly allowed
        servo->setAcceleration(_maxStepAcceleration);
        servo->applySpeedAcceleration();
        servo->stopMove();

        // Set state
        _state = READY;

#ifdef DEBUG_VERBOSE
        Serial.println("Motion stopped");
#endif

    }
    
#ifdef DEBUG_VERBOSE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

void StrokeEngine::enableAndHome(int pin, bool activeLow, void(*callBackHoming)(bool), float speed) {
    // Store callback
    _callBackHomeing = callBackHoming;

    // enable and home
    enableAndHome(pin, activeLow, speed);
}

void StrokeEngine::enableAndHome(int pin, bool activeLow, float speed) {
    // set homing pin as input
    _homeingPin = pin;
    pinMode(_homeingPin, INPUT);
    _homeingActiveLow = activeLow;
    _homeingSpeed = speed * _motor->stepsPerMillimeter;

    // first stop current motion and delete stroke task
    stopMotion();

    // Enable Servo
    servo->enableOutputs();

    // Create homing task
    xTaskCreate(
        this->_homingProcedureImpl,     // Function that should be called
        "Homing",                       // Name of the task (for debugging)
        2048,                           // Stack size (bytes)
        this,                           // Pass reference to this class instance
        20,                             // Pretty high task piority
        &_taskHomingHandle              // Task handle
    ); 
#ifdef DEBUG_VERBOSE
    Serial.println("Homing task started");
#endif

}

void StrokeEngine::thisIsHome(float speed) {
    // set homeing speed
    _homeingSpeed = speed * _motor->stepsPerMillimeter;

    if (_state != ERROR) {
        // Enable Servo
        servo->enableOutputs();

        // Stet current position as home
        servo->setCurrentPosition(-_motor->stepsPerMillimeter * _physics->keepoutBoundary);

        // Set feedrate for homing
        servo->setSpeedInHz(_homeingSpeed);       
        servo->setAcceleration(_maxStepAcceleration / 10);

        // drive free of switch and set axis to 0
        servo->moveTo(0);
        
        // Change state
        _isHomed = true;
        _state = READY;
    }
}

bool StrokeEngine::moveToMax(float speed) {

#ifdef DEBUG_VERBOSE
    Serial.println("Move to max");
#endif

    if (_isHomed) {
        // Stop motion immideately
        stopMotion();

        // Set feedrate for safe move 
        // Constrain speed between 1 step/sec and _maxStepPerSecond
        servo->setSpeedInHz(constrain(speed * _motor->stepsPerMillimeter, 1, _maxStepPerSecond));
        servo->setAcceleration(_maxStepAcceleration / 10);
        servo->moveTo(_maxStep);

        // Set state
        _state = READY;

#ifdef DEBUG_VERBOSE
        Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

        // Return success
        return true;

    } else {
        // Return failure
        return false;
    }
}

bool StrokeEngine::moveToMin(float speed) {

#ifdef DEBUG_VERBOSE
    Serial.println("Move to min");
#endif

    if (_isHomed) {
        // Stop motion immideately
        stopMotion();

        // Set feedrate for safe move 
        // Constrain speed between 1 step/sec and _maxStepPerSecond
        servo->setSpeedInHz(constrain(speed * _motor->stepsPerMillimeter, 1, _maxStepPerSecond));
        servo->setAcceleration(_maxStepAcceleration / 10);
        servo->moveTo(_minStep);

        // Set state
        _state = READY;

#ifdef DEBUG_VERBOSE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

        // Return success
        return true;

    } else {
        // Return failure
        return false;
    }
}

bool StrokeEngine::setupDepth(float speed, bool fancy) {
#ifdef DEBUG_VERBOSE
    Serial.println("Move to Depth");
#endif
    // store fanciness
    _fancyAdjustment = fancy;

    // returns true on success, and false if in wrong state
    bool allowed = false;

    // isHomed is only true in states READY, PATTERN and SETUPDEPTH
    if (_isHomed) {
        // Stop motion immideately
        stopMotion();

        // Set feedrate for safe move 
        // Constrain speed between 1 step/sec and _maxStepPerSecond
        servo->setSpeedInHz(constrain(speed * _motor->stepsPerMillimeter, 1, _maxStepPerSecond));
        servo->setAcceleration(_maxStepAcceleration / 10);
        servo->moveTo(_depth);

        // Set new state
        _state = SETUPDEPTH;

        // set return value to true
        allowed = true;
    }
#ifdef DEBUG_VERBOSE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
    return allowed;
}

ServoState StrokeEngine::getState() {
    return _state;
}

void StrokeEngine::disable() {
    _state = UNDEFINED;
    _isHomed = false;

    // Disable servo motor
    servo->disableOutputs();

    // Delete homing Task
    if (_taskHomingHandle != NULL) {
        vTaskDelete(_taskHomingHandle);
        _taskHomingHandle = NULL;
    }

#ifdef DEBUG_VERBOSE
    Serial.println("Servo disabled. Call home to continue.");
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

}

String StrokeEngine::getPatternName(int index) {
    if (index >= 0 && index <= patternTableSize) {
        return String(patternTable[index]->getName());
    } else {
        return String("Invalid");
    }
    
}

void StrokeEngine::motorFault() {
//TODO: propably not interrupt safe. But does it matter?

    // call disable
    disable();
    
    // Set error state
    _state = ERROR;

    // Safe State can only be cleared by removing power from Servo and ESP32 a.k.a. reboot
    Serial.println("Servo entered Safe State. Remove power to clear fault.");
#ifdef DEBUG_VERBOSE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

void StrokeEngine::setMaxSpeed(float maxSpeed){
    _maxStepPerSecond = int(0.5 + _motor->maxSpeed * _motor->stepsPerMillimeter);
}

float StrokeEngine::getMaxSpeed() {
    return float(_maxStepPerSecond) / _motor->stepsPerMillimeter;
}

void StrokeEngine::setMaxAcceleration(float maxAcceleration) {
    _maxStepAcceleration = int(0.5 + _motor->maxAcceleration * _motor->stepsPerMillimeter);
}

float StrokeEngine::getMaxAcceleration() {
    return float(_maxStepAcceleration) / _motor->stepsPerMillimeter;
}

void StrokeEngine::_homingProcedure() {
    // Set feedrate for homing
    servo->setSpeedInHz(_homeingSpeed);       
    servo->setAcceleration(_maxStepAcceleration / 10);    

    // Check if we are aleady at the homing switch
    if (digitalRead(_homeingPin) == !_homeingActiveLow) {
        //back off 5 mm from switch
        servo->move(_motor->stepsPerMillimeter * 2 * _physics->keepoutBoundary);

        // wait for move to complete
        while (servo->isRunning()) {
            // Pause the task for 100ms while waiting for move to complete
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        // move back towards endstop
        servo->move(-_motor->stepsPerMillimeter * 4 * _physics->keepoutBoundary);

    } else {
        // Move MAX_TRAVEL towards the homing switch
        servo->move(-_motor->stepsPerMillimeter * _physics->physicalTravel);
    }

    // Poll homing switch
    while (servo->isRunning()) {

        // Switch is active low
        if (digitalRead(_homeingPin) == LOW) {

            //Switch is at -KEEPOUT_BOUNDARY
            servo->forceStopAndNewPosition(-_motor->stepsPerMillimeter * _physics->keepoutBoundary);
            _isHomed = true;

            // drive free of switch and set axis to 0
            servo->moveTo(0);
            
            // Break loop, home was found
            break;
        }

        // Pause the task for 20ms to allow other tasks
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    
    // disable Servo if homing has not found the homing switch
    if (!_isHomed) {
        servo->disableOutputs();
        _state = UNDEFINED;

#ifdef DEBUG_VERBOSE
        Serial.println("Homing failed");
#endif

    } else {
        // Set state to ready
        _state = READY;

#ifdef DEBUG_VERBOSE
        Serial.println("Homing succeded");
#endif
    }

    // Call notification callback, if it was defined.
    if (_callBackHomeing != NULL) {
        _callBackHomeing(_isHomed);
    }

#ifdef DEBUG_VERBOSE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif

    // delete one-time task
    _taskHomingHandle = NULL;
    vTaskDelete(NULL);
}

void StrokeEngine::_stroking() {
    motionParameter currentMotion;

    while(1) { // infinite loop

        // Delete task, if not in PATTERN state
        if (_state != PATTERN) {
            _taskStrokingHandle = NULL;
            vTaskDelete(NULL);
        }

        // If motor has stopped issue moveTo command to next position
        if (servo->isRunning() == false) {
            // Increment index for pattern
            _index++;

#ifdef DEBUG_STROKE
            Serial.println("Stroking Index: " + String(_index));
#endif

            // Ask pattern for update on motion parameters
            currentMotion = patternTable[_patternIndex]->nextTarget(_index);

            // Apply new trapezoidal motion profile to servo
            _applyMotionProfile(&currentMotion);
        }
        
        // Delay 10ms 
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void StrokeEngine::_applyMotionProfile(motionParameter* motion) {
    // Apply new trapezoidal motion profile to servo
    // Constrain speed between 1 step/sec and _maxStepPerSecond
    servo->setSpeedInHz(constrain(motion->speed, 1, _maxStepPerSecond));

    // Constrain acceleration between 1 step/sec^2 and _maxStepAcceleration
    servo->setAcceleration(constrain(motion->acceleration, 1, _maxStepAcceleration));

    // Constrain position between max(0, (_depth - _stroke)) and _depth
    servo->moveTo(constrain(motion->position, max(0, (_depth - _stroke)), _depth));

#ifdef DEBUG_STROKE
    Serial.println("motion.position: " + String(motion->position));
    Serial.println("motion.speed: " + String(motion->speed));
    Serial.println("motion.acceleration: " + String(motion->acceleration));
#endif

}

void StrokeEngine::_setupDepths() {
    // set depth to _depth
    int depth = _depth;

    // in fancy mode we need to calculate exact position based on sensation, stroke & depth
    if (_fancyAdjustment == true) {
        // map sensation into the interval [depth-stroke, depth]
        depth = map(_sensation, -100, 100, _depth - _stroke, _depth);

#ifdef DEBUG_VERBOSE
        Serial.println("map sensation " + String(_sensation)
            + " to interval [" + String(_depth - _stroke)
            + ", " + String(_depth) 
            + "] = " + String(depth));
#endif
    } 

    // move servo to desired position
    servo->moveTo(depth);

#ifdef DEBUG_VERBOSE
    Serial.println("setup new depth: " + String(depth));
#endif
}
