#include <Arduino.h>
#include <StrokeEngine.h>
#include <pattern.h>

void StrokeEngine::attachMotor(MotorInterface* motor) {
  // store the machine geometry and motor properties pointer
  this->motor = motor;
        
  // Initialize with default values
  motionBounds bounds = motor->getBounds();

  // TODO - Create new Motor Attached but not Ready state
  _state = ServoState::UNDEFINED;

  this->maxDepth = abs(bounds.end - bounds.start);
  this->depth = this->maxDepth; 
  this->stroke = this->maxDepth / 3;
  this->timeOfStroke = 1.0;
  this->sensation = 0.0;

  ESP_LOGD("StrokeEngine", "Stroke Parameter Max Depth = %f", this->maxDepth);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Depth = %f", this->depth);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Stroke = %f", this->stroke);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Time of Stroke = %f", this->timeOfStroke);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Sensation = %f", this->sensation);

  ESP_LOGI("StrokeEngine", "Attached Motor succesfully to Stroke Engine!");
}

void StrokeEngine::setParameter(StrokeParameter parameter, float value, bool applyNow) {
  if (xSemaphoreTake(_patternMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  String name;
  float debugValue;

  switch (parameter) {
    case StrokeParameter::RATE:
      name = "Rate";
      debugValue = this->timeOfStroke = constrain(60.0 / value, 0.01, 120.0);
      patternTable[_patternIndex]->setTimeOfStroke(this->timeOfStroke);
      break;
    
    case StrokeParameter::DEPTH:
      name = "Depth";
      debugValue = this->depth = constrain(int(value), 0, this->maxDepth); 
      patternTable[_patternIndex]->setDepth(depth);
      break;

    case StrokeParameter::STROKE:
      name = "Stroke";
      debugValue = this->stroke = constrain(int(value), 0, this->depth); 
      patternTable[_patternIndex]->setStroke(this->stroke);
      break;

    case StrokeParameter::SENSATION:
      name = "Sensation";
      debugValue = this->sensation = constrain(sensation, -100, 100); 
      patternTable[_patternIndex]->setSensation(this->sensation);
      break;

    case StrokeParameter::PATTERN:
      name = "Pattern";
      debugValue = this->_patternIndex = value;
      patternTable[_patternIndex]->setSpeedLimit(
        this->motor->getMaxSpeed(), 
        this->motor->getMaxAcceleration(),
        1
      );
      patternTable[_patternIndex]->setTimeOfStroke(this->timeOfStroke);
      patternTable[_patternIndex]->setStroke(this->stroke);
      patternTable[_patternIndex]->setDepth(this->depth);
      patternTable[_patternIndex]->setSensation(this->sensation);
      this->_index = 0;
      break;
  }
  
  // TODO - Need to specify parameter name
  ESP_LOGD("StrokeEngine", "Stroke Parameter %s - %f", name, debugValue);
  
  // When running a pattern and immediate update requested: 
  if ((_state == ServoState::PATTERN) && (applyNow == true)) {
    this->applyUpdate = true;
    ESP_LOGD("StrokeEngine", "Setting Apply Update Flag!");
  }
  
  // if in state SETUPDEPTH then adjust
  if (_state == ServoState::SETUPDEPTH) {
      _setupDepths();
  }

  xSemaphoreGive(_patternMutex);
}

float StrokeEngine::getParameter(StrokeParameter parameter) {
  switch (parameter) {
    case StrokeParameter::RATE:
      return 60.0 / this->timeOfStroke;
    case StrokeParameter::DEPTH:
      return this->depth;
    case StrokeParameter::STROKE:
      return this->stroke;
    case StrokeParameter::SENSATION:
      return this->sensation;
    case StrokeParameter::PATTERN:
      return this->_patternIndex;
  }
}

bool StrokeEngine::startPattern() {
  // Only valid if state is ready
  if (_state != ServoState::READY && _state != ServoState::SETUPDEPTH) {
    ESP_LOGE("StrokeEngine", "Failed to start motion! Incorrect State!");
    return false;
  }

  // Stop current move, should one be pending (moveToMax or moveToMin)
  if (this->motor->hasStatusFlag(MOTOR_FLAG_MOTION_ACTIVE)) {
    this->motor->stopMotion();
  }

  // Set state to PATTERN
  _state = ServoState::PATTERN;

  // Reset Stroke and Motion parameters
  _index = -1;
  if (xSemaphoreTake(_patternMutex, portMAX_DELAY) == pdTRUE) {
    patternTable[_patternIndex]->setSpeedLimit(
      this->motor->getMaxSpeed(), 
      this->motor->getMaxAcceleration(),
      1
    );
    patternTable[_patternIndex]->setTimeOfStroke(this->timeOfStroke);
    patternTable[_patternIndex]->setStroke(this->stroke);
    patternTable[_patternIndex]->setDepth(this->depth);
    patternTable[_patternIndex]->setSensation(this->sensation);         
    xSemaphoreGive(_patternMutex);
  }

  #ifdef DEBUG_TALKATIVE
    Serial.print(" _timeOfStroke: " + String(_timeOfStroke));
    Serial.print(" | _depth: " + String(_depth));
    Serial.print(" | _stroke: " + String(_stroke));
    Serial.println(" | _sensation: " + String(_sensation));
  #endif

  if (_taskStrokingHandle == NULL) {
    // Create Stroke Task
    xTaskCreatePinnedToCore(
      this->_strokingImpl,    // Function that should be called
      "Stroking",             // Name of the task (for debugging)
      4096,                   // Stack size (bytes)
      this,                   // Pass reference to this class instance
      24,                     // Pretty high task priority
      &_taskStrokingHandle,   // Task handle
      1                       // Pin to application core
    ); 
  } else {
    // Resume task, if it already exists
    vTaskResume(_taskStrokingHandle);
  }

  #ifdef DEBUG_TALKATIVE
    Serial.println("Started motion task");
    Serial.println("Stroke Engine State: " + verboseState[_state]);
  #endif

  return true;
}

void StrokeEngine::stopPattern() {
    // only valid when 
    if (_state == ServoState::PATTERN || _state == ServoState::SETUPDEPTH) {
        // Set state
        _state = ServoState::READY;
        this->motor->stopMotion();

#ifdef DEBUG_TALKATIVE
        Serial.println("Motion stopped");
#endif

        // Send telemetry data - TODO
        //if (_callbackTelemetry != NULL) {
        //    _callbackTelemetry(float(servo->getCurrentPosition() / _motor->stepsPerMillimeter), 0.0, false);
        //}
    }
    
#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
}

bool StrokeEngine::setupDepth(float speed, bool fancy) {
#ifdef DEBUG_TALKATIVE
    Serial.println("Move to Depth");
#endif
    // store fanciness
    _fancyAdjustment = fancy;

    // returns true on success, and false if in wrong state
    bool allowed = false;

    // isHomed is only true in states READY, PATTERN and SETUPDEPTH
    if (this->motor->hasStatusFlag(MOTOR_FLAG_HOMED)) {
        // Stop motion immediately
        this->motor->stopMotion();

        // Set new state
        _state = ServoState::SETUPDEPTH;

        // move to current depth position
        _setupDepths();

        // set return value to true
        allowed = true;
    }
#ifdef DEBUG_TALKATIVE
    Serial.println("Stroke Engine State: " + verboseState[_state]);
#endif
    return allowed;
}

ServoState StrokeEngine::getState() {
    return _state;
}

void StrokeEngine::disable() {
    _state = ServoState::UNDEFINED;

#ifdef DEBUG_TALKATIVE
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

void StrokeEngine::registerTelemetryCallback(void(*callbackTelemetry)(float, float, bool)) {
    _callbackTelemetry = callbackTelemetry;
}

void StrokeEngine::_stroking() {
    motionParameter currentMotion;

    while(1) { // infinite loop

        // Suspend task, if not in PATTERN state
        if (_state != ServoState::PATTERN) {
            vTaskSuspend(_taskStrokingHandle);
        }

        // Take mutex to ensure no interference / race condition with communication threat on other core
        if (xSemaphoreTake(_patternMutex, 0) == pdTRUE) {

            if (this->applyUpdate == true) {
                // Ask pattern for update on motion parameters
                currentMotion = patternTable[_patternIndex]->nextTarget(_index);

                // Apply new trapezoidal motion profile to servo
                ESP_LOGI("StrokeEngine", "Stroking Index (FORCE): %d @ %f %f %f", _index, currentMotion.stroke, currentMotion.speed, currentMotion.acceleration);
                this->motor->goToPos(
                  currentMotion.stroke,
                  currentMotion.speed,
                  currentMotion.acceleration
                );

                // clear update flag
                this->applyUpdate = false;
            }

            // If motor has stopped issue moveTo command to next position
            else if (this->motor->hasStatusFlag(MOTOR_FLAG_AT_TARGET)) {
                // Increment index for pattern
                _index++;

                // Querey new set of pattern parameters
                currentMotion = patternTable[_patternIndex]->nextTarget(_index);

                // Pattern may introduce pauses between strokes
                if (currentMotion.skip == false) {
                    ESP_LOGI("StrokeEngine", "Stroking Index (AT_TARGET): %d @ %f %f %f", _index, currentMotion.stroke, currentMotion.speed, currentMotion.acceleration);

                    this->motor->goToPos(
                      currentMotion.stroke,
                      currentMotion.speed,
                      currentMotion.acceleration
                    );

                } else {
                    // decrement _index so that it stays the same until the next valid stroke parameters are delivered
                    _index--;
                }
            }

            // give back mutex
            xSemaphoreGive(_patternMutex);
        }
        
        // Delay 10ms 
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

void StrokeEngine::_streaming() {

    while(1) { // infinite loop

        // Suspend task, if not in STREAMING state
        if (_state != ServoState::STREAMING) {
            vTaskSuspend(_taskStreamingHandle);
        }
        
        // Delay 10ms 
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void StrokeEngine::_setupDepths() {
    // set depth to _depth
    int depth = this->depth;

    // in fancy mode we need to calculate exact position based on sensation, stroke & depth
    if (_fancyAdjustment == true) {
        // map sensation into the interval [depth-stroke, depth]
        depth = map(this->sensation, -100, 100, this->depth - this->stroke, this->depth);

#ifdef DEBUG_TALKATIVE
        Serial.println("map sensation " + String(_sensation)
            + " to interval [" + String(_depth - _stroke)
            + ", " + String(_depth) 
            + "] = " + String(depth));
#endif
    } 
    
    this->motor->goToPos(
      depth,
      1000,
      1000
    );

    // Send telemetry data
    ///if (_callbackTelemetry != NULL) {
    ///    _callbackTelemetry(float(depth / _motor->stepsPerMillimeter), 
    ///        float(servo->getSpeedInMilliHz() * 1000 / _motor->stepsPerMillimeter), 
    ///        false);
    ///} 

#ifdef DEBUG_TALKATIVE
    Serial.println("setup new depth: " + String(depth));
#endif
}
