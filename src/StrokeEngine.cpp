#include <Arduino.h>
#include <StrokeEngine.h>
#include <pattern.h>

void StrokeEngine::attachMotor(MotorInterface* motor) {
  // store the machine geometry and motor properties pointer
  this->motor = motor;
        
  // Initialize with default values
  this->maxDepth = motor->getMaxPosition();
  this->depth = this->maxDepth; 
  this->stroke = this->maxDepth / 3;
  this->strokeRate = 1.0;
  this->sensation = 0.0;

  ESP_LOGD("StrokeEngine", "Stroke Parameter Max Depth = %f", this->maxDepth);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Depth = %f", this->depth);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Stroke = %f", this->stroke);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Stroke Rate = %f", this->strokeRate);
  ESP_LOGD("StrokeEngine", "Stroke Parameter Sensation = %f", this->sensation);

  ESP_LOGI("StrokeEngine", "Attached Motor succesfully to Stroke Engine!");
}

void StrokeEngine::setParameter(StrokeParameter parameter, float value, bool applyNow) {
  if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) != pdTRUE) {
    return;
  }

  String name;
  float debugValue;

  switch (parameter) {
    // TODO - When rate is set to 1 it bugs out and stops all motion
    case StrokeParameter::RATE:
      name = "Stroke Rate";
      debugValue = this->strokeRate = constrain(value, 1, 60 * 10);
      break;
    
    case StrokeParameter::DEPTH:
      name = "Depth";
      debugValue = this->depth = constrain(int(value), 0, this->maxDepth); 
      break;

    case StrokeParameter::STROKE:
      name = "Stroke";
      debugValue = this->stroke = constrain(int(value), 0, this->maxDepth); 
      break;

    case StrokeParameter::SENSATION:
      name = "Sensation";
      debugValue = this->sensation = constrain(sensation, -100, 100); 
      break;

    case StrokeParameter::PATTERN:
      name = "Pattern";
      debugValue = this->_patternIndex = value;
      this->_index = 0;
      break;
  }
  
  this->sendParameters(this->_patternIndex);
  
  ESP_LOGD("StrokeEngine", "Stroke Parameter %s - %f", name, debugValue);
  
  // When running a pattern and immediate update requested: 
  if (applyNow == true) {
    this->applyUpdate = true;
    ESP_LOGD("StrokeEngine", "Setting Apply Update Flag!");
  }

  xSemaphoreGive(_parameterMutex);
}

// WARNING: This function must be called only within the scope of a Taken _parameterMutex
void StrokeEngine::sendParameters(int patternIndex) {
  // Stroke is dynamically constrainted based on depth.
  // This allows depth to change, and allow stroke to fill the available space, 
  // rather than having to set the parameter again

  patternTable[patternIndex]->setSpeedLimit(
    this->motor->getMaxSpeed(), 
    this->motor->getMaxAcceleration(),
    1
  );

  patternTable[patternIndex]->setTimeOfStroke(constrain(60.0 / this->strokeRate, 0.01, 120.0));
  patternTable[patternIndex]->setStroke(constrain(int(this->stroke), 0, this->depth));
  patternTable[patternIndex]->setDepth(this->depth);
  patternTable[patternIndex]->setSensation(this->sensation);
}

float StrokeEngine::getParameter(StrokeParameter parameter) {
  switch (parameter) {
    case StrokeParameter::RATE:
      return this->strokeRate;
    case StrokeParameter::DEPTH:
      return this->depth;
    case StrokeParameter::STROKE:
      return this->stroke;
    case StrokeParameter::SENSATION:
      return this->sensation;
    case StrokeParameter::PATTERN:
      return this->_patternIndex;
    default:
      return 0; // Should never be reached
  }
}

bool StrokeEngine::startPattern() {
  // Only valid if state is ready
  if (!this->motor->isInState(MotorState::ACTIVE)) {
    ESP_LOGE("StrokeEngine", "Failed to start pattern! Motor is not active!");
    return false;
  }

  Pattern* pattern = patternTable[_patternIndex];
  ESP_LOGE("StrokeEngine", "Starting pattern %s", pattern->getName());

  // Stop current move, should one be pending (moveToMax or moveToMin)
  if (this->motor->hasStatusFlag(MOTOR_FLAG_MOTION_ACTIVE)) {
    this->motor->stopMotion();
  }

  // Reset Stroke and Motion parameters
  _index = -1;
  if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
    this->sendParameters(this->_patternIndex);        
    xSemaphoreGive(_parameterMutex);
  }

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
  this->active = true;

  return true;
}

void StrokeEngine::stopPattern() {
  ESP_LOGI("StrokeEngine", "Suspending Pattern!");

  if (_taskStrokingHandle != NULL) {
    vTaskDelete(_taskStrokingHandle);
  }
  this->active = false;
  this->motor->stopMotion();
}

String StrokeEngine::getPatternName(int index) {
    if (index >= 0 && index <= patternTableSize) {
        return String(patternTable[index]->getName());
    } else {
        return String("Invalid");
    }
    
}

void StrokeEngine::_stroking() {
    motionParameter currentMotion;

    SemaphoreHandle_t semaphore = this->motor->takeSemaphore();

    while(1) { // infinite loop

        // Suspend task, if motor is not active
        // TODO - This doesn't look quite right
        if (!this->motor->isInState(MotorState::ACTIVE)) {
          ESP_LOGI("StrokeEngine", "Motor is no longer active! Attempting to suspend pattern.");
          this->stopPattern();
        }

        // Take mutex to ensure no interference / race condition with communication threat on other core
        if (xSemaphoreTake(_parameterMutex, 0) == pdTRUE) {

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
                    //ESP_LOGI("StrokeEngine", "Stroking Index (AT_TARGET): %d @ %d %d %d", _index, currentMotion.stroke, currentMotion.speed, currentMotion.acceleration);

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
            xSemaphoreGive(_parameterMutex);
        }
        
        // Delay 1ms while waiting for an Movement Exit notification
        BaseType_t xResult = xTaskNotifyWait(notifyMovementExit, 0, 0, NULL, 10 / portTICK_PERIOD_MS);
        if (xResult & MOVEMENT_TASK_FLAG_EXIT) {
          // Exit was requested
          this->motor->giveSemaphore(semaphore);
          this->stopPattern();
        }

        if (xResult & MOVEMENT_TASK_FLAG_NEXT) {
          currentMotion = patternTable[_patternIndex]->nextTarget(_index);
        }
    }
}

// TODO - Move to a Depth Pattern
/*
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
*/