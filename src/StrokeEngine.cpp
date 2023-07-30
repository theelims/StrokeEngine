#include <Arduino.h>
#include <StrokeEngine.h>
#include <pattern.h>

StrokeEngine::StrokeEngine() {
  listeners = new StrokeEngineListener*[10];
}

void StrokeEngine::attachMotor(MotorInterface* motor) {
  // store the machine geometry and motor properties pointer
  this->motor = motor;
        
  // Initialize with default values
  maxDepth = 200;
  depth = maxDepth; 
  stroke = maxDepth / 3;
  strokeRate = 1.0;
  sensation = 0.0;
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
      debugValue = strokeRate = constrain(value, 1, 60 * 10);
      break;
    
    case StrokeParameter::DEPTH:
      name = "Depth";
      debugValue = depth = constrain(int(value), 0, maxDepth); 
      break;

    case StrokeParameter::STROKE:
      name = "Stroke";
      debugValue = stroke = constrain(int(value), 0, maxDepth); 
      break;

    case StrokeParameter::SENSATION:
      name = "Sensation";
      debugValue = sensation = constrain(value, -100, 100); 
      break;

    case StrokeParameter::PATTERN:
      name = "Pattern";
      debugValue = _patternIndex = value;
      _index = 0;
      break;
  }
  
  sendParameters(_patternIndex);
  for (int i = 0; i < listenerCount; i++) {
    listeners[i]->engine_parameterUpdated(parameter, debugValue);
  }
  
  ESP_LOGD("StrokeEngine", "Stroke Parameter %s - %f > %f", name, value, debugValue);
  
  // When running a pattern and immediate update requested: 
  if (applyNow == true) {
    applyUpdate = true;
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
    motor->getMaxSpeed(), 
    motor->getMaxAcceleration(),
    1
  );

  patternTable[patternIndex]->setTimeOfStroke(constrain(60.0 / strokeRate, 0.01, 120.0));
  patternTable[patternIndex]->setStroke(constrain(int(stroke), 0, depth));
  patternTable[patternIndex]->setDepth(depth);
  patternTable[patternIndex]->setSensation(sensation);
}

// TODO - Replace Depth/Stroke with Envelope, and move everything else out of parameter
float StrokeEngine::getParameter(StrokeParameter parameter) {
  switch (parameter) {
    case StrokeParameter::RATE:
      return strokeRate;
    case StrokeParameter::DEPTH:
      return depth;
    case StrokeParameter::STROKE:
      return stroke;
    case StrokeParameter::SENSATION:
      return sensation;
    case StrokeParameter::PATTERN:
      return _patternIndex;
    default:
      return 0; // Should never be reached
  }
}

bool StrokeEngine::startPattern() {
  // Only valid if state is ready
  if (!motor->isRunning()) {
    ESP_LOGE("StrokeEngine", "Failed to start pattern! Motor is not active!");
    return false;
  }

  Pattern* pattern = patternTable[_patternIndex];
  ESP_LOGI("StrokeEngine", "Starting pattern %s", pattern->getName());

  // Stop current move, should one be pending (moveToMax or moveToMin)
  if (motor->isInMotion()) {
    motor->stopMotion();
  }

  // Reset Stroke and Motion parameters
  _index = -1;
  if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
    sendParameters(_patternIndex);        
    xSemaphoreGive(_parameterMutex);
  }

  if (_taskStrokingHandle == NULL) {
    // Create Stroke Task
    xTaskCreatePinnedToCore(
      _strokingImpl,    // Function that should be called
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
  active = true;

  return true;
}

void StrokeEngine::stopPattern() {
  ESP_LOGI("StrokeEngine", "Suspending Pattern!");

  if (_taskStrokingHandle != NULL) {
    vTaskSuspend(_taskStrokingHandle);
  }
  active = false;
  motor->stopMotion();
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

    while(1) { // infinite loop

        // Suspend task, if motor is not active
        // TODO - This doesn't look quite right
        if (!motor->isRunning()) {
          ESP_LOGI("StrokeEngine", "Motor is no longer active! Attempting to suspend pattern.");
          stopPattern();
        }

        // Take mutex to ensure no interference / race condition with communication threat on other core
        if (xSemaphoreTake(_parameterMutex, 0) == pdTRUE) {

            if (applyUpdate == true) {
                // Ask pattern for update on motion parameters
                currentMotion = patternTable[_patternIndex]->nextTarget(_index);

                // Apply new trapezoidal motion profile to servo
                ESP_LOGI("StrokeEngine", "Stroking Index (FORCE): %d @ %f %f %f", _index, currentMotion.stroke, currentMotion.speed, currentMotion.acceleration);
                motor->goToPosition(
                  currentMotion.stroke,
                  currentMotion.speed,
                  currentMotion.acceleration
                );

                // clear update flag
                applyUpdate = false;
            }

            // If motor has stopped issue moveTo command to next position
            else if (motor->isMotionCompleted()) {
                // Increment index for pattern
                _index++;

                // Querey new set of pattern parameters
                currentMotion = patternTable[_patternIndex]->nextTarget(_index);

                // Pattern may introduce pauses between strokes
                if (currentMotion.skip == false) {
                    //ESP_LOGI("StrokeEngine", "Stroking Index (AT_TARGET): %d @ %d %d %d", _index, currentMotion.stroke, currentMotion.speed, currentMotion.acceleration);

                    motor->goToPosition(
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
        
        // Delay 10ms 
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// TODO - Move to a Depth Pattern
/*
void StrokeEngine::_setupDepths() {
    // set depth to _depth
    int depth = depth;

    // in fancy mode we need to calculate exact position based on sensation, stroke & depth
    if (_fancyAdjustment == true) {
        // map sensation into the interval [depth-stroke, depth]
        depth = map(sensation, -100, 100, depth - stroke, depth);

#ifdef DEBUG_TALKATIVE
        Serial.println("map sensation " + String(_sensation)
            + " to interval [" + String(_depth - _stroke)
            + ", " + String(_depth) 
            + "] = " + String(depth));
#endif
    } 
    
    motor->goToPos(
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