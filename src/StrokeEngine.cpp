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
    this->notifyMotionTask(MotionTaskEvent::REQUEST_NEXT);
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

  // Reset Stroke and Motion parameters
  _index = -1;
  if (xSemaphoreTake(_parameterMutex, portMAX_DELAY) == pdTRUE) {
    this->sendParameters(this->_patternIndex);        
    xSemaphoreGive(_parameterMutex);
  }

  this->startMotionTask();

  return true;
}

void StrokeEngine::stopPattern() {
  ESP_LOGI("StrokeEngine", "Suspending Pattern!");

  this->exitMotionTask();
}

String StrokeEngine::getPatternName(int index) {
    if (index >= 0 && index <= patternTableSize) {
        return String(patternTable[index]->getName());
    } else {
        return String("Invalid");
    }
}

void StrokeEngine::motion_task_init() {
  this->active = true;
  
  if (xSemaphoreTake(_parameterMutex, 0) == pdTRUE) {
    motionParameter currentMotion = patternTable[_patternIndex]->nextTarget(_index);
    this->motor->goToPos(
      currentMotion.stroke,
      currentMotion.speed,
      currentMotion.acceleration
    );

    _index += 1;
    xSemaphoreGive(_parameterMutex);
  }
}

void StrokeEngine::motion_task_event(MotionTaskEvent event) {
  if (event == MotionTaskEvent::MOTION_COMPLETED) {
    ESP_LOGI("StrokeEngine", "Motion Task: Next Target was requested - Motion Completed");
    _index++;
  } else {
    ESP_LOGI("StrokeEngine", "Motion Task: Next Target was requested - Next");
  }
  
  bool atTarget = this->motor->hasStatusFlag(MOTOR_FLAG_AT_TARGET);

  if (xSemaphoreTake(_parameterMutex, 0) == pdTRUE) {
    motionParameter currentMotion = patternTable[_patternIndex]->nextTarget(_index);
    this->motor->goToPos(
      currentMotion.stroke,
      currentMotion.speed,
      currentMotion.acceleration
    );

    xSemaphoreGive(_parameterMutex);
  }
}
