#ifndef MOTOR_STEPPER_H
#define MOTOR_STEPPER_H

#include <time.h>
#include <FastAccelStepper.h>

#include "motor.hpp"

FastAccelStepperEngine engine = FastAccelStepperEngine();

class StepperMotor: public MotorInterface {
  public:
    // Motion
    void goToHome();
    void stopMotion();

    // Stepper
    void setStepsPerMm(int steps);
    void setMovementPin(int stepPin, int dirPin);
    void setEnablePin(int enablePin, bool activeHigh = true);

    // Assumes always homing to back of machine for safety
    void setHoming(int homePin = 0, uint8_t pinMode = INPUT_PULLDOWN, bool activeHigh = true);

    // Tasks
    void task_motion();

  protected:
    void unsafeGoToPos(float position, float speed, float acceleration);

  private:
    FastAccelStepper *servo;
    bool hasInitialized = false;
};

#endif