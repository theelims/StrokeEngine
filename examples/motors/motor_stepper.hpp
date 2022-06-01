#ifndef MOTOR_LINMOT_H
#define MOTOR_LINMOT_H

#include <time.h>
#include <FastAccelStepper.h>

#include "motor.hpp"

FastAccelStepperEngine engine = FastAccelStepperEngine();

class StepperMotor: public MotorInterface {
  public:
    // Motion
    //void goToHome();
    void goToPos(float position, float speed, float acceleration);
    void stopMotion();

    // General
    void registerTasks();

    // Stepper
    void setStepsPerMm(int steps);
    void setMovementPin(int stepPin, int dirPin);
    void setEnablePin(int enablePin, bool activeHigh = true);

    // TODO - Match this up with OSSM strategy for homing
    // Assumes always homing to back of machine for safety - Is this valid?
    void setSensorlessHoming(int currentPin, float currentLevel);
    void setSensoredHoming(int homePin = 0, uint8_t pinMode = INPUT_PULLDOWN, bool activeHigh = true);

    // Tasks
    void task_motion();
    void task_heartbeat();

  private:
    FastAccelStepper *servo;

    uint16_t position = 0;
    bool hasInitialized = false;
};

#endif