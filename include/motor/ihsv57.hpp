#ifndef MOTOR_IHSV57_H
#define MOTOR_IHSV57_H

#include <time.h>
#include <FastAccelStepper.h>

#include "motor.hpp"

FastAccelStepperEngine engine = FastAccelStepperEngine();

class IHSV57Motor: public MotorInterface {
  public:
    // Motion
    void goToHome();
    void stopMotion();

    // Stepper
    void setStepsPerMm(int steps);
    void setMovementPin(int stepPin, int dirPin);
    void setEnablePin(int enablePin, bool activeHigh = true);
    void setModBusPin(int txPin, int rxPin);

    // Tasks
    void task_motion();
    void task_heartbeat();

  protected:
    void unsafeGoToPos(float position, float speed, float acceleration);

  private:
    FastAccelStepper *servo;
    bool hasInitialized = false;
};

#endif