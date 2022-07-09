#pragma once

#include <FastAccelStepper.h>

#include "motor.h"

FastAccelStepperEngine engine = FastAccelStepperEngine();

/**************************************************************************/
/*!
  @brief  Struct defining the motor (stepper or servo with STEP/DIR 
  interface) and the motion system translating the rotation into a 
  linear motion.
*/
/**************************************************************************/
typedef struct {
  int stepsPerMillimeter;   /*> Number of steps per millimeter */
  bool invertDirection;       /*> Set to true to invert the direction signal
                               *  The firmware expects the home switch to be located at the 
                               *  end of an retraction move. That way the machine homes 
                               *  itself away from the body. Home position is -KEEPOUTBOUNDARY */
  bool enableActiveLow;       /*> Polarity of the enable signal. True for active low. */
  int stepPin;                /*> Pin connected to the STEP input */
  int directionPin;           /*> Pin connected to the DIR input */
  int enablePin;              /*> Pin connected to the ENA input */
} motorProperties;

class StepperMotor: public MotorInterface {
  public:
    void enable() { addStatusFlag(MOTOR_FLAG_ENABLED); }
    void disable() { removeStatusFlag(MOTOR_FLAG_ENABLED); }

    // Motion
    void goToHome();
    void stopMotion();
    void motionCompleted();
    void goToHome(float speed = 5.0);
    void goToHome(void(*callBackHoming)(bool), float speed = 5.0);

    // Stepper
    void init(motorProperties *motor);

    // Assumes always homing to back of machine for safety
    void setSensoredHoming(int homePin = 0, uint8_t pinMode = INPUT_PULLDOWN, bool activeHigh = true);

    // Tasks
    void task_motion();


  protected:
    void unsafeGoToPos(float position, float speed, float acceleration);

  private:
    FastAccelStepper *stepper;
    bool hasInitialized = false;
    motorProperties *_motor;
    static void _homingProcedureImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_homingProcedure(); }
    void _homingProcedure();
    TaskHandle_t _taskHomingHandle = NULL;
    void(*_callBackHomeing)(bool) = NULL;

};


