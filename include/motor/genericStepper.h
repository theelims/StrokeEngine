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
  int stepsPerMillimeter;     /*> Number of steps per millimeter */
  bool invertDirection;       /*> Set to true to invert the direction signal
                               *  The firmware expects the home switch to be located at the 
                               *  end of an retraction move. That way the machine homes 
                               *  itself away from the body. Home position is -KEEPOUTBOUNDARY */
  bool enableActiveLow;       /*> Polarity of the enable signal. True for active low. */
  int stepPin;                /*> Pin connected to the STEP input */
  int directionPin;           /*> Pin connected to the DIR input */
  int enablePin;              /*> Pin connected to the ENA input */
} motorProperties;

class GenericStepperMotor: public MotorInterface {
  public:

    // Init
    void begin(motorProperties *motor) { _motor = motor; }
    void setSensoredHoming(int homePin = 0, uint8_t pinMode = INPUT_PULLDOWN, bool activeHigh = true); // Assumes always homing to back of machine for safety
    void home();
    void home(float speed = 5.0);
    void home(void(*callBackHoming)(bool), float speed = 5.0);

    // Control
    void enable();
    void disable();

    // Motion
    void stopMotion();
    bool motionCompleted();

  protected:
    void _unsafeGoToPos(float position, float speed, float acceleration);

  private:
    FastAccelStepper *stepper;
    int _stepPerMm; 
    motorProperties *_motor;
    static void _homingProcedureImpl(void* _this) { static_cast<GenericStepperMotor*>(_this)->_homingProcedure(); }
    void _homingProcedure();
    TaskHandle_t _taskHomingHandle = NULL;
    void(*_callBackHomeing)(bool) = NULL;

};


