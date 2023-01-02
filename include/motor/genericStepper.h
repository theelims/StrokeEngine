/**
 *   Generic Stepper Motor Driver of StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine 
 *
 * Copyright (C) 2023 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */


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

/**************************************************************************/
/*!
  @brief  Stroke Engine provides a convenient package for stroking motions
  created by stepper or servo motors. It's internal states are handled by a 
  finite state machine. A pattern generator allows to creat a variety of 
  motion profiles. Under the hood FastAccelStepper is used for interfacing
  a stepper or servo motor vie a STEP/DIR interface.  
*/
/**************************************************************************/
class GenericStepperMotor: public MotorInterface {
  public:

    // Init
    void begin(motorProperties *motor);
    void setMachineGeometry(float start, float end, float keepout = 5.0);
    void setSensoredHoming(int homePin = 0, uint8_t pinMode = INPUT_PULLDOWN, bool activeHigh = true); // Assumes always homing to back of machine for safety
    void home(float speed = 5.0);
    void home(void(*callBackHoming)(bool), float speed = 5.0); 
    void attachPositionFeedback(void(*cbMotionPoint)(float, float, float), unsigned int timeInMs = 50); 

    // Control
    void enable(); 
    void disable();

    // Motion
    void stopMotion(); 
    bool motionCompleted();

    // Misc
    FastAccelStepperEngine& fastAccelStepperEngineReference() { return engine; } 

  protected:
    void _unsafeGoToPosition(float position, float speed, float acceleration);

  private:
    FastAccelStepper *stepper;
    motorProperties *_motor;
    int _minStep;
    int _maxStep;
    int _maxStepPerSecond;
    int _maxStepAcceleration;
    bool _motionCompleted = true;
    static void _homingProcedureImpl(void* _this) { static_cast<GenericStepperMotor*>(_this)->_homingProcedure(); }
    void _homingProcedure();
    int _homingSpeed;
    int _homingPin;
    bool _homingActiveLow;      /*> Polarity of the homing signal*/
    TaskHandle_t _taskHomingHandle = NULL;
    void(*_callBackHoming)(bool) = NULL;
    void(*_cbMotionPoint)(float, float, float) = NULL;
    TickType_t _timeSliceInMs = 50;
    static void _positionFeedbackTaskImpl(void* _this) { static_cast<GenericStepperMotor*>(_this)->_positionFeedbackTask(); }
    void _positionFeedbackTask();
    TaskHandle_t _taskPositionFeedbackHandle = NULL;
};


