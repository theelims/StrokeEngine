/**
 *   StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine 
 *
 * Copyright (C) 2021 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#pragma once

#include <Arduino.h>
#include <pattern.h>

// Debug Levels
#define DEBUG_VERBOSE               // Show debug messages from the StrokeEngine on Serial
#define DEBUG_STROKE                // Show debug messaged for each individual stroke on Serial
#define DEBUG_PATTERN               // Show debug messages from inside pattern generator on Serial


/**************************************************************************/
/*!
  @brief  Struct defining the physical propoerties of the stroking machine.
*/
/**************************************************************************/
typedef struct {
  float physicalTravel;       /*> What is the maximum physical travel in mm */
  float keepoutBoundary;      /*> Soft endstop preventing hard crashes in mm. Will be 
                               *  subtracted twice from pysicalTravel. Should be 
                               *  sufficiently to completley drive clear from 
                               *  homing switch */
} machineGeometry;

/**************************************************************************/
/*!
  @brief  Struct defining the motor (stepper or servo with STEP/DIR 
  interface) and the motion system translating the rotation into a 
  linear motion.
*/
/**************************************************************************/
typedef struct {
  int stepsPerRevolution;     /*> How many steps per revolution of the motor */
  int maxRPM;                 /*> What is the maximum RPM of the servo */
  int maxAcceleration;        /*> Maximum acceleration in mm/s^2 */
  float stepsPerMillimeter;   /*> Number of steps per millimeter */
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
  @brief  Enum containing the states of the state machine
*/
/**************************************************************************/
typedef enum {
  SERVO_DISABLED,          //!< No power to the servo. We don't know its position
  SERVO_READY,             //!< Servo is energized and knows it position. Not running.
  SERVO_ERROR,             //!< Servo is on error state. Needs to be cleared by removing power
  SERVO_RUNNING,           //!< Stroke Engine is running and servo is moving according to defined pattern
  SERVO_STREAMING          //!< Stroke Engine is running and is receaiving a stream of position data
} ServoState;

// Verbose strings of states for debugging purposes
static String verboseState[] = {
  "[0] Servo disabled",
  "[1] Servo ready",
  "[2] Servo error",
  "[3] Servo running",
  "[4] Servo streaming"
};

/**************************************************************************/
/*!
  @brief  Stroke Engine provides a conveniant package for stroking motions
  created by stepper or servo motors. It's internal states are handled by a 
  finite state machine. A pattern generator allows to creat a variaty of 
  motion profiles. Under the hood FastAccelStepper is used for interfacing
  a stepper or servo motor vie a STEP/DIR interface.  
*/
/**************************************************************************/
class StrokeEngine {
    public:

        /**************************************************************************/
        /*!
          @brief  Initializes FastAccelStepper and configures all pins and outputs
          accordingly. StrokeEngine is in state SERVO_DISABLED
        */
        /**************************************************************************/
        void begin(machineGeometry *physics, motorProperties *motor);

        /**************************************************************************/
        /*!
          @brief  Set the speed of a stroke. Speed is given in Strokes per Minute
          and internally calculated to the time a full stroke needs to complete. 
          Settings tale effect with next stroke, or after calling 
          applyNewSettingsNow().
          @param speed Strokes per Minute. Is constrained from 0.5 to 6000 
        */
        /**************************************************************************/
        void setSpeed(float speed);

        /**************************************************************************/
        /*!
          @brief  Set the depth of a stroke. Settings tale effect with next stroke, 
          or after calling applyNewSettingsNow().
          @param speed Depth in [mm]. Is constrained from 0 to TRAVEL 
        */
        /**************************************************************************/
        void setDepth(float depth);

        /**************************************************************************/
        /*!
          @brief  Set the stroke length of a stroke. Settings tale effect with next 
          stroke, or after calling applyNewSettingsNow().
          @param speed Stroke length in [mm]. Is constrained from 0 to TRAVEL 
        */
        /**************************************************************************/
        void setStroke(float stroke);

        /**************************************************************************/
        /*!
          @brief  Set the sensation of a pattern. Sensation is an additional 
          parameter a pattern may use to alter its behaviour. Settings takes 
          effect with next stroke, or after calling applyNewSettingsNow().
          @param sensation  Sensation in [a.u.]. Is constrained from -100 to 100  
                        with 0 beeing assumed as neutral.
        */
        /**************************************************************************/
        void setSensation(float sensation);

        /**************************************************************************/
        /*!
          @brief  Choose a pattern for the StrokeEngine. Settings take effect with 
          next stroke, or after calling applyNewSettingsNow(). 
          @param patternIndex  Index of a pattern
          @return TRUE on success, FALSE, if patternIndex is invalid. Previous 
                        pattern will be retained.
        */
        /**************************************************************************/
        bool setPattern(int patternIndex);

        /**************************************************************************/
        /*!
          @brief  Normally parameter changes take effect only with next stroke. If
          immideate parameter changes are desired call applyNewSettingsNow() to 
          change even mid-stroke.
          @return 
        */
        /**************************************************************************/
        bool applyNewSettingsNow();

        /**************************************************************************/
        /*!
          @brief  Creates a FreeRTOS task to run a stroking pattern. Only valid in
          state SERVO_READY. Pattern is initilized with the values from the set 
          functions. If the task is running, state is SERVO_RUNNING.
          @return TRUE when task was created and motion starts, FALSE on failure.
        */
        /**************************************************************************/
        bool startMotion();

        /**************************************************************************/
        /*!
          @brief  Stops the motion with MAX_ACCEL and deletes the stroking task. Is
          in state SERVO_READY faterwards.
        */
        /**************************************************************************/
        void stopMotion();

        /**************************************************************************/
        /*!
          @brief  Enable the servo/stepper and do the homing procedure. Drives towards
          the endstop with HOMING_SPEED. Function is non-blocking and backed by a task.
          Optionally a callback can be given to receive feedback if homing succeded 
          going in state SERVO_READY. If homing switch is not found after traveling 
          MAX_TRAVEL it times out, disables the servo and goes into SERVO_DISABLED.
          @param pin    The pin used by the homeing switch
          @param aciveLow True if the switch id active low (pressed = 0V), 
                        False otherwise.
          @param speed  Speed in mm/s used for finding the homing switch. 
                        Defaults to 5.0 mm/s
          @param callBackHoming Callback function is called after homing is done. 
                        Function parametere holds a bool containing the success (TRUE)
                        or failure (FALSE) of homing.
        */
        /**************************************************************************/
        void enableAndHome(int pin, bool activeLow, float speed = 5.0);
        void enableAndHome(int pin, bool activeLow, void(*callBackHoming)(bool), float speed = 5.0);

        /**************************************************************************/
        /*!
          @brief  If no homing switch is present homing can be done manually. Push 
          the endeffector all the way in and call thisIsHome(). This enables the
          the servo and sets the position to -KEEPOUT_BOUNDARY
          @param speed  Speed in mm/s used for finding the homing switch. 
                        Defaults to 5.0 mm/s
        */
        /**************************************************************************/
        void thisIsHome(float speed = 5.0);

        /**************************************************************************/
        /*!
          @brief  In state SERVO_RUNNING and SERVO_READY this moves the endeffector
          to TRAVEL with SAFE_SPEED. Can be used for adjustments. Stops any running
          pattern and ends in state SERVO_READY.
          @param speed  Speed in mm/s used for driving to max. 
                        Defaults to 10.0 mm/s
          @return TRUE on success, FALSE if state does not allow this.
        */
        /**************************************************************************/
        bool moveToMax(float speed = 10.0);

        /**************************************************************************/
        /*!
          @brief  In state SERVO_RUNNING and SERVO_READY this moves the endeffector
          to 0 with SAFE_SPEED. Can be used for adjustments. Stops any running
          pattern and ends in state SERVO_READY.
          @param speed  Speed in mm/s used for driving to min. 
                        Defaults to 10.0 mm/s
          @return TRUE on success, FALSE if state does not allow this.
        */
        /**************************************************************************/
        bool moveToMin(float speed = 10.0);

        /**************************************************************************/
        /*!
          @brief  Retrieves the current servo state from the internal state machine.
          @return Current state of the state machine
        */
        /**************************************************************************/
        ServoState getState();

        /**************************************************************************/
        /*!
          @brief  Disables the servo motor instantly and deletes any motion task. 
          Sets state machine to SERVO_DISABLED. Must be followed by homing to enable
          servo again. 
        */
        /**************************************************************************/
        void disable();

        /**************************************************************************/
        /*!
          @brief  This function internally calls disable() and sets the state machine
          to SERVO_ERROR. Intended to be called from an interrupt if a servo/stepper
          fault signal should be monitored. Propably not interrupt safe, but this 
          maybe even doesn't matter.
        */
        /**************************************************************************/
        void motorFault();

        /**************************************************************************/
        /*!
          @brief  Makes the pattern list available for the main program to retreive 
          informations like pattern names.
          @param index index of a pattern.
          @return String holding a pattern name with a certain index. If index is 
                        out of range it returns "Invalid"
        */
        /**************************************************************************/
        String getPatternName(int index);

        /**************************************************************************/
        /*!
          @brief  Makes the pattern list available for the main program to retreive 
          informations like pattern names.
          @return The number of pattern available.
        */
        /**************************************************************************/
        unsigned int getNumberOfPattern() { 
          return patternTableSize; 
        };


    protected:
        ServoState _state = SERVO_DISABLED;
        motorProperties *_motor;
        machineGeometry *_physics;
        float _travel;
        int _minStep;
        int _maxStep;
        int _maxStepPerSecond;
        int _maxStepAcceleration;
        int _patternIndex = 0;
        bool _isHomed = false;
        int _index = 0;
        int _depth;
        int _stroke;
        float _timeOfStroke;
        float _sensation;
        static void _homingProcedureImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_homingProcedure(); }
        void _homingProcedure();
        static void _strokingImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_stroking(); }
        void _stroking();
        TaskHandle_t _taskStrokingHandle = NULL;
        TaskHandle_t _taskHomingHandle = NULL;
        void _applyMotionProfile(motionParameter* motion);
        void(*_callBackHomeing)(bool);
        int _homeingSpeed;
        int _homeingPin;
        bool _homeingActiveLow;      /*> Polarity of the homing signal*/
};

