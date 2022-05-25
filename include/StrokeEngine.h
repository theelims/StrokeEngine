/**
 *   StrokeEngine
 *   A library to create a variety of stroking motions with a stepper or servo motor on an ESP32.
 *   https://github.com/theelims/StrokeEngine 
 *
 * Copyright (C) 2022 theelims <elims@gmx.net>
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#pragma once
#ifndef STROKE_ENGINE_H
#define STROKE_ENGINE_H

#include <pattern.h>
#include <motor.hpp>

// Debug Levels
//#define DEBUG_TALKATIVE             // Show debug messages from the StrokeEngine on Serial
//#define DEBUG_STROKE                // Show debug messaged for each individual stroke on Serial
#define DEBUG_CLIPPING              // Show debug messages when motions violating the machine 
                                    // physics are commanded

/**************************************************************************/
/*!
  @brief  Enum containing the states of the state machine
*/
/**************************************************************************/
// TODO - Change to EngineState
enum class ServoState {
  UNDEFINED,          //!< No power to the servo. We don't know its position
  READY,             //!< Servo is energized and knows it position. Not running.
  PATTERN,           //!< Stroke Engine is running and servo is moving according to defined pattern.
  SETUPDEPTH,        //!< Interactive adjustment mode to setup depth and stroke
  STREAMING          //!< Tracks the depth-position whenever depth is updated.
};

enum class StrokeParameter {
  // RATE & SPEED are mutually exclusive. Only one can be specified at a time!
  // RATE - Range 0.5 to 6000 Strokes / Min
  // Can allow better control typically than just SPEED, as other machines use
  RATE,

  // SPEED - Range 1 to 5000 mm / Sec
  // Standard Speed Control
  SPEED,

  // DEPTH - Range is constrainted by motionBounds from MotorInterface
  // Is the point at which the stroke ends
  DEPTH, 

  // STROKE - Range is constrainted by motionBounds from MotorInterface
  // How far the stroke will retract from DEPTH point
  STROKE, 

  // SENSATION - Range is -100 to 100
  // Serves as a generic parameter for usage by patterns to adjust sensation
  SENSATION,

  PATTERN
};

// Verbose strings of states for debugging purposes
static String verboseState[] = {
  "[0] Servo disabled",
  "[1] Servo ready",
  "[2] Servo pattern running",
  "[3] Servo setup depth",
  "[4] Servo position streaming"
};

/**************************************************************************/
/*!
  @brief  Stroke Engine provides a convenient package for stroking motions
  created by stepper or servo motors. It's internal states are handled by a 
  finite state machine. A pattern generator allows to creat a variety of 
  motion profiles. Under the hood FastAccelStepper is used for interfacing
  a stepper or servo motor vie a STEP/DIR interface.  
*/
/**************************************************************************/
class StrokeEngine {
    public:

        /**************************************************************************/
        /*!
          @brief  Initializes FastAccelStepper and configures all pins and outputs
          accordingly. StrokeEngine is in state UNDEFINED
        */
        /**************************************************************************/
        void attachMotor(MotorInterface *motor);
        void ready() { this->_state = ServoState::READY; };

        /**************************************************************************/
        /*!
          @brief Settings tale effect with next stroke, or after calling applyNewSettingsNow().
          @param speed Strokes per Minute. Is constrained from 0.5 to 6000 
          @param applyNow Set to true if changes should take effect immediately 
        */
        /**************************************************************************/
        void setParameter(StrokeParameter parameter, float value, bool applyNow = false);
        float getParameter(StrokeParameter parameter);

        /**************************************************************************/
        /*!
          @brief  Creates a FreeRTOS task to run a stroking pattern. Only valid in
          state READY. Pattern is initialized with the values from the set 
          functions. If the task is running, state is PATTERN.
          @return TRUE when task was created and motion starts, FALSE on failure.
        */
        /**************************************************************************/
        bool startPattern();

        /**************************************************************************/
        /*!
          @brief  Stops the motion with MAX_ACCEL and deletes the stroking task. Is
          in state READY afterwards.
        */
        /**************************************************************************/
        void stopPattern();

        /**************************************************************************/
        /*!
          @brief  In state PATTERN and READY this moves the endeffector
          to DEPTH and enters state SETUPDEPTH. Follows the DEPTH postion 
          whenever setDepth() is called. Can be used for adjustments. Stops any running
          pattern. 
          @param speed  Speed in mm/s used for driving to min. 
                        Defaults to 10.0 mm/s
          @param fancy  In fancy mode sensation allows to adjust both, depth and 
                        stroke. +100 adjusts the depth position, -100 adjusts the
                        stroke position. 0 adjusts the midpoint depth-stroke/2.
          @return TRUE on success, FALSE if state does not allow this.
        */
        /**************************************************************************/
        bool setupDepth(float speed = 10.0, bool fancy = false);

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
          Sets state machine to UNDEFINED. Must be followed by homing to enable
          servo again. 
        */
        /**************************************************************************/
        void disable();

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

        /**************************************************************************/
        /*!
          @brief  Register a callback function that will update telemetry information
          about StrokeEngine. The provided function will be called whenever a motion 
          is executed by a manual command or by a pattern. The returned values are the
          target position of this move, its top speed and wether clipping occurred. 
          @param callbackTelemetry Function must be of type: 
          void callbackTelemetry(float position, float speed, bool clipping)
        */
        /**************************************************************************/
        void registerTelemetryCallback(void(*callbackTelemetry)(float, float, bool));

    protected:
        ServoState _state = ServoState::UNDEFINED;
        MotorInterface *motor;
        int _patternIndex = 0;

        int _index = 0;

        float maxDepth;
        float depth;
        float stroke;
        float timeOfStroke;
        float sensation;

        bool applyUpdate = false;

        static void _strokingImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_stroking(); }
        void _stroking();
        static void _streamingImpl(void* _this) { static_cast<StrokeEngine*>(_this)->_streaming(); }
        void _streaming();
        TaskHandle_t _taskStrokingHandle = NULL;
        TaskHandle_t _taskStreamingHandle = NULL;

        SemaphoreHandle_t _patternMutex = xSemaphoreCreateMutex();
        void _applyMotionProfile(motionParameter* motion);
        void(*_callbackTelemetry)(float, float, bool) = NULL;

        bool _fancyAdjustment;
        void _setupDepths();
};

#endif
