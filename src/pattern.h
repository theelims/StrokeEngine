/**
 *   Patterns of the StrokeEngine
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
#include <math.h>
#include "PatternMath.h"

#define DEBUG_PATTERN                 // Print some debug informations over Serial

#ifndef STRING_LEN
  #define STRING_LEN           64     // Bytes used to initialize char array. No path, topic, name, etc. should exceed this value
#endif

/**************************************************************************/
/*!
  @brief  struct to return all parameters FastAccelStepper needs to calculate
  the trapezoidal profile.
*/
/**************************************************************************/
typedef struct {
    int stroke;         //!< Absolute and properly constrainted target position of a move in steps 
    int speed;          //!< Speed of a move in Steps/second 
    int acceleration;   //!< Acceleration to get to speed or halt 
    bool skip;          //!< no valid stroke, skip this set an query for the next --> allows pauses between strokes
} motionParameter;


/**************************************************************************/
/*!
  @class Pattern 
  @brief  Base class to derive your pattern from. Offers a unified set of
          functions to store all relevant paramteres. These function can be
          overridenid necessary. Pattern should be self-containted and not 
          rely on any stepper/servo related properties. Internal book keeping
          is done in steps. The translation from real word units to steps is
          provided by the StrokeEngine. Also the sanity check whether motion
          parameters are physically possible are done by the StrokeEngine. 
          Imposible motion commands are clipped, cropped or adjusted while 
          still having a smooth appearance.  
*/
/**************************************************************************/
class Pattern {

    public:
        //! Constructor
        /*!
          @param str String containing the name of a pattern 
        */
        Pattern(const char *str) { strcpy(_name, str); }

        //! Set the time a normal stroke should take to complete
        /*! 
          @param speed time of a full stroke in [sec] 
        */
        virtual void setTimeOfStroke(float speed) { _timeOfStroke = speed; }

        //! Set the maximum stroke a pattern may have
        /*! 
          @param stroke stroke distance in mm 
        */
        virtual void setStroke(float stroke) { _stroke = stroke; }

        //! Set the maximum depth a pattern may have
        /*! 
          @param stroke stroke distance in Steps 
        */
        virtual void setDepth(float depth) { _depth = depth; }

        //! Sensation is an additional parameter a pattern can take to alter its behaviour
        /*! 
          @param sensation Arbitrary value from -100 to 100, with 0 beeing neutral 
        */
        virtual void setSensation(float sensation) { _sensation = sensation; } 

        //! Retrives the name of a pattern
        /*! 
          @return c_string containing the name of a pattern 
        */
        char *getName() { return _name; }

        //! Calculate the position of the next stroke based on the various parameters
        /*! 
          @param index index of a stroke. Increments with every new stroke. 
          @return Set of motion parameteres like speed, acceleration & position
        */
        virtual motionParameter nextTarget(unsigned int index) {
            _index = index;
            return _nextMove;
        } 

        //! Communicates the maximum possible speed and acceleration limits of the machine to a pattern.
        /*! 
          @param maxSpeed maximum speed which is possible. Higher speeds get truncated inside StrokeEngine anyway.
          @param maxAcceleration maximum possible acceleration. Get also truncated, if impossible.
          @param stepsPerMM 
        */
        virtual void setSpeedLimit(unsigned int maxSpeed, unsigned int maxAcceleration, unsigned int stepsPerMM) { _maxSpeed = maxSpeed; _maxAcceleration = maxAcceleration; _stepsPerMM = stepsPerMM; } 

    protected:
        float _stroke;
        float _depth;
        float _timeOfStroke;
        float _sensation = 0.0;

        int _index = -1;
        char _name[STRING_LEN]; 
        motionParameter _nextMove = {0, 0, 0, false};
        int _startDelayMillis = 0;
        int _delayInMillis = 0;
        unsigned int _maxSpeed = 0;
        unsigned int _maxAcceleration = 0;
        unsigned int _stepsPerMM = 0;

        /*!
          @brief Start a delay timer which can be polled by calling _isStillDelayed(). 
          Uses internally the millis()-function.
        */
        void _startDelay() {
            _startDelayMillis = millis();
        } 

        /*! 
          @brief Update a delay timer which can be polled by calling _isStillDelayed(). 
          Uses internally the millis()-function.
          @param delayInMillis delay in milliseconds 
        */
        void _updateDelay(int delayInMillis) {
            _delayInMillis = delayInMillis;
        } 

        /*! 
          @brief Poll the state of a internal timer to create pauses between strokes. 
          Uses internally the millis()-function.
          @return True, if the timer is running, false if it is expired.
        */
        bool _isStillDelayed() {
            return (millis() > (_startDelayMillis + _delayInMillis)) ? false : true; 
        }

};

/**************************************************************************/
/*
  Array holding all different patterns. Please include any custom pattern here.
*/
/**************************************************************************/
extern Pattern* patternTable[9];
const unsigned int patternTableSize = sizeof(patternTable) / sizeof(patternTable[0]);
