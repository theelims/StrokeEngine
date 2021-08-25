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
#include <StrokeEngine.h>
#include <math.h>
#include "PatternMath.h"

#define DEBUG_PATTERN                 // Print some debug informations over Serial

#ifndef STRING_LEN
  #define STRING_LEN           64     // Bytes used to initalize char array. No path, topic, name, etc. should exceed this value
#endif

/**************************************************************************/
/*!
  @brief  struct to return all parameters FastAccelStepper needs to calculate
  the trapezoidal profile.
*/
/**************************************************************************/
typedef struct {
    int position;       //!< Absolute target position of a move in steps 
    int speed;          //!< Speed of a move in Hz 
    int acceleration;   //!< Acceleration to get to speed or halt 
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
          @param str Sring containing the name of a pattern 
        */
        Pattern(char *str) { strcpy(_name, str); }

        //! Set the time a normal stroke should take to complete
        /*! 
          @param speed time of a full stroke in [sec] 
        */
        virtual void setTimeOfStroke(float speed) { _timeOfStroke = speed; }

        //! Set the depth of a stroke. Depth is the furthest position a stroke ever reaches
        /*! 
          @param depth depth in Steps 
        */
        virtual void setDepth(int depth) { _depth = depth; }

        //! Set the maximum stroke a pattern may have
        /*! 
          @param stroke stroke distance in Steps 
        */
        virtual void setStroke(int stroke) { _stroke = stroke; }

        //! Sensation is an additional parameter a pattern can take to alter its behaviour
        /*! 
          @param sesation Arbitrary value from -100 to 100, with 0 beeing neutral 
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

    protected:
        int _depth;
        int _stroke;
        float _timeOfStroke;
        float _sensation = 0.0;
        unsigned int _index;
        char _name[STRING_LEN]; 
        motionParameter _nextMove = {0, 0, 0};

};

/**************************************************************************/
/*!
  @brief  Simple Stroke Pattern. It creates a trapezoidal stroke profile
  with 1/3 acceleration, 1/3 coasting, 1/3 deceleration. Sensation has 
  no effect.
*/
/**************************************************************************/
class SimpleStroke : public Pattern {
    public:
        SimpleStroke(char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to devide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);  

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.position = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.position = _depth;
            }

            _index = index;
            return _nextMove;
        }
};

/**************************************************************************/
/*!
  @brief  Simple pattern where the sensation value can change the speed 
  ratio between in and out. Sensation > 0 make the in move faster (up to 5x)
  giving a hard pounding sensation. Values < 0 make the out move going 
  faster. This gives a more pleasing sensation. The time for the overall 
  stroke remains the same. 
*/
/**************************************************************************/
class TeasingPounding : public Pattern {
    public:
        TeasingPounding(char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateStrokeTiming();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateStrokeTiming();
        }
        motionParameter nextTarget(unsigned int index) {
            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfOutStroke);  
                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfOutStroke);    
                _nextMove.position = _depth - _stroke;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfInStroke);        
                // acceleration to meet the profile            
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfInStroke);    
                _nextMove.position = _depth;
            }
            _index = index;
            return _nextMove;
        }
    protected:
        float _timeOfFastStroke = 1.0;
        float _timeOfInStroke = 1.0;
        float _timeOfOutStroke = 1.0;
        void _updateStrokeTiming() {
            // calculate the time it takes to complete the faster stroke
            // Division by 2 because reference is a half stroke
            _timeOfFastStroke = (0.5 * _timeOfStroke) / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0);
            // positive sensation, in is faster
            if (_sensation > 0.0) {
                _timeOfInStroke = _timeOfFastStroke;
                _timeOfOutStroke = _timeOfStroke - _timeOfFastStroke;
            // negative sensation, out is faster
            } else {
                _timeOfOutStroke = _timeOfFastStroke;
                _timeOfInStroke = _timeOfStroke - _timeOfFastStroke;
            }
#ifdef DEBUG_PATTERN
            Serial.println("TimeOfInStroke: " + String(_timeOfInStroke));
            Serial.println("TimeOfOutStroke: " + String(_timeOfOutStroke));
#endif
        }
};


/**************************************************************************/
/*!
  @brief  Robot Stroke Pattern. Sensation controls the acceleration of the
  stroke. Positive value increase acceleration until it is a constant speed
  motion (feels robotic). Neutral is equal to simple stroke (1/3, 1/3, 1/3).
  Negative a reduces acceleration into a triangle profile.
*/
/**************************************************************************/ 
class RoboStroke : public Pattern {
    public:
        RoboStroke(char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to devide by 2
            _timeOfStroke = 0.5 * speed; 
        }

        void setSensation(float sensation = 0) { 
            _sensation = sensation;
            // scale sensation into the range [0.05, 0.5] where 0 = 1/3
            if (sensation >= 0 ) {
              _x = fscale(0.0, 100.0, 1.0/3.0, 0.5, sensation, 0.0);
            } else {
              _x = fscale(0.0, -100.0, 1.0/3.0, 0.05, sensation, 0.0);
            }
#ifdef DEBUG_PATTERN
            Serial.println("Sensation:" + String(sensation,0) + " --> " + String(_x,6));
#endif
        }

        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion
            float speed = float(_stroke) / ((1 - _x) * _timeOfStroke);
            _nextMove.speed = int(speed);  

            // acceleration to meet the profile
            _nextMove.acceleration = int(speed / (_x * _timeOfStroke));

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.position = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.position = _depth;
            }

            _index = index;
            return _nextMove;
        }
    protected:
        float _x = 1.0/3.0;
};

/**************************************************************************/
/*!
  @brief  Like Teasing or Pounding, but every second stroke is only half the
  depth. The sensation value can change the speed ratio between in and out. 
  Sensation > 0 make the in move faster (up to 5x) giving a hard pounding 
  sensation. Values < 0 make the out move going faster. This gives a more 
  pleasing sensation. The time for the overall stroke remains the same for
  all strokes, even half ones. 
*/
/**************************************************************************/
class HalfnHalf : public Pattern {
    public:
        HalfnHalf(char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateStrokeTiming();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
            _updateStrokeTiming();
        }
        motionParameter nextTarget(unsigned int index) {
            // check if this is the very first 
            if (index == 0) {
              //pattern started for the very fist time, so we start gentle with a half move
              _half = true;
            }

            // set-up the stroke length
            int stroke = _stroke;
            int insert = _depth;
            if (_half == true) {
                // half the stroke length
                stroke = _stroke / 2;
                // stroke only extends half the insertion depth
                insert = _depth - stroke;
            } 

            // odd stroke is moving out
            if (index % 2) {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * stroke/_timeOfOutStroke);  
                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfOutStroke);    
                _nextMove.position = _depth - _stroke;
                // every second move is half
                _half = !_half;
            // even stroke is moving in
            } else {
                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * stroke/_timeOfInStroke);        
                // acceleration to meet the profile            
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfInStroke);    
                _nextMove.position = insert;  
            }
            _index = index;
            return _nextMove;
        }
    protected:
        float _timeOfFastStroke = 1.0;
        float _timeOfInStroke = 1.0;
        float _timeOfOutStroke = 1.0;
        bool _half = true;
        void _updateStrokeTiming() {
            // calculate the time it takes to complete the faster stroke
            // Division by 2 because reference is a half stroke
            _timeOfFastStroke = (0.5 * _timeOfStroke) / fscale(0.0, 100.0, 1.0, 5.0, abs(_sensation), 0.0);
            // positive sensation, in is faster
            if (_sensation > 0.0) {
                _timeOfInStroke = _timeOfFastStroke;
                _timeOfOutStroke = _timeOfStroke - _timeOfFastStroke;
            // negative sensation, out is faster
            } else {
                _timeOfOutStroke = _timeOfFastStroke;
                _timeOfInStroke = _timeOfStroke - _timeOfFastStroke;
            }
#ifdef DEBUG_PATTERN
            Serial.println("TimeOfInStroke: " + String(_timeOfInStroke));
            Serial.println("TimeOfOutStroke: " + String(_timeOfOutStroke));
#endif
        }
};

/**************************************************************************/
/*!
  @brief  The insertion depth ramps up gradually with each stroke until it
  reaches its maximum. It then resets and restars. Sensations controls the
  slope of the ramp. 
*/
/**************************************************************************/
class Deeper : public Pattern {
    public:
        Deeper(char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to devide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        void setSensation(float sensation) { 
            _sensation = sensation;

            // maps sensation to useful values [2,22] with 12 beeing neutral
            if (sensation < 0) {
                _countStrokesForRamp = map(sensation, -100, 0, 2, 11);
            } else {
                _countStrokesForRamp = map(sensation, 0, 100, 11, 32);
            }
#ifdef DEBUG_PATTERN
            Serial.println("_countStrokesForrRamp: " + String(_countStrokesForRamp));
#endif
        }

        motionParameter nextTarget(unsigned int index) {
            // How many steps is each stroke advancing         
            int slope = _stroke / (_countStrokesForRamp);

            // The pattern recycles so we use modulo to get a cycling index.
            // Factor 2 because index increments with each full stroke twice
            // add 1 because modulor = 0 is index = 1
            int cycleIndex = (index / 2) % _countStrokesForRamp + 1;

            // This might be not smooth, as the insertion depth may jump when 
            // sensation is adjusted.

            // Amplitude is slope * cycleIndex
            int amplitude = slope * cycleIndex;
#ifdef DEBUG_PATTERN
            Serial.println("amplitude: " + String(amplitude)
                         + " cycleIndex: " + String(cycleIndex));
#endif

            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * amplitude/_timeOfStroke);  

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.position = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.position = _depth - _stroke + amplitude;
            }

            _index = index;
            return _nextMove;
        }
    
    protected:
        int _countStrokesForRamp = 2;

};

/**************************************************************************/
/*
  Array holding all different patterns. Please include any custom pattern here.
*/
/**************************************************************************/
static Pattern *patternTable[] = { 
  new SimpleStroke("Simple Stroke"),
  new TeasingPounding("Teasing or Pounding"),
  new RoboStroke("Robo Stroke"),
  new HalfnHalf("Half'n'Half")
  // <-- insert your new pattern class here!
 };

static const unsigned int patternTableSize = sizeof(patternTable) / sizeof(patternTable[0]);
