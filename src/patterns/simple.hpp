#pragma once
#include "pattern.h"

/**************************************************************************/
/*!
  @brief  Simple Stroke Pattern. It creates a trapezoidal stroke profile
  with 1/3 acceleration, 1/3 coasting, 1/3 deceleration. Sensation has 
  no effect.
*/
/**************************************************************************/
class SimpleStroke : public Pattern {
    public:
        SimpleStroke(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   

        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion 
            _nextMove.speed = 1.5 * _stroke/_timeOfStroke;

            // acceleration to meet the profile
            _nextMove.acceleration = 3.0 * _nextMove.speed/_timeOfStroke;

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = _depth;
            }

            _index = index;
            return _nextMove;
        }
};