#pragma once
#include "pattern.h"

/**************************************************************************/
/*!
  @brief  Robot Stroke Pattern. Sensation controls the acceleration of the
  stroke. Positive value increase acceleration until it is a constant speed
  motion (feels robotic). Neutral is equal to simple stroke (1/3, 1/3, 1/3).
  Negative reduces acceleration into a triangle profile.
*/
/**************************************************************************/ 
class RoboStroke : public Pattern {
    public:
        RoboStroke(const char *str) : Pattern(str) {}

        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }

        void setSensation(float sensation = 0) { 
            _sensation = sensation;
            // scale sensation into the range [0.05, 0.5] where 0 = 1/3
            if (sensation >= 0 ) {
              _x = fscale(0.0, 100.0, 1.0/3.0, 0.5, sensation, 0.0);
            } else {
              _x = fscale(0.0, 100.0, 1.0/3.0, 0.05, -sensation, 0.0);
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
                _nextMove.stroke = _depth - _stroke;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = _depth;
            }

            _index = index;
            return _nextMove;
        }
    protected:
        float _x = 1.0/3.0;
};