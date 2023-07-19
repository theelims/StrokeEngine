#pragma once
#include "pattern.h"

/**************************************************************************/
/*!
  @brief  Vibrational pattern that works like a jack hammer. Vibrates on the 
  way in and pulls out smoothly in one go. Sensation sets the vibration 
  amplitude.
*/
/**************************************************************************/
class JackHammer : public Pattern {
    public:
        JackHammer(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateVibrationParameters();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = 0.5 * speed;
            _updateVibrationParameters();
        }
        void setStroke(int stroke) { 
            _stroke = stroke;
            _updateVibrationParameters();
        }
        void setSpeedLimit(unsigned int maxSpeed, unsigned int maxAcceleration, unsigned int stepsPerMM) { 
            _maxSpeed = maxSpeed; 
            _maxAcceleration = maxAcceleration; 
            _stepsPerMM = stepsPerMM;
            _updateVibrationParameters(); 
        }
        motionParameter nextTarget(unsigned int index) {

            // revert position for the first move or if depth is exceeded
            if (index == 0 || _nextMove.stroke >= _depth) {
                // Return strokes goes at regular speed without vibration back to 0

                // maximum speed of the trapezoidal motion
                _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);  
                // acceleration to meet the profile                  
                _nextMove.acceleration = int(3.0 * float(_nextMove.speed)/_timeOfStroke);  
                // all they way out to start
                _nextMove.stroke = _depth - _stroke;
                // we are done here
                return _nextMove;
            }

            // only calculate new position, if index has incremented: no mid-stroke update, as vibration is sufficiently fast
            // except for return stroke if depth is exceeded.
            if (index != _index) {
             
                // Vibration happens at maximum speed and acceleration of the machine
                _nextMove.speed = _maxSpeed;
                _nextMove.acceleration = _maxAcceleration;

                // odd stroke is shaking out
                if (index % 2) {  
                    _nextMove.stroke = _nextMove.stroke - _outVibrationDistance;
                // even stroke is shaking in
                } else {
                    // limit range to _depth
                    // TODO - Fix this - _nextMove.stroke = min((_nextMove.stroke + _inVibrationDistance), _depth);
                }
            }
            _index = index;
            return _nextMove;
        }
    protected:
        int _inVibrationDistance = 0;
        int _outVibrationDistance = 0;
        int _strokeInSpeed = 0;
        void _updateVibrationParameters() {
            // Hammering in takes considerable longer then backing off
            _strokeInSpeed = int(0.5 * _stroke/_timeOfStroke);

            // Scale vibration amplitude from 1mm to 15mm with sensation
            _inVibrationDistance = (int)fscale(-100.0, 100.0, (float)(3.0*_stepsPerMM), (float)(25.0*_stepsPerMM), _sensation, 0.0);

            /* Calculate _outVibrationDistance to match with stroking speed
               d_out = d_in * (v_vib - v_stroke) / (v_vib + v_stroke)
               Formula neglects acceleration. Real timing will be slower due to finite acceleration & deceleration
            */
           _outVibrationDistance = _inVibrationDistance * (_maxSpeed - _strokeInSpeed) / (_maxSpeed + _strokeInSpeed);

#ifdef DEBUG_PATTERN
            Serial.println("_maxSpeed: " + String(_maxSpeed) + " _strokeInSpeed: " + String(_strokeInSpeed)  + " _strokeOutSpeed: " + String(int(1.5 * _stroke/_timeOfStroke)));
            Serial.println("inDist: " + String(_inVibrationDistance) + " outDist: " + String(_outVibrationDistance));
#endif
            
        }
};