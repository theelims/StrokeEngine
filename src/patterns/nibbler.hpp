#pragma once
#include "pattern.h"

/**************************************************************************/
/*!
  @brief  Simple vibrational overlay pattern. Vibrates on the way in and out. 
  Sensation sets the vibration amplitude.
*/
/**************************************************************************/
class StrokeNibbler : public Pattern {
    public:
        StrokeNibbler(const char *str) : Pattern(str) {}
        void setSensation(float sensation) { 
            _sensation = sensation;
            _updateVibrationParameters();
        }
        void setTimeOfStroke(float speed = 0) {
            _timeOfStroke = speed;
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

            // revert position to start for the first stroke
            if (index == 0) {
                // Set motion parameter
                _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);
                _nextMove.acceleration = _maxAcceleration;

                // go to back position
                _nextMove.stroke = _depth - _stroke;

                // store index and return
                _index = index;
                return _nextMove;
            }

            // Vibration happens at maximum speed and acceleration of the machine
            _nextMove.speed = _maxSpeed;
            _nextMove.acceleration = _maxAcceleration;

            // check if we have reached one of the ends and reverse direction
            if (_nextMove.stroke >= _depth) {
                _returnStroke = true;
            } 
            if (_nextMove.stroke <= (_depth - _stroke)) {
                _returnStroke = false;
            }

            // only calculate new position, if index has incremented: no mid-stroke update, as vibration is sufficiently fast
            if (index != _index) {
                if (_returnStroke == true) {
                    // long vibration distance on way out
                    // odd stroke is shaking out
                    if (index % 2) {  
                        // limit stroke to _depth - _stroke
                        // TODO - Fix this - _nextMove.stroke = max(_nextMove.stroke - _inVibrationDistance, _depth - _stroke);

                    // even stroke is shaking in
                    } else {
                        _nextMove.stroke = _nextMove.stroke + _outVibrationDistance;
                    }

                } else {
                    // long vibration distance on way in
                    // odd stroke is shaking out
                    if (index % 2) {  
                        _nextMove.stroke = _nextMove.stroke - _outVibrationDistance;

                    // even stroke is shaking in
                    } else {
                        // limit stroke to _depth
                        // TODO - Fix this - _nextMove.stroke = min(_nextMove.stroke + _inVibrationDistance, _depth);
                    }
                }
            }

            _index = index;
            return _nextMove;
        }
    protected:
        bool _returnStroke = false;
        int _inVibrationDistance = 0;
        int _outVibrationDistance = 0;
        int _strokeSpeed = 0;
        void _updateVibrationParameters() {
            // Empirical factor to compensate time losses due to finite acceleration.
            _strokeSpeed = int(5.0 * _stroke/_timeOfStroke);

            // Scale vibration amplitude from 3mm to 25mm with sensation
            _inVibrationDistance = (int)fscale(-100.0, 100.0, (float)(3.0*_stepsPerMM), (float)(25.0*_stepsPerMM), _sensation, 0.0);

            /* Calculate _outVibrationDistance to match with stroking speed
               d_out = d_in * (v_vib - v_stroke) / (v_vib + v_stroke)
               Formula neglects acceleration. Real timing will be slower due to finite acceleration & deceleration
            */
           _outVibrationDistance = _inVibrationDistance * (_maxSpeed - _strokeSpeed) / (_maxSpeed + _strokeSpeed);

#ifdef DEBUG_PATTERN
            Serial.println("_maxSpeed: " + String(_maxSpeed) + " _strokeSpeed: " + String(_strokeSpeed));
            Serial.println("inDist: " + String(_inVibrationDistance) + " outDist: " + String(_outVibrationDistance));
#endif
            
        }
};