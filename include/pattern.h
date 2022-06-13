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

#include "duktape.h"

#include "PatternMath.h"

typedef enum {
  LINEAR_MOVE = 0,
  ROTATE_MOVE = 1,
  VIBRATE = 2,
  AUXILLIARY = 3,
  DWELL = 4,
  DEVICE = 5,
  PARAMETERS = 6
} MotionCommandType;

class MotionCommand {
  MotionCommandType type;
};

class MotionLinearMove : public MotionCommand {
  float magnitude;
  float duration;
  float speed;
};

typedef struct {
    int stroke;         //!< Absolute and properly constrainted target position of a move in steps 
    int speed;          //!< Speed of a move in Steps/second 
    int acceleration;   //!< Acceleration to get to speed or halt 
    bool skip;          //!< no valid stroke, skip this set an query for the next --> allows pauses between strokes
} motionParameter;

class Pattern {
  public:
    static register(const char *name, )
  
      Pattern(const char *str) { strcpy(_name, str); }
      char *getName() { return _name; }

      virtual motionParameter nextTarget(unsigned int index) {
          _index = index;
          return _nextMove;
      } 
  protected:
      float _stroke;
      float _depth;
      float _timeOfStroke;
      float _sensation = 0.0;

      int _index = -1;
      char _name[256]; 
      motionParameter _nextMove = {0, 0, 0, false};
};

static Pattern *patternTable[] = {

};

static const unsigned int patternTableSize = sizeof(patternTable) / sizeof(patternTable[0]);
