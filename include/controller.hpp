#pragma once

#include "engines/stroke.hpp"
#include "motor.hpp"

class Controller {
    void attach(StrokeEngine* engine, MotorInterface* motor);
    virtual void loop();

  protected:
    StrokeEngine* engine;
};