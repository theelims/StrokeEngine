# StrokeEngine
StrokeEngine provides a unified way of handling Motion Generation and feeding these Motions into Motor Drivers.
It comes bundled with a set of generic Patterns and Drivers allowing anyone with a [OSSM](https://github.com/KinkyMakers/OSSM-hardware), or other similar hardware, to be quickly up and running.

This Library will allow you to take full advantage of the freedom a linear-positionable stroking or fucking machine can provide over fixed cam-driven designs. To this date there are only few commercial offerings using this advantage. And often so the implementation is rather boring, not utilizing the full possibilities of such a linear position drive.

This project has full support for usage with
- [Open Source Sex Machine](https://github.com/KinkyMakers/OSSM-hardware) (Recommended approach)
- LinMot-based Custom Machines
- Step/Dir-based Custom Machines

but can be adapted to any DIY fucking machine.

There are several Libraries which provide more advanced integration on top of StrokeEngine
- [FuckIO](https://github.com/theelims/FuckIO)
- [CANFuck](https://github.com/zylos146/CANFuck)

> Table of contents  
> <span class="mono">¶</span>&emsp;[Getting Started](#getting-started)  
> <span class="mono">¶</span>&emsp;[Motion Generation](#motion-supported)  
> <span class="mono">¶</span>&emsp;[Drives Supported](#drives-supported)  
> <span class="mono">¶</span>&emsp;[Internals](#internals)

<a name="getting-started"></a>
# Getting Started

## Usage
StrokeEngine aims to have a simple and straight forward, yet powerful API. The following describes the minimum case to get up and running. All input parameters need to be specified in real world (metric) units.

### > Initialize
First all parameters of the machine and the servo need to be set. Including the pins for interacting with the driver and an (optionally) homing switch.
```cpp
#include "esp_log.h"

#include "motors/stepper.hpp"
#include "StrokeEngine.h"

// Pin Definitions
#define SERVO_PULSE       4
#define SERVO_DIR         16
#define SERVO_ENABLE      17
#define SERVO_ENDSTOP     25        // Optional: Only needed if you have a homing switch

// Calculation Aid:
#define STEP_PER_REV      2000      // How many steps per revolution of the motor (S1 off, S2 on, S3 on, S4 off)
#define PULLEY_TEETH      20        // How many teeth has the pulley
#define BELT_PITCH        2         // What is the timing belt pitch in mm
#define MAX_RPM           3000.0    // Maximum RPM of motor
#define STEP_PER_MM       STEP_PER_REV / (PULLEY_TEETH * BELT_PITCH)
#define MAX_SPEED         (MAX_RPM / 60.0) * PULLEY_TEETH * BELT_PITCH // Max Speed in mm/s

StepperMotor* motor;
StrokeEngine* engine;
```
Inside `void setup()` call the following functions to initialize the StrokeEngine:
```cpp
// TODO - Can we move app_motion into some basic wrapper? People shouldn't need to handle this part
float position = 0;
void app_motion(void *pvParameter) {
  while (true) {
    if (motor->isInState(MotorState::ACTIVE) && motor->hasStatusFlag(MOTOR_FLAG_HOMED) && !engine->isActive()) {
      ESP_LOGE("main", "Motor ready and homed. Attempting to start Stroke Engine");
      engine->startPattern();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void loop() {}
void setup() {
  Serial.begin(115200);

  ESP_LOGI("main", "Configuring Motor");
  motor = new StepperMotor();

  motionBounds bounds = {
    .start = 100, // mm
    .end = 0, // mm
    .keepout = 5 // mm
  };
  motor->setMaxSpeed(MAX_SPEED); // 5 m/s
  motor->setMaxAcceleration(25000); // 25 m/s^2
  motor->setBounds(bounds);
  motor->setStepsPerMm(STEP_PER_MM);
  motor->setMovementPin(SERVO_PULSE, SERVO_DIR);
  motor->setEnablePin(SERVO_ENABLE, true);
  motor->setSensoredHoming(SERVO_ENDSTOP, INPUT_PULLUP, false);

  ESP_LOGI("main", "Configuring Stroke Engine");
  engine = new StrokeEngine();
  c->attachMotor(motor);
  engine->setParameter(StrokeParameter::PATTERN, 0);
  engine->setParameter(StrokeParameter::RATE, 50);
  engine->setParameter(StrokeParameter::DEPTH, 100);
  engine->setParameter(StrokeParameter::STROKE, 50);
  engine->setParameter(StrokeParameter::SENSATION, 0);

  ESP_LOGI("main", "Homing Motor");
  motor->enable();
  motor->goToHome();
  
  ESP_LOGI("main", "Starting Motion Task!");
  xTaskCreate(&app_motion, "app_motion", 4096, NULL, 5, NULL);
}
```

#### > Retrieve Available Patterns as JSON-String
This is an example snippet showing how `Stroker.getNumberOfPattern()` and `Stroker.getPatternName(i)` may be used to iterate through the available patterns and composing a JSON-String.
```cpp
String getPatternJSON() {
    String JSON = "[{\"";
    for (size_t i = 0; i < engine->getNumberOfPattern(); i++) {
        JSON += String( engine->getPatternName(i));
        JSON += "\": ";
        JSON += String(i, DEC);
        if (i < engine->getNumberOfPattern() - 1) {
            JSON += "},{\"";
        } else {
            JSON += "}]";
        }
    }
    Serial.println(JSON);
    return JSON;
}
```

### Running
#### > Start & Stop the Stroking Action
Use `engine->startPattern();` and `engine->stopPattern();` to start and stop the motion. Stop is immediate and with the highest possible acceleration.

#### > Move to the Minimum or Maximum Position
You can move to either end of the machine for setting up reaches. Call `motor->goToPos(0, speed, acceleratio)` to move all they way back towards home. With `motor->goToPos(motor->getBounds().end, speed, acceleration)` it moves all the way out.

#### > Setup Optimal Depth/Stroke Interactively
There is a special pattern `Adjust Depth` which allows setting up an optimal depth/stroke. 

This pattern allows not only to interactively adjust `depth`, but also `stroke` by using the sensation slider. `sensation` gets mapped into the interval `[depth-stroke, depth]`: `sensation = 100` adjusts `depth`-position, whereas `sensation = -100` adjusts the `stroke`-position. `sensation = 0` yields the midpoint of the stroke.

#### > Set Parameters
```cpp
enum class StrokeParameter {
  // RATE & SPEED are mutually exclusive. Only one can be specified at a time!
  // RATE - Range 0.5 to 600 Strokes / Min
  // Can allow better control typically than just SPEED, as other machines use
  RATE,

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

void setParameter(StrokeParameter parameter, float value, bool applyNow = false);
float getParameter(StrokeParameter parameter);
```
Parameters can be updated in any state and are stored internally. On `engine->startPattern();` they will be used to initialize the pattern. Each one may be get/set individually. The argument given to the function is constrained to the physical limits of the machine (depth/stroke within `[0, maxTravel]`)
```cpp
engine->setParameter(StrokeParameter::PATTERN, 0); // Pattern, index must be < Stroker.getNumberOfPattern()
engine->setParameter(StrokeParameter::RATE, 50); // Speed in Cycles (in & out) per minute
engine->setParameter(StrokeParameter::DEPTH, 100); // Depth in mm
engine->setParameter(StrokeParameter::STROKE, 50); // Stroke length in mm
engine->setParameter(StrokeParameter::SENSATION, 0); // Sensation (arbitrary value a pattern may use to alter its behavior), constrained to [-100, 100] with 0 being neutral.
```
Normally a parameter change is only executed after the current stroke has finished. However, sometimes it is desired to have the changes take effect immediately, even mid-stroke. In that case set the argument `bool applyNow` to `true`. 

#### > Get Parameters
```cpp
float value = engine->getParameter(StrokeParameter::PATTERN); // Pattern, index must be < Stroker.getNumberOfPattern()
float value = engine->setParameter(StrokeParameter::RATE); // Speed in Cycles (in & out) per minute
float value = engine->setParameter(StrokeParameter::DEPTH); // Depth in mm
float value = engine->setParameter(StrokeParameter::STROKE); // Stroke length in mm
float value = engine->setParameter(StrokeParameter::SENSATION); // Sensation (arbitrary value a pattern may use to alter its behavior), constrained to [-100, 100] with 0 being neutral.
```

<a name="motion-supported"></a>
# Motion Generation

## Streaming
StrokeEngine will support ingesting Motion Commands or T-Codes and sending them directly to the Motor. These will act in a similar way to G-Code. Either a pre-built file can be loaded, or an external integration can stream them in.

Streaming/Loading T-Codes is not currently supported, but is a intended goal.

Once built, the hope is other devices like Deep Throat Trainer can be integrated to send commands to mimic the sucking motion, on a OSSM perhaps equiped with a Fleshlight or similar. 

## Patterns
Patterns allow a set of pre-built, real-time configurable strokes to be generated on-demand.
These Strokes will typically be cyclic, always eventually repeating what has happened before.

## Directors
Directors are complex Meta-Patterns that allow orchestrating a Scene, or set of Patterns. This allows, for example, a Throat Fucking Director to randomly vary between deep thrusts, short thrusts, pauses, throat fucking, and other patterns.

The Directors are meant to provide a more varied, but automatic experience, without needing manual user input.

<a name="drives-supported"></a>
# Drives Supported
Motors in StrokeEngine are abstracted away, to allow the end-user to provide a compatible motor implementation.
So if your motor uses a new communication protocol, like EtherCAT or Ethernet/IP, the work needed to integrate StrokeEngine is kept minimal.

StrokeEngine Core offers a few basic drivers for
- Step/Dir Pulse-based drives (TB6600)
- iHSV
Motors in StrokeEngine are abstracted away, to allow the end-user to provide a compatible motor implementation.
So if your motor uses a new communication protocol, like EtherCAT or Ethernet/IP, the work needed to integrate StrokeEngine is kept minimal.

StrokeEngine Core supports a few motors by default. Other motors are provided via External Libraries
### StrokeEngine Core
- ModBus 
  - :heavy_check_mark: iHSV57 V6xx Servos 100/140/180W (`IHSV57Motor` in `motor/ihsv57.hpp`)
  - :x: iHSV57 V5xx is **NOT SUPPORTED** due to differing communication protocol with V6xx drivers
- Step/Dir Pulses (`StepperMotor` in `motor/stepper.hpp`)
  - :heavy_check_mark: TB6600 drives
  - :heavy_check_mark: Other generic Step/Dir stepper/servo drives

### [StrokeEngine-CANOpen](https://github.com/zylos146/StrokeEngine-CANOpen)
- CANOpen
  - LinMot CiA 402 CANOpen capable drives
    - :x: E1200-EC / E1200-DC are not supported unless EtherCat support is built
    - :heavy_check_mark: E1200-GP-xx (Haven't confirmed yet, but should work)
    - :heavy_check_mark: E1100-GP-xx (Haven't confirmed yet, but should work)
    - :heavy_check_mark: E1100-CO-xx (Haven't confirmed yet, but should work)
    - :heavy_check_mark: B1100-GP-xx
    - :heavy_check_mark: A1100-GP-LC
  - Other CiA 402 drives could likely be ported with minimal effort
- EtherCat - Not yet supported, but might be in the future if demand arises
  - CiA 402
    - :question: Rtelligent ECT60 / ECT40 Closed-Loop Servos
    - :question: LinMot EtherCat CiA 402 capable drives (E1200-DC, etc)

<a name="internals"></a>
# Internals
## Homing
Homing can be handled in two different ways.
- Sensor-less - Home will be determined by moving in a specific direction until a desired current, and thus torqe/force threshold has been reached. This threshold indicates the machine has reached it's start of motion point.
- Sensored - Home will be determined by moving in a specific direction until a physical switch has been activated

### Coordinate System
The machine uses an internal metric (mm, m/s, m/s<sup>2</sup>) coordinate system for all motion planning. This offers an advantage, that this is independent of a specific implementation and works with all machine sizes and regardless of the motor chosen.

Each motor is expected to handle converting from metric Motion Commands into their own relative coordinate system. There is a standard way mapping these two coordinate systems is handled.

![Coordinate System](./doc/coordinates.svg)
* The system is 1-dimensional and the positive move direction is towards the front a.k.a. towards the body.
* The __physicalTravel__ is the real physical travel the machine has from one hard endstop to the other. 
* From `physicalTravel` a safety distance called __keepoutBoundary__ is subtracted on each side giving the real working distance **_travel**: 
  ```
  _travel = physicalTravel - (2 * keepoutBoundary)
  ``` 
  This gives a safety margin to avoid crashes into a hard endstop.
* The __Home__-position is expected to be at `-keepoutBoundary`. Albeit not recommended for safety reasons, it is possible to mount the home switch in the front at `physicalTravel` as well.
* Zero __MIN = 0__ is `keepoutBoundary` away from the home position.
* Pattern make use of __Depth__ and __Stroke__. These values are dynamic parameter and may be adjusted during runtime:
  * __Depth__ is the furthest point the machine can extract at any given time. This is useful to find a sweet spot in positioning the body relative to the machine.
  * __Stroke__ is the longest working distance a stroking motion will have.

Think of __Stroke__ as the amplitude and __Depth__ a linear offset that is added.

### Pattern
One of the biggest benefits of a linear position drive over a cam-driven motion is its versatility. StrokeEngine uses a pattern generator to provide a wide variety of sensations where parameters like speed, stroke and depth are adjusted dynamically on a motion by motion basis. It uses a trapezoidal motion profile with a defined acceleration and deceleration distance. In between it moves with a constant speed. Pattern take __depth__, __stroke__, __speed__ and an arbitrary __sensation__ parameter. In [Pattern.md](./Pattern.md) you can find a detailed description of each available pattern. Also some information how to write your own patterns and contribute them to this project.

### Graceful Behavior
One design goal was to have a unobtrusive failure handling when invalid parameters are given. Either from the user with values that lay outside the physics of the machine, or from a pattern commanding an impossible speed, position or acceleration. 

All `MotorInterface` implementations are expected to constrain any Motion Command which will exit the provided `motionBounds`. The command will still execute, but the full range of motion will be cut short. This manifests in a distortion of the motion. Strokes may be shortened when position targets outside of the machine bounds are requested (e.g. `stroke > depth`). Acceleration and speed are limited leading to distorted ramps. The motion is executed over the full distance, but may take slightly longer then expected to reach the target position. 

### Mid-Stroke Parameter Update
It is possible to update any parameter like depth, stroke, speed and pattern mid-stroke. This gives a very responsive and fluid user experience. Safeguards are in place to ensure the move stays inside the bounds of the machine at any time.
