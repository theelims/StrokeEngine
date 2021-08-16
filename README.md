# StrokeEngine
A library to create a variety of stroking motions with a stepper or servo motor on an ESP32. A usage example can be found in my other related project: [FuckIO](https://github.com/theelims/FuckIO). It will work with all kinds of stepper / servo operated fucking and stroking machines. An other popular example is the Kinky Makers [OSSM-Project](https://github.com/KinkyMakers/OSSM-hardware)

Every DIY fucking machine with a linear position drive powered by a stepper or servo motor can be used with this library. 

## Concepts
StrokeEngine takes full advantage of the freedom a servo / stepper driven stroking or fucking machine can provide over fixed cam-driven designs. To this date there are only few commercial offerings using this advantage. And often so the implementation is rather boring, not utilizing the full possibilities of such a linear position drive. 

Unter the hood it uses the fabulous [FastAccelStepper](https://github.com/gin66/FastAccelStepper) library to interface stepper or servo motors with commonly found STEP / DIR interfaces.

Understanding the underlying concepts will help you to get up and running with StrokeEngine faster. 

### Coordinate System
The machine spans it's own internal cooridnate system. It takes the real world (metric) units and converts them into the internal coordiante system just counting the encoder / stepper steps of the motor and vice versa. This offers the advantage, that this is independent of a specific implementation and works with all machine sizes and regardless of the motor choosen. 

![Coordinate System](./doc/coordinates.svg)
* The system is 1-dimensional and the positive move direction is towards the front a.k.a. towards the body.
* The __physicalTravel__ is the real physical travel the machine has from one hard endstop to the other. 
* From `physicalTravel` a safety distance called __keepoutBoundary__ is subtracted on each side giving the real working distance **_travel**: 
  ```
  _travel = physicalTravel - (2 * keepoutBoundary)
  ``` 
  This gives a safety margin to avoid crashes into a hard endstop.
* The __Home__-position is expected to be at `-keepoutBoundary`.
* Zero __MIN = 0__ is `keepoutBoundary` away from the home position.
* Pattern make use of __Depth__ and __Stroke__. These values are dynamic parameter and may be adjusted during runtime:
  * __Depth__ is the furthest point the machine can extract at any given time. This is usefull to find a sweet spot in positioning the body relative to the machine.
  * __Stroke__ is the longest working distance a stroking motion will have.

Think of __Stroke__ as the amplitude and __Depth__ a linear offset that is added.

### Pattern
One of the biggest benefits of a linear position drive over a cam-driven motion is its versatility. StrokeEngine uses a pattern generator to provide a wide varity of sensations where parameters like speed, stroke and depth are adjusted dynamically on a motion by motion basis. It uses a trapezoidal motion profile with a defined acceleration and deceleartion distance. In between it moves with a constant speed. Pattern take __depth__, __stroke__, __speed__ and an arbitrary __sensation__ parameter. In [Pattern.md](./Pattern.md) you can find a detailed description of each available pattern. Also some information how to write your own patterns and contribute them to this project.

### Graceful Behaviour
One design goal was to have a unobtrusive failure handling when invalid parameters are given. Either from the user with values that lay outside the physics of the machine, or from a pattern commanding an impossible speed, position or acceleration. All set-functions make use of a `constrain()`-function to limit the input to the physical capabilities of the given machine. Values outside the bounds are simply cropped. 

Also on the pattern side `constrain()` is used to ensure no impossible motion commands leading to crahses or step losses are executed. This manifests in a distortion of the motion. Strokes may be shortened when position targets outside of the machine bounds are requested (e.g. `stroke > depth`). Acceleration and speed are limited leading to  distorted ramps. The motion is executed over the full distance, but may take slightly longer then expected to reach the target position. 

### Mid-Stroke Parameter Update
It is possible to update any parameter like depth, stroke, speed and pattern mid-stroke. This gives a very responsive and fluid user experience. Safeguards are in place to ensure the move stays inside the bounds of the machine at any time.

### State Machine
An internal finite state machine handles the different states of the machine. See the below graph with all functions relating to the state machine and how the cause transitions:

![State Machine](./doc/state-machine.svg)
* __SERVO_DISABLED:__ The initial state prior to homeing. Stepper / Servo are disabled and the position is undefined.
* __SERVO_READY:__ Homeing defines the position inside the internal coordinate system. Machine is now ready to be used and accepts motion commands.
* __SERVO_RUNNING:__ The cyclic motion has started and the pattern generator is commanding a sequence of trapezoidal motions until stopped.
* __SERVO_ERROR:__ If a motor fault is detected the state machine enters this state. This is a dead-end. To clear this fault the power from the drive has to be removed and the EPS32 needs to be resetted. 

## Usage
StrokeEngine aims to have a simple and straight forward, yet powerful API. The follwoing describes the minimum case to get up and running. All input parameteres need to be specified in real world (metric) units.
### Initialize
First all parameteres of the machine and the servo need to be set. Including the pins for interacting with the driver and an (optional) homeing switch.
```cpp
#include <StrokeEngine.h>

// Pin Definitions
#define SERVO_PULSE       4
#define SERVO_DIR         16
#define SERVO_ENABLE      17
#define SERVO_ENDSTOP     25          // Optional: Only needed if you have a homeing switch

// Step per mm calculation helper:
#define STEP_PER_REV      3200        // How many steps per revolution has the motor
#define PULLEY_TEETH      20          // How many teeth has the pulley
#define BELT_PITCH        2           // What is the timing belt pitch in mm
#define STEP_PER_MM       STEP_PER_REV / (PULLEY_TEETH * BELT_PITCH)

static motorProperties servoMotor {
  .stepsPerRevolution = STEP_PER_REV, // How many steps per revolution has the motor    
  .maxRPM = 2900,                     // Maximum RPM your motor can / should go
  .maxAcceleration = 300000,          // Maximum linear acceleration in mm/s²
  .stepsPerMillimeter = STEP_PER_MM,  // Steps per millimeter 
  .invertDirection = true,            // One of many ways to change the direction of 
                                      // things move the wrong way
  .enableActiveLow = true,            // Polarity of the enable signal      
  .stepPin = SERVO_PULSE,             // Pin of the STEP signal
  .directionPin = SERVO_DIR,          // Pin of the DIR signal
  .enablePin = SERVO_ENABLE           // Pin of the enable signal
};

static machineGeometry strokingMachine = {
  .physicalTravel = 160.0,            // Real physical travel from one hard endstop to the other
  .keepoutBoundary = 5.0              // Safe distance the motion is constrained to avoiding crashes
};

StrokeEngine Stroker;
```
Inside `void setup()` call the follwing functions to initialize the StrokeEngine:
```cpp
void setup() 
{
  // Setup Stroke Engine
  Stroker.begin(&strokingMachine, &servoMotor);
  Stroker.enableAndHome(SERVO_ENDSTOP, true);   // Pin and polarity (active low = true) of the
                                                // homeing switch
  
  // other initialisation code
  
  // wait for homeing to complete
  while (Stroker.getState() != SERVO_READY) {
    delay(100);
  }
}
```

#### Alternate Manual Homeing Procedure __[Dangerous]__
Some machines may not have a homeing switch mounted. For these you may use a manual homeing procedure instead of `Stroker.enableAndHome(SERVO_ENDSTOP, true);`. Manually move back until the physical endstop and then call:
```cpp
Stroker.thisIsHome();
```
This enables the driver and sets the current position as `-keepoutBoundary`. It then slowly moves to 0. 

__Be sure to know what you do. If this function is called while not at the physical endstop the internal coordinate system is off resulting in a certain crash! This could damage your machine!__

#### Retrieve Available Patterns as JSON-String
This is an example snippet showing how `Stroker.getNumberOfPattern()` and `Stroker.getPatternName(i)` may be used to iterrate through the available patterns and composing a JSON-String.
```cpp
String getPatternJSON() {
    String JSON = "[{\"";
    for (size_t i = 0; i < Stroker.getNumberOfPattern(); i++) {
        JSON += String(Stroker.getPatternName(i));
        JSON += "\": ";
        JSON += String(i, DEC);
        if (i < Stroker.getNumberOfPattern() - 1) {
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
#### Start & Stop the Stroking Action
Use `Stroker.startMotion();` and `Stroker.stopMotion();` to start and stop the motion. Stop is immideate and with the the highest possible acceleration.

#### Move to the Minimum or Maximum Position
You can move to either end of the machine for setting up reaches. Call `Stroker.moveToMin();` to move all they way back towards home. With `Stroker.moveToMax();` it moves all the way out. Takes the speed in mm/s as an argument: e.g. `Stroker.moveToMax(10.0);` Speed defaults to 10 mm/s. Can be called from states `SERVO_RUNNING` and `SERVO_READY` and stops any current motion. Returns `false` if called in a wrong state.

#### Change Parameters
Parameters can be updated at any state and are stored internally. On `Stroker.startMotion();` they will be used to initialize the pattern. Each one may be called individually. The argument given to the function is constrained to the physical limits of the machine:
```cpp
Stroker.setSpeed(speed);          // Speed in Cycles (in & out) per minute, constrained from 0.5 to 6000
Stroker.setDepth(depth);          // Depth in mm, constrained to [0, _travel]
Stroker.setStroke(stroke);        // Stroke length in mm, constrained to [0, _travel]
Stroker.setSensation(sensation);  // Sensation (arbitrary value a pattern may use to alter its behaviour), 
                                  // constrained to [-100, 100] with 0 beeing neutral.
Stroker.setPattern(index);        // Pattern, index must be < Stroker.getNumberOfPattern()
```
Normally a parameter change is only executed after the current stroke has finished. However, sometimes it is desired to have the changes take effect immideately, even mid-stroke. In that case call 
```cpp
Stroker.applyNewSettingsNow();
```
directly after changing one or more parameter.

#### Advanced Functions
Consult [StrokeEngine.h](./src/StrokeEngine.h) for further functions and a more detailed documentation of each function. Some functions are overloaded and may have additional useful functionalities.
