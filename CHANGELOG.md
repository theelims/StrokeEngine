# Upcoming Release 0.2.1
- set and get functions for maximum speed and maximum acceleration. Allows to change these limits during runtime.
- Renamed `#define DEBUG_VERBOSE` to `#define DEBUG_TALKATIVE` to make StrokeEngine play nice with WifiManager.
- Removed state `ERROR` from state machine. After tests with the servo this has become pointless. `disable()` and homing will clear any error state. 
- Fixed bug with missing implementation of inverted direction signal: https://github.com/theelims/StrokeEngine/issues/3
- Added an improved pause handling mechanism to pattern:
  - removed `void patternDelay(int milliseconds)` function as unfit for the purpose
  - Pattern-Class has 3 private functions `void _startDelay()`, `void _updateDelay(int delayInMillis)` and `bool _isStillDelayed()` to add an internal delay function based on comparing `millis()`.
  - A pattern can request StrokeEngine to skip the current motion command by returning `_nextMove.skip = true;`. 
- Speed and acceleration limits are available inside pattern, now. 
- New pattern available:
  - Stop'n'Go
  - Insist

## Planned Features
- New Pattern:
  - Closing Gap
  - Stroke Nibble
  - Jack Hammer

# Release 0.2.0
- Stroking-task uses suspend and resume instead of deleting and recreation to prevent heap fragmentation.
- Refined homing procedure. Fully configurable via a struct including front or back  switch and internal pull-up or pull-down resistors. 
- Added a delay-function for patterns: `void patternDelay(int milliseconds)`
- Debug messages report real world units instead of steps.
- Special debug messages for clipping when speed or acceleration limits are hit.
- Several changes how pattern feed their information back:
  - Removed `setDepth()`, as this information is not needed inside a pattern.
  - Pattern return relative stroke and not absolute coordinates.
  - Depth-offset is calculated and properly constrained inside StrokeEngine.
  - All pattern updated accordingly. 
  - Also fixed the erratic move bug. When depth changes the necessary transfer motion is carried out at the same speed as the overall motion.
- Some minor default parameter changes in the example code snippets.

## Update Notes
### Homing Procedure
New syntax for `Stroker.enableAndHome()`. It now expects a pointer to a `endstopProperties`-Struct configuring the  behavior.
```cpp
// Configure Homing Procedure
static endstopProperties endstop = {
  .homeToBack = true,             // Endstop sits at the rear of the machine
  .activeLow = true,              // switch is wired active low
  .endstopPin = SERVO_ENDSTOP,    // Pin number
  .pinMode = INPUT                // pinmode INPUT with external pull-up resistor
};

...

// Home StrokeEngine
Stroker.enableAndHome(&endstop);
```

# Release 0.1.1
- Constructor of Pattern-class changed to type `const char*` to get rid of compiler warning.
- Fixed homing-Bug.

# Release 0.1.0
- First "official" release
- Renamed function `Stroker.startMotion()` to `Stroker.startPattern()`
- Renamed all states of the statemachine
- Provide set and get functions for maxSpeed and maxAcceleration
- Fancy adjustment mode
- Changed members of struct `motorProperties` to streamline interface:
  - introduced `maxSpeed`
  - deleted `maxRPM` and `stepsPerRevolution`
