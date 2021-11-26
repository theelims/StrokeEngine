# Release 0.2.0
- Stroking-task uses suspend and resume instead of deleting and recreation to prevent heap fragmentation.
- Refined homeing procedure. Fully configurable via a struct including front or back homeing switch and internal pull-up or pull-down resistors. 
- Added a delay-function for patterns: `void patternDelay(int milliseconds)`
- Debug messages report real world units instead of steps.
- Special debug messages for clipping when speed or acceleration limits are hit.
- Several changes how pattern feed their information back:
  - Removed `setDepth()`, as this information is not needed inside a pattern.
  - Pattern return relative stroke and not absolute coordinates.
  - Depth-offset is calculated and properly constrained inside StrokeEgine.
  - All pattern updated accordingly. 
  - Also fixed the erratic move bug. When depth changes the necessary transfer motion is carried out at the same speed as the overall motion.
- Some minor default parameter changes in the example code snippets.

## Update Notes
### Homeing Procedure
New syntax for `Stroker.enableAndHome()`. It now expects a pointer to a `endstopProperties`-Struct configuring the homeing behaviour.
```cpp
// Configure Homeing Procedure
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
