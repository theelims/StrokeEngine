<<<<<<< Updated upstream
=======
# Upcomming Release 0.2.0
- Constructor of Pattern-class changed to type `const char*` to get rid of compiler warning.
- Stroking-task uses suspend and resume instead of deleting and recreation to prevent heap fragmentation.
- Refined homeing procedure. Fully configurable via a struct including front or back homeing switch and internal pull-up or pull-down resistors. 
- Added a delay-function for patterns: `void patternDelay(int milliseconds)`

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

>>>>>>> Stashed changes
# Release 0.1.0
- First "official" release
- Renamed function `Stroker.startMotion()` to `Stroker.startPattern()`
- Renamed all states of the statemachine
- Provide set and get functions for maxSpeed and maxAcceleration
- Fancy adjustment mode
- Changed members of struct `motorProperties` to streamline interface:
  - introduced `maxSpeed`
  - deleted `maxRPM` and `stepsPerRevolution`
