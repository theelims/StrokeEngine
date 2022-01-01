# Pattern
Patterns are what set this StrokeEngine appart. They allow to bring a lot of variety into play. Pattern are small programs that the StrokeEngine is using to create the next set of trapezoidal motion parameter (Target Position, Speed & Acceleration).

## Description of Available Pattern
### Simple Stroke
Simple Stroke Pattern. It creates a trapezoidal stroke profile with 1/3 acceleration, 1/3 coasting, 1/3 deceleration. Sensation has no effect.

### Teasing or Pounding
The sensation value can change the speed ratio between in and out. Sensation > 0 makes the in-move faster (up to 5x) giving a hard pounding sensation. Values < 0 make the out-move going faster. This gives a more pleasing sensation. The time for the overall stroke remains always the same and only depends on the global speed parameter.

### Robo Stroke
Sensation controls the acceleration of the stroke. Positive value increase acceleration until it is a constant speed motion (feels robotic). Neutral is equal to simple stroke (1/3, 1/3, 1/3). Negative reduces acceleration into a triangle profile.

### Half'n'Half
Similar to Teasing or Pounding, but every second stroke is only half the depth. The sensation value can change the speed ratio between in and out. Sensation > 0 make the in move faster (up to 5x) giving a hard pounding sensation. Values < 0 make the out move going faster. This gives a more pleasing sensation. The time for the overall stroke remains the same for all strokes, even half ones.

### Deeper
The insertion depth ramps up gradually with each stroke until it reaches its maximum. It then resets and restarts. Sensations controls how many strokes there are in a ramp.

## Contribute a Pattern
Making your own pattern is not that hard. They can be found in the header only [pattern.h](./src/pattern.h) and easily extended.

### Subclass Pattern in pattern.h
To create a new pattern just subclass from `class Pattern`. Have a look at `class SimpleStroke` for the most basic implementation:
```cpp
class SimpleStroke : public Pattern {
    public:
        SimpleStroke(const char *str) : Pattern(str) {} 
```
Add the constructor to store the patterns name string.

Reimplement set-functions if you need them to do some math:
```cpp
        void setTimeOfStroke(float speed = 0) { 
             // In & Out have same time, so we need to divide by 2
            _timeOfStroke = 0.5 * speed; 
        }   
```
Reimplement the core function `motionParameter nextTarget(unsigned int index)`. This is mandatory, as the StrokeEngine will call this fuction after each stroke to get the next set of `motionParameter`. Each time this function is called the `index` increments by 1. If the pattern is called the very first time this always starts as 0. This can be used to create pattern that vary over time:
```cpp
        motionParameter nextTarget(unsigned int index) {
            // maximum speed of the trapezoidal motion 
            _nextMove.speed = int(1.5 * _stroke/_timeOfStroke);  

            // acceleration to meet the profile
            _nextMove.acceleration = int(3.0 * _nextMove.speed/_timeOfStroke);

            // odd stroke is moving out    
            if (index % 2) {
                _nextMove.stroke = 0;
            
            // even stroke is moving in
            } else {
                _nextMove.stroke = _stroke;
            }

            _index = index;
            return _nextMove;
        }
};
```
If you need further helper functions and variables use the `protected:` section to implement them.

For debugging and verifying the math it can be handy to have something on the Serial Monitor. Please encapsulate the `Serial.print()` statement so it can be turned on and off.
```cpp
#ifdef DEBUG_PATTERN
            Serial.println("TimeOfInStroke: " + String(_timeOfInStroke));
            Serial.println("TimeOfOutStroke: " + String(_timeOfOutStroke));
#endif
```
Don't forget to add an instance of your new pattern class at the very bottom of the file to the `*patternTable[]`-Array.
```cpp
static Pattern *patternTable[] = { 
  new SimpleStroke("Simple Stroke"),
  new TeasingPounding("Teasing or Pounding")
  // <-- insert your new pattern class here!
 };
```
### Expected Behavior
#### Adhere to Depth & Stroke at All Times
Depth and Stroke set in StrokeEngine are axiomatic. StrokeEngine closely monitors the returned `motionParameter` and ensures no violation against the machines physics were returned. Pattern only return a stroke information which is offset by depth in the StrokeEngine. Your return value may be anywhere in the interval [0, stroke]. Positions outside the interval [depth - stroke, depth] will be truncated, leading to a distortion of your intended stroke profile. This is an integral safety feature to prevent injuries. This sets the envelope the pattern may use. Similar for speed a.k.a. timeOfStroke. 

#### Use `index` Properly 
`index` provides further information then just the stroke count:
* It always starts at `0` if a pattern is called the first time. It resets with every call of `StrokeEngine.setPattern(int)` or `StrokeEngine.startMotion()`.
* It increments after each successfully executed move.
* Store the last index in `_index` before returning. By comparing `index == _index` you can determine that this time it is not a new stroke, but rather an update of a current stroke. This information can be handy in pattern varying over time.

### Pull Request
Make a pull request for your new [pattern.h](./src/pattern.h) after you thoroughly tested it. 
