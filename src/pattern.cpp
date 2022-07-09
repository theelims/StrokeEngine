#include "pattern.h"

#include "patterns/deeper.hpp"
#include "patterns/halfnhalf.hpp"
#include "patterns/insist.hpp"
#include "patterns/jackhammer.hpp"
#include "patterns/nibbler.hpp"
#include "patterns/robostroke.hpp"
#include "patterns/simple.hpp"
#include "patterns/stopngo.hpp"
#include "patterns/teasing.hpp"

/**************************************************************************/
/*
  Array holding all different patterns. Please include any custom pattern here.
*/
/**************************************************************************/
Pattern* patternTable[9] = { 
  new SimpleStroke("Simple Stroke"),
  new TeasingPounding("Teasing or Pounding"),
  new RoboStroke("Robo Stroke"),
  new HalfnHalf("Half'n'Half"),
  new Deeper("Deeper"),
  new StopNGo("Stop'n'Go"),
  new Insist("Insist"),
  new JackHammer("Jack Hammer"),
  new StrokeNibbler("Stroke Nibbler")
  // <-- insert your new pattern class here!
 };