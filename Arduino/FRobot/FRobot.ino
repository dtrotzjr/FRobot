#include <Servo.h>
#include "FRobot.h"

FRobot robot;

void setup() {
  robot.Initialize();
}







void loop() {
  robot.Step();
}
