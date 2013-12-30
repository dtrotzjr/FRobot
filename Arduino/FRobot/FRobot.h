#include "Arduino.h"
#include <Servo.h>

class FRobot {
  public:
  FRobot();
  virtual ~FRobot();
  
  
  void Initialize();
  void Step();
  
  protected:
  void GoForward(boolean fadeIn, byte maxSpeed);
  void GoBackward(boolean fadeIn, byte maxSpeed);
  void Stop();
  
  float ReadSonarDistance();
  byte PerformScanForBestPath();
  void NavigateTowardClearestPath();
  
  Servo mSteerServo;
  Servo mSonarServo;

  static const byte MOTOR_CTL_A;
  static const byte MOTOR_CTL_B;
  static const byte MOTOR_CTL_ENABLE;
  static const byte MOTOR_CTL_C;
  static const byte MOTOR_CTL_D;
  static const byte SERVO_STEERING_PIN;
  static const byte SERVO_SONAR_PIN;
  static const byte SONAR_TRIGGER;
  static const byte SONAR_ECHO;
  static const byte STEERING_CENTER_ANGLE;
  static const byte SONAR_CENTER_ANGLE;
  
  static const byte MAX_STEERING_ANGLE;
  static const byte MAX_SCAN_ANGLE;
};
