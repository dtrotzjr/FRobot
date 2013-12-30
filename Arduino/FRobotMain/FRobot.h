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
  int PerformScanForBestPath();
  void NavigateTowardClearestPath();
  
  void PostStatus();
  
  // Servos
  Servo mSteerServo;
  Servo mSonarServo;
  
  // Current State
  float* mLastScanValues;
  int* mLastScanAngles;
  byte mLastScanValuesLength;
  int mCurrentForwardClearance;
  boolean mMovingForward;
  
  // Pin Assignments
  static const byte MOTOR_CTL_A_PIN;
  static const byte MOTOR_CTL_B_PIN;
  static const byte MOTOR_CTL_ENABLE_PIN;
  static const byte MOTOR_CTL_C_PIN;
  static const byte MOTOR_CTL_D_PIN;
  static const byte SERVO_STEERING_PIN;
  static const byte SERVO_SONAR_PIN;
  static const byte SONAR_TRIGGER_PIN;
  static const byte SONAR_ECHO_PIN;
  
  // Servo Centering Values
  static const byte STEERING_CENTER_ANGLE;
  static const byte SONAR_CENTER_ANGLE;
  
  // Magic Numbers for Servos
  static const byte MAX_STEERING_ANGLE;
  static const byte MAX_SCAN_ANGLE;
  static const byte SCAN_ANGLE_STEP;
};
