#include "FRobot.h"

const byte FRobot::MOTOR_CTL_A = 7;
const byte FRobot::MOTOR_CTL_B = 8;
const byte FRobot::MOTOR_CTL_ENABLE = 3;
const byte FRobot::MOTOR_CTL_C = 12;
const byte FRobot::MOTOR_CTL_D = 13;
const byte FRobot::SERVO_STEERING_PIN = 9;
const byte FRobot::SERVO_SONAR_PIN = 10;
const byte FRobot::FRobot::SONAR_TRIGGER = 4;
const byte FRobot::SONAR_ECHO = 5;
const byte FRobot::STEERING_CENTER_ANGLE = 90;
const byte FRobot::SONAR_CENTER_ANGLE = 90;

const byte FRobot::MAX_STEERING_ANGLE = 25;
const byte FRobot::MAX_SCAN_ANGLE = 50;

FRobot::FRobot() {
}

FRobot::~FRobot() {
}

void FRobot::Initialize() {
	Serial.begin(115200);
 
	mSteerServo.attach(SERVO_STEERING_PIN);
	mSteerServo.write(STEERING_CENTER_ANGLE);  
	mSonarServo.attach(SERVO_SONAR_PIN);
	mSonarServo.write(SONAR_CENTER_ANGLE);
	pinMode(SONAR_TRIGGER, OUTPUT);
	pinMode(SONAR_ECHO, INPUT);
	pinMode(MOTOR_CTL_A, OUTPUT);
	pinMode(MOTOR_CTL_B, OUTPUT);
	pinMode(MOTOR_CTL_ENABLE, OUTPUT);
	pinMode(MOTOR_CTL_C, OUTPUT);
	pinMode(MOTOR_CTL_D, OUTPUT);
	digitalWrite(MOTOR_CTL_ENABLE, LOW);
}

void FRobot::Step() {
	int clearance = (int)ReadSonarDistance();
	if (clearance > 50) {
		GoForward(false, 255);
	} else {
		NavigateTowardClearestPath();
	}
}

void FRobot::NavigateTowardClearestPath() {
	Stop();
	int angle = PerformScanForBestPath();
	int absAngle = abs(angle);
	int multiplier = 0;
	if (angle < 0)
		multiplier = -1;
	else if (angle > 0)
		multiplier = 1;
	if (multiplier != 0) {
		mSteerServo.write(STEERING_CENTER_ANGLE + (multiplier * MAX_STEERING_ANGLE));
		GoBackward(true, 255);
		double timeMultiplier = ((double)absAngle/(double)MAX_SCAN_ANGLE);
		delay(1000 * timeMultiplier);
		mSteerServo.write(STEERING_CENTER_ANGLE + (-1 * multiplier * MAX_STEERING_ANGLE));
		GoForward(true, 255);
		delay(750 * timeMultiplier);
		Stop();
		mSteerServo.write(STEERING_CENTER_ANGLE);
	}
}

void FRobot::GoForward(boolean fadeIn, byte maxSpeed)
{
	digitalWrite(MOTOR_CTL_ENABLE, LOW);
	digitalWrite(MOTOR_CTL_A, LOW);
	digitalWrite(MOTOR_CTL_B, HIGH);
	digitalWrite(MOTOR_CTL_C, LOW);  
	digitalWrite(MOTOR_CTL_D, HIGH);    
	if (fadeIn) { 
		for(byte i = (maxSpeed/2); i <= maxSpeed; i += (maxSpeed/16)) {
			analogWrite(MOTOR_CTL_ENABLE, i);  
			delay(maxSpeed/8);
		}
	}
	if (!fadeIn || maxSpeed == 255)
		digitalWrite(MOTOR_CTL_ENABLE, HIGH);
}

void FRobot::Stop()
{
	digitalWrite(MOTOR_CTL_ENABLE, LOW);
	digitalWrite(MOTOR_CTL_A, HIGH);
	digitalWrite(MOTOR_CTL_B, HIGH);
	digitalWrite(MOTOR_CTL_C, HIGH);  
	digitalWrite(MOTOR_CTL_D, HIGH);    
	digitalWrite(MOTOR_CTL_ENABLE, HIGH);  
	delay(1000);
	digitalWrite(MOTOR_CTL_ENABLE, LOW);  
}

void FRobot::GoBackward(boolean fadeIn, byte maxSpeed)
{
	digitalWrite(MOTOR_CTL_ENABLE, LOW);
	digitalWrite(MOTOR_CTL_A, HIGH);
	digitalWrite(MOTOR_CTL_B, LOW);
	digitalWrite(MOTOR_CTL_C, HIGH);  
	digitalWrite(MOTOR_CTL_D, LOW); 
	if (fadeIn) { 
		for(byte i = (maxSpeed/2); i <= maxSpeed; i += (maxSpeed/16)) {
			analogWrite(MOTOR_CTL_ENABLE, i);  
			delay(maxSpeed/8);
		}
	}
	if (!fadeIn || maxSpeed == 255)
		digitalWrite(MOTOR_CTL_ENABLE, HIGH);
}

float FRobot::ReadSonarDistance()
{
	/* The following trigPin/echoPin cycle is used to determine the
	distance of the nearest object by bouncing soundwaves off of it. 
	*/ 
	digitalWrite(SONAR_TRIGGER, LOW); 
	delayMicroseconds(2);

	digitalWrite(SONAR_TRIGGER, HIGH);
	delayMicroseconds(10); 
 
	digitalWrite(SONAR_TRIGGER, LOW);
	unsigned long duration = pulseIn(SONAR_ECHO, HIGH);
   
	//Calculate the distance (in cm) based on the speed of sound.
	return  duration/58.2f;
}

/* 
 * Note: Servo seems to take less than 4ms per degree of motion.
 * negative values indicate the right side of vehicle
 * positive values indicate the left side of vehicle
 */
byte FRobot::PerformScanForBestPath() {
	float furthestDistance = 0.0f;
	float currentDistance = 0.0f;  
	byte furthestDistanceAngle = 0;
	mSonarServo.write(SONAR_CENTER_ANGLE - MAX_SCAN_ANGLE);
	delay(MAX_SCAN_ANGLE*4 + 25);
	for(byte currentAngle = -MAX_SCAN_ANGLE; currentAngle <= MAX_SCAN_ANGLE; currentAngle += 5) {
		mSonarServo.write(SONAR_CENTER_ANGLE + currentAngle);
		delay(4*5 + 25);
		currentDistance = ReadSonarDistance();
		if (currentDistance < 3000 && currentDistance > furthestDistance) {
			furthestDistance = currentDistance;
			furthestDistanceAngle = currentAngle;
		}
	}
	mSonarServo.write(SONAR_CENTER_ANGLE);
	delay(MAX_SCAN_ANGLE*4 + 25);
	return furthestDistanceAngle;
}
