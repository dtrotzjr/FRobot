#include "FRobot.h"

// Pin Assignments
const byte FRobot::MOTOR_CTL_A_PIN 			= 7;
const byte FRobot::MOTOR_CTL_B_PIN 			= 8;
const byte FRobot::MOTOR_CTL_ENABLE_PIN 	= 3;
const byte FRobot::MOTOR_CTL_C_PIN 			= 12;
const byte FRobot::MOTOR_CTL_D_PIN 			= 13;
const byte FRobot::SERVO_STEERING_PIN 		= 9;
const byte FRobot::SERVO_SONAR_PIN 			= 10;
const byte FRobot::SONAR_TRIGGER_PIN		= 4;
const byte FRobot::SONAR_ECHO_PIN 			= 5;

// Servo Centering Values
const byte FRobot::STEERING_CENTER_ANGLE 	= 90;
const byte FRobot::SONAR_CENTER_ANGLE 		= 90;

// Magic Numbers for Servos
const byte FRobot::MAX_STEERING_ANGLE 		= 25;
const byte FRobot::MAX_SCAN_ANGLE 			= 50;
const byte FRobot::SCAN_ANGLE_STEP			= 5;


FRobot::FRobot() {
	mLastScanValuesLength = (2 * (MAX_SCAN_ANGLE/SCAN_ANGLE_STEP)) + 1;
	mLastScanValues = new float[mLastScanValuesLength];
	mLastScanAngles = new int[mLastScanValuesLength];
}

FRobot::~FRobot() {
	delete [] mLastScanValues;
	delete [] mLastScanAngles;
}

void FRobot::Initialize() {
	Serial.begin(57600);
 
	mSteerServo.attach(SERVO_STEERING_PIN);
	mSteerServo.write(STEERING_CENTER_ANGLE);  
	mSonarServo.attach(SERVO_SONAR_PIN);
	mSonarServo.write(SONAR_CENTER_ANGLE);
	pinMode(SONAR_TRIGGER_PIN, OUTPUT);
	pinMode(SONAR_ECHO_PIN, INPUT);
	pinMode(MOTOR_CTL_A_PIN, OUTPUT);
	pinMode(MOTOR_CTL_B_PIN, OUTPUT);
	pinMode(MOTOR_CTL_ENABLE_PIN, OUTPUT);
	pinMode(MOTOR_CTL_C_PIN, OUTPUT);
	pinMode(MOTOR_CTL_D_PIN, OUTPUT);
	digitalWrite(MOTOR_CTL_ENABLE_PIN, LOW);
	
	mCurrentForwardClearance = 0;
	for (int arrayIndex = 0; arrayIndex < mLastScanValuesLength; arrayIndex++) {
		mLastScanValues[arrayIndex] = 0;
		mLastScanAngles[arrayIndex] = 0;
	}
	mMovingForward = false;
	mAutoMode = true;
}

void FRobot::Step() {
	mCurrentForwardClearance = (int)ReadSonarDistance();
	if (mCurrentForwardClearance > 50) {
		if (!mMovingForward)
			GoForward(false, 255);
		mMovingForward = true;
	} else {
		mMovingForward = false;
		NavigateTowardClearestPath();
	}
	PostStatus();
}

void FRobot::PostStatus() {
	Serial.print("FR+CLR:");
	Serial.println(mCurrentForwardClearance);
	Serial.print("FR+MOV:");
	Serial.println(mMovingForward);	
	
	if(!mMovingForward)
	{
		Serial.print("FR+SONAR:{");		
		for(int i = 0; i < mLastScanValuesLength; ++i)
		{
			Serial.print("(");
			Serial.print(mLastScanAngles[i]);
			Serial.print(",");
			Serial.print(mLastScanValues[i]);		
			Serial.print(") ");
		}
		Serial.println("}");	
	}
	else
	{
		Serial.println("FR+SONAR:{}");
	}
}

boolean FRobot::InAutoMode() {
	if(Serial.peek() != -1) {
		
		int length = Serial.readBytesUntil('\n', mInputBuffer, MAX_INPUT_BUFFER_LEN);
		ParseInputBufer(length);
	}
	return mAutoMode;
	
}

void FRobot::NavigateTowardClearestPath() {
	Stop();
	int angle = PerformScanForBestPath();
	Serial.print("Scanned... ");Serial.println(angle);
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
		Serial.print("Sleeping... ");Serial.println(timeMultiplier);
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
	digitalWrite(MOTOR_CTL_ENABLE_PIN, LOW);
	digitalWrite(MOTOR_CTL_A_PIN, LOW);
	digitalWrite(MOTOR_CTL_B_PIN, HIGH);
	digitalWrite(MOTOR_CTL_C_PIN, LOW);  
	digitalWrite(MOTOR_CTL_D_PIN, HIGH);    
	if (fadeIn) { 
		for(int i = (maxSpeed/2); i <= maxSpeed; i += (maxSpeed/16)) {
			analogWrite(MOTOR_CTL_ENABLE_PIN, i);  
			delay(maxSpeed/8);
		}
	}
	if (maxSpeed == 255)
		digitalWrite(MOTOR_CTL_ENABLE_PIN, HIGH);
	else
		analogWrite(MOTOR_CTL_ENABLE_PIN, maxSpeed);
}

void FRobot::Stop()
{
	digitalWrite(MOTOR_CTL_ENABLE_PIN, LOW);
	digitalWrite(MOTOR_CTL_A_PIN, HIGH);
	digitalWrite(MOTOR_CTL_B_PIN, HIGH);
	digitalWrite(MOTOR_CTL_C_PIN, HIGH);  
	digitalWrite(MOTOR_CTL_D_PIN, HIGH);    
	digitalWrite(MOTOR_CTL_ENABLE_PIN, HIGH);  
	delay(1000);
	digitalWrite(MOTOR_CTL_ENABLE_PIN, LOW);  
}

void FRobot::GoBackward(boolean fadeIn, byte maxSpeed)
{
	digitalWrite(MOTOR_CTL_ENABLE_PIN, LOW);
	digitalWrite(MOTOR_CTL_A_PIN, HIGH);
	digitalWrite(MOTOR_CTL_B_PIN, LOW);
	digitalWrite(MOTOR_CTL_C_PIN, HIGH);  
	digitalWrite(MOTOR_CTL_D_PIN, LOW); 
	if (fadeIn) { 
		for(int i = (maxSpeed/2); i <= maxSpeed; i += (maxSpeed/16)) {
			analogWrite(MOTOR_CTL_ENABLE_PIN, i);  
			delay(maxSpeed/8);
		}
	}
	if (maxSpeed == 255)
		digitalWrite(MOTOR_CTL_ENABLE_PIN, HIGH);
	else
		analogWrite(MOTOR_CTL_ENABLE_PIN, maxSpeed);
}

float FRobot::ReadSonarDistance()
{
	/* The following trigPin/echoPin cycle is used to determine the
	distance of the nearest object by bouncing soundwaves off of it. 
	*/ 
	digitalWrite(SONAR_TRIGGER_PIN, LOW); 
	delayMicroseconds(2);

	digitalWrite(SONAR_TRIGGER_PIN, HIGH);
	delayMicroseconds(10); 
 
	digitalWrite(SONAR_TRIGGER_PIN, LOW);
	unsigned long duration = pulseIn(SONAR_ECHO_PIN, HIGH);
   
	//Calculate the distance (in cm) based on the speed of sound.
	return  duration/58.2f;
}

/* 
 * Note: Servo seems to take less than 4ms per degree of motion.
 * So you will see appropriate delays after each write command
 * to the servo.
 *
 * negative values indicate the right side of vehicle
 * positive values indicate the left side of vehicle
 */
int FRobot::PerformScanForBestPath() {
	float furthestDistance = 0.0f;
	float currentDistance = 0.0f;  
	int furthestDistanceAngle = 0;
	
	mSonarServo.write(SONAR_CENTER_ANGLE - MAX_SCAN_ANGLE);
	delay(MAX_SCAN_ANGLE*4 + 25);
	
	int arrayIndex = 0;
	for(int currentAngle = -MAX_SCAN_ANGLE; currentAngle <= MAX_SCAN_ANGLE; currentAngle += SCAN_ANGLE_STEP) {
		mSonarServo.write(SONAR_CENTER_ANGLE + currentAngle);
		delay(4*5 + 25);
		
		currentDistance = ReadSonarDistance();
		
		mLastScanValues[arrayIndex] = currentDistance;
		mLastScanAngles[arrayIndex] = currentAngle;
		arrayIndex++;
		
		if (currentDistance < 3000 && currentDistance > furthestDistance) {
			furthestDistance = currentDistance;
			furthestDistanceAngle = currentAngle;
		}
	}
	
	mSonarServo.write(SONAR_CENTER_ANGLE);
	delay(MAX_SCAN_ANGLE*4 + 25);
	
	return furthestDistanceAngle;
}
