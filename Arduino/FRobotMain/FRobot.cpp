#include "FRobot.h"

const byte FRobot::SONAR_RING_BUFFER_MAX = 3;

// Pin Assignments
const byte FRobot::MOTOR_CTL_A_PIN          = 7;
const byte FRobot::MOTOR_CTL_B_PIN          = 8;
const byte FRobot::MOTOR_CTL_ENABLE_PIN     = 3;
const byte FRobot::MOTOR_CTL_C_PIN          = 12;
const byte FRobot::MOTOR_CTL_D_PIN          = 13;
const byte FRobot::SERVO_STEERING_PIN       = 9;
const byte FRobot::SERVO_SONAR_PIN          = 10;
const byte FRobot::SONAR_TRIGGER_PIN        = 5;
const byte FRobot::SONAR_ECHO_PIN           = 4;

// Servo Centering Values
const byte FRobot::STEERING_CENTER_ANGLE    = 85;
const byte FRobot::SONAR_CENTER_ANGLE       = 90;

// Magic Numbers for Servos
const byte FRobot::MAX_STEERING_ANGLE       = 25;
const byte FRobot::MAX_SCAN_ANGLE           = 50;
const byte FRobot::SCAN_ANGLE_STEP          = 5;

const int FRobot::MAX_INPUT_BUFFER_LEN      = 256;

// Serial Commands
String FRobot::SERIAL_SEND_CLEARANCE                = "FS+CLR:";
String FRobot::SERIAL_SEND_MOVING                   = "FS+MOV:";
String FRobot::SERIAL_SEND_AUTO                     = "FS+AUTO:";
String FRobot::SERIAL_SEND_SONAR_PREFIX             = "FS+SONAR:{";
String FRobot::SERIAL_SEND_SONAR_POSTFIX            = "}";

String FRobot::SERIAL_RECEIVE_AUTO                  = "FR+AUTO";
String FRobot::SERIAL_RECEIVE_OVERRIDE              = "FR+OVERRIDE";        
String FRobot::SERIAL_RECEIVE_DPAD_PREFIX           = "FR+DPAD:";
String FRobot::SERIAL_RECEIVE_DPAD_FORWARD_POSTFIX  = "FWD:";
String FRobot::SERIAL_RECEIVE_DPAD_REVERSE_POSTFIX  = "REV:";
String FRobot::SERIAL_RECEIVE_DPAD_LEFT_POSTFIX     = "LT:"; 
String FRobot::SERIAL_RECEIVE_DPAD_RIGHT_POSTFIX    = "RT:";
String FRobot::SERIAL_RECEIVE_DPAD_STOP_POSTFIX     = "S:";

// #define TEST_WITHOUT_MOTORS

FRobot::FRobot() {
    mLastScanValuesLength = (2 * (MAX_SCAN_ANGLE/SCAN_ANGLE_STEP)) + 1;
    mLastScanValues = new float[mLastScanValuesLength];
    mLastScanAngles = new int[mLastScanValuesLength];
    mInputBuffer = new char[MAX_INPUT_BUFFER_LEN];
    mSonarRingBuffer = new unsigned long[SONAR_RING_BUFFER_MAX];
}

FRobot::~FRobot() {
    delete [] mLastScanValues;
    delete [] mLastScanAngles;
    delete [] mInputBuffer;
}

void FRobot::Initialize() {
    Serial.begin(115200);
 
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
    
    for(int bufferIndex = 0; bufferIndex < MAX_INPUT_BUFFER_LEN; ++bufferIndex)
        mInputBuffer[bufferIndex] = 0;
    
    mCurrentForwardClearance = 0;
    for (int arrayIndex = 0; arrayIndex < mLastScanValuesLength; arrayIndex++) {
        mLastScanValues[arrayIndex] = 0;
        mLastScanAngles[arrayIndex] = 0;
    }
    mMovingForward = false;
    mAutoMode = true;
    
    for (int ringBufferIndex = 0; ringBufferIndex < SONAR_RING_BUFFER_MAX; ringBufferIndex++)
        mSonarRingBuffer[ringBufferIndex] = 0;
    mSonarRingBufferIndex = 0;
}

void FRobot::Step() {
    mCurrentForwardClearance = (int)ReadSonarDistance();
    ParseInputBufer();
    if (mAutoMode) {
        if (mCurrentForwardClearance > 50) {
            if (!mMovingForward)
                GoForward(false, 255);
            mMovingForward = true;
        } else {
            mMovingForward = false;
            NavigateTowardClearestPath();
        }
    }
    PostStatus();
}

void FRobot::PostStatus() {
    ParseInputBufer();
    Serial.print(SERIAL_SEND_CLEARANCE);
    Serial.println(mCurrentForwardClearance);
    ParseInputBufer();
    Serial.print(SERIAL_SEND_MOVING);
    Serial.println(mMovingForward); 
    ParseInputBufer();
    Serial.print(SERIAL_SEND_AUTO);
    Serial.println(mAutoMode); 
    PostSonarStatus(); 
}

void FRobot::PostSonarStatus() {
    ParseInputBufer();
    Serial.print(SERIAL_SEND_SONAR_PREFIX);
    if(!mMovingForward && mAutoMode)
    {
        for(int i = 0; i < mLastScanValuesLength; ++i)
        {
            Serial.print("(");
            Serial.print(mLastScanAngles[i]);
            Serial.print(",");
            Serial.print(mLastScanValues[i]);       
            Serial.print(")|");
        }
    } else {
        Serial.print("(");
        Serial.print(0);
        Serial.print(",");
        Serial.print(mCurrentForwardClearance);     
        Serial.print(") ");
    }
    Serial.println(SERIAL_SEND_SONAR_POSTFIX);
}

boolean FRobot::InAutoMode() {
    return mAutoMode;
}

void FRobot::ParseInputBufer() {
    while(Serial.peek() != -1) {
        int len = Serial.readBytesUntil('\n', mInputBuffer, MAX_INPUT_BUFFER_LEN);
        mInputBuffer[len] = 0;
        String input = String(mInputBuffer);
        if(input.equals(SERIAL_RECEIVE_AUTO)) {
            mAutoMode = true;
        } else if(mAutoMode && input.equals(SERIAL_RECEIVE_OVERRIDE)) {
            mAutoMode = false;
        } else if(!mAutoMode && input.startsWith(SERIAL_RECEIVE_DPAD_PREFIX)) {
            int prefixLen = SERIAL_RECEIVE_DPAD_PREFIX.length();
            int cmdEndIndex = input.indexOf(':', prefixLen) + 1;
            String direction = input.substring(prefixLen, cmdEndIndex);
            String magnitudeStr = input.substring(cmdEndIndex);
            int magnitude = magnitudeStr.toInt();            
            if (SERIAL_RECEIVE_DPAD_FORWARD_POSTFIX.equals(direction))
                GoForward(false, magnitude);
            else if (SERIAL_RECEIVE_DPAD_REVERSE_POSTFIX.equals(direction))
                GoBackward(false, magnitude);
            else if (SERIAL_RECEIVE_DPAD_LEFT_POSTFIX.equals(direction))
                TurnWheelsLeft(magnitude);
            else if (SERIAL_RECEIVE_DPAD_RIGHT_POSTFIX.equals(direction))
                TurnWheelsRight(magnitude);
            else if (SERIAL_RECEIVE_DPAD_STOP_POSTFIX.equals(direction))
                Stop(magnitude);            
        }
    }
}

/*
 * There are a lot of checks in here to Parse the Input Buffer 
 * this is to give the bluetooth remote a chance to cancel this action
 * as soon as possible rather than to wait for this entire method to
 * finish out.
 */
void FRobot::NavigateTowardClearestPath() {
    Stop(true);
    int angle = PerformScanForBestPath();
    ParseInputBufer();
    if (mAutoMode) {
        int absAngle = abs(angle);
        int multiplier = 0;
        if (angle < 0)
            multiplier = -1;
        else if (angle > 0)
            multiplier = 1;
        if (multiplier != 0) {
            ParseInputBufer();
            if(mAutoMode)
            {
                TurnWheels(MAX_STEERING_ANGLE, multiplier);
                GoBackward(true, 255);
                double timeMultiplier = ((double)absAngle/(double)MAX_SCAN_ANGLE);
                delay(1000 * timeMultiplier);
                ParseInputBufer();
                if(mAutoMode)
                {
                    TurnWheels(MAX_STEERING_ANGLE, -multiplier);
                    GoForward(true, 255);
                    delay(750 * timeMultiplier);
                }
            }
            Stop(true);
            mSteerServo.write(STEERING_CENTER_ANGLE);
        }
    }
}

void FRobot::GoForward(boolean fadeIn, byte maxSpeed)
{
#ifndef TEST_WITHOUT_MOTORS
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
#endif    
}

void FRobot::TurnWheelsLeft(byte angle)
{
    TurnWheels(angle, -1);
}

void FRobot::TurnWheelsRight(byte angle)
{
    TurnWheels(angle, 1);    
}

void FRobot::TurnWheels(byte angle, int multiplier)
{
    if (angle > MAX_STEERING_ANGLE)
        angle = MAX_STEERING_ANGLE;
    mSteerServo.write(STEERING_CENTER_ANGLE + (multiplier * angle));
}

void FRobot::Stop(boolean useBreak)
{
    digitalWrite(MOTOR_CTL_ENABLE_PIN, LOW);
    if (useBreak) {
        digitalWrite(MOTOR_CTL_A_PIN, HIGH);
        digitalWrite(MOTOR_CTL_B_PIN, HIGH);
        digitalWrite(MOTOR_CTL_C_PIN, HIGH);  
        digitalWrite(MOTOR_CTL_D_PIN, HIGH);    
        digitalWrite(MOTOR_CTL_ENABLE_PIN, HIGH);  
        delay(1000);
        digitalWrite(MOTOR_CTL_ENABLE_PIN, LOW);
    }  
}

void FRobot::GoBackward(boolean fadeIn, byte maxSpeed)
{
#ifndef TEST_WITHOUT_MOTORS    
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
#endif
}

unsigned long FRobot::ReadRawSonarValue()
{
    /* The following trigPin/echoPin cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. 
    */ 
    digitalWrite(SONAR_TRIGGER_PIN, LOW); 
    delayMicroseconds(2);

    digitalWrite(SONAR_TRIGGER_PIN, HIGH);
    delayMicroseconds(10); 
 
    digitalWrite(SONAR_TRIGGER_PIN, LOW);
    unsigned long rawValue = pulseIn(SONAR_ECHO_PIN, HIGH);    
    return rawValue;
}

unsigned long FRobot::MovingAverageRawSonarValue()
{
    unsigned long avg = 0;
    unsigned long sum = 0;
    int div = 0;
    for (int bufferIndex = 0; bufferIndex < SONAR_RING_BUFFER_MAX; bufferIndex++) {
        if(mSonarRingBuffer[bufferIndex] > 0) {
            sum += mSonarRingBuffer[bufferIndex];
            div++;
        }
    }
    if(div == SONAR_RING_BUFFER_MAX)
        avg = sum / div;
    return avg;
}

float FRobot::ReadSonarDistance()
{
    unsigned long duration = 0;
    const int maxTries = 5;
    int tries = 0;
    // This loop tries to filer out bad values by keeping a running
    // average and comapring current values against that running 
    // average. If the values deviate too much we try another reading 
    // if the deviations continue we just take what we have and move on.
    while (tries < 5) {
        unsigned long avg = MovingAverageRawSonarValue();
        duration = ReadRawSonarValue();
        // A delta longer than 2000 equates to around 35 cm
        unsigned long delta = abs((long)duration - (long)avg);
        if(avg == 0 || delta < 2000)
        {
            break;
        } else {
            tries++;
            duration = ReadRawSonarValue();            
        }
    }
    
    mSonarRingBuffer[mSonarRingBufferIndex] = duration;
    mSonarRingBufferIndex = (mSonarRingBufferIndex + 1) % SONAR_RING_BUFFER_MAX;
    
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
    int furthestDistanceAngle = 0;
    ParseInputBufer();
    if(mAutoMode)
    {
        float currentDistance = 0.0f;  
        float furthestDistance = 0.0f;
    
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
    
        // During this final write to center the sonar servo
        // we will take advantage of the time needed and post
        // the results
        mSonarServo.write(SONAR_CENTER_ANGLE);
        unsigned long totalWaitMicros = (MAX_SCAN_ANGLE*4L + 25L) * 1000L;
        unsigned long t0 = micros();
        PostSonarStatus();
        unsigned long t1 = micros();
        long remaining = totalWaitMicros - (t1 - t0);
        if (remaining > 0)
            delayMicroseconds(remaining);
    }
    return furthestDistanceAngle;
}
