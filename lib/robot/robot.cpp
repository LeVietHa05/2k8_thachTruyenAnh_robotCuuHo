#include "Robot.h"
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>

#define PULSES_PER_REV 400    // encoder pulses per revolution
#define WHEEL_DIAMETER_MM 100 // wheel diameter in mm
#define MM_PER_PULSE (3.14159 * WHEEL_DIAMETER_MM / PULSES_PER_REV)

MPU6050 mpu;
ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

// Constructor
Robot::Robot(double Kp, double Ki, double Ke)
    : Kp(Kp), Ki(Ki), Ke(Ke), kalmanFilter(2, 2, 0.01)
{
}

// Destructor
Robot::~Robot()
{
    delete leftMotor;
    delete rightMotor;
}

// Attach motors to the robot
void Robot::attachMotors(int pwmL_R, int pwmL_L, int pwmR_R, int pwmR_L)
{
    leftMotor =new  Motor(pwmL_R, pwmL_L, this->Kp, this->Ki, this->Ke);
    rightMotor =new  Motor(pwmR_R, pwmR_L, this->Kp, this->Ki, this->Ke);
}

// Attach encoders to the robot
void Robot::attachEncoders(int leftEncA, int leftEncB, int rightEncA, int rightEncB)
{
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    leftEncoder.attachHalfQuad(leftEncA, leftEncB);
    rightEncoder.attachHalfQuad(rightEncA, rightEncB);
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

// Reset encoders
void Robot::resetEncoders()
{
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

// return distance in mm
double Robot::getDistanceTraveled()
{
    return ((leftEncoder.getCount() + rightEncoder.getCount()) / 2.0) * MM_PER_PULSE;
}


// Initialize IMU
void Robot::initIMU()
{
    Wire.begin();
    mpu.initialize();
}

// Get IMU data
void Robot::getIMUData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
{
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
    ax = ax_raw / 16384.0;
    ay = ay_raw / 16384.0;
    az = az_raw / 16384.0;
    gx = gx_raw / 131.0;
    gy = gy_raw / 131.0;
    gz = gz_raw / 131.0;
}

// Get filtered angle
float Robot::getFilteredAngle()
{
    int16_t gx_raw, gy_raw, gz_raw;
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
    float gz = gz_raw / 131.0;
    return kalmanFilter.updateEstimate(gz);
}

// Get raw angle
float Robot::getRawAngle()
{
    int16_t gx_raw, gy_raw, gz_raw;
    mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);
    return gz_raw / 131.0;
}

// Move forward
void Robot::moveForward(double speed)
{
    leftMotor->setTargetSpeed(speed);
    rightMotor->setTargetSpeed(speed);
}

// Move backward
void Robot::moveBackward(double speed)
{
    leftMotor->setTargetSpeed(-speed);
    rightMotor->setTargetSpeed(-speed);
}

// Turn left
void Robot::turnLeft(double speed)
{
    leftMotor->setTargetSpeed(-speed);
    rightMotor->setTargetSpeed(speed);
}

// Turn right
void Robot::turnRight(double speed)
{
    leftMotor->setTargetSpeed(speed);
    rightMotor->setTargetSpeed(-speed);
}

// Stop
void Robot::stop()
{
    leftMotor->stop();
    rightMotor->stop();
}
