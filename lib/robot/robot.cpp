#include "Robot.h"
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>

#define PULSES_PER_REV 400
#define WHEEL_DIAMETER_MM 100
#define MM_PER_PULSE (3.14159 * WHEEL_DIAMETER_MM / PULSES_PER_REV)

MPU6050 mpu;
ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

Robot::Robot(double Kp, double Ki, double Kd)
    : leftMotor(0, 0, Kp, Ki, Kd), rightMotor(0, 0, Kp, Ki, Kd)
{
    // Ban đầu chưa gán chân PWM, cần gọi attachMotors()
}

void Robot::attachMotors(int pwmL_R, int pwmL_L, int pwmR_R, int pwmR_L)
{
    leftMotor = Motor(pwmL_R, pwmL_L, Kp, Ki, Kd);
    rightMotor = Motor(pwmR_R, pwmR_L, Kp, Ki, Kd);
}

void Robot::attachEncoders(int leftEncA, int leftEncB, int rightEncA, int rightEncB)
{
    ESP32Encoder::useInternalWeakPullResistors = UP;
    leftEncoder.attachHalfQuad(leftEncA, leftEncB);
    rightEncoder.attachHalfQuad(rightEncA, rightEncB);
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

void Robot::resetEncoders()
{
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

double Robot::getDistanceTraveled()
{
    return ((leftEncoder.getCount() + rightEncoder.getCount()) / 2.0) * MM_PER_PULSE;
}

void Robot::initIMU()
{
    Wire.begin();
    mpu.initialize();
}

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

void Robot::moveForward(double speed)
{
    leftMotor.setTargetSpeed(speed);
    rightMotor.setTargetSpeed(speed);
}

void Robot::moveBackward(double speed)
{
    leftMotor.setTargetSpeed(-speed);
    rightMotor.setTargetSpeed(-speed);
}

void Robot::turnLeft(double speed)
{
    leftMotor.setTargetSpeed(-speed);
    rightMotor.setTargetSpeed(speed);
}

void Robot::turnRight(double speed)
{
    leftMotor.setTargetSpeed(speed);
    rightMotor.setTargetSpeed(-speed);
}

void Robot::stop()
{
    leftMotor.stop();
    rightMotor.stop();
}
