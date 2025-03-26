#include "Robot.h"
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>
#include <MahonyAHRS.h>

#define PULSES_PER_REV 400    // encoder pulses per revolution
#define WHEEL_DIAMETER_MM 100 // wheel diameter in mm
#define MM_PER_PULSE (3.14159 * WHEEL_DIAMETER_MM / PULSES_PER_REV)

MPU6050 mpu;
Mahony filter;
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
    leftMotor = new Motor(pwmL_R, pwmL_L, this->Kp, this->Ki, this->Ke);
    rightMotor = new Motor(pwmR_R, pwmR_L, this->Kp, this->Ki, this->Ke);
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
    calibrateIMU();
}

// Calibrate IMU
void Robot::calibrateIMU()
{
    int numReadings = 1000;
    long axSum = 0, aySum = 0, azSum = 0;
    long gxSum = 0, gySum = 0, gzSum = 0;

    for (int i = 0; i < numReadings; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        axSum += ax;
        aySum += ay;
        azSum += az;
        gxSum += gx;
        gySum += gy;
        gzSum += gz;

        delay(2);
    }

    ax_offset = axSum / numReadings;
    ay_offset = aySum / numReadings;
    az_offset = azSum / numReadings;
    gx_offset = gxSum / numReadings;
    gy_offset = gySum / numReadings;
    gz_offset = gzSum / numReadings;
}

// Get IMU data
void Robot::updateIMUdata()
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    // Chuyển đổi giá trị raw thành đơn vị vật lý
    float accelX = ax / 16384.0; // Đơn vị: g
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    float gyroX = gx / 131.0; // Đơn vị: độ/giây (dps)
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
}

// Get filtered angle
float Robot::getFilteredAngle()
{
    // Lấy giá trị góc từ bộ lọc
    float yaw = filter.getYaw();
    // Chuyển đổi góc từ -180 đến 180 thành -900 đến 900 (khong hieu vi sao can phai lam nhu nay, nhung khong lam thi no khong chuan)
    yaw = (yaw - 180) * 25.714f;
    return yaw;
}

// Get raw angle
float Robot::getRawAngle()
{
    return filter.getYaw();
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
