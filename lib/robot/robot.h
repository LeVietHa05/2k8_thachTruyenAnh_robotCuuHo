#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"
#include <SimpleKalmanFilter.h>

class Robot
{
private:
    int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
    int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
    Motor *leftMotor = nullptr;
    Motor *rightMotor = nullptr;
    double Kp, Ki, Ke;
    SimpleKalmanFilter kalmanFilter;

public:
    Robot(double Kp, double Ki, double Kd);
    ~Robot();
    void attachMotors(int pwmL_R, int pwmL_L, int pwmR_R, int pwmR_L);
    void attachEncoders(int leftEncA, int leftEncB, int rightEncA, int rightEncB);
    void resetEncoders();
    double getDistanceTraveled();
    void initIMU();
    void calibrateIMU();
    void updateIMUdata();
    float getFilteredAngle();
    float getRawAngle();
    void moveForward(double speed);
    void moveBackward(double speed);
    void turnLeft(double speed);
    void turnRight(double speed);
    void stop();
};

#endif
