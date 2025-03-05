#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"

class Robot
{
private:
    Motor leftMotor;
    Motor rightMotor;

public:
    Robot(double Kp, double Ki, double Kd);
    void attachMotors(int pwmL_R, int pwmL_L, int pwmR_R, int pwmR_L);
    void attachEncoders(int leftEncA, int leftEncB, int rightEncA, int rightEncB);
    void resetEncoders();
    double getDistanceTraveled();
    void getIMUData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);
    void moveForward(double speed);
    void moveBackward(double speed);
    void turnLeft(double speed);
    void turnRight(double speed);
    void stop();
};

#endif
