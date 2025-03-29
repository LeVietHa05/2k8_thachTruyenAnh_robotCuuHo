#ifndef ROBOT_H
#define ROBOT_H

#include "Motor.h"

class Robot
{
private:
    int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
    int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
    Motor leftMotor;
    Motor rightMotor;
    double Kp, Ki, Kd;
    PID syncPID; // PID để đồng bộ tốc độ
    double syncSetpoint, syncInput, syncOutput;
    int offset = 0; // for leftmotor (it slower than right motor)
    int leftTargetSpeed = 0;
    int rightTargetSpeed = 0;
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
    void setOffset(int offset);
    int getOffset();
    void updateMotorSpeeds();
    void debugRobot();
    void initSyncPID(double Kp, double Ki, double Kd);
    void balanceSpdNoPWM(int targetSpeed);
    void turnLeftNoPWM(int targetSpeed);
    void turnRightNoPWM(int targetSpeed);
    bool isStop();
};

#endif
