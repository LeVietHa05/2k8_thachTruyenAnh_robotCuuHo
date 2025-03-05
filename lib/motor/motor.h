// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>

class Motor
{
private:
    int pwmRPin;
    int pwmLPin;
    double setpoint, input, output;
    double Kp, Ki, Kd;
    PID pid;

public:
    Motor(int pwmR, int pwmL, double Kp, double Ki, double Kd);
    void setTargetSpeed(double speed);
    void updateSpeed(double measuredSpeed);
    void run();
    void stop();
};

#endif
