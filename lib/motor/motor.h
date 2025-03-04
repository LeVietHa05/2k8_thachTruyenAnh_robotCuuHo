// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
private:
    int pwmRPin;
    int pwmLPin;

public:
    Motor(int pwmR, int pwmL);
    void run(int speed);
    void stop();
};

#endif
