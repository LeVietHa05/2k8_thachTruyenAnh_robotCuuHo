// Motor.cpp
#include "Motor.h"

Motor::Motor(int pwmR, int pwmL)
{
    pwmRPin = pwmR;
    pwmLPin = pwmL;
    pinMode(pwmRPin, OUTPUT);
    pinMode(pwmLPin, OUTPUT);
    stop();
}

void Motor::run(int speed)
{
    speed = constrain(speed, -255, 255);
    if (speed > 0)
    {
        analogWrite(pwmRPin, speed);
        analogWrite(pwmLPin, 0);
    }
    else if (speed < 0)
    {
        analogWrite(pwmRPin, 0);
        analogWrite(pwmLPin, speed);
    }
    else
    {
        stop();
    }
}

void Motor::stop()
{
    analogWrite(pwmRPin, 0);
    analogWrite(pwmLPin, 0);
}
