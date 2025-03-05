// Motor.cpp
#include "Motor.h"

Motor::Motor(int pwmR, int pwmL, double Kp, double Ki, double Kd)
    : pwmRPin(pwmR), pwmLPin(pwmL), Kp(Kp), Ki(Ki), Kd(Kd),
      pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT)
{
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255); // Chỉ từ 0-255
}

void Motor::setTargetSpeed(double speed)
{
    setpoint = speed;
}

void Motor::updateSpeed(double measuredSpeed)
{
    input = abs(measuredSpeed);
    pid.Compute();
    run();
}

void Motor::run()
{
    int pwmValue = constrain(output, 0, 255);
    if (setpoint > 0)
    {
        analogWrite(pwmRPin, pwmValue);
        analogWrite(pwmLPin, 0);
    }
    else if (setpoint < 0)
    {
        analogWrite(pwmRPin, 0);
        analogWrite(pwmLPin, pwmValue);
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
