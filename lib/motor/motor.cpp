// Motor.cpp
#include "Motor.h"

Motor::Motor(int pwmR, int pwmL, double Kp, double Ki, double Kd)
    : pwmRPin(pwmR), pwmLPin(pwmL), Kp(Kp), Ki(Ki), Kd(Kd),
      pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT)
{
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255); // Chỉ từ 0-255
    int channel = find_free_ledc_channel();
    if (channel == -1)
    {
        Serial.println("No free LEDC channel found");
    }
    else
    {
        ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(pwmRPin, channel);
    }
    channel = find_free_ledc_channel();
    if (channel == -1)
    {
        Serial.println("No free LEDC channel found");
    }
    else
    {
        ledcSetup(channel, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(pwmLPin, channel);
    }
}

int Motor::find_free_ledc_channel()
{
    for (int i = 0; i < 16; i++)
    {
        if (!ledcReadFreq(i))
        {
            return i;
        }
    }
    return -1;
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
        ledcWrite(pwmRPin, pwmValue);
        ledcWrite(pwmLPin, 0);
    }
    else if (setpoint < 0)
    {
        ledcWrite(pwmRPin, 0);
        ledcWrite(pwmLPin, pwmValue);
    }
    else
    {
        stop();
    }
}

void Motor::stop()
{
    ledcWrite(pwmRPin, 0);
    ledcWrite(pwmLPin, 0);
}
