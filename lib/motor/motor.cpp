// Motor.cpp
#include "Motor.h"

Motor::Motor(int pwmR, int pwmL, double Kp, double Ki, double Kd)
    : pwmRPin(pwmR), pwmLPin(pwmL), Kp(Kp), Ki(Ki), Kd(Kd),
      pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT)
{
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255); // Chỉ từ 0-255
    channelR = find_free_ledc_channel();
    if (channelR == -1)
    {
        Serial.println("No free LEDC channel found");
    }
    else
    {
        ledcSetup(channelR, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(pwmRPin, channelR);
    }
    channelL = find_free_ledc_channel();
    if (channelL == -1)
    {
        Serial.println("No free LEDC channel found");
    }
    else
    {
        ledcSetup(channelL, PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(pwmLPin, channelL);
    }
}

#define LEDC_MAX_CHANNELS 16

bool ledc_used_channels[LEDC_MAX_CHANNELS] = {false};

int Motor::find_free_ledc_channel()
{
    for (int i = 0; i < LEDC_MAX_CHANNELS; i++)
    {
        if (!ledc_used_channels[i])
        {
            ledc_used_channels[i] = true; // Mark as used
            return i;
        }
    }
    return -1; // No free channel found
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
        ledcWrite(channelR, pwmValue);
        ledcWrite(channelL, 0);
    }
    else if (setpoint < 0)
    {
        ledcWrite(channelR, 0);
        ledcWrite(channelL, pwmValue);
    }
    else
    {
        stop();
    }
}

void Motor::stop()
{
    ledcWrite(channelR, 0);
    ledcWrite(channelL, 0);
}
