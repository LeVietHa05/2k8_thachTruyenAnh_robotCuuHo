// Motor.cpp
#include "Motor.h"

Motor::Motor() : pid(&input, &output, &setpoint, 0, 0, 0, DIRECT)
{
    setpoint = 0;
    input = 0;
    output = 0;
}

void Motor::begin(int pwmR, int pwmL)
{
    pwmRPin = pwmR;
    pwmLPin = pwmL;
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

void Motor::beginPID(double Kp, double Ki, double Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    pid.SetMode(AUTOMATIC); // Bật chế độ tự động
    pid.SetTunings(Kp, Ki, Kd);
    pid.SetSampleTime(10); // 10ms
    pid.SetOutputLimits(-255, 255);
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
    static double lastOutput = 0;
    input = measuredSpeed;
    pid.Compute();
    double maxChange = 20;
    if (output > lastOutput + maxChange)
        output = lastOutput + maxChange;
    else if (output < lastOutput - maxChange)
        output = lastOutput - maxChange;
    lastOutput = output;
    run();
}

void Motor::run()
{
    int pwmValue = constrain(output, 0, MAX_SPEED);
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
    setTargetSpeed(0);
}

void Motor::debugMotor()
{
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(input);
    Serial.print(",");
    Serial.println(output);
}

void Motor::runPWM(int pwmValue)
{
    ledcWrite(channelR, pwmValue);
    ledcWrite(channelL, 0);
}

double Motor::getSetpoint()
{
    return setpoint;
}

void Motor::setSpdNoPID(int setSpeed)
{
    #define SPEED_STEP 5 // Bước tăng tốc độ
    if (setSpeed == 0)
    {
        spdNoPWM = 0; // Dừng động cơ
        runPWM(0);
        return;
    }
    // Giới hạn setSpeed trong khoảng 0-255
    setSpeed = constrain(setSpeed, 0, MAX_SPEED);

    // Tăng hoặc giảm tốc độ từ từ
    if (spdNoPWM < setSpeed)
    {
        spdNoPWM += SPEED_STEP;
        if (spdNoPWM > setSpeed)
            spdNoPWM = setSpeed; // Không vượt quá setSpeed
    }
    else if (spdNoPWM > setSpeed)
    {
        spdNoPWM -= SPEED_STEP;
        if (spdNoPWM < setSpeed)
            spdNoPWM = setSpeed; // Không thấp hơn setSpeed
    }

    runPWM(spdNoPWM);
}

int Motor::getSpdNoPID()
{
    return spdNoPWM;
}