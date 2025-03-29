#include "Robot.h"
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>
#include <MahonyAHRS.h>

#define PULSES_PER_REV 800    // encoder pulses per revolution
#define WHEEL_DIAMETER_MM 100 // wheel diameter in mm
#define MM_PER_PULSE (3.14159 * WHEEL_DIAMETER_MM / PULSES_PER_REV)
#define MAX_SPEED_RPM 333 // RPM

MPU6050 mpu;
Mahony filter;
ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

// Constructor
Robot::Robot(double Kp, double Ki, double Kd)
    : Kp(Kp), Ki(Ki), Kd(Kd), syncPID(&syncInput, &syncOutput, &syncSetpoint, 0, 0, 0, DIRECT)
{
    leftMotor.beginPID(Kp, Ki, Kd);
    rightMotor.beginPID(Kp, Ki, Kd);
    initSyncPID(0.1, 0.0, 1.0);
}

// Destructor
Robot::~Robot()
{
    leftMotor.stop();
    rightMotor.stop();
}

// Attach motors to the robot
void Robot::attachMotors(int pwmL_R, int pwmL_L, int pwmR_R, int pwmR_L)
{
    leftMotor.begin(pwmL_R, pwmL_L);
    rightMotor.begin(pwmR_R, pwmR_L);
}

// Attach encoders to the robot
void Robot::attachEncoders(int leftEncA, int leftEncB, int rightEncA, int rightEncB)
{
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    leftEncoder.attachHalfQuad(leftEncA, leftEncB);
    rightEncoder.attachHalfQuad(rightEncA, rightEncB);
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

// Reset encoders
void Robot::resetEncoders()
{
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}

// return distance in mm
double Robot::getDistanceTraveled()
{
    return ((leftEncoder.getCount() + rightEncoder.getCount()) / 2.0) * MM_PER_PULSE;
}

// Initialize IMU
void Robot::initIMU()
{
    Wire.begin();
    mpu.initialize();
    calibrateIMU();
}

// Calibrate IMU
void Robot::calibrateIMU()
{
    int numReadings = 1000;
    long axSum = 0, aySum = 0, azSum = 0;
    long gxSum = 0, gySum = 0, gzSum = 0;

    for (int i = 0; i < numReadings; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        axSum += ax;
        aySum += ay;
        azSum += az;
        gxSum += gx;
        gySum += gy;
        gzSum += gz;

        delay(2);
    }

    ax_offset = axSum / numReadings;
    ay_offset = aySum / numReadings;
    az_offset = azSum / numReadings;
    gx_offset = gxSum / numReadings;
    gy_offset = gySum / numReadings;
    gz_offset = gzSum / numReadings;
}

// Get IMU data
void Robot::updateIMUdata()
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    // Chuyển đổi giá trị raw thành đơn vị vật lý
    float accelX = ax / 16384.0; // Đơn vị: g
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;
    float gyroX = gx / 131.0; // Đơn vị: độ/giây (dps)
    float gyroY = gy / 131.0;
    float gyroZ = gz / 131.0;

    filter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
}

// Get filtered angle
float Robot::getFilteredAngle()
{
    // Lấy giá trị góc từ bộ lọc
    float yaw = filter.getYaw();
    // Chuyển đổi góc từ -180 đến 180 thành -900 đến 900 (khong hieu vi sao can phai lam nhu nay, nhung khong lam thi no khong chuan)
    yaw = (yaw - 180) * 25.714f;
    return yaw;
}

// Get raw angle
float Robot::getRawAngle()
{
    return filter.getYaw();
}

// Move forward
void Robot::moveForward(double speed)
{
    // spd: 0 - 100 %
    // targetspd: 0 - 800 (pulse/s)
    int targetSpeed = speed * MAX_SPEED_RPM * PULSES_PER_REV / 60 / 100;
    leftMotor.setTargetSpeed(targetSpeed + offset);
    rightMotor.setTargetSpeed(targetSpeed);
    leftMotor.run();
    rightMotor.run();
    updateMotorSpeeds(); // Cập nhật tốc độ
}

// Move backward
void Robot::moveBackward(double speed)
{
    int targetSpeed = speed * MAX_SPEED_RPM * PULSES_PER_REV / 60 / 100;
    leftMotor.setTargetSpeed(-targetSpeed - offset);
    rightMotor.setTargetSpeed(-targetSpeed);
    leftMotor.run();
    rightMotor.run();
    updateMotorSpeeds(); // Cập nhật tốc độ
}

// Turn left
void Robot::turnLeft(double speed)
{
    int targetSpeed = speed * MAX_SPEED_RPM * PULSES_PER_REV / 60 / 100;
    leftMotor.setTargetSpeed(0);
    rightMotor.setTargetSpeed(targetSpeed);
    leftMotor.run();
    rightMotor.run();
    updateMotorSpeeds(); // Cập nhật tốc độ
}

// Turn right
void Robot::turnRight(double speed)
{
    int targetSpeed = speed * MAX_SPEED * PULSES_PER_REV / 60 / 100;
    leftMotor.setTargetSpeed(targetSpeed + offset);
    rightMotor.setTargetSpeed(0);
    leftMotor.run();
    rightMotor.run();
    updateMotorSpeeds(); // Cập nhật tốc độ
}

// Stop
void Robot::stop()
{
    leftMotor.setTargetSpeed(0);
    rightMotor.setTargetSpeed(0);
    updateMotorSpeeds(); // Cập nhật tốc độ
}

// offset
void Robot::setOffset(int offset)
{
    this->offset = offset;
}

int Robot::getOffset()
{
    return offset;
}

// Update motor speeds based on encoder feedback
void Robot::updateMotorSpeeds()
{
    static int32_t lastLeftCount = 0;
    static int32_t lastRightCount = 0;
    static unsigned long lastTime = 0;

    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 10)
    { // Cập nhật mỗi 10ms
        int32_t leftCount = leftEncoder.getCount();
        int32_t rightCount = rightEncoder.getCount();

        double deltaTime = (currentTime - lastTime) / 1000.0;
        double leftSpeed = (leftCount - lastLeftCount) / deltaTime;
        double rightSpeed = (rightCount - lastRightCount) / deltaTime;

        lastLeftCount = leftCount;
        lastRightCount = rightCount;
        lastTime = currentTime;

        // PID cấp thấp
        leftMotor.updateSpeed(leftSpeed);
        rightMotor.updateSpeed(rightSpeed);

        // PID cấp cao để đồng bộ
        syncInput = leftSpeed - rightSpeed; // Chênh lệch tốc độ
        syncPID.Compute();
        offset = syncOutput; // Điều chỉnh offset động

        // Áp dụng offset
        leftMotor.setTargetSpeed(leftMotor.getSetpoint() + offset);
        rightMotor.setTargetSpeed(rightMotor.getSetpoint());

        // Debug
        Serial.print(" - pid speed ");
        Serial.print(leftSpeed);
        Serial.print(" | ");
        Serial.print(rightSpeed);
        Serial.print(" | ");
        Serial.println(offset);
    }
}

void Robot::debugRobot()
{
    Serial.print(" - encoder ");
    Serial.print(leftEncoder.getCount());
    Serial.print(" | ");
    Serial.print(rightEncoder.getCount());
    Serial.print("");
    // leftMotor.debugMotor();
    // Serial.print("Right motor: ");
    // rightMotor.debugMotor();
}

void Robot::initSyncPID(double Kp, double Ki, double Kd)
{
    syncSetpoint = 0; // Chênh lệch tốc độ mong muốn = 0
    syncPID.SetMode(AUTOMATIC);
    syncPID.SetTunings(Kp, Ki, Kd);
    syncPID.SetSampleTime(10);
    syncPID.SetOutputLimits(-20, 20); // Giới hạn offset
}

// only for forward and backward
void Robot::balanceSpdNoPWM(int targetSpeed)
{
    // Nếu tốc độ mục tiêu là 0, dừng động cơ
    if (targetSpeed == 0)
    {
        leftMotor.setSpdNoPID(0);
        rightMotor.setSpdNoPID(0);
        return;
    }
    // Lấy số xung từ encoder
    long leftCount = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();

    // Giới hạn tốc độ mục tiêu
    targetSpeed = constrain(targetSpeed, 0, MAX_SPEED);

    // Tính sai lệch giữa 2 encoder
    long error = (leftCount - rightCount); // Sai lệch giữa 2 động cơ

    // Điều chỉnh tốc độ dựa trên sai lệch
    int leftAdjustSpeed = targetSpeed - 0.1 * (error / 2);  // Động cơ trái
    int rightAdjustSpeed = targetSpeed + 0.1 * (error / 2); // Động cơ phải

    // Giới hạn tốc độ sau điều chỉnh
    leftAdjustSpeed = constrain(leftAdjustSpeed, 0, MAX_SPEED);
    rightAdjustSpeed = constrain(rightAdjustSpeed, 0, MAX_SPEED);

    // Cập nhật tốc độ mục tiêu
    leftTargetSpeed = leftAdjustSpeed;
    rightTargetSpeed = rightAdjustSpeed;

    // Thay đổi tốc độ từ từ đến giá trị mục tiêu
    leftMotor.setSpdNoPID(leftTargetSpeed);
    rightMotor.setSpdNoPID(rightTargetSpeed);
}

// turn left without PWM
void Robot::turnLeftNoPWM(int targetSpeed)
{
    leftMotor.setSpdNoPID(0);            // Dừng động cơ trái
    rightMotor.setSpdNoPID(targetSpeed); // Chạy động cơ phải
}

// turn right without PWM
void Robot::turnRightNoPWM(int targetSpeed)
{
    leftMotor.setSpdNoPID(targetSpeed); // Chạy động cơ trái
    rightMotor.setSpdNoPID(0);          // Dừng động cơ phải
}

bool Robot::isStop()
{
    return (leftMotor.getSpdNoPID() == 0 && rightMotor.getSpdNoPID() == 0);
}