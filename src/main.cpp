#include <Arduino.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Encoder.h>

#include "config.h" // Include configuration for host and API key

// Define the variables
const char *apiKey = "5b3ce3597851110001cf62489b1a4894dc59438fa047b32238086ce2";
const char *host = "api.openrouteservice.org";
Step steps[MAX_STEPS_ALLOWED]; // Define the steps array

#include "get_data.h"

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// MPU6050 and Encoder objects
MPU6050 mpu;
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// Variables for sensor data and motor control
float yaw = 0;
long lastLeftCount = 0;
long lastRightCount = 0;
unsigned long lastSensorUpdate = 0;

// PID constants
const float Kp = 1.0;
const float Ki = 0.01;
const float Kd = 0.1;

// Motor control variables
float leftSpeed = 0;
float rightSpeed = 0;
float leftError = 0;
float rightError = 0;
float leftIntegral = 0;
float rightIntegral = 0;
float leftDerivative = 0;
float rightDerivative = 0;

unsigned long lastGpsUpdate = 0;
const unsigned long GpsTimeout = 10000; // 10 seconds

int currentStep = 0;

void moveAccordingToStep(int type);
void turnLeft();
void turnRight();
void moveForward();
void stopMotors();
void calculateMotorSpeeds();
void updateMotorControl();
void maintainHeading(float targetYaw);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);
bool checkGpsValid(float &lat, float &lon);
float normalizeAngle(float angle);

void setup()
{
  // Initialize I2C for MPU6050
  Wire.begin(MPU6050_SDA, MPU6050_SCL);
  mpu.initialize();

  // Initialize watchdog timer (5 second timeout)
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL); // Add current task to watchdog

  Serial.begin(115200); // Initialize serial communication at a baud rate of 115200
  // Initialize GPS serial communication
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  WiFiManager wm;

  bool res = wm.autoConnect("AutoConnectAP", "password");
  if (!res)
  {
    Serial.println("Failed to connect to WiFi");
    pinMode(LED_BUILTIN, OUTPUT);
    while (1)
    {
      dw(LED_BUILTIN, HIGH);
      delay(300);
      dw(LED_BUILTIN, LOW);
      delay(300);
    }
  }

  Serial.println("Connected!");
  if (fetchRoute(host, apiKey))
  {
    Serial.println("Route fetched successfully");
  }
  else
  {
    Serial.println("Failed to fetch route");
  }

  // Configure PWM channels with frequency and resolution
  ledcSetup(PWM_CHANNEL0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL3, PWM_FREQ, PWM_RESOLUTION);

  // Attach PWM channels to motor control pins
  ledcAttachPin(MT1_L, PWM_CHANNEL0);
  ledcAttachPin(MT1_R, PWM_CHANNEL1);
  ledcAttachPin(MT2_L, PWM_CHANNEL2);
  ledcAttachPin(MT2_R, PWM_CHANNEL3);

  ESP32Encoder::useInternalWeakPullResistors = puType::up; // Enable internal pull-up resistors for encoder pins

  encoderLeft.attachHalfQuad(ENC_L_A, ENC_L_B);
  encoderRight.attachHalfQuad(ENC_R_A, ENC_R_B);

  encoderLeft.clearCount();
  encoderRight.clearCount();

  encoderLeft.setCount(0);
  encoderRight.setCount(0);
}

void loop()
{
  // Read and process MPU6050 data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate yaw from gyro data
  float dt = (millis() - lastSensorUpdate) / 1000.0;
  yaw += gz / 131.0 * dt; // 131 LSB/degree/sec

  // Read and process encoder data
  int64_t leftPos = encoderLeft.getCount();
  int64_t rightPos = encoderRight.getCount();

  // Update last values
  lastLeftCount = leftPos;
  lastRightCount = rightPos;
  lastSensorUpdate = millis();

  // Reset watchdog timer
  esp_task_wdt_reset();

  // Check for GPS signal
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
    lastGpsUpdate = millis();
  }

  // Handle GPS signal loss
  if (millis() - lastGpsUpdate > GpsTimeout)
  {
    Serial.println("Warning: GPS signal lost!");
    stopMotors();
    return; // Skip navigation until signal is restored
  }

  if (gps.location.isUpdated())
  {
    float lat_now = gps.location.lat();
    float lon_now = gps.location.lng();

    Serial.printf("Current: %f, %f\n", lat_now, lon_now);

    if (currentStep < MAX_STEPS_ALLOWED)
    {
      Serial.printf("Instruction: %s\n", steps[currentStep].instruction.c_str());
      // Only move if we have valid GPS signal
      if (millis() - lastGpsUpdate <= GpsTimeout)
      {
        moveAccordingToStep(steps[currentStep].type);
      }

      // Simulate completing the route segment
      delay(steps[currentStep].duration * 1000);
      currentStep++;
    }
    else
    {
      stopMotors();
      Serial.println("Route completed!");
      while (true)
        ;
    }
  }
}

// 📌 Điều khiển robot theo từng loại bước
void moveAccordingToStep(int type)
{
  switch (type)
  {
  case 11:
    moveForward();
    Serial.println("Moving forward...");
    break;
  case 1:
    turnRight();
    Serial.println("Turning right...");
    break;
  case 2:
    turnLeft();
    Serial.println("Turning left...");
    break;
  case 10:
    stopMotors();
    Serial.println("Arrived at destination!");
    break;
  default:
    stopMotors();
    Serial.println("Unknown step type");
    break;
  }
}

void calculateMotorSpeeds()
{
  // Calculate speed from encoder deltas
  float dt = (millis() - lastSensorUpdate) / 1000.0;
  leftSpeed = (encoderLeft.getCount() - lastLeftCount) / dt;
  rightSpeed = (encoderRight.getCount() - lastRightCount) / dt;

  // Update last counts
  lastLeftCount = encoderLeft.getCount();
  lastRightCount = encoderRight.getCount();
}

void updateMotorControl()
{
  // Calculate errors
  float leftTarget = 255; // Target speed
  float rightTarget = 255;

  float leftPrevError = leftError;
  float rightPrevError = rightError;

  leftError = leftTarget - leftSpeed;
  rightError = rightTarget - rightSpeed;

  // Calculate integral terms
  leftIntegral += leftError;
  rightIntegral += rightError;

  // Calculate derivative terms
  leftDerivative = leftError - leftPrevError;
  rightDerivative = rightError - rightPrevError;

  // Calculate PID outputs
  float leftOutput = Kp * leftError + Ki * leftIntegral + Kd * leftDerivative;
  float rightOutput = Kp * rightError + Ki * rightIntegral + Kd * rightDerivative;

  // Apply motor control
  ledcWrite(PWM_CHANNEL0, constrain(leftOutput, 0, 255));
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, constrain(rightOutput, 0, 255));
  ledcWrite(PWM_CHANNEL3, 0);
}

void maintainHeading(float targetYaw)
{
  // Normalize target yaw to be within -180 to 180 degrees
  targetYaw = normalizeAngle(targetYaw);
  // Calculate yaw error
  float yawError = targetYaw - yaw;

  // Simple P controller for yaw correction
  float correction = yawError * 0.5; // Adjust gain as needed

  // Apply correction to motors
  ledcWrite(PWM_CHANNEL0, constrain(255 + correction, 0, 255));
  ledcWrite(PWM_CHANNEL2, constrain(255 - correction, 0, 255));
}

// 📌 Tính toán góc giữa 2 điểm GPS
float calculateBearing(float lat1, float lon1, float lat2, float lon2)
{
  // Check if latitude and longitude values are within valid ranges
  if (checkGpsValid(lat1, lon1) || checkGpsValid(lat2, lon2))
  {
    Serial.println("Invalid latitude or longitude values");
    return 0; // Return 0 or handle error as needed
  }
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  return degrees(atan2(y, x));
}

// 📌 Di chuyển robot về phía trước
void moveForward()
{
  calculateMotorSpeeds();

  // Get current GPS position
  float currentLat = gps.location.lat();
  float currentLon = gps.location.lng();

  // Get next waypoint from steps
  float targetLat = steps[currentStep].targetLat;
  float targetLon = steps[currentStep].targetLon;

  // Check if GPS coordinates are valid
  if (checkGpsValid(currentLat, currentLon) && checkGpsValid(targetLat, targetLon))
  {
    Serial.println("Invalid GPS coordinates");
    return;
  }

  // Calculate target bearing
  float targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);

  // Maintain heading towards waypoint
  maintainHeading(targetBearing);

  // Update motor speeds
  updateMotorControl();
}

// 📌 Quay trái
void turnLeft()
{
  // Set target yaw 90 degrees left
  float targetYaw = yaw - 90;

  targetYaw = normalizeAngle(targetYaw);

  // Turn until target yaw is reached
  while (abs(yaw - targetYaw) > 5)
  { // 2 degree tolerance
    ledcWrite(PWM_CHANNEL0, 100);
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 255);
    ledcWrite(PWM_CHANNEL3, 0);

    // Update yaw reading
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float dt = (millis() - lastSensorUpdate) / 1000.0;
    yaw += gz / 131.0 * dt;
    lastSensorUpdate = millis();
  }

  stopMotors();
}

void turnRight()
{
  // Set target yaw 90 degrees right
  float targetYaw = yaw + 90;

  targetYaw = normalizeAngle(targetYaw);

  // Turn until target yaw is reached
  while (abs(yaw - targetYaw) > 5)
  { // 2 degree tolerance
    ledcWrite(PWM_CHANNEL0, 255);
    ledcWrite(PWM_CHANNEL1, 0);
    ledcWrite(PWM_CHANNEL2, 100);
    ledcWrite(PWM_CHANNEL3, 0);

    // Update yaw reading
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float dt = (millis() - lastSensorUpdate) / 1000.0;
    yaw += gz / 131.0 * dt;
    lastSensorUpdate = millis();
  }

  stopMotors();
}

void stopMotors()
{
  ledcWrite(PWM_CHANNEL0, 0);
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, 0);
  ledcWrite(PWM_CHANNEL3, 0);
}

bool checkGpsValid(float &lat, float &lon)
{
  if (lat == 0.0 && lon == 0.0)
  {
    Serial.println("Invalid GPS coordinates");
    return false;
  }
  else if (lat < -90 || lat > 90 || lon < -180 || lon > 180)
  {
    Serial.println("Invalid latitude or longitude values");
    return false;
  }
  else
  {
    return true;
  }
}

float normalizeAngle(float angle)
{
  while (angle > 180)
    angle -= 360;
  while (angle < -180)
    angle += 360;
  return angle;
}