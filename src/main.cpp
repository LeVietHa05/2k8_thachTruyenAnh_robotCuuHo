#include <Arduino.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>
#include <Wire.h>

#include "config.h" // Include configuration for host and API key
#define TESTING 1
#define SERIAL_TESTING 0

// Define the variables
const char *apiKey = "5b3ce3597851110001cf62489b1a4894dc59438fa047b32238086ce2";
const char *host = "api.openrouteservice.org";
Step steps[MAX_STEPS_ALLOWED]; // Define the steps array

#include "get_data.h"

#include "robot.h"

Robot *robot; // Initialize the robot with PID constants
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Variables for sensor data and motor control
float yaw = 0;

unsigned long lastGpsUpdate = 0;
const unsigned long GpsTimeout = 10000; // 10 seconds

// Variables
int currentStepIndex = 0;
int totalSteps = 0;
unsigned long lastNavigationUpdate = 0;
const unsigned long NAVIGATION_UPDATE_INTERVAL = 1000; // Update navigation every 1 second
bool isGpsWorking = false;
unsigned long lastUpdateTime = 0;
const int updateInterval = 40;    // Update interval in milliseconds
const float angleThreshold = 5.0; // Angle threshold for turning
float isInitialized = false;      // Flag to check if the robot is initialized
double referenceAngle = 0.0;

// Current position and orientation
double currentLat = 0.0;
double currentLon = 0.0;
double targetLat = 0.0;
double targetLon = 0.0;
float currentHeading = 0.0;
bool routeLoaded = false;

void maintainHeading(float targetYaw, double speed);
bool checkGpsValid(double &lat, double &lon);
float normalizeAngle(float angle);
float calculateDistance(double lat1, double lon1, double lat2, double lon2);
float calculateBearing(double lat1, double lon1, double lat2, double lon2);
void updateNavigation();
void executeCurrentStep(float targetBearing, float bearingDiff);
void updateCurLocation();

void setup()
{
  Serial.begin(115200); // Initialize serial communication at a baud rate of 115200
  // Initialize GPS serial communication
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  //   WiFiManager wm;
  //
  //   bool res = wm.autoConnect("AutoConnectAP", "password");
  //   if (!res)
  //   {
  //     Serial.println("Failed to connect to WiFi");
  //     pinMode(LED_BUILTIN, OUTPUT);
  //     while (1)
  //     {
  //       dw(LED_BUILTIN, HIGH);
  //       delay(300);
  //       dw(LED_BUILTIN, LOW);
  //       delay(300);
  //     }
  //   }
  //   Serial.println("Connected!");
  // Update GPS data
  updateCurLocation();

  if (TESTING || !isGpsWorking)
  {
    steps[0] = {0, 1.0f, FORWARD, 21.01731031312228, 105.79836476525836, 21.017901101150503, 105.79856804153376};
    steps[1] = {1, 0.2f, TURN_RIGHT, 21.017533048366026, 105.7976265862394, 21.01731031312228, 105.79836476525836};
    steps[2] = {2, 1.50f, FORWARD, 21.017012264642755, 105.79740932732533, 21.017533048366026, 105.7976265862394};
    steps[3] = {3, 0.1f, STOP, 0, 0, 0, 0};
    totalSteps = sizeof(steps) / sizeof(steps[0]);
    Serial.println(calculateBearing(steps[0].startLat, steps[0].startLon, steps[0].targetLat, steps[0].targetLon));
    // delay(10000);
  }
  else
  {
    if (fetchRoute(host, apiKey, 21.01867661642452, 105.7985191732634, 21.01643685342504, 105.80130384889998))
    {
      Serial.println("Route fetched successfully");
    }
    else
    {
      Serial.println("Failed to fetch route");
    }
    totalSteps = sizeof(steps) / sizeof(steps[0]);
    for (int i = 0; i < totalSteps; i++)
    {
      // calculate the bearing and distance
      double bearing = calculateBearing(steps[i].startLat, steps[i].startLon, steps[i].targetLat, steps[i].targetLon);
      // TODO:  and then set the TYPE of steps
      //  steps[i].type = getStepType(bearing);
    }
  }

  robot = new Robot(0.15, 0.001, 1);

  robot->attachMotors(MT1_R, MT1_L, MT2_R, MT2_L);
  robot->attachEncoders(ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B);
  robot->initIMU();
  robot->setOffset(0);
}

void loop()
{
  if (TESTING)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdateTime < updateInterval)
      return;
    robot->updateIMUdata();
    lastUpdateTime = currentMillis;

    if (currentStepIndex >= totalSteps)
    {
      // End of route reached
      robot->balanceSpdNoPWM(0, 0); // Dừng robot
      Serial.println("Destination reached!");
      return;
    }

    Step currentStep = steps[currentStepIndex];

    updateCurLocation(); // Cập nhật vị trí hiện tại từ GPS
    double distanceTraveled;
    double targetBearing = 0.0f;
    double currentBearing = 0.0f;
    double targetDistance = currentStep.distance; // Độ dài của bước hiện tại
    if (currentLat == 0.0f || currentLon == 0.0f)
    {
      currentBearing = robot->getFilteredAngle() - referenceAngle;                                                                // Lấy góc hiện tại từ IMU
      distanceTraveled = robot->getDistanceTraveled();                                                                            // Lấy khoảng cách từ encoder
      targetBearing = calculateBearing(currentStep.startLat, currentStep.startLon, currentStep.targetLat, currentStep.targetLon); // Tính toán góc đến điểm đích
    }
    else
    {
      currentBearing = gps.course.deg(); // Lấy góc hiện tại từ GPS
      distanceTraveled = calculateDistance(currentLat, currentLon, currentStep.targetLat, currentStep.targetLon);
      targetBearing = calculateBearing(currentLat, currentLon, currentStep.targetLat, currentStep.targetLon); // Tính toán góc đến điểm đích
    }

    double angleDiff = targetBearing - currentBearing; // Tính toán độ chênh lệch giữa góc
    if (angleDiff > 180.0f)
      angleDiff -= 360.0f; // Chuyển đổi về khoảng -180 đến 180 độ
    if (angleDiff < -180.0f)
      angleDiff += 360.0f;
    Serial.print("heading: ");
    Serial.print(currentBearing);
    Serial.print(", target: ");
    Serial.print(targetBearing);
    Serial.print(", diff: ");
    Serial.print(angleDiff);

    // // Căn chỉnh ban đầu nếu chưa khởi tạo
    // if (!isInitialized)
    // {
    //   if (abs(angleDiff) > angleThreshold)
    //   {
    //     if (angleDiff > 0)
    //     {
    //       robot->turnLeftNoPWM(50); // Quay trái nếu lệch dương
    //     }
    //     else
    //     {
    //       robot->turnRightNoPWM(50); // Quay phải nếu lệch âm
    //     }
    //   }
    //   else
    //   {
    //     robot->balanceSpdNoPWM(0, 0);
    //     if (robot->isStop())
    //     {
    //       referenceAngle = robot->getFilteredAngle(); // Cập nhật lại để currentAngle = 0
    //       isInitialized = true;
    //       Serial.println("Initial alignment completed");
    //       delay(1000);
    //     }
    //   }
    //   return;
    // }

    switch (currentStep.type)
    {
    case FORWARD:
      if (distanceTraveled < (targetDistance - 0.2f) * 1000.0f) // Nếu khoảng cách di chuyển nhỏ hơn 20cm
      {
        Serial.print(" - case: ");
        Serial.print("FORWARD");
        Serial.print(" - distance: ");
        Serial.print(distanceTraveled);
        Serial.print(" - targetDistance: ");
        Serial.println(targetDistance * 1000.0f);
        robot->balanceSpdNoPWM(50, 50); // Chạy về phía trước
      }
      else
      {
        robot->resetEncoders(); // Reset encoder
        currentStepIndex++;
        Serial.println("Moving to next step");
      }
      break;

    case TURN_LEFT:
      if (angleDiff > 5.0f)
      {
        robot->turnLeftNoPWM(50); // Quay trái
      }
      else
      {
        robot->resetEncoders(); // Reset encoder
        currentStepIndex++;
        Serial.println("Moving to next step");
      }
      break;

    case TURN_RIGHT:
      if (angleDiff < -5.0f)
      {
        robot->turnRightNoPWM(50); // Quay phải
      }
      else
      {
        robot->resetEncoders(); // Reset encoder
        currentStepIndex++;
        Serial.println("Moving to next step");
      }

    default:
      robot->balanceSpdNoPWM(0, 0); // Dừng robot
      break;
    }
  }
  else if (SERIAL_TESTING)
  {
    static char c;
    if (Serial.available() > 0)
    {
      c = Serial.read();
      if (c == 'w')
      {
        robot->balanceSpdNoPWM(50, 50);
      }
      else if (c == 's')
      {
        robot->balanceSpdNoPWM(0, 0);
      }
      else if (c == 'a')
      {
        robot->turnLeftNoPWM(50);
      }
      else if (c == 'd')
      {
        robot->turnRightNoPWM(50);
      }
      else if (c == 'x')
      {
        robot->stop();
      }
    }
  }
  else
  {

    // Update GPS data
    while (gpsSerial.available() > 0)
    {
      gps.encode(gpsSerial.read());
      isGpsWorking = true;
    }

    // loop through the steps
    for (const Step &step : steps)
    {
      Serial.print("Step ");
      Serial.print(step.type);
      Serial.print(" - ");
      Serial.print(step.distance);
      Serial.print(" - ");
      Serial.print(step.targetLat);
      Serial.print(" - ");
      Serial.print(step.targetLon);
      Serial.print(" - ");
      Serial.print(step.startLat);
      Serial.print(" - ");
      Serial.println(step.startLon);

      float targetBearing = 0.0f;
      float bearingDiff = 0.0f;
      float distance = 0.0f;
      targetLat = step.targetLat;
      targetLon = step.targetLon;
      float startLat = step.startLat;
      float startLon = step.startLon;

      // keep turning till bearing difference is less than 5 degrees
      // to do this: need current heading and target bearing
      do
      {
        robot->updateMotorSpeeds();
        // Update GPS data again if available
        while (gpsSerial.available() > 0)
        {
          gps.encode(gpsSerial.read());
          isGpsWorking = true;
        }
        // if gps good
        if (gps.location.isValid() && !TESTING)
        {
          isGpsWorking = true;
          currentLat = gps.location.lat();
          currentLon = gps.location.lng();
          targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);
          // Calculate distance to target using GPS
          distance = calculateDistance(currentLat, currentLon, targetLat, targetLon); // Distance in meters
        }
        else
        {
          // or else use the default GPS data
          isGpsWorking = false;
          targetBearing = calculateBearing(startLat, startLon, targetLat, targetLon);
          // fix gps distance
          distance = step.distance; // Distance in meters
        }
        Serial.print(distance);
        Serial.print(" - ");
        Serial.print(targetBearing);
        Serial.print(" - ");

        // Get the current heading using GPS or IMU
        if (gps.course.isValid() && !TESTING)
        {
          currentHeading = gps.course.deg();
        }
        else
        {
          robot->updateIMUdata();
          currentHeading = robot->getFilteredAngle();
        }
        Serial.print(currentHeading);
        Serial.println("");

        // Calculate bearing difference (-180 to 180 degrees)
        bearingDiff = targetBearing - currentHeading;
        if (bearingDiff > 180)
          bearingDiff -= 360;
        if (bearingDiff < -180)
          bearingDiff += 360;
        // Adjust heading if
        if (bearingDiff > 5)
        {
          robot->turnRight(10);
        }
        else if (bearingDiff < -5)
        {
          robot->turnLeft(10);
        }
        delay(45);
      } while (abs(bearingDiff) > 5);

      // move forward
      if (isGpsWorking && !TESTING)
      {
        do
        {
          delay(50);
          robot->updateMotorSpeeds();
          robot->moveForward(30);
          updateCurLocation();
          distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
        } while (distance > 5 && distance != -1.0f);
      }
      else
      {
        do
        {
          delay(50);
          robot->updateMotorSpeeds();
          robot->moveForward(30);
        } while (robot->getDistanceTraveled() < (step.distance - 1.0f) * 1000.0f);
      }
      // stop the robot
      robot->stop();
      delay(1000);
      // reset encoders to ready for next step
      robot->resetEncoders();
    }
  }
}

void maintainHeading(float targetYaw, double speed)
{
  float currentYaw;
  do
  {
    currentYaw = robot->getFilteredAngle();
    float error = targetYaw - currentYaw;
    if (error > 5)
    {
      robot->turnRight(50);
    }
    else if (error < -5)
    {
      robot->turnLeft(50);
    }
    else
    {
      robot->moveForward(speed);
    }
  } while (robot->getDistanceTraveled() < 1000); // 1 mét
  robot->stop();
}

// calculate distance between 2 GPS points in meters
float calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
  // Convert degrees to radians
  float lat1Rad = lat1 * PI / 180.0;
  float lon1Rad = lon1 * PI / 180.0;
  float lat2Rad = lat2 * PI / 180.0;
  float lon2Rad = lon2 * PI / 180.0;

  // Haversine formula
  float dlon = lon2Rad - lon1Rad;
  float dlat = lat2Rad - lat1Rad;
  float a = pow(sin(dlat / 2), 2) + cos(lat1Rad) * cos(lat2Rad) * pow(sin(dlon / 2), 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  // Earth radius in meters
  float radius = 6371000;
  return radius * c;
}

// 📌 Tính toán góc giữa 2 điểm GPS don vi do
float calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
  if (!checkGpsValid(lat1, lon1) || !checkGpsValid(lat2, lon2))
  {
    Serial.println("Invalid latitude or longitude values");
    return 0; // Return 0 or handle error as needed
  }
  float dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float bearing = degrees(atan2(y, x));

  // Chuẩn hóa về [-180°, 180°]
  if (bearing > 180.0)
    bearing -= 360.0;
  if (bearing < -180.0)
    bearing += 360.0;
  return bearing;
}

void updateNavigation()
{
  if (currentStepIndex >= totalSteps)
  {
    // End of route reached
    robot->stop();
    Serial.println("Destination reached!");
    return;
  }

  // Get current target from our step
  targetLat = steps[currentStepIndex].targetLat;
  targetLon = steps[currentStepIndex].targetLon;

  // Calculate distance to target
  float distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);

  // Calculate bearing to target
  float targetBearing = calculateBearing(currentLat, currentLon, targetLat, targetLon);

  // Calculate bearing difference (-180 to 180 degrees)
  float bearingDiff = targetBearing - currentHeading;
  if (bearingDiff > 180)
    bearingDiff -= 360;
  if (bearingDiff < -180)
    bearingDiff += 360;

  Serial.print("Step ");
  Serial.print(currentStepIndex + 1);
  Serial.print("/");
  Serial.print(totalSteps);

  Serial.print("Distance to next point: ");
  Serial.print(distance);
  Serial.println(" meters");

  Serial.print("Bearing difference: ");
  Serial.print(bearingDiff);
  Serial.println(" degrees");

  // Execute navigation based on step type and bearing
  executeCurrentStep(targetBearing, bearingDiff);

  // Check if we're close enough to target to move to next step
  if (distance < 5.0)
  { // Within 5 meters of target
    currentStepIndex++;
    Serial.println("Moving to next step");

    // If this was the last step, stop the robot
    if (currentStepIndex >= totalSteps)
    {
      robot->stop();
      Serial.println("Route completed!");
    }
  }
  else
  {
    // Maintain heading
    maintainHeading(targetBearing, 50);
  }
}

void executeCurrentStep(float targetBearing, float bearingDiff)
{
  while (abs(bearingDiff) > 5)
  {
    // Adjust heading
    if (bearingDiff > 5)
    {
      robot->turnRight(50);
    }
    else if (bearingDiff < -5)
    {
      robot->turnLeft(50);
    }
    robot->updateIMUdata();
    currentHeading = robot->getFilteredAngle();
    bearingDiff = targetBearing - currentHeading;
  }
}

// return distance in meters or -1 if GPS is not working
void updateCurLocation()
{
  while (gpsSerial.available() > 0)
  {
    gps.encode(gpsSerial.read());
    isGpsWorking = true;
  }
  if (gps.location.isValid())
  {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
  }
  else
  {
    isGpsWorking = false;
  }
}

bool checkGpsValid(double &lat, double &lon)
{
  if (lat == 0.0f && lon == 0.0f)
  {
    Serial.println("Invalid GPS coordinates");
    return false;
  }
  else if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f)
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
  while (angle > 180.0f)
    angle -= 360.0f;
  while (angle < -180.0f)
    angle += 360.0f;
  return angle;
}