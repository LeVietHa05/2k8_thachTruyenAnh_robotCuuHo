#include <Arduino.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>
#include <Wire.h>

#include "config.h" // Include configuration for host and API key
#define TESTING 1

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

// Current position and orientation
float currentLat = 0.0;
float currentLon = 0.0;
float targetLat = 0.0;
float targetLon = 0.0;
float currentHeading = 0.0;
bool routeLoaded = false;

void maintainHeading(float targetYaw, double speed);
bool checkGpsValid(float &lat, float &lon);
float normalizeAngle(float angle);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);
void updateNavigation();
void executeCurrentStep(float targetBearing, float bearingDiff);
void updateCurLocation();

void setup()
{
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
  // Update GPS data
  updateCurLocation();

  if (TESTING || !isGpsWorking)
  {
    steps[0].distance = 10.0f;
    steps[0].type = 0;
    steps[0].targetLat = 21.01731031312228;
    steps[0].targetLon = 105.79836476525836;
    steps[0].startLat = 21.017901101150503;
    steps[0].startLon = 105.79856804153376;

    steps[1].distance = 12.0f;
    steps[1].type = 0;
    steps[1].targetLat = 21.017533048366026;
    steps[1].targetLon = 105.7976265862394;
    steps[1].startLat = 21.01731031312228;
    steps[1].startLon = 105.79836476525836;

    steps[2].distance = 15.0f;
    steps[2].type = 0;
    steps[2].targetLat = 21.017012264642755;
    steps[2].targetLon = 105.79740932732533;
    steps[2].startLat = 21.017533048366026;
    steps[2].startLon = 105.7976265862394;
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
  }

  robot = new Robot(2.0, 1.0, 5.0);

  robot->attachMotors(MT1_R, MT1_L, MT2_R, MT2_L);
  robot->attachEncoders(ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B);
  robot->initIMU();
}

void loop()
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

      // Calculate bearing difference (-180 to 180 degrees)
      bearingDiff = targetBearing - currentHeading;
      if (bearingDiff > 180)
        bearingDiff -= 360;
      if (bearingDiff < -180)
        bearingDiff += 360;
      // Adjust heading if
      if (bearingDiff > 5)
      {
        robot->turnRight(150);
      }
      else if (bearingDiff < -5)
      {
        robot->turnLeft(150);
      }
    } while (abs(bearingDiff) > 5);

    // move forward
    if (isGpsWorking && !TESTING)
    {
      do
      {
        robot->moveForward(200);
        updateCurLocation();
        distance = calculateDistance(currentLat, currentLon, targetLat, targetLon);
      } while (distance > 5 && distance != -1.0f);
    }
    else
    {
      do
      {
        robot->moveForward(200);
      } while (robot->getDistanceTraveled() < (step.distance - 1.0f) * 1000.0f);
    }
    // stop the robot
    robot->stop();
    // reset encoders to ready for next step
    robot->resetEncoders();
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
float calculateDistance(float lat1, float lon1, float lat2, float lon2)
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