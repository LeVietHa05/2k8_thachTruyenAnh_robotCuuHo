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

void moveAccordingToStep(String instruction);
void maintainHeading(float targetYaw, double speed);
bool checkGpsValid(float &lat, float &lon);
float normalizeAngle(float angle);
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearing(float lat1, float lon1, float lat2, float lon2);
void updateNavigation();
void executeCurrentStep(float targetBearing, float bearingDiff);

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
  if (fetchRoute(host, apiKey, 21.01867661642452, 105.7985191732634, 21.01643685342504, 105.80130384889998))
  {
    Serial.println("Route fetched successfully");
  }
  else
  {
    Serial.println("Failed to fetch route");
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

  // If we have valid GPS data and a route
  if (gps.location.isValid() && routeLoaded && isGpsWorking)
  {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();

    if (gps.course.isValid())
    {
      currentHeading = gps.course.deg();
    }
    else
    {
      // Use IMU for heading if GPS course is not available
      float ax, ay, az, gx, gy, gz;
      currentHeading = robot->getFilteredAngle();
    }

    // Update navigation at regular intervals
    if (millis() - lastNavigationUpdate > NAVIGATION_UPDATE_INTERVAL)
    {
      updateNavigation();
      lastNavigationUpdate = millis();
    }
  }
  if (!isGpsWorking)
  {
    // GPS signal lost, stop robot
    if (TESTING)
    {
    }
    else
    {
      robot->stop();
      Serial.println("GPS signal lost");
    }
  }
}

void moveAccordingToStep(String instruction)
{
  instruction.toLowerCase();
  if (instruction.indexOf("left") != -1)
  {
    // TODO: turn left
  }
  else if (instruction.indexOf("right") != -1)
  {
    // TODO: turn right
  }
  else if (instruction.indexOf("straight") != -1)
  {
    // TODO: go straight
  }
  else if (instruction.indexOf("arrive") != -1)
  {
    // TODO: stop robot
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
  Serial.print(": ");
  Serial.println(steps[currentStepIndex].instruction);

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
  // If we need to adjust heading by more than 20 degrees, turn first
  if (abs(bearingDiff) > 10)
  {
    if (bearingDiff > 0)
    {
      // Turn right
      Serial.println("Turning right to adjust heading");
      robot->turnRight(150); // Adjust speed as needed
      delay(100);            // Short turn duration
    }
    else
    {
      // Turn left
      Serial.println("Turning left to adjust heading");
      robot->turnLeft(150); // Adjust speed as needed
      delay(100);           // Short turn duration
    }
  }
  else
  {
    // Heading is close enough, move forward
    int stepType = steps[currentStepIndex].type;
    String instruction = steps[currentStepIndex].instruction;

    // Log current action
    Serial.print("Executing: ");
    Serial.println(instruction);

    moveAccordingToStep(instruction);
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