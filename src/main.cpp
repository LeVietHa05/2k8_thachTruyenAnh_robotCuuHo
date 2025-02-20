#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <WiFiManager.h>

#define API_URL "https://api.openrouteservice.org/v2/directions/driving-hgv"
#define API_KEY "5b3ce3597851110001cf62489b1a4894dc59438fa047b32238086ce2"
#define temp "?start=%20105.79887063405867,%2021.01852430824737&end=105.79762065889055,%2021.017536256274408"

#define dw digitalWrite
#define dr digitalRead

#define MT1_L 25
#define MT1_R 26
#define MT2_L 27
#define MT2_R 14

#define PWM_CHANNEL0 0
#define PWM_CHANNEL1 1
#define PWM_CHANNEL2 2
#define PWM_CHANNEL3 3
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

struct Step
{
  float distance;
  float duration;
  int type;
  String instruction;
};

Step steps[10]; // Lưu tối đa 10 bước
int currentStep = 0;

void fetchRoute();
void moveAccordingToStep(int type);
void turnLeft();
void turnRight();
void moveForward();
void stopMotors();

void setup()
{
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  WiFiManager wm;

  bool res = wm.autoConnect("AutoConnectAP", "password");
  if (!res)
  {
    Serial.println("Failed to connect to WiFi");
    while (1)
    {
      dw(LED_BUILTIN, HIGH);
      delay(300);
      dw(LED_BUILTIN, LOW);
      delay(300);
    }
  }

  Serial.println("Connected!");
  fetchRoute();

  ledcSetup(PWM_CHANNEL0, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MT1_L, PWM_CHANNEL0);
  ledcAttachPin(MT1_R, PWM_CHANNEL1);
  ledcAttachPin(MT2_L, PWM_CHANNEL2);
  ledcAttachPin(MT2_R, PWM_CHANNEL3);
}

void loop()
{
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated())
    {
      float lat_now = gps.location.lat();
      float lon_now = gps.location.lng();

      Serial.printf("Current: %f, %f\n", lat_now, lon_now);

      if (currentStep < 10)
      {
        Serial.printf("Instruction: %s\n", steps[currentStep].instruction.c_str());
        moveAccordingToStep(steps[currentStep].type);

        // Giả lập đi xong đoạn đường
        delay(steps[currentStep].duration * 1000);
        currentStep++;
      }
      else
      {
        stopMotors();
        Serial.println("Route completed!");
        while (1)
          ;
      }
    }
  }
}

// 📌 Lấy dữ liệu tuyến đường từ API
void fetchRoute()
{
  HTTPClient http;
  http.begin(API_URL);
  int httpCode = http.GET();

  if (httpCode == 200)
  {
    String payload = http.getString();
    Serial.println("Received route:");
    Serial.println(payload);

    StaticJsonDocument<2048> doc;
    deserializeJson(doc, payload);

    JsonArray stepsJson = doc["segments"][0]["steps"];
    int i = 0;
    for (JsonObject step : stepsJson)
    {
      steps[i].distance = step["distance"];
      steps[i].duration = step["duration"];
      steps[i].type = step["type"];
      steps[i].instruction = step["instruction"].as<String>();
      i++;
    }
  }
  else
  {
    Serial.printf("HTTP error: %d\n", httpCode);
  }
  http.end();
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
    Serial.println("Unknown step type");
    break;
  }
}

// 📌 Hàm di chuyển robot
void moveForward()
{
  ledcWrite(PWM_CHANNEL0, 255);
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, 255);
  ledcWrite(PWM_CHANNEL3, 0);
}

void turnLeft()
{
  ledcWrite(PWM_CHANNEL0, 100);
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, 255);
  ledcWrite(PWM_CHANNEL3, 0);
}

void turnRight()
{
  ledcWrite(PWM_CHANNEL0, 255);
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, 100);
  ledcWrite(PWM_CHANNEL3, 0);
}
void stopMotors()
{
  ledcWrite(PWM_CHANNEL0, 0);
  ledcWrite(PWM_CHANNEL1, 0);
  ledcWrite(PWM_CHANNEL2, 0);
  ledcWrite(PWM_CHANNEL3, 0);
}
