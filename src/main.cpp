#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define WIFI_SSID "your_wifi"
#define WIFI_PASSWORD "your_password"
#define API_URL "https://api.openrouteservice.org/v2/directions/driving-hgv?api_key=5b3ce3597851110001cf62489b1a4894dc59438fa047b32238086ce2&start=%20105.79887063405867,%2021.01852430824737&end=105.79762065889055,%2021.017536256274408"

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

void setup()
{
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected!");
  fetchRoute();
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
  digitalWrite(25, HIGH);
  digitalWrite(26, HIGH);
  analogWrite(27, 100);
  analogWrite(14, 100);
}

void turnLeft()
{
  digitalWrite(25, LOW);
  digitalWrite(26, HIGH);
  analogWrite(27, 50);
  analogWrite(14, 100);
}

void turnRight()
{
  digitalWrite(25, HIGH);
  digitalWrite(26, LOW);
  analogWrite(27, 100);
  analogWrite(14, 50);
}

void stopMotors()
{
  analogWrite(27, 0);
  analogWrite(14, 0);
}
