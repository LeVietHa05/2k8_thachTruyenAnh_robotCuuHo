#include <Arduino.h>
#include "get_data.h"

// 📌 Lấy dữ liệu tuyến đường từ API
bool fetchRoute(const char *host, const char *apiKey)
{
    WiFiClientSecure client;
    client.setInsecure();
    Serial.println("Connecting to API...");

    if (!client.connect(host, 443))
    {
        Serial.println("Connection failed!");
        return false;
    }

    String url = "/v2/directions/driving-car?api_key=" + String(apiKey) + "&start=8.681495,49.41461&end=8.687872,49.420318&format=json";
    Serial.println("Requesting: " + url);

    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");

    String payload;
    while (client.connected() || client.available())
    {
        if (client.available())
        {
            String line = client.readStringUntil('\n');
            if (line == "\r")
                break;
        }
    }
    while (client.available())
    {
        payload += client.readString();
    }
    Serial.println("Done getting response");
    client.stop(); // Close the connection

    int stepCount = 0;
    if (!extractStepsFromGeoJSON(payload.c_str(), steps, stepCount)) {
        Serial.println("Failed to extract steps from GeoJSON");
        return false;
    }

    for (int i = 0; i < stepCount; i++)
    {
        Serial.print("Step ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(steps[i].instruction);
    }
    return true;
}

bool extractStepsFromGeoJSON(const char *json, Step steps[], int &stepCount)
{
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error) {
        Serial.println("Failed to parse JSON");
        return false;
    }

    JsonArray features = doc["features"];
    JsonObject properties = features[0]["properties"];
    JsonArray stepsArray = properties["segments"][0]["steps"];
    JsonArray coordinates = features[0]["geometry"]["coordinates"];

    stepCount = 0; // Reset step count

    for (JsonObject step : stepsArray)
    {
        int startIdx = step["way_points"][0];
        int endIdx = step["way_points"][1];

        steps[stepCount] = {
            step["distance"],
            step["duration"],
            step["type"],
            step["instruction"].as<String>(),
            coordinates[endIdx][1], // Latitude
            coordinates[endIdx][0]  // Longitude
        };
        stepCount++;
    }
    return true; // Successfully extracted steps
}
