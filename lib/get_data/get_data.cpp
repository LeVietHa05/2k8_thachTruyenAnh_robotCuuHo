#include <Arduino.h>
#include "get_data.h"

// 📌 Lấy dữ liệu tuyến đường từ API
bool fetchRoute(const char *host, const char *apiKey, float startLat, float startLon, float endLat, float endLon)
{
    WiFiClientSecure client;
    client.setInsecure();
    Serial.println("Connecting to API...");

    if (!client.connect(host, 443))
    {
        Serial.println("Connection failed!");
        return false;
    }

    String url = "/v2/directions/driving-car?api_key=" + String(apiKey) +
                 "&start=" + String(startLon, 6) + "," + String(startLat, 6) +
                 "&end=" + String(endLon, 6) + "," + String(endLat, 6) +
                 "&format=json";

    Serial.println("Requesting: " + url);

    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");

    while (client.connected() || client.available())
    {
        if (client.available())
        {
            String line = client.readStringUntil('\n');
            if (line == "\r")
                break;
        }
    }
    String payload = readChunkedResponse(client);
    client.stop(); // Close the connection

    Serial.println("Done getting response");

    int stepCount = 0;
    if (!extractStepsFromGeoJSON(payload.c_str(), steps, stepCount))
    {
        Serial.println("Failed to extract steps from GeoJSON");
        return false;
    }

    for (int i = 0; i < stepCount; i++)
    {
        Serial.print("Step ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(steps[i].distance);
    }
    return true;
}

bool extractStepsFromGeoJSON(const char *json, Step steps[], int &stepCount)
{
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error)
    {
        Serial.print("JSON parsing failed: ");
        Serial.println(error.c_str());
        return false;
    }

    JsonArray features = doc["features"];
    // check null or empty
    if (features.isNull() || features.size() == 0)
    {
        Serial.println("No features found in GeoJSON");
        return false;
    }

    JsonObject properties = features[0]["properties"];
    if (properties.isNull())
    {
        Serial.println("No properties found in JSON");
        return false;
    }

    JsonArray segments = properties["segments"];
    if (segments.isNull() || segments.size() == 0)
    {
        Serial.println("No segments found in JSON");
        return false;
    }

    JsonArray stepsArray = segments[0]["steps"];
    if (stepsArray.isNull())
    {
        Serial.println("No steps found in JSON");
        return false;
    }

    JsonArray coordinates = features[0]["geometry"]["coordinates"];
    if (coordinates.isNull() || coordinates.size() == 0)
    {
        Serial.println("No coordinates found in JSON");
        return false;
    }

    stepCount = 0; // Reset step count

    for (JsonObject step : stepsArray)
    {
        if (stepCount >= MAX_STEPS_ALLOWED)
        {
            Serial.println("Warning: Maximum steps exceeded, truncating route");
            break;
        }

        JsonArray wayPoints = step["way_points"];
        if (wayPoints.isNull() || wayPoints.size() < 2)
        {
            continue; // Skip invalid steps
        }

        int startIdx = wayPoints[0];
        int endIdx = wayPoints[1];

        if (endIdx >= coordinates.size())
        {
            Serial.println("Warning: Invalid coordinate index");
            continue;
        }

        float longitude = coordinates[endIdx][0];
        float latitude = coordinates[endIdx][1];

        steps[stepCount] = {
            step["distance"],
            step["type"],
            targetLat : latitude,
            targetLon : longitude,
            startLat : coordinates[startIdx][1],
            startLon : coordinates[startIdx][0]
        };

        stepCount++;
    }

    return stepCount > 0; // Successfully extracted at least one step
}

String readChunkedResponse(WiFiClientSecure &client)
{
    String payload = "";
    while (client.available())
    {
        String chunkSizeStr = client.readStringUntil('\n');
        int chunkSize = strtol(chunkSizeStr.c_str(), NULL, 16);
        if (chunkSize <= 0)
            break;
        while (chunkSize > 0)
        {
            char c = client.read();
            payload += c;
            chunkSize--;
        }
        client.read(); // Read the trailing '\r'
        client.read(); // Read the trailing '\n'
    }
    return payload;
}