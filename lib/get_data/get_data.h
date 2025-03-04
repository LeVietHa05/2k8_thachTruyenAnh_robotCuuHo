#ifndef GET_DATA_H
#define GET_DATA_H

#include "step.h" // Include the Step struct definition
#include <iostream>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

#include "config.h" // Include config after step.h

bool fetchRoute(const char *host, const char *apiKey);
bool extractStepsFromGeoJSON(const char *json, Step steps[], int &stepCount);

#endif // GET_DATA_H