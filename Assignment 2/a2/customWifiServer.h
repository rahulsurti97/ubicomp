#ifndef CUSTOM_WIFI_SERVER_H
#define CUSTOM_WIFI_SERVER_H

#define USING_WIFI_SERVER true

#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>

void connectToWifi();
void sendDataOverWifi(String data);

#endif  // CUSTOM_WIFI_SERVER_H