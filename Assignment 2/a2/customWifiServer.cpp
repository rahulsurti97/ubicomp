#include "customWifiServer.h"

void sendDataOverWifi(String data) {
  connectToWifi();

  HTTPClient http;

  // Start HTTP POST request
  http.begin(SERVER_ADDRESS);
  http.addHeader("Content-Type", "text/plain");

  // Send data
  int httpResponseCode = http.POST(data);

  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();
}

void connectToWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Reconnected to WiFi");
}