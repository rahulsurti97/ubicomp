#include "adafruitio.h"

void connectToAdafruit(){
  Serial.print("Connecting to Adafruit IO");
  io.connect();

  // Wait for a connection
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

    // We are connected
  Serial.println();
  Serial.println(io.statusText());
}
