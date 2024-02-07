#include "config.h"
#include <WiFi.h>
#include <HTTPClient.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

const int BATTERY_PIN = A13;
const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2
const float MIN_BATTERY_VOLTAGE = 3.2; // MIN LiPoly voltage of a 3.7 battery is 3.2

const int SERIAL_BAUD_RATE = 115200; // make sure this matches the value in AccelRecorder.pde

const int numAccReadings = 5; // Number of readings to average
float mag_data[numAccReadings]; // Array to store accelerometer magnitude data for smoothing
float magnitudeSum = 0;
int accIndex = 0; // Index for storing the latest reading

const int stepDetectionWindow = 20; //store 20 samples, loop delay is 50ms, so window is 1 second worth of data
float step_data[stepDetectionWindow]; // Array to store accelerometer magnitude data for step detection
int stepIndex = 0;
int totalSteps = 0;

const unsigned long DISPLAY_INTERVAL = 500;
unsigned long t_last_display_time = 0;
const unsigned long WIFI_DATA_INTERVAL = 250;
unsigned long t_last_wifi_data_time = 0;

void setup(void) {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  pinMode(BATTERY_PIN, INPUT);

  Serial.println("LIS3DH test!");

  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  // lis.setDataRate(LIS3DH_DATARATE_50_HZ);
  Serial.print("Data rate set to: ");
  switch (lis.getDataRate()) {
    case LIS3DH_DATARATE_1_HZ: Serial.println("1 Hz"); break;
    case LIS3DH_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3DH_DATARATE_25_HZ: Serial.println("25 Hz"); break;
    case LIS3DH_DATARATE_50_HZ: Serial.println("50 Hz"); break;
    case LIS3DH_DATARATE_100_HZ: Serial.println("100 Hz"); break;
    case LIS3DH_DATARATE_200_HZ: Serial.println("200 Hz"); break;
    case LIS3DH_DATARATE_400_HZ: Serial.println("400 Hz"); break;

    case LIS3DH_DATARATE_POWERDOWN: Serial.println("Powered Down"); break;
    case LIS3DH_DATARATE_LOWPOWER_5KHZ: Serial.println("5 Khz Low Power"); break;
    case LIS3DH_DATARATE_LOWPOWER_1K6HZ: Serial.println("16 Khz Low Power"); break;
  }

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  for(int i = 0; i < numAccReadings; i++) {
    mag_data[i] = 0;
  }

  for(int i = 0; i < stepDetectionWindow; i++) {
    step_data[i] = 0;
  }

  totalSteps = 0;

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(1000); // Pause for 1 seconds

  // Clear the buffer
  display.clearDisplay();

  connectToWifi();
}

void loop() {
  unsigned long currentTime = millis();

  sensors_event_t event;
  float currentMagnitude = getSmoothedMagnitude(&event); 

  int stepCount = detectStep(currentMagnitude);
  totalSteps += stepCount;

  if(currentTime - t_last_display_time > DISPLAY_INTERVAL) {
    display.clearDisplay();
    writeAccToDisplay(&event);
    writeBatteryToDisplay();
    writeStepCountToDisplay();
    display.display();
    t_last_display_time = currentTime;
  }

  if(currentTime - t_last_wifi_data_time > WIFI_DATA_INTERVAL && stepCount > 0) {
    sendDataOverWifi((String)currentTime + "," + totalSteps);
    t_last_wifi_data_time = currentTime;
  }

  delay(50); // record ~20 samples/second 
}

int detectStep(float mag) {
  Serial.println(mag);
  step_data[stepIndex] = mag;
  stepIndex++;

  if(stepIndex < stepDetectionWindow) {
    return 0;
  }

  // Buffer is full, start processing
  stepIndex = 0;

  int peakCount = 0;
  float peakAccumulate = 0;
  float backwardSlope = 0;
  float forwardSlope = 0;
  for(int i = 1; i < stepDetectionWindow - 1; i++) {
    backwardSlope = step_data[i] - step_data[i-1];
    forwardSlope = step_data[i+1] - step_data[i];
    if(forwardSlope < 0 && backwardSlope > 0) {
      peakCount += 1;
      peakAccumulate += step_data[i];
    }
  }
  float peakMean = peakAccumulate/peakCount;

  int stepCount = 0;
  for(int i = 1; i < stepDetectionWindow - 1; i++){
    backwardSlope = step_data[i] - step_data[i-1];
    forwardSlope = step_data[i+1] - step_data[i];
    if(forwardSlope < 0 && backwardSlope > 0 && 
       step_data[i] > 0.6 * peakMean && step_data[0] > 10) {
        stepCount += 1;
    }
  }

  return min(stepCount, 3); //no more than 3 steps/second
}

float getSmoothedMagnitude(sensors_event_t *event){
  lis.read();
  lis.getEvent(event);

  float currentMagnitude = sqrt(event->acceleration.x * event->acceleration.x + 
                                event->acceleration.y * event->acceleration.y + 
                                event->acceleration.z * event->acceleration.z);

  magnitudeSum = magnitudeSum - mag_data[accIndex] + currentMagnitude;
  mag_data[accIndex] = currentMagnitude;
  accIndex = (accIndex + 1) % numAccReadings;
  return magnitudeSum / numAccReadings; // return avg magnitude
}

void writeAccToDisplay(sensors_event_t *event){
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(SSD1306_WHITE);

  display.println((String)"X:" + event->acceleration.x + "m/s^2");
  display.println((String)"Y:" + event->acceleration.y + "m/s^2");
  display.println((String)"Z:" + event->acceleration.z + "m/s^2");
}

void writeBatteryToDisplay(){
  int rawValue = analogRead(BATTERY_PIN); // Read raw ADC value
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calculate voltage level
  float batteryPercentage = calculateBatteryPercentage(voltageLevel);

  // Write battery text
  String percent = (String)batteryPercentage + "%";
  int16_t x, y;
  uint16_t w, h;
  display.getTextBounds(percent, 0, 0, &x, &y, &w, &h);
  display.setCursor(SCREEN_WIDTH - w, 0);
  display.println(percent);

  String text = (String)voltageLevel + "V";
  display.getTextBounds(text, 0, 0, &x, &y, &w, &h);
  display.setCursor(SCREEN_WIDTH - w, h);
  display.println(text);
}

void writeStepCountToDisplay(){
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.setTextColor(SSD1306_WHITE);
  display.println((String)"Step Count:" + totalSteps);
}

int calculateBatteryPercentage(float voltage) {
  if (voltage <= MIN_BATTERY_VOLTAGE) {
    return 0;
  } else if (voltage >= MAX_BATTERY_VOLTAGE) {
    return 100;
  } else {
    return map(voltage, MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE, 0, 100);
  }
}

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
