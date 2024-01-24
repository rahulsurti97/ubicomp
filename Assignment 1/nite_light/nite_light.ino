const boolean COMMON_ANODE = true;

const int RGB_RED_PIN = 6;
const int RGB_GREEN_PIN = 5;
const int RGB_BLUE_PIN = 3;
const int DELAY_MS = 20;  // delay in ms between changing colors
const int MAX_COLOR_VALUE = 255;

enum RGB {
  RED,
  GREEN,
  BLUE,
  NUM_COLORS
};

int _rgbLedValues[] = { 0, 255, 0 };  // Red, Green, Blue
enum RGB _curFadingUpColor = GREEN;
enum RGB _curFadingDownColor = RED;
const int FADE_STEP = 5;
int lastCrossfadeTime = 0;

// Button
const int INPUT_BUTTON_PIN = 2;
const int DEBOUNCE_WINDOW = 40; // in milliseconds
int _prevRawButtonVal = HIGH;  //starts high because using pull-up resistor
int _debouncedButtonVal = HIGH;
unsigned long _buttonStateChangeTimestamp = 0;  // the last time the input pin was toggled
int _buttonMode = 0;

// Photoresistor
const int INPUT_PHOTORESISTOR_PIN = A0;

// Microphone
const int INPUT_MIC_PIN = A1;
const int MAX_ANALOG_IN = 1024; 
const int MAX_ANALOG_OUT = 255;
const int MIC_SAMPLE_WINDOW = 40; // in milliseconds
unsigned long _lastMicrophoneRead;
int _micSignalMax = 0;
int _micSignalMin = 1024;

// Potentiometer
const int INPUT_POT_PIN = A2;

// Serial in buffer
const int BUFFER_SIZE = 12;
char serialBuffer[BUFFER_SIZE] = "\0";

void setup() {
  // Turn on Serial so we can verify expected colors via Serial Monitor
  Serial.begin(9600);

  // Setup button input and starting mode
  pinMode(INPUT_BUTTON_PIN, INPUT_PULLUP); // Set the button pin as INPUT with internal pull-up resistor
  _buttonMode = 0;

  // Setup RGB LED
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);

  // Set initial color and timing window
  setColor(_rgbLedValues[RED], _rgbLedValues[GREEN], _rgbLedValues[BLUE], 0);
  lastCrossfadeTime = millis();

  // Setup mic input and timing window
  pinMode(INPUT_MIC_PIN, INPUT);
  _lastMicrophoneRead = millis();
}

void loop() {
  // Evaluate button mode
  readButtonMode();
  crossfadeLed(_buttonMode == 0);
  readMicrophone(_buttonMode == 1);
  readPotentiometer(_buttonMode == 2);
  readSerialInput(_buttonMode == 3);
}

void readSerialInput(bool enabled) {
  if(!enabled) {
    return;
  }

  if (Serial.available() > 0) {
    // Read the incoming serial data
    char incomingChar = Serial.read();
    
    // If end of line, parse entire buffer
    if(incomingChar == '\n'){
      if (validateSerialBufferHexCode()) {
        Serial.print("Valid hex code received: ");
        Serial.println(serialBuffer);
        setHexColor();
      } else {
        Serial.print("Invalid input. Expected 6 digit hex code, received: ");
        Serial.println(serialBuffer);
      }

      serialBuffer[0] = '\0'; // Assign an empty string
    } else {
      appendChar(incomingChar) ;
    }
  }
}

void setHexColor() {
  // Convert hex code to RGB values
  unsigned long colorValue;
  sscanf(serialBuffer, "%lx", &colorValue);

  // Extract red, green, and blue components
  int red = (colorValue >> 16) & 0xFF;
  int green = (colorValue >> 8) & 0xFF;
  int blue = colorValue & 0xFF;
  Serial.print("R:");
  Serial.print(red);
  Serial.print(" G:");
  Serial.print(green); 
  Serial.print(" B:");
  Serial.println(blue);

  // Set the color on the GRB LED
  setColor(red, green, blue, 1.0);
}

bool validateSerialBufferHexCode() {
  // Validate input length and '0x' prefix
  if(strlen(serialBuffer) != 8 || serialBuffer[0] != '0' && serialBuffer[1] != 'x' && serialBuffer[1] != 'X') {
    return false;
  }

  // Validate input is 0-9a-fA-F
  for(int i = 2; i < 8; i++){
    if(!((serialBuffer[i] >= 'a' && serialBuffer[i] <= 'f') || 
         (serialBuffer[i] >= 'A' && serialBuffer[i] <= 'F') ||
         (serialBuffer[i] >= '0' && serialBuffer[i] <= '9'))) {
      return false;
    }
  }
  
  return true;
}

void appendChar(char newChar) {
  int currentLength = strlen(serialBuffer);

  // Check if there is enough space to append the character
  if (currentLength < BUFFER_SIZE - 1) {
    serialBuffer[currentLength] = newChar;
    serialBuffer[currentLength + 1] = '\0'; // Null-terminate the string
  } else {
    // Handle the case where the buffer is full
    Serial.println("Buffer is full, cannot append more characters.");
  }
}

void readPotentiometer(bool enabled) {
  if(!enabled) {
    return;
  }

  int potReading = analogRead(INPUT_POT_PIN);
  int redValue = 0;
  int greenValue = 0;
  int blueValue = 0;

  // Set color based on potentiometer reading
  // red peaks at 255, green peaks at 512, blue peaks at 768, 0 and 1024 have zero led output.
  if(potReading < 256) {
    redValue = potReading;
  } else if(potReading < 512) {
    redValue = 256 - (potReading - 256);
    greenValue = potReading - 256;
  } else if(potReading < 768) {
    greenValue = 256 - (potReading - 512);
    blueValue = potReading - 512;
  } else {
    blueValue = 256 - (potReading - 768);
  }

  Serial.print("potentiometer RGB values: ");
  Serial.print(redValue);
  Serial.print(",");
  Serial.print(greenValue);
  Serial.print(",");
  Serial.println(blueValue);

  setColor(redValue, greenValue, blueValue, 1.0);
}

void readMicrophone(bool enabled) {
  if(!enabled) {
    return;
  }

  // Set min and max reading during the last timing window
  unsigned long curMillis = millis();
  if(curMillis - _lastMicrophoneRead < MIC_SAMPLE_WINDOW) {
    int soundLevel = analogRead(INPUT_MIC_PIN);
    if(soundLevel < 1024) {
      if(soundLevel > _micSignalMax) {
        _micSignalMax = soundLevel;
      }
      if(soundLevel < _micSignalMin) {
        _micSignalMin = soundLevel;
      }
    }
  } else {
    // Gather readings from current timing window
    unsigned int peakToPeak = _micSignalMax - _micSignalMin;
    double volts = (peakToPeak * 5.0) / 1024;
    Serial.print("mic voltage: ");
    Serial.println(volts);
    // Constrain brightness to just 0.0-1.0V of the reading so light is more sensitive at lower volumes
    double brightness = constrain(volts, 0, 1.0);
    setColor(255, 255, 255, brightness);

    // Reset state for next timing window
    _lastMicrophoneRead = curMillis;
    _micSignalMax = 0;
    _micSignalMin = 1024;
  }
}

void readButtonMode() {
  // Assume a pull-down resistor button configuration so button will be HIGH when pressed and LOW when not pressed
  int rawButtonVal = digitalRead(INPUT_BUTTON_PIN);

  // Grab timestamp of button state change
  if(rawButtonVal != _prevRawButtonVal) {
    _buttonStateChangeTimestamp = millis();
  }

  // Steady state if button hasn't changed during debounce window
  if((millis() - _buttonStateChangeTimestamp) >= DEBOUNCE_WINDOW) {
    // Transition mode during button release
    if(rawButtonVal == HIGH && _debouncedButtonVal == LOW) {
      _buttonMode = (_buttonMode + 1) % 4;
      Serial.print("_buttonMode: ");
      Serial.println(_buttonMode);

      // No serial input yet, so zero LED
      if(_buttonMode == 3) {
        setColor(0,0,0,0.0);
      }
    }

    _debouncedButtonVal = rawButtonVal;
  }
  _prevRawButtonVal = rawButtonVal;
}

void crossfadeLed(bool enabled) {
  if(!enabled) {
    return;
  }

  int curTime = millis();
  if(curTime - lastCrossfadeTime < 20) {
    return;
  }

  lastCrossfadeTime = curTime;

  // Increment and decrement the RGB LED values for the current
  // fade up color and the current fade down color
  _rgbLedValues[_curFadingUpColor] += FADE_STEP;
  _rgbLedValues[_curFadingDownColor] -= FADE_STEP;

  // Check to see if we've reached our maximum color value for fading up
  // If so, go to the next fade up color (we go from RED to GREEN to BLUE
  // as specified by the RGB enum)
  // This fade code partially based on: https://gist.github.com/jamesotron/766994
  if (_rgbLedValues[_curFadingUpColor] > MAX_COLOR_VALUE) {
    _rgbLedValues[_curFadingUpColor] = MAX_COLOR_VALUE;
    _curFadingUpColor = (RGB)((int)_curFadingUpColor + 1);

    if (_curFadingUpColor > (int)BLUE) {
      _curFadingUpColor = RED;
    }
  }

  // Check to see if the current LED we are fading down has gotten to zero
  // If so, select the next LED to start fading down (again, we go from RED to
  // GREEN to BLUE as specified by the RGB enum)
  if (_rgbLedValues[_curFadingDownColor] < 0) {
    _rgbLedValues[_curFadingDownColor] = 0;
    _curFadingDownColor = (RGB)((int)_curFadingDownColor + 1);

    if (_curFadingDownColor > (int)BLUE) {
      _curFadingDownColor = RED;
    }
  }

  // Set the color and brightness
  int photoresistorVal = analogRead(INPUT_PHOTORESISTOR_PIN); // read in photoresistor val
  double brightnessLevel = static_cast<double>(photoresistorVal) / 1023.0;
  Serial.print("brightnessLevel: ");
  Serial.println(brightnessLevel);
  setColor(_rgbLedValues[RED], _rgbLedValues[GREEN], _rgbLedValues[BLUE], brightnessLevel);
}

/**
 * setColor takes in values between 0 - 255 for the amount of red, green, and blue, respectively
 * where 255 is the maximum amount of that color and 0 is none of that color. You can illuminate
 * all colors by intermixing different combinations of red, green, and blue
 * 
 * This function is based on https://gist.github.com/jamesotron/766994
 */
void setColor(int red, int green, int blue, double brightness) {

  // If a common anode LED, invert values
  if (COMMON_ANODE == true) {
    red = MAX_COLOR_VALUE - (int)(red * brightness);
    green = MAX_COLOR_VALUE - (int)(green * brightness);
    blue = MAX_COLOR_VALUE - (int)(blue * brightness);
  }
  analogWrite(RGB_RED_PIN, red);
  analogWrite(RGB_GREEN_PIN, green);
  analogWrite(RGB_BLUE_PIN, blue);
}
