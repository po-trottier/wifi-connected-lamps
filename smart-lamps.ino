#include <ArduinoJson.h>
#include <neopixel.h>

/////////////////////////////////////////////////////////
// START CONFIGURATION
/////////////////////////////////////////////////////////

// Debug Variables
// Change this variable to 'true' to see debug events 
#define DEBUG_LOG false

// Configuration Variables
#define LAMPS_COUNT 2                     // Number of connected lamps
#define BASELINE_VARIANCE 512.0           // The higher the number the less the baseline is affected by current readings.
#define BASELINE_SENSITIVITY 16           // Integer. This is a trigger point such that values exceeding this point will not affect the baseline. Higher values make the trigger point sooner.
#define SENSITIVITY 20                    // Integer. Higher is more sensitive
#define SAMPLE_SIZE 512                   // Number of samples to take for one reading. Higher is more accurate but large values cause some latency.
#define SAMPLES_BETWEEN_PIXEL_UPDATES 32
#define LOOPS_TO_FINAL_COLOR 150

// Define Constants
#define PRE_ATTACK 0
#define ATTACK 1
#define DECAY 2
#define SUSTAIN 3
#define RELEASE1 4
#define RELEASE2 5
#define OFF 6
#define END_VALUE 0
#define TIME 1
#define PIXEL_PIN D2
#define PIXEL_COUNT 16
#define PIXEL_TYPE WS2812B
#define NO_EVENT 0
#define TOUCH_EVENT 1
#define RELEASE_EVENT 2

// If you have more than 2 lamps, make sure you change the value of the 'LAMPS_COUNT' variable
String lamps[] = {
    "XXXXXXXXXXXXXXXXXXXXXXXX",
    "YYYYYYYYYYYYYYYYYYYYYYYY",
};

const int colorDiff[2][2] = {
    {5, 20},    // min/Max if color change last color change from same Lamp
    {50, 128}   // min/Max if color change last color change from different Lamp
};

const long envelopes[6][2] = {
    // 160 is approximately 1 second
    {0, 0},      // OFF
    {255, 30},   // ATTACK
    {200, 240},  // DECAY
    {200, 1000}, // SUSTAIN
    {150, 60},   // RELEASE1
    {0, 320}     // RELEASE2
};

int sPin = D4;
int rPin = D3;

/////////////////////////////////////////////////////////
// END CONFIGURATION
/////////////////////////////////////////////////////////

// Initialize all variables
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
int currentEvent = NO_EVENT;
int initColor = 0;      // 0 to 255
int finalColor = 0;     // 0 to 255
int currentColor = 0;   // 0 to 255
int brightness = 0;     // 0 to 255
int initBrightness = 0; // 0 to 255
int brightnessDistanceToNextState = 0;
int colorChangeToNextState = 0;
int touchEvent = 0;
long loopCount = 0;
long colorLoopCount = 0;
long touchReading = 0;
float touchBaseline = 0;
double touchDelay = 0;
double touchBaselineExternal = 0;
uint8_t updateServer = NO_EVENT;
uint32_t colorAndBrightness = 0;
unsigned char state = OFF;
unsigned char prevState = OFF;
unsigned long timeStampTouch;
volatile unsigned long timeStampRelease;
String myID = "0";
String deviceId = "0";
String lastDeviceId = "0";

// Setup the device
void setup()
{
  Particle.subscribe("SEND", receiveEvent);
  if (DEBUG_LOG)
    Particle.publish("DEBUG_STARTED", Particle.deviceID(), PRIVATE);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  pinMode(sPin, OUTPUT);
  attachInterrupt(rPin, touchSense, RISING);
  for (int i = 0; i < LAMPS_COUNT; i++)
  {
    if (!lamps[i].compareTo(Particle.deviceID()))
    {
      myID = lamps[i];
      break;
    }
    finalColor = random(256);
  }

  for (byte i = 0; i < 1; i++)
  {
    for (byte j = 0; j < strip.numPixels(); j++)
    {
      strip.setPixelColor(j, 255, 255, 255);
    }
    strip.show();
    delay(250);
    for (byte j = 0; j < strip.numPixels(); j++)
    {
      strip.setPixelColor(j, 0, 0, 0);
    }
    strip.show();
    delay(250);
  }

  // Calibrate touch sensor- Keep hands off!!!
  touchBaseline = touchSampling(); // initialize to first reading

  for (int i = 0; i < 256; i++)
  {
    uint32_t color = wheelColor(i, 255);
    for (byte j = 0; j < strip.numPixels(); j++)
    {
      strip.setPixelColor(j, color);
      strip.show();
    }
    delay(1);
  }

  for (int j = 255; j >= 0; j--)
  {
    uint32_t color = wheelColor(255, j);
    for (byte k = 0; k < strip.numPixels(); k++)
    {
      strip.setPixelColor(k, color);
      strip.show();
    }
    delay(1);
  }
}

// Listen to events
void loop()
{
  touchEvent = touchEventCheck();
  if (touchEvent == TOUCH_EVENT)
  {
    currentEvent = touchEvent;
    state = PRE_ATTACK;
    String json = "{\"id\":\"" + Particle.deviceID() + "\",\"color\":" + String(finalColor) + ",\"event\":" + String(RELEASE_EVENT) + "}";
    Particle.publish("SEND", json, PUBLIC);
  }
  else if (touchEvent == RELEASE_EVENT)
  {
    currentEvent = touchEvent;
    String json = "{\"id\":\"" + Particle.deviceID() + "\",\"color\":" + String(finalColor) + ",\"event\":" + String(TOUCH_EVENT) + "}";
    Particle.publish("SEND", json, PUBLIC);
  }
}

// Helper Methods
void touchSense()
{
  timeStampRelease = micros();
}

long touchSampling()
{
  long tDelay = 0;
  int mSample = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++)
  {
    if (!(i % SAMPLES_BETWEEN_PIXEL_UPDATES))
    {
      stateAndPixelMagic();
    }
    pinMode(rPin, OUTPUT); // discharge capacitance at rPin
    digitalWrite(sPin, LOW);
    digitalWrite(rPin, LOW);
    pinMode(rPin, INPUT); // revert to high impedance input
    // timestamp & transition sPin to HIGH and wait for interrupt in a read loop
    timeStampTouch = micros();
    timeStampRelease = timeStampTouch;
    digitalWrite(sPin, HIGH);
    do
    {
      // wait for transition
    } while (digitalRead(rPin) == LOW);
    // accumulate the RC delay samples
    // ignore readings when micros() overflows
    if (timeStampRelease > timeStampTouch)
    {
      tDelay = tDelay + (timeStampRelease - timeStampTouch);
      mSample++;
    }
  }
  // calculate average RC delay [usec]
  if ((tDelay > 0) && (mSample > 0))
  {
    tDelay = tDelay / mSample;
  }
  else
  {
    tDelay = 0; // this is an error condition!
  }
  //autocalibration using exponential moving average on data below specified point
  if (tDelay < (touchBaseline + touchBaseline / BASELINE_SENSITIVITY))
  {
    touchBaseline = touchBaseline + (tDelay - touchBaseline) / BASELINE_VARIANCE;
  }
  return tDelay;
}

int touchEventCheck()
{
  int touchSense;                  // current reading
  static int touchSenseLast = LOW; // last reading

  static unsigned long touchDebounceTimeLast = 0; // debounce timer
  int touchDebounceTime = 50;                     // debounce time

  static int touchNow = LOW;  // current debounced state
  static int touchLast = LOW; // last debounced state

  int tEvent = NO_EVENT; // default event

  // read touch sensor
  touchReading = touchSampling();

  // touch sensor is HIGH if trigger point some threshold above Baseline
  if (touchReading > (touchBaseline + touchBaseline / SENSITIVITY))
  {
    touchSense = HIGH;
  }
  else
  {
    touchSense = LOW;
  }

  // debounce touch sensor
  // if state changed then reset debounce timer
  if (touchSense != touchSenseLast)
  {
    touchDebounceTimeLast = millis();
  }
  touchSenseLast = touchSense;

  // accept as a stable sensor reading if the debounce time is exceeded without reset
  if (millis() > touchDebounceTimeLast + touchDebounceTime)
  {
    touchNow = touchSense;
  }

  // set events based on transitions between readings
  if (!touchLast && touchNow)
  {
    tEvent = TOUCH_EVENT;
  }

  if (touchLast && !touchNow)
  {
    tEvent = RELEASE_EVENT;
  }

  // update last reading
  touchLast = touchNow;
  return tEvent;
}

uint32_t wheelColor(byte WheelPos, byte iBrightness)
{
  float R, G, B;
  float brightness = iBrightness / 255.0;

  if (WheelPos < 85)
  {
    R = WheelPos * 3;
    G = 255 - WheelPos * 3;
    B = 0;
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    R = 255 - WheelPos * 3;
    G = 0;
    B = WheelPos * 3;
  }
  else
  {
    WheelPos -= 170;
    R = 0;
    G = WheelPos * 3;
    B = 255 - WheelPos * 3;
  }
  R = R * brightness + .5;
  G = G * brightness + .5;
  B = B * brightness + .5;
  return strip.Color((byte)R, (byte)G, (byte)B);
}

void updateNeoPixels(uint32_t color)
{
  for (char i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void generateColor()
{
  int newColor = finalColor;
  if (prevState == OFF)
  {
    finalColor = newColor = currentColor = random(256);
  }
  else
  {
    bool diffId = (lastDeviceId != myID);
    finalColor += (random(2) * 2 - 1) * (colorDiff[diffId][0] + random(colorDiff[diffId][1] - colorDiff[diffId][0] + 1));
    finalColor = (finalColor + 256) % 256;
  }
  colorChangeToNextState = finalColor - currentColor;
  colorChangeToNextState += ((colorChangeToNextState < 0) * 2 - 1) * (abs(colorChangeToNextState) > 127) * 256;
  initColor = currentColor;
  lastDeviceId = myID;
}

void getColorFromServer(int color)
{
  finalColor = color;
  colorChangeToNextState = finalColor - currentColor;
  colorChangeToNextState += ((colorChangeToNextState < 0) * 2 - 1) * (abs(colorChangeToNextState) > 127) * 256;
  initColor = currentColor;
  if (DEBUG_LOG)
  {
    Particle.publish("DEBUG_GET_COLOR", finalColor + ", " + colorChangeToNextState, PRIVATE);
  }
  lastDeviceId = deviceId;
  colorLoopCount = 0;
}

void changeState(unsigned char newState)
{
  prevState = state;
  state = newState;
  loopCount = 0;
  initBrightness = brightness;
  brightnessDistanceToNextState = envelopes[newState][END_VALUE] - brightness;
}

void updatePixelSettings()
{
  brightness = min(255, max(0, initBrightness + loopCount * brightnessDistanceToNextState / envelopes[state][TIME]));
  if (colorLoopCount > LOOPS_TO_FINAL_COLOR)
  {
    currentColor = finalColor;
  }
  if (currentColor != finalColor)
  {
    currentColor = (initColor + 256 + colorLoopCount * colorChangeToNextState / LOOPS_TO_FINAL_COLOR) % 256;
  }
}

void stateAndPixelMagic()
{
  switch (state)
  {
  case PRE_ATTACK:
    generateColor();
    updateServer = TOUCH_EVENT;
    changeState(ATTACK);
    colorLoopCount = 0;
    break;
  case ATTACK:
    updatePixelSettings();
    currentEvent = NO_EVENT;
    if (loopCount >= envelopes[ATTACK][TIME])
    {
      changeState(DECAY);
    }
    break;
  case DECAY:
    updatePixelSettings();
    if ((loopCount >= envelopes[DECAY][TIME]) || (currentEvent == RELEASE_EVENT))
    {
      changeState(SUSTAIN);
    }
    break;
  case SUSTAIN:
    updatePixelSettings();
    if ((loopCount >= envelopes[SUSTAIN][TIME]) || (currentEvent == RELEASE_EVENT))
    {
      changeState(RELEASE1);
      updateServer = RELEASE_EVENT;
      currentEvent = NO_EVENT;
    }
    break;
  case RELEASE1:
    updatePixelSettings();
    if (loopCount >= envelopes[RELEASE1][TIME])
    {
      changeState(RELEASE2);
    }
    break;
  case RELEASE2:
    updatePixelSettings();
    if (loopCount >= envelopes[RELEASE2][TIME])
    {
      changeState(OFF);
    }
    break;
  case OFF:
    brightness = 0;
    break;
  }
  colorAndBrightness = wheelColor(currentColor, brightness);
  updateNeoPixels(colorAndBrightness);
  loopCount++;
  colorLoopCount++;
}

// Event Handler
void receiveEvent(const String e, const String data)
{
  int event = NO_EVENT;
  int color = 0;
  deviceId = "0";

  // TODO Parse the data
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, data.c_str());

  if (err)
  {
    Particle.publish("ERROR", "Error Parsing the JSON Data", PRIVATE);
    return;
  }

  event = doc["event"];
  color = doc["color"];
  const char *temp = doc["id"];
  deviceId = temp;

  if (DEBUG_LOG)
  {
    Particle.publish("DEBUG_COLOR", String(color), PRIVATE);
    Particle.publish("DEBUG_ID", String(deviceId), PRIVATE);
  }

  if ((deviceId != myID))
  {
    if (updateServer)
    {
      if (DEBUG_LOG)
        Particle.publish("DEBUG_UPDATE", String(updateServer), PRIVATE);
        
      if (updateServer == TOUCH_EVENT)
      {
        if (event == TOUCH_EVENT)
        {
          lastDeviceId = deviceId;
          generateColor();
          changeState(ATTACK);
        }
        else
        {
          generateColor();
        }
      }
      else
      {
        if (event == TOUCH_EVENT)
        {
          getColorFromServer(color);
          changeState(ATTACK);
        }
        else
        {
          changeState(RELEASE1);
        }
      }
    }
    else
    {
      if (event == TOUCH_EVENT)
      {
        getColorFromServer(color);
        changeState(ATTACK);
      }
      else
      {
        changeState(RELEASE1);
      }
    }
  }
}
