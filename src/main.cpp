// LED_Touch button and optionally WiFi controlled light

// Topic structure
// <location>/light/<which_light>
// Messages on this topic:
// {state:<current_state>,intensity:<current_intensity>}
// {reset:1} perform soft reset of the esp
// {delay_off:1} initiate delay off timer
// {info:1} request to send status info
// Commands sent to <location>/light/<which_light>/command
// {state:1,intensity:255}

#include <Arduino.h>


#ifdef ARDUINO_ARCH_ESP8266
#include <ArduinoOTA.h>

#include "ArduinoJson.h"
#include "CronAlarms.h"
#include "PubSubClient.h"
#endif

#ifdef ARDUINO_AVR_DIGISPARK
#include <Print.h>
#endif

#include "PinButton.h"

// Secrets
// #define SSID "SSID"
// #define PASS "PASSW"
// #define MQTT_UNAME "MQTT_UNAME"
// #define MQTT_PW "MQTT_PASSW"
// #define OTA_PW "OTA_PASSW"
#include ".secrets.h"

// Board ID
// #define BOARDID "light_companionway"
// #define LIGHTID "companionway/light/main"
// Salon light
#define BOARDID "light_salon"
#define LIGHTID "salon/light/main"  // Topic to ID the light for MQTT control

// Pins
#define GATE_WHITE 0   // Gate pin of white light
#define BUTTON1_PIN 2  // Input pin for button 1

#ifndef ARDUINO_ARCH_ESP8266
#define BUTTON2_PIN 3  // Input pin for button 2
#endif

// Constants
#define BRIGHTNESS_STEP_DEFAULT 2  // Brighness step adjust
#ifdef ARDUINO_ARCH_ESP8266
#define STEP_DELAY 12  // Delay before next brightness step in ms
#endif
#ifdef ARDUINO_AVR_DIGISPARK
#define STEP_DELAY 16  // Delay before next brightness step in ms
#endif
#define BRIGHTNESS_MIN 50     // Minimum brightness level
#define BRIGHTNESS_MAX 255    // Maximum brightness level
#define DELAY_OFF_TIME_DEFAULT 15000  // Delay before turning off in milliseconds
#ifdef ARDUINO_ARCH_ESP8266
#define WIFI_RECONNECT_INTERVAL 20000UL  // Interval before attempting to connect again
#endif

// R = (255 * log10(2))/(log10(255)); // Calculate the R value required for
// logatithmic dimming 
// Alternatively use R = (255 * log10(2))/(log10(255 - (BRIGHTNESS_MIN / 2))); for better 
// minimun values for startup (not used here)
#define R \
  31.897513915796375228513177  // Magic!! Used for the logarhythmic LED fading
                               // formula

// MQTT command defines
#define MQTT_CMD_ERROR -1  // MQTT command not set || error
#define MQTT_CMD_ADJ 0     // MQTT command is for adjusting light
#define MQTT_CMD_CRON 1    // MQTT command for setting cron
#define MQTT_CMD_RESET 2   // MQTT command for resetting board
#define MQTT_CMD_INFO 3    // MQTT command for requesting to send current state
#define MQTT_CMD_DELAY 4   // MQTT command for delay off

// DEBUG
//#define DEBUG // DEBUGging flag

// State machine
enum lightStates {
  //             * Transition/unstable states
  OFF,        // Light is off
  MANUAL,     // Manual brightness, standard 'ON' mode
  CHANGING,   // *Increasing or decreasing in brightness
  DELAY_OFF,  // *Delay before turning off
  MAX_STATES
};

enum lightStates lightState;

#ifdef ARDUINO_ARCH_ESP8266
// MQTT
namespace mqtt {
  IPAddress serverIP(192, 168, 5, 1);
  const char *user = MQTT_UNAME;
  const char *password = MQTT_PW;
  const char *id = BOARDID;
  WiFiClient espClient;
  PubSubClient client(espClient);
  unsigned long lastReconnectAttempt{0};
  unsigned long lastLoop{0};
  bool reconnect();
  // long lastmsg { 0 };
  // char msg[50];
  // int value { 0 };
  void mqtt_cb(char *, uint8_t *, unsigned int);

  struct commands {
    bool flag = false;
    int action = MQTT_CMD_ERROR;
    int state = 2;
    int intensity = 128;
    const char *cronStr = NULL;  // min hour day-of-month month day-of-week
  } command;
}  // namespace mqtt

time_t now;  // Current timestamp
unsigned long lastWiFiConnectionAttempt { WIFI_RECONNECT_INTERVAL };

#endif

/*
Button functions
Simple state diagram:

Single click:
  OFF --> CHANGING
  MANUAL --> CHANGING (turn off)
  CHANGING --> MANUAL
Double click:
  OFF --> MANUAL (min brightness)
  MANUAL --> MANUAL (min brightness)
  CHANGING --> MANUAL (min brightness)
Long Press:
  OFF --> MANUAL (min brightness)
  MANUAL --> CHANGING
  CHANGING --> CHANGING
Release: (On double or long press only)
  OFF --> OFF
  MANUAL --> MANUAL
  CHANGING --> MANUAL
*/

PinButton touch(BUTTON1_PIN);  // Button object for detecting different presses

#ifdef ARDUINO_ARCH_ESP8266

void WiFiconnect() {
  
  if (WiFi.status() == WL_CONNECTED) {
    // Already connected
    return;
  } else if (millis() - lastWiFiConnectionAttempt < WIFI_RECONNECT_INTERVAL) {
    // Not yet ready to try again
    return;
  }
  
  // Connect to the WiFi
  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

#ifdef DEBUG
    printf("Connecting to WiFi ... \n");
#endif

  for (int i = 0; i < 10; i++)
  {
#ifdef DEBUG
    printf(".");
#endif
    if (WiFi.status() == WL_CONNECTED) {
      break;
    } else {
      delay(500);
    }
  }

  if ( WiFi.status() == WL_CONNECTED ) {
    // Connected to WiFi
    // Start all connected services
    configTime(0, 0, "au.pool.ntp.org");  // Get UTC time over NTP
    #ifdef DEBUG
    printf("Time set to: %lld\n", time(&now));
    #endif

    #ifdef DEBUG
    printf("Ready \nIP address: %s\n", WiFi.localIP().toString().c_str());
    #endif


  } else {
    // No WiFi connection could be established
    lastWiFiConnectionAttempt = millis();
    return;
  }

}

void StartOTA() {

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(BOARDID);

  // No authentication by default
  ArduinoOTA.setPassword((const char *)OTA_PW);
  // ArduinoOTA.setPassword(NULL);

  ArduinoOTA.onStart([]() {
#ifdef DEBUG
    printf("OTA Start\n");
#endif
  });
  ArduinoOTA.onEnd([]() {
#ifdef DEBUG
    printf("\nOTA End\n");
#endif
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
#ifdef DEBUG
    printf("OTA Progress: %u%%\r", (progress / (total / 100)));
#endif
  });
  ArduinoOTA.onError([](ota_error_t error) {
#ifdef DEBUG
    printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      printf("Auth Failed\n");
    else if (error == OTA_BEGIN_ERROR)
      printf("Begin Failed\n");
    else if (error == OTA_CONNECT_ERROR)
      printf("Connect Failed\n");
    else if (error == OTA_RECEIVE_ERROR)
      printf("Receive Failed\n");
    else if (error == OTA_END_ERROR)
      printf("End Failed\n");
#endif
  });
  ArduinoOTA.begin();

}


void mqtt::mqtt_cb(char *topic, byte *message, unsigned int length) {
  // Callback when message arrives on MQTT

  // Build the string form the bytestream
  String messageTemp;
  for (unsigned int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

// Print the message received
#ifdef DEBUG
  printf("Message arrived on topic: '%s'. Message: '%s'.\n", topic,
         messageTemp.c_str());
#endif

  // Message parser
  if (topic != (char *)"housekeeping") {
#ifdef DEBUG
    printf("Command received, parsing ...\n");
#endif
    // Messages to be parsed:
    // {state:<0...4],intensity:<0...255>}
    StaticJsonDocument<200> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, messageTemp);

    // Test if parsing succeeds.
    if (error) {
#ifdef DEBUG
      printf("JSON deserialization failed: %s.\n", error.c_str());
#endif
      return;
    }

    int cmd_state = lightStates::MAX_STATES;
    int cmd_intensity = -1;

    if (doc["reset"] == 1) {
      // Reset command given
      mqtt::command.action = MQTT_CMD_RESET;
    } else if (doc["info"] == 1) {
      mqtt::command.action = MQTT_CMD_INFO;
    } else if (doc["delay_off"] == 1) {
      mqtt::command.action = MQTT_CMD_DELAY;
    } else {
      cmd_state = doc["state"];
      cmd_intensity = doc["intensity"];
      mqtt::command.action = MQTT_CMD_ADJ;
      // if (doc["cronStr"] != "\0") {
      //   mqtt::command.action = MQTT_CMD_CRON;
      //   mqtt::command.cronStr = doc["cronStr"];
      // }
    }

    // Check validity of values passed
    if (cmd_state < 0 || cmd_state >= MAX_STATES) {
// Invalid state
#ifdef DEBUG
      printf("Error: state out of bounds (%d)\n", cmd_state);
#endif
      mqtt::command.state = 0;
      // return;
    } else {
      mqtt::command.state = cmd_state;
    }
    if (cmd_intensity < 0) {
// Invalid intensity
#ifdef DEBUG
      printf("Error: intensity out of bounds (%d)\n", cmd_intensity);
#endif
      mqtt::command.intensity = 0;
      // return;
    } else if (cmd_intensity > 255) {
      mqtt::command.intensity = 255;
    } else {
      mqtt::command.intensity = cmd_intensity;
    }

    // Set flag for new command received
    mqtt::command.flag = true;
  }

  return;
}

bool mqtt::reconnect() {
  if (mqtt::client.connect(mqtt::id, mqtt::user, mqtt::password)) {
// Once connected, publish an announcement to the housekeeing topic
#ifdef DEBUG
    printf("Connected to MQTT broker.\n");
#endif

    mqtt::client.publish("housekeeping", BOARDID);
    // And (re)subscribe
    mqtt::client.subscribe("housekeeping");
    mqtt::client.subscribe(((String)LIGHTID + (String) "/command").c_str());
  } else {
#ifdef DEBUG
    printf("Failed to connect to MQTT broker. \n");
#endif
  }
  return mqtt::client.connected();
}
#endif

int changeState(lightStates newState) {
  // Change the state
  // TODO: Make lightState not global!
  lightState = newState;

// Publish new state to MQTT
#ifdef ARDUINO_ARCH_ESP8266
  String payload = String("{\"state\":");
  payload += String(newState);
  payload += String("}");
  mqtt::client.publish(LIGHTID, payload.c_str(), true);
#endif

// Debugging info
#ifdef DEBUG
  printf("State chage: %d \n", newState);
#endif

  return newState;
}

int changeState(int newState) {
  changeState((lightStates)newState);
  return newState;
}

void setTarget(int target, int step, int &brightnessCurrent,
               int &brightnessTarget, int &brightnessStep) {
  // This sets the target intensity value for the light
  // and the step value of how fast to get there.
  // Automatically calculates direction and validates input.
  // int target: 0-255
  // int step: 0-255
  if (target > BRIGHTNESS_MAX) {
    target = BRIGHTNESS_MAX;
  } else if (target <= 0) {
    // Target of 0 is valid because this will turn off the light
    target = 0;
  } else if (target < BRIGHTNESS_MIN) {
    // Do not allow to go under minimum brightness
    target = BRIGHTNESS_MIN;
  }

  if (step > 255 || step < 0) {
    step = BRIGHTNESS_STEP_DEFAULT;
  }

  if (brightnessCurrent == target) {
    step = BRIGHTNESS_STEP_DEFAULT;
  } else if (brightnessCurrent > target) {
    // Brightness needs to go down, use negative stepping
    step *= -1;
  }

  brightnessTarget = target;
  brightnessStep = step;

#ifdef DEBUG
  printf("Target set: %d. BrightnessStep: %d \n", brightnessTarget,
         brightnessStep);
#endif
}

bool stepIntensity(unsigned long &lastIntensityChange, int &brightnessCurrent,
                   int &brightnessStep, int &brightnessTarget,
                   unsigned long &delayTime) {
  // Steps the intensity of the light towards the target with one step
  // Returns true if change is done (target is reached)
  // Otherwise returns false
  if (millis() - lastIntensityChange < STEP_DELAY) {
    return false;
  }
  // Check some issues with the delay off when lastIntensityChange is set in the future
  if (lastIntensityChange - delayTime > millis()) {
    lastIntensityChange = millis(); 
    return false;
  } else if (lastIntensityChange > millis()) {
    return false;
  }

  bool changeDone = false;
  int adjustedBrightness = 0;

  brightnessCurrent +=
      brightnessStep;  // Add the step to the current brightness

  // Check is result is valid
  if (brightnessCurrent > brightnessTarget && brightnessStep > 0) {
    brightnessCurrent = brightnessTarget;
    changeDone = true;
  } else if (brightnessCurrent < brightnessTarget && brightnessStep < 0) {
    brightnessCurrent = brightnessTarget;
    changeDone = true;
  }

  // Adjust the make the dimming logarithmic
  adjustedBrightness = pow(2, (brightnessCurrent / R)) - 1;  // Original
  // adjustedBrightness = pow ( 2, ( brightnessCurrent / R ) ) - 1 + (
  // BRIGHTNESS_MIN / 2 ); // Adjusted for better minimum values at startup

  analogWrite(GATE_WHITE, adjustedBrightness);
  lastIntensityChange = millis();

  // Publish new state to MQTT
#ifdef ARDUINO_ARCH_ESP8266
  String payload = String("{\"intensity\":");
  payload += String(brightnessCurrent);
  payload += String("}");
  mqtt::client.publish(LIGHTID, payload.c_str(), true);
#endif

  return changeDone;
}

void setup() {
#ifdef ARDUINO_AVR_DIGISPARK
  // Set the timer prescaler
  // This sets the ATtiny85 PWM register to change PWM speed
  // TCCR0B = ((TCCR0B) & 0b11111000) | 0b001 ; // no prescaling (PWM ~32 kHz
  // (measured)) TCCR0B = ((TCCR0B) & 0b11111000) | 0b010 ; // clk/8 from
  // prescaler (PWM ~4.069 kHz (measured))
  TCCR0B = ((TCCR0B)&0b11111000) |
           0b011;  // clk/64 from prescaler (PWM ~500 Hz (measured))
// TCCR0B = ((TCCR0B) & 0b11111000) | 0b100 ; // clk/256 from prescaler (PWM ~60
// Hz) TCCR0B = ((TCCR0B) & 0b11111000) | 0b101 ; // clk/1024 from prescaler
// (PWM ~7.5 Hz)
#endif

#ifdef DEBUG
  Serial.begin(115200);
  printf("Welcome to light control. Board: %s \n", BOARDID);
#endif

  analogWrite(GATE_WHITE, 0);  // Set PWM signel to 0
  changeState(OFF);            // Set initial state

// ESP8266 setup
#ifdef ARDUINO_ARCH_ESP8266
  // Set the PWM frequency
  // analogWriteRange(4096);
  analogWriteFreq(200);

  WiFiconnect();
  StartOTA();

  // MQTT init
  // ---------
  mqtt::client.setServer(mqtt::serverIP, 1883);  // Standard port 1883
  mqtt::client.setCallback(mqtt::mqtt_cb);

#endif  // For ESP8266
}

#ifdef ARDUINO_ARCH_ESP8266
OnTick_t cronCbOn(int &brightnessTargetCron, int &brightnessCurrent,
                  int &brightnessTarget, int &brightnessStep) {
  // Turn off the light
  setTarget(brightnessTargetCron, BRIGHTNESS_STEP_DEFAULT, brightnessCurrent,
            brightnessTarget, brightnessStep);
  return 0;
}

OnTick_t cronCbOff(int &brightnessCurrent, int &brightnessTarget,
                   int &brightnessStep) {
  // Turn on the light
  setTarget(0, BRIGHTNESS_STEP_DEFAULT, brightnessCurrent, brightnessTarget,
            brightnessStep);
  return 0;
}
#endif

void loop() {
  // Variables
  static int brightnessCurrent = BRIGHTNESS_MIN;  // Current brightness
  static int brightnessTarget = 128;  // Brightness target to be reached
  static int brightnessStep =
      BRIGHTNESS_STEP_DEFAULT;      // Step rate for brightness change, can be
                                    // negative
  static int lastBrightness = 128;  // Last known brightness before turning off
  static unsigned long delayTime = DELAY_OFF_TIME_DEFAULT;

  static unsigned long lastIntensityChange =
      0;  // For governing intensity changes

  static bool longclickActive = false;  // To identify if a long click is active
  static unsigned long longClickStartTime = 0UL;
  static bool disableTouchSensor =
      false;  // Flag to disable the touch input sensor

#ifdef ARDUINO_ARCH_ESP8266
  static int brightnessTargetCron = 128;  // Intenstiy for cron turning on

  static CronId cronOn;
  static CronId cronOff;
#endif

#ifdef ARDUINO_ARCH_ESP8266
  // WiFi connection check
  WiFiconnect();
  // OTA check
  ArduinoOTA.handle();

  String payload = "";

  // Keep connected to MQTT broker
  if (WiFi.status() == WL_CONNECTED) {
    // WiFi connection available, try connecting to MQTT
    long tnow = millis();
    if (!mqtt::client.connected()) {
      // Not connected, try to connect now
      if (tnow - mqtt::lastReconnectAttempt > 10000UL) {
        mqtt::lastReconnectAttempt = tnow;
        // Attempt to reconnect
        mqtt::reconnect();
      }
    } else {
      // Client connected
      // TODO: Time this event to only go at set intervals
      if ( tnow - mqtt::lastLoop > 50UL ) {
        mqtt::lastLoop = tnow;
        mqtt::client.loop();
      }
    }
  }

  // Check the mqtt flag (set when new commands have arrived)
  if (mqtt::command.flag) {
    // New command has been received
    switch (mqtt::command.action) {
      case MQTT_CMD_ADJ:
        if (lightState == DELAY_OFF) {
          lastIntensityChange = millis() - STEP_DELAY - 1;  // Nullify the delay
        }
        setTarget(mqtt::command.intensity, BRIGHTNESS_STEP_DEFAULT,
                  brightnessCurrent, brightnessTarget, brightnessStep);
        changeState(mqtt::command.state);
        break;
      case MQTT_CMD_CRON:
        if (mqtt::command.intensity == 0) {
          // use Cron.free(id) to disable a timer and recycle its memory.
          Cron.free(cronOff);
          cronOff = Cron.create(
              (char *)mqtt::command.cronStr,
              cronCbOff(brightnessCurrent, brightnessTarget, brightnessStep),
              false);
        } else {
          // disable timer
          Cron.free(cronOn);
          brightnessTargetCron = mqtt::command.intensity;
          cronOn = Cron.create((char *)mqtt::command.cronStr,
                               cronCbOn(brightnessTargetCron, brightnessCurrent,
                                        brightnessTarget, brightnessStep),
                               false);
        }
        break;
      case MQTT_CMD_RESET:
// Restart the ESP
#ifdef DEBUG
        printf("Resettig ESP ...");
#endif
        ESP.restart();
        break;
      case MQTT_CMD_INFO:
        // Send the current state over MQTT
        payload += String("{\"state\":");
        payload += String(lightState);
        payload += String(",\"intensity\":");
        payload += String(brightnessCurrent);
        payload += String("}");
        mqtt::client.publish(LIGHTID, payload.c_str(), true);
        break;
      case MQTT_CMD_DELAY:
        if (lightState == MANUAL) {
          // --> Turn off light with delay
          lastBrightness = brightnessCurrent;

          /*
          // First set the brightness down a bit to show its about to turn off
          brightnessCurrent = brightnessCurrent - 32 < BRIGHTNESS_MIN
                                  ? BRIGHTNESS_MIN
                                  : brightnessCurrent - 32;
          stepIntensity(lastIntensityChange, brightnessCurrent, brightnessStep,
                        brightnessTarget);  // Apply the brightness change
          */

          // Delay for next intensity change
          lastIntensityChange = millis() + delayTime;

          // Target is off
          setTarget(0, BRIGHTNESS_STEP_DEFAULT, brightnessCurrent,
                    brightnessTarget, brightnessStep);
          changeState(DELAY_OFF);
        }
        break;
      default:
        break;
    }
    // Reset the flag
    mqtt::command.flag = false;
  }

  Cron.delay();  // Call in loop for hanlding cron
#endif

  touch.update();  // Make sure to check for button press at least every 30ms

  if (disableTouchSensor) {
    // The touch sensor is disabled
    // This is built in to avoid hanging up of the touch sensor if a fly is on there or something :)
    // Also happened a few times during an electric storm
    // Check if it's still long clicking
    if (touch.isLongClick() && longclickActive) {
      goto execute;
    } else {
      // Re-enable the touch sensor
      longclickActive = false;
      disableTouchSensor = false;
      longClickStartTime = 0UL;
    }
  }

  if (touch.isSingleClick()) {
// Single click
#ifdef DEBUG
    printf("--- Single Click! ---\n");
#endif

    switch (lightState) {
      case OFF:
        // --> CHANGING (target = default)
        if (lastBrightness < BRIGHTNESS_MIN) {
          lastBrightness = BRIGHTNESS_MIN;
        }
        // Start with the minimum brightness minus one step
        brightnessCurrent = BRIGHTNESS_MIN - BRIGHTNESS_STEP_DEFAULT;
        // Apply the brightness change immediately
        stepIntensity(lastIntensityChange, brightnessCurrent, brightnessStep,
                      brightnessTarget, delayTime);
        // Set the target
        setTarget(lastBrightness, BRIGHTNESS_STEP_DEFAULT, brightnessCurrent,
                  brightnessTarget, brightnessStep);
        changeState(CHANGING);
        break;
      case MANUAL:
        // --> Turn off light
        lastBrightness = brightnessCurrent;

        // First set the brightness down a bit to show its about to turn off
        brightnessCurrent = brightnessCurrent - 32 < BRIGHTNESS_MIN
                                ? BRIGHTNESS_MIN
                                : brightnessCurrent - 32;
        stepIntensity(lastIntensityChange, brightnessCurrent, brightnessStep,
                      brightnessTarget, delayTime);  // Apply the brightness change

        // Delay for next intensity change
        lastIntensityChange = millis() + delayTime;

        // Target is off
        setTarget(0, BRIGHTNESS_STEP_DEFAULT, brightnessCurrent,
                  brightnessTarget, brightnessStep);
        changeState(DELAY_OFF);
        break;
      case DELAY_OFF:
        // Turn light out immediately
        setTarget(0, 255, brightnessCurrent, brightnessTarget, brightnessStep);
        lastIntensityChange = millis() - STEP_DELAY - 1;  // Nullify the delay
        changeState(CHANGING);
        break;
      case CHANGING:
        // --> MANUAL
        if (brightnessCurrent < BRIGHTNESS_MIN) {
          // Light is already going out, turn off completely
          setTarget(0, 255, brightnessCurrent, brightnessTarget,
                    brightnessStep);
          changeState(CHANGING);
        } else {
          // Stop changing intensity and set state to manual
          setTarget(brightnessCurrent, BRIGHTNESS_STEP_DEFAULT,
                    brightnessCurrent, brightnessTarget, brightnessStep);
          changeState(MANUAL);
        }
        break;
      case MAX_STATES:
        // max states reached
        break;
    }
  }

  if (touch.isDoubleClick()) {
// Double click
#ifdef DEBUG
    printf("--- Double Click! ---\n");
#endif

    switch (lightState) {
      case OFF:
      case MANUAL:
      case CHANGING:
        // --> CHANGING (min brightness)
        // Double click always set the light to min brightness, even when off.
        setTarget(BRIGHTNESS_MIN, 9, brightnessCurrent, brightnessTarget,
                  brightnessStep);
        changeState(CHANGING);
        break;
      case DELAY_OFF:
        // Turn light out immediately
        setTarget(0, 255, brightnessCurrent, brightnessTarget, brightnessStep);
        lastIntensityChange = millis() - STEP_DELAY - 1;  // Nullify the delay
        changeState(CHANGING);
        break;
      case MAX_STATES:
        // Max
        break;
    }
  }

  if (touch.isLongClick() || longclickActive == true) {
    // Long click starts the increasing routine

#ifdef DEBUG
    printf("--- Long Click! ---\n");
#endif

    // The problem with long click:
    // If the touch sensor goes rogue it will keep long clicking for a long
    // time. Here we try to detect that and disable the touch sensor temporarily
    // until the sensor has regained itself.
    // The touch senser can go rogue for all sorts of reasons and most of them
    // are temporary so we will enable the touch senser automatically again if
    // the sensor is behaving again.
    // Possible reasons:
    // - Flies/insects are activating the sensor
    // - Atmospheric conditions, i.e. electrical storms
    // - Water/Moisture
    // - Bad connection
    // - Etc., etc.
    if (longclickActive == false) {
      longClickStartTime = millis();
    }
    if (millis() - longClickStartTime > 15e3) {
      // Longpress is activated for over 15 seconds
      disableTouchSensor = true;
      goto execute;
    }

    longclickActive = true;

    switch (lightState) {
      case OFF:
        // --> MANUAL (min brightness)
        setTarget(BRIGHTNESS_MIN, BRIGHTNESS_MIN, brightnessCurrent,
                  brightnessTarget, brightnessStep);
        changeState(CHANGING);
        break;
      case MANUAL:
      case CHANGING:
        // --> CHANGING
        // Set new target a little further then now
        if (brightnessCurrent >= BRIGHTNESS_MAX) {
          // Maximum brightness reached, do nothing
          break;
        }
        setTarget(brightnessCurrent + BRIGHTNESS_STEP_DEFAULT,
                  BRIGHTNESS_STEP_DEFAULT, brightnessCurrent, brightnessTarget,
                  brightnessStep);
        changeState(CHANGING);
        break;
      case DELAY_OFF:
        // Turn light back on again
        lastIntensityChange = millis() - STEP_DELAY - 1;  // Nullify the delay
        changeState(CHANGING);
        break;
      case MAX_STATES:
        // Max
        break;
    }
  }

  if (touch.isReleased()) {
// Long or double click released
#ifdef DEBUG
    printf("--- Released! ---\n");
#endif

    longclickActive = false;
  }

execute:
  // Execute current state
  if (lightState == CHANGING || lightState == DELAY_OFF) {
    // Brightness is 0 and the target is set lower then the min brightness then
    // switch to OFF
    if (brightnessCurrent <= 0 && brightnessTarget < BRIGHTNESS_MIN) {
      setTarget(0, 255, brightnessCurrent, brightnessTarget, brightnessStep);
      stepIntensity(lastIntensityChange, brightnessCurrent, brightnessStep,
                    brightnessTarget, delayTime);
      changeState(OFF);

#ifdef DEBUG
      printf("Switch off target lower then min brightness\n");
#endif
    } else {
      // Execute the change required
      if (stepIntensity(lastIntensityChange, brightnessCurrent, brightnessStep,
                        brightnessTarget, delayTime) == true) {
        // Target reached, set state to manual
        changeState(MANUAL);
      }
#ifdef DEBUG
      printf("Execute change in brightness\n");
#endif
    }
#ifdef DEBUG
    printf("brightnessCurrent: %d\n", brightnessCurrent);
#endif
  }

  if (lightState == MANUAL && brightnessCurrent <= 0) {
    brightnessCurrent = 0;
    changeState(OFF);
#ifdef DEBUG
    printf("Cutoff at 0 brightness!\n");
#endif
  }
}