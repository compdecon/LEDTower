/*
I asked ChatGPT 4, Bard and Hugging Face to create this. This is mostly from
ChatGPT 4. I've made a few minor changes but will make more.

As the end user I want the R/Y/G/B LED Tower to be used to indicate information about on going events. I want the LEDS to be able to slow flash on and off or fast flash on and off or fade from off to on back to on or 

- ESP32
  - GPIO 4 - Red
  - GPIO 5 - Yellow
  - GPIO 6 - Green
  - GPIO 7 - Blue
- Arduino enviroment
- MQTT
  - accept command strings from the device/cmd topic
    - coomands are in the format of "color cmd"
      - where command is:
        - on
        - off
        - fflash (fast flash)
        - sflash (slow flash)
      - where color is
        - red
        - green
        - yellow
        - blue
      - addition commands:
        - all off
        - all on
      - commands can be issued asynchronously
  - states can be monitored on device/state
- Flash fast
- Flash slow
- Fade on and off
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "myWiFi.h"
#include "myMQTT.h"

#ifndef ssid
// WiFi credentials.
const char* ssid     = "your_SSID";
const char* password = "your_PASSWORD";
#endif

#ifndef mqtt_server
// MQTT details
const char* mqtt_server   = "your_MQTT_SERVER";
const char* command_topic = "device/cmd";
const char* state_topic   = "device/state";
#ednif

WiFiClient espClient;
PubSubClient client(espClient);

// LED GPIOs
int redPin = 4;
int yellowPin = 5;
int greenPin = 6;
int bluePin = 7;

// Global state of LEDs
DynamicJsonDocument doc(1024);
JsonObject ledState = doc.to<JsonObject>();

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  ledState["red"] = "off";
  ledState["yellow"] = "off";
  ledState["green"] = "off";
  ledState["blue"] = "off";
}

void setup_wifi() {
  delay(10);
  // Connect to WiFi network
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect() {
  // Loop until reconnected
  while (!client.connected()) {
    if (client.connect("ESP32Client")) {
      client.subscribe(command_topic);
    } else {
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  processCommand(message);
}

void processCommand(String command) {
  // Split command into parts
  int    index  = command.indexOf(' ');
  String led    = command.substring(0, index);
  String action = command.substring(index + 1);

  // Determine LED pin
  int ledPin = 0;
  if (led == "red") {
    ledPin = redPin;
  } else if (led == "yellow") {
    ledPin = yellowPin;
  } else if (led == "green") {
    ledPin = greenPin;
  } else if (led == "blue") {
    ledPin = bluePin;
  } else if (led == "all") {
    // Process 'all' command separately
    processAllCommand(action);
    return;
  }

  // Determine action
  if (action == "on") {
    digitalWrite(ledPin, HIGH);
    ledState[led] = "on";  // update state
  } else if (action == "off") {
    digitalWrite(ledPin, LOW);
    ledState[led] = "off";  // update state
  } else if (action == "fflash") {
    // Fast flash
    for(int i = 0; i < 10; i++) { // flash for 10 times
      digitalWrite(ledPin, HIGH);
      delay(100); // 100 ms delay for fast flash
      digitalWrite(ledPin, LOW);
      delay(100); // 100 ms delay for fast flash
    }
    ledState[led] = "fflash";  // update state
  } else if (action == "sflash") {
    // Slow flash
    for(int i = 0; i < 5; i++) { // flash for 5 times
      digitalWrite(ledPin, HIGH);
      delay(500); // 500 ms delay for slow flash
      digitalWrite(ledPin, LOW);
      delay(500); // 500 ms delay for slow flash
    }
    ledState[led] = "sflash";  // update state
  } 
  publishState();  // publish updated state
}

void processAllCommand(String action) {
  // Process 'all' command
  if (action == "on") {
    digitalWrite(redPin, HIGH);
    digitalWrite(yellowPin, HIGH);
    digitalWrite(greenPin, HIGH);
    digitalWrite(bluePin, HIGH);

    ledState["red"] = "on";  // update state
    ledState["yellow"] = "on";  // update state
    ledState["green"] = "on";  // update state
    ledState["blue"] = "on";  // update state
  } else if (action == "off") {
    digitalWrite(redPin, LOW);
    digitalWrite(yellowPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);

    ledState["red"] = "off";  // update state
    ledState["yellow"] = "off";  // update state
    ledState["green"] = "off";  // update state
    ledState["blue"] = "off";  // update state
  }
  publishState();  // publish updated state
}

void publishState() {
  String output;
  serializeJson(ledState, output);
  client.publish(state_topic, output.c_str());
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

/* Local Variables: */
/* mode:c           */
/* End:             */
