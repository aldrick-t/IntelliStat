/******************************************
 *
 * IntelliStat
 * Main program code for ESP32 running on Arduino Framework
 * Developed by:
 * 
 * 
 * Ubitdots ESP32 MQTT Function by:
 * Jose Garcia, https://github.com/jotathebest/
 *
 * ****************************************/
// Basic Libraries
#include <Arduino.h>
// Ubidots / MQTT Libraries
#include "UbidotsEsp32Mqtt.h"

// Sensor Libraries
#include <DHT.h>
#include <DHT_U.h>

// Define DHT Sensor
#define DHTTYPE DHT11 // DHT 11
#define DHTPIN 2
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// Define config constants
// Ubidots
const char *UBIDOTS_TOKEN = "BBFF-qEjEkvY4IIQlR6npGuDcr7eYIx3aOx";
const char *DEVICE_LABEL = "esp32";   // Device label to which data will be published
const char *VARIABLE_LABEL = "humid"; // Variable label to which data will be published
const char *VARIABLE_LABEL_TWO = "temp";
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
// WiFi
const char *WIFI_SSID = "Andromeda";
const char *WIFI_PASS = "andromeda";

// Define global variables
unsigned long timer;
float MQ2_val;

//Define pins
//const int DHT11_s = 2;
const int MQ2_a0 = 4;
const int MQ2_d0 = 16;
const int LED_gasWarn = 5;
const int LED_humidLim = 18;


// Define Ubidots object
Ubidots ubidots(UBIDOTS_TOKEN);

// Define callback function (for use with MQTT)
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


void setup() {
  // Serial Setup
  Serial.begin(115200);
  
  // Ubidots Setup
  ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  
  // Time Setup
  timer = millis();

  // Sensor Setup
  dht.begin();
  sensor_t sensor;

  // Pin Setup
  pinMode(MQ2_a0, INPUT);
  pinMode(MQ2_d0, INPUT);
  pinMode(LED_gasWarn, OUTPUT);
  pinMode(LED_humidLim, OUTPUT);

}

void loop() {
  // DHT Sensor Readings
  sensors_event_t event;
  // Get temperature event and print its value.
  dht.temperature().getEvent(&event);
  Serial.print(F("Temperature: "));
  Serial.print(event.temperature);
  Serial.println(F("°C"));
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  Serial.print(F("Humidity: "));
  Serial.print(event.relative_humidity);
  Serial.println(F("%"));
  // Blink LED
  digitalWrite(18, 1);
  delay(1000);
  digitalWrite(18, 0);
  delay(1000);

  // Ubidots Loop
  if (!ubidots.connected()) {
    ubidots.reconnect();
  }
  if ((millis() - timer) > PUBLISH_FREQUENCY) { // triggers the routine every 5 second
    float value = event.relative_humidity;
    dht.temperature().getEvent(&event);
    float val = event.temperature;
    ubidots.add(VARIABLE_LABEL, value); // Insert your variable Labels and the value to be sent
    ubidots.add(VARIABLE_LABEL_TWO, val);
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
  ubidots.loop();
  
  // MQ2 Sensor Readings
  MQ2_val = analogRead(MQ2_a0);
  Serial.print("MQ2 Value: ");
  Serial.println(MQ2_val);
  delay(1000);
  // Gas Warning LED
  if (MQ2_val > 1000) {
    digitalWrite(LED_gasWarn, 1);
  }
  else {
    digitalWrite(LED_gasWarn, 0);
  }

  // Light Sensor Readings

  // PIR Sensor Readings

}