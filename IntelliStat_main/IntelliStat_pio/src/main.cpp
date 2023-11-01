/******************************************
 *
 * IntelliStat
 * 
 * Main program code for ESP32 running on Arduino Framework
 * Developed by:
 * Aldrick Tadeo, https://github.com/aldrick-t/
 * Sebastian Colin,
 * Chimali Nava,
 * Andrea Medina,
 * 
 * 
 * Ubidots ESP32 MQTT Function by:
 * Jose Garcia, https://github.com/jotathebest/
 *
 * ****************************************/
// Basic Libraries
#include <Arduino.h>
#include <WiFi.h>
// Ubidots / MQTT Libraries
#include "UbidotsEsp32Mqtt.h"

// Sensor Libraries
#include <DHT.h>
#include <DHT_U.h>

// Define DHT Sensor
#define DHTTYPE DHT11 // DHT 11
#define DHTPIN 15
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
sensors_event_t event;

// Define config constants
// Ubidots
const char *UBIDOTS_TOKEN = "BBFF-qEjEkvY4IIQlR6npGuDcr7eYIx3aOx";
const char *DEVICE_LABEL = "esp32";   // Device label to which data will be published
const char *VARIABLE_LABEL = "humid"; // Variable label to which data will be published
const char *VARIABLE_LABEL_TWO = "temp";
const char *VARIABLE_LABEL_THREE = "MQ2";
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds
// WiFi
const char *WIFI_SSID = "Andromeda";
const char *WIFI_PASS = "andromeda";

// Define global variables
unsigned long timer;
float MQ2_read;

//Define pins
//const int DHT11_s = 15;
const int MQ2_a0 = 32;
const int MQ2_d0 = 16;
const int LED_MQ2Warn = 5;
const int LED_humidLim = 18;

// Define Ubidots object
Ubidots ubidots(UBIDOTS_TOKEN);

// Define callback function (for use with MQTT)
void callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}


void setup() {
    // Serial Setup
    Serial.begin(115200);
    
    
    // Time Setup
    timer = millis();

    // Sensor Setup
    dht.begin();
    sensor_t sensor;

    // Pin Setup
    pinMode(MQ2_a0, INPUT);
    pinMode(MQ2_d0, INPUT);
    pinMode(LED_MQ2Warn, OUTPUT);
    pinMode(LED_humidLim, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    //Startup Sequence
    Serial.println("IntelliStat v0.1");
    Serial.println("Initializing...");

    //Sensor Startup
    Serial.println("Warming up sensors...");
    for (int i = 0; i < 40; i++) { //Number set in seconds = n * 0.5
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
    }
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Sensors ready!");
    delay(1000);
    //digitalWrite(LED_BUILTIN, LOW);

    // WiFi Setup
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("Connecting to WiFi");
    int i = 0;
    while (WiFi.status() != WL_CONNECTED) {
        i++;
        Serial.print(".");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
        if (WiFi.status() == WL_NO_SSID_AVAIL && i >= 20) {
            Serial.println("SSID not available!");
            Serial.println("To retry connection, restart device.");
            Serial.println("Continuing without WiFi...");
            break;
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi Network: " + WiFi.SSID());
        Serial.println("Local ESP32 IP Address: " + WiFi.localIP().toString());
        digitalWrite(LED_BUILTIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);

        // Ubidots Setup
        //ubidots.setDebug(true);  // uncomment this to make debug messages available
        ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
        ubidots.setCallback(callback);
        ubidots.setup();
        ubidots.reconnect();
    } else {
        Serial.println("No WiFi connection, continuing offline without DB.");

        
    }

}

void LED_Busy() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}

void Ubidots_loop() {
    //Ubidots Loop
    if (!ubidots.connected()) {
        LED_Busy();
        ubidots.reconnect();
    }
    if ((millis() - timer) > PUBLISH_FREQUENCY) { // triggers the routine every 5 second
        float value = event.relative_humidity;
        dht.temperature().getEvent(&event);
        float val = event.temperature;
        ubidots.add(VARIABLE_LABEL, value); // Insert your variable Labels and the value to be sent
        ubidots.add(VARIABLE_LABEL_TWO, val);
        ubidots.add(VARIABLE_LABEL_THREE, MQ2_read);
        ubidots.publish(DEVICE_LABEL);
        timer = millis();
    }
    ubidots.loop();
}

void loop() {
    // DHT Sensor Readings
    // Get temperature event and print its value.
    dht.temperature().getEvent(&event);
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));\

    // Humid Linit Warning LED
    if (event.relative_humidity > 75) {
        digitalWrite(LED_humidLim, 1);
    } else {
        digitalWrite(LED_humidLim, 0);
    }
    
    // MQ2 Sensor Readings
    MQ2_read = analogRead(MQ2_a0);
    Serial.print("MQ2 Value: ");
    Serial.println(MQ2_read); 
    delay(1000);

    // Gas Warning LED
    if (MQ2_read > 1000) {
        digitalWrite(LED_MQ2Warn, 1);
    } else {
        digitalWrite(LED_MQ2Warn, 0);
    }
    
    // Light Sensor Readings

    // PIR Sensor Readings

    // Ubidots
    if (WiFi.status() == WL_CONNECTED) {
        Ubidots_loop();
    }
}