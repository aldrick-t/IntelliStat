
/******************************************
 *
 * IntelliStat
 * 
 * Main program code for ESP32 running on Arduino Framework
 * Developed by:
 * Aldrick T, https://github.com/aldrick-t/
 * Sebastian C,
 * Chimali N,
 * Andrea M,
 * 
 *
 * ****************************************/
// Basic Libs
#include <Arduino.h>
#include <WiFi.h>
#include <Buzzer.h>
//#include <WiFiClient.h>

// Ubidots / MQTT Libs
//#include "UbidotsEsp32Mqtt.h"
#include <PubSubClient.h>

//Arduino cloud Libs
#include "thingProperties.h" //ignore errors and upload from Arduino IDE or Cloud Editor. PIO does not support this library.

// Sensor Libs
#include <DHT.h>
#include <DHT_U.h>
#include <MQUnifiedsensor.h>

// Define config constants
// Ubidots
const char *UBIDOTS_TOKEN = "BBFF-qEjEkvY4IIQlR6npGuDcr7eYIx3aOx";
const char *DEVICE_LABEL = "esp32";   // Device label to which data will be published
const char *VARIABLE_LABEL = "humid"; // Variable label to which data will be published
const char *VARIABLE_LABEL_TWO = "temp";
const char *VARIABLE_LABEL_THREE = "MQ2";
const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

// WiFi (non-ArduinoCloud)
const char *WIFI_SSID = "Andromeda";
const char *WIFI_PASS = "andromeda";

//HiveMQ
const char *mqtt_server_url = "broker.emqx.io";
const char *hiveMQ_API_tok = "eyJraWQiOiIwOWZlZDNlZS03ZTgzLTRmZWEtOWRkYy1lMjNkYzllNjJmYmYiLCJhbGciOiJSUzI1NiJ9.eyJuYW1lIjoiaW50ZWxsaXN0YXRFU1AzMiIsInN1YiI6InFtN2VuZSIsImlzcyI6ImhpdmVtcS1jbG91ZCIsImF1ZCI6WyJ3b29sLWNhcmRlciJdLCJpYXQiOjE2OTk4OTc0NDAsImV4cCI6MTcwMDUwMjI0MCwicm9sZXMiOlsid3JpdGUvYWxsL3FtN2VuZSIsInJlYWQvYWxsL3FtN2VuZSIsImRlbGV0ZS9hbGwvcW03ZW5lIiwicmVhZC9saXN0TXF0dFBlcm1pc3Npb25zL3FtN2VuZSJdLCJ2ZXJzaW9uIjoiVjEifQ.GbzatcjEkKMbzGflL-E8qJx78AjLNVQ4Vpb1wzqoGGaQ6TySIILQ-dTbpHgyxDslMlMmdE6bflPmwAGDW0W0LOxD5Z2HNEjCPeSm7f6qQgtrQzCMmOa5C7whJLbNd8GGo27Z1HbuY8P4_XmzbLOPyL_Cbly6IC8Q5VTWqCy8jtUO1rnPID2AZwF-cw3_C-ggdAdrpgiJbzFYj3DDcSm8dvsQ-wHTLrjvAIdIcr-S7sfSOV0PNYDsYy_v2zuSEKpzS0AWPy7loWhq7f4cAeDTONE-t-hnGINaRCn6QRyuz6XdkBAOwqPYQc6BTWa7AhcDvnAcxdybnG9wdlrohasxzw";
const char *mqtt_username = "Intellistat";
const char *mqtt_password = "IntellistatESP32";
const int mqtt_port = 1883;

// Define Network Objects
// Define MQTT object
WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
const char* dht11_temp_topic= "temp";
const char* dht11_humid_topic= "humid";
const char* mq2_topic= "mq2";

//Define Sensor Objects
//Define DHT Sensor
#define DHTTYPE DHT11 // DHT 11
#define DHTPIN 33
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;
sensors_event_t event;

//Define Buzzer
#define BUZZER_PIN 27
Buzzer buzzer_1(BUZZER_PIN);

//Define MQ-X Sensors
#define         Board                   ("ESP-32") // Wemos ESP-32 or other board, whatever have ESP32 core.
#define         Pin                     (35)
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-2") //MQ2 or other MQ Sensor, if change this verify your a and b values.
#define         Voltage_Resolution      (3.3) // 3V3 <- IMPORTANT. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         ADC_Bit_Resolution      (12) // ESP-32 bit resolution. Source: https://randomnerdtutorials.com/esp32-adc-analog-read-arduino-ide/
#define         RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm 
#define R0_default (9.06) //Default R0 value
MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

//Define pins
const int MQ2_a0 = 32; //MQ2 A0
const int MQ2_d0 = 26; //MQ2 D0
const int LED_MQ2Warn = 13; //MQ2 Warning LED
const int LED_humidLim = 12; //Humidity Warning LED
const int LED_tempWarn = 14; //Temperature Warning LED
const int calib_switch = 25; //Calibration Enable Switch
const int fan_enable = 18; //Fan Pin
const int LED_status = 4; //Status LED
const int LED_const = 15; //Constant LED

// Define global variables
unsigned long timer;
float MQ2_read;
float MQ2_A0_raw;
unsigned long fanTime;

// Define callback function (for use with MQTT)
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void LED_Busy() {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_status, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_status, LOW);
    delay(250);
}

void buzzer_warning_01() {
    //Buzzer Warning
    buzzer_1.sound(NOTE_A4, 500);
    buzzer_1.sound(NOTE_G3, 500);
    buzzer_1.end(0);
}

void WiFi_connect(const char *ssid, const char *pass) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    Serial.println("Connecting to WiFi");
    int i = 0;
    while (WiFi.status() != WL_CONNECTED) {
        i++;
        Serial.print(".");
        LED_Busy();

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

    } else {
        Serial.println("No WiFi connection, continuing offline without DB.");   
    }
}

void mqtt_connect(const char *mqtt_server_url) {
    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        if (client.connect("ESP32Client")) {
            Serial.println("connected!");
            //client.publish("esp32");
        } else {
            Serial.print("failed with state ");
            Serial.print(client.state() + "\n");
            delay(2000);
        }
    }
    
}

void mqtt_reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publishMessage(const char* topic, String payload , boolean retained){
if (client.publish(topic, payload.c_str(), true))
Serial.println("Message published ["+String(topic)+"]: "+payload);
}

void MQ_calibration() {
    /*****************************  MQ CAlibration ********************************************/ 
    // Explanation: 
    // In this routine the sensor will measure the resistance of the sensor supposedly before being pre-heated
    // and on clean air (Calibration conditions), setting up R0 value.
    // We recomend executing this routine only on setup in laboratory conditions.
    // This routine does not need to be executed on each restart, you can load your R0 value from eeprom.
    // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor
    Serial.print("Calibrating please wait.");
    float calcR0 = 0;
    for(int i = 1; i<=10; i ++)
    {
        MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
        calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
        Serial.print(".");
    }
    MQ2.setR0(calcR0/10);
    Serial.println("  done!.");
    Serial.print("R0= ");
    Serial.println(calcR0/10);
    
    if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
    if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}
    /*****************************  MQ CAlibration ********************************************/ 
}

void setup() {
    // Serial Setup
    // Initialize serial and wait for port to open:
    Serial.begin(115200);
    // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
    delay(1500); 
    
    //Board Status LED
    digitalWrite(LED_const, HIGH);

    // Time Setup
    timer = millis();

    // Pin Setup
    //LED / User Interface Pins
    pinMode(calib_switch, INPUT);
    pinMode(LED_MQ2Warn, OUTPUT);
    pinMode(LED_humidLim, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LED_tempWarn, OUTPUT);
    pinMode(fan_enable, OUTPUT);
    pinMode(LED_status, OUTPUT);
    pinMode(LED_const, OUTPUT);
    //pinMode(buzzer_1_pin, OUTPUT);
    //ledcAttachPin(buzzer_1_pin, 0);
    

    dht.begin();
    sensor_t sensor;

    MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ2.setA(574.25); MQ2.setB(-2.222); // Configure the equation to to calculate LPG concentration
    /*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
    */
    MQ2.init();
    //MQ2.setRL(5); //Review RL value (datasheet).
    
    // Sensor Setup
    if (digitalRead(calib_switch) == HIGH) {
        MQ_calibration();
    } else {
        MQ2.setR0(R0_default);
        Serial.println("Calibration disabled. Continuing with default R0.");
    }

    //Sensor Pins
    pinMode(MQ2_a0, INPUT);
    pinMode(MQ2_d0, INPUT);

    //Startup Sequence
    Serial.println("=============================================");
    Serial.println("IntelliStat v1.1");
    Serial.println("Initializing...");
    Serial.println("=============================================");


    //Sensor Startup
    Serial.println("Warming up sensors...");
    for (int i = 0; i < 40; i++) { //Number set in seconds = n * 0.5
        LED_Busy();
    }
    //MQ2.serialDebug(true); 
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_MQ2Warn, HIGH);
    digitalWrite(LED_humidLim, HIGH);
    digitalWrite(LED_tempWarn, HIGH);
    digitalWrite(LED_status, HIGH);
    digitalWrite(fan_enable, HIGH);
    Serial.println("Sensors ready!");
    delay(1000);

    // WiFi Setup
    //WiFi_connect(WIFI_SSID, WIFI_PASS);
    //Arduino Cloud Setup
    // Defined in thingProperties.h
    initProperties();

    // Connect to Arduino IoT Cloud
    ArduinoCloud.begin(ArduinoIoTPreferredConnection);
    
    /*
        The following function allows you to obtain more information
        related to the state of network and IoT Cloud connection and errors
        the higher number the more granular information you’ll get.
        The default is 0 (only errors).
        Maximum is 4
    */
    setDebugMessageLevel(4);
    ArduinoCloud.printDebugInfo();
    ArduinoCloud.update();
    

    // MQTT Setup
    client.setServer(mqtt_server_url, mqtt_port);
    client.setCallback(callback);
}

void loop() {
    ArduinoCloud.update();
    digitalWrite(LED_const, HIGH);
    digitalWrite(LED_status, HIGH);

    // DHT Sensor Readings
    // Get temperature event and print its value.
    dht.temperature().getEvent(&event);
    temp = event.temperature;
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    humid = event.relative_humidity;

    // Humid Limit Warning LED
    if (humid > 75) {
        digitalWrite(LED_humidLim, HIGH);
        //digitalWrite(fan_enable, HIGH);
        //buzzer_warning_01();
    } else {
        digitalWrite(LED_humidLim, LOW);
        //digitalWrite(fan_enable, LOW);
    }

    if (temp > 45) {
        digitalWrite(LED_tempWarn, HIGH);
        //digitalWrite(fan_enable, HIGH);
        buzzer_warning_01();
    } else {
        digitalWrite(LED_tempWarn, LOW);
        //digitalWrite(fan_enable, LOW);
    }
    // MQ2 Sensor Readings
    MQ2.update(); // Update data, the esp32 will read the voltage from the analog pin
    MQ2_read = MQ2.readSensor(); // Sensor will read PPM concentration using the model, a and b values set previously or from the setup
    delay(500); //Sampling frequency
    MQ2_A0_raw = analogRead(Pin);
    mQ2 = MQ2_read; 
    
    // Gas Warning LED
    if (MQ2_A0_raw > 500) {
        digitalWrite(LED_MQ2Warn, HIGH);
        buzzer_warning_01();
        //digitalWrite(fan_enable, HIGH);
    } else {
        digitalWrite(LED_MQ2Warn, LOW);
        //digitalWrite(fan_enable, LOW);
    }
    
    // Fan enable const
    digitalWrite(fan_enable, HIGH);

    // Ubidots
    if (WiFi.status() == WL_CONNECTED) {
        //Ubidots_loop();
    }

    // Fan Control  =========================================
    // timer = millis();

    // if (timer - fanTime >= 120000 && fan_enable == LOW) {
    //     digitalWrite(fan_enable, HIGH);
    //     fanTime = timer;
    // }
    // else if (timer - fanTime > 60000 && fan_enable == HIGH) {
    //     digitalWrite(fan_enable, LOW);
    //     fanTime = timer;
    // }
    // Fan Control  =========================================

    // MQTT =================================================
    // if (!client.connected()) {
    //     mqtt_reconnect();
    // }
    // client.loop();
    
    // String payload = "test_iot_esp32_intellistat";
    // publishMessage(dht11_humid_topic,String(humid),true);    
    // publishMessage(dht11_temp_topic,String(temp),true);
    // //Pub Payload
    // MQTT =================================================

    //*******SERIAL DEBUG OUTPUT**********//
    // uncomment to enable serial debug output.

    // This command will read the voltage at the analog pin of the sensor.
    //Serial.print("MQ2 A0 Voltage: "); Serial.print(MQ2.getVoltage(true)); Serial.print("\n"); 
    //Serial.print("MQ2 A0 RAW: "); Serial.print(MQ2_A0_raw); Serial.print("\n");
    // Serial.print(F("Temperature: "));
    // Serial.print(temp);
    // Serial.println(F("°C"));
    // Serial.print(F("Humidity: "));
    // Serial.print(humid);
    // Serial.println(F("%"));
    // Serial.print("LPG: "); Serial.print(MQ2_read);
    // Serial.print(" ppm\t\n");
    //Serial.print("FAN_TIME:"); Serial.print(fanTime); Serial.print("\n");
    //Serial.print("TIMER   :"); Serial.print(timer); Serial.print("\n");
    //Serial.print("FAN_EN  :"); Serial.print(fan_enable); Serial.print("\n");
    // MQ2.serialDebug(); // Will print the table on the serial port

    //*******SERIAL DEBUG OUTPUT**********//
    
}