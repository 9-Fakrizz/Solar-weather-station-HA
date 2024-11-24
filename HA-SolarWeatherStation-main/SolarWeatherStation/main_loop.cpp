#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <PMS.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "SHT31.h" // Include SHT31 library

#define WIFI_STA_NAME "Ok"
#define WIFI_STA_PASS "q12345678"
#define MQTT_SERVER   "mqtt-dashboard.com"
#define MQTT_PORT     1883
//#define ACCESS_TOKEN  "iIla9F6X0EssBUMoybSL" // Replace with your ThingsBoard device token

#define LED_BUILTIN 5
#define SENSOR_EN_PIN 13
#define TX2_PIN 25
#define RX2_PIN 26
#define BATT_PIN 35

WiFiClient client;
PubSubClient mqtt(client);

// PMS sensor
PMS pms(Serial2);
PMS::DATA data;

// SHT3x sensor
SHT31 sht;

// Sensor values
float vBatt = 0;
int PM1_0 = 0;
int PM2_5 = 0;
int PM10_0 = 0;
float temperature = 0;
float humidity = 0;

float getBattVoltage() {
    long sum = 0;
    float voltage = 0.0;
    float R1 = 100000.0;
    float R2 = 100000.0;

    for (int i = 0; i < 500; i++) {
        sum += analogRead(BATT_PIN);
        delayMicroseconds(1000);
    }
    voltage = sum / (float)500;
    voltage = (voltage * 3.6) / 4096.0;
    voltage = voltage / (R2/(R1+R2));

    if (voltage > 10 || voltage < 0)
        voltage = 0;

    return voltage;
}

bool fetchPMS() {
    Serial.print("Activating PMS Sensor...");
    digitalWrite(SENSOR_EN_PIN, HIGH); // Activate PMS sensor
    delay(1000); // Wait for sensor to stabilize

    Serial.print("Reading PMS Sensor...");
    pms.passiveMode();
    if (pms.readUntil(data)) {
        PM1_0 = data.PM_AE_UG_1_0;
        PM2_5 = data.PM_AE_UG_2_5;
        PM10_0 = data.PM_AE_UG_10_0;
        Serial.printf("PM1 = %d, PM2.5 = %d, PM10 = %d\n", PM1_0, PM2_5, PM10_0);
        digitalWrite(SENSOR_EN_PIN, LOW); // Deactivate PMS sensor
        return true;
    } else {
        Serial.println("No PMS data available.");
        return false;
    }
    digitalWrite(SENSOR_EN_PIN, LOW); // Deactivate PMS sensor
}

bool fetchSHT3x() {
    if (sht.read()) {
        temperature = sht.getTemperature();
        humidity = sht.getHumidity();
        Serial.printf("Temperature = %.1f, Humidity = %.1f\n", temperature, humidity);
        return true;
    } else {
        Serial.println("No SHT3x data available.");
        return false;
    }
}

bool connectAndSend() {
    Serial.print("Connecting to Wi-Fi... ");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");
    }
    Serial.println(" Connected.");

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);

    if (!mqtt.connect("WeatherStationClient")) {
        Serial.println("MQTT Connection Failed");
        return false;
    }
    // Publish each value to its respective topic
    if (!mqtt.publish("okha/vbatt", String(vBatt).c_str())) {
        Serial.println("Failed to publish vBatt.");
    } else {
        Serial.println("vBatt published successfully.");
    }

    if (!mqtt.publish("okha/pm1_0", String(PM1_0).c_str())) {
        Serial.println("Failed to publish PM1_0.");
    } else {
        Serial.println("PM1_0 published successfully.");
    }

    if (!mqtt.publish("okha/pm2_5", String(PM2_5).c_str())) {
        Serial.println("Failed to publish PM2_5.");
    } else {
        Serial.println("PM2_5 published successfully.");
    }

    if (!mqtt.publish("okha/pm10_0", String(PM10_0).c_str())) {
        Serial.println("Failed to publish PM10_0.");
    } else {
        Serial.println("PM10_0 published successfully.");
    }

    if (!mqtt.publish("okha/temperature", String(temperature).c_str())) {
        Serial.println("Failed to publish temperature.");
    } else {
        Serial.println("Temperature published successfully.");
    }

    if (!mqtt.publish("okha/humidity", String(humidity).c_str())) {
        Serial.println("Failed to publish humidity.");
    } else {
        Serial.println("Humidity published successfully.");
    }

    // // Create JSON payload
    // String payload = "{";
    // payload += "\"vBatt\":" + String(vBatt) + ",";
    // payload += "\"PM1_0\":" + String(PM1_0) + ",";
    // payload += "\"PM2_5\":" + String(PM2_5) + ",";
    // payload += "\"PM10_0\":" + String(PM10_0) + ",";
    // payload += "\"temperature\":" + String(temperature) + ",";
    // payload += "\"humidity\":" + String(humidity);
    // payload += "}";
    // Serial.println(payload);
    // // Publish telemetry data to the "okha/" topic
    // if (!mqtt.publish("okha/", payload.c_str())) {
    //     Serial.println("Failed to publish MQTT message.");
    //     return false;
    // }
    // Serial.println("MQTT message published successfully.");
    return true;
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BATT_PIN, INPUT);
    pinMode(SENSOR_EN_PIN, OUTPUT);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);

    vBatt = getBattVoltage();
    digitalWrite(SENSOR_EN_PIN, LOW); // Ensure PMS sensor is off initially

    // Initialize SHT3x sensor
    Wire.begin();
    sht.begin(); // I2C address for SHT3x

    connectAndSend();
}

void loop() {
    fetchPMS();
    fetchSHT3x();
    vBatt = getBattVoltage();
    connectAndSend();
    delay(5000);  // wait 5 seconds before the next reading
}
