#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <PMS.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include "SHT31.h" // Include SHT31 librarywsqaa"


#define WIFI_STA_NAME "Ok"
#define WIFI_STA_PASS "q12345678"
#define MQTT_SERVER   "broker.hivemq.com" // ThingsBoard server
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
float batt_map = 0;
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
    int retries = 3;
    Serial.print("Reading PMS Sensor...");
    for (int i = 0; i < retries; i++) {
        if (pms.readUntil(data)) {
            PM1_0 = data.PM_AE_UG_1_0;
            PM2_5 = data.PM_AE_UG_2_5;
            PM10_0 = data.PM_AE_UG_10_0;
            Serial.printf("PM1 = %d, PM2.5 = %d, PM10 = %d\n", PM1_0, PM2_5, PM10_0);
            digitalWrite(SENSOR_EN_PIN, LOW); // Deactivate PMS sensor
            return true;
        }
        delay(100); // Retry delay
    }
    Serial.println("No PMS data available.");
    digitalWrite(SENSOR_EN_PIN, LOW); // Deactivate PMS sensor
    return false;
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
bool connectMQTT() {
    int retries = 3;
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    for (int i = 0; i < retries; i++) {
        if (mqtt.connect("okhastation")) {
            Serial.println("MQTT Connected!");
            return true;
        }
        Serial.print("Retrying MQTT Connection... Attempt ");
        Serial.println(i + 1);
        delay(2000); // Wait before retrying
    }
    Serial.println("MQTT Connection Failed after retries.");
    return false;
}
void connectToWiFi() {
    Serial.print("Connecting to Wi-Fi... ");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS);

    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(" Failed to connect to Wi-Fi.");
    } else {
        Serial.println(" Connected. IP: " + WiFi.localIP().toString());
    }
}
void ensureMQTTConnected() {
    if (!mqtt.connected()) {
        connectMQTT();
    }
}
void ensureWiFiConnected() {
    if (WiFi.status() != WL_CONNECTED) {
        connectToWiFi();
    }
}


void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(BATT_PIN, INPUT);
    pinMode(SENSOR_EN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    vBatt = getBattVoltage();
    batt_map = map(vBatt, 3.10, 4.20, 5.00, 100.00);

    if (vBatt < 3.15) {
        Serial.println("Battery low. Going back to deep sleep.");
        esp_sleep_enable_timer_wakeup(100 * 1000000); // 5 minutes in microseconds
        esp_deep_sleep_start();
    }
    else{
        connectToWiFi();
        digitalWrite(SENSOR_EN_PIN, HIGH);
        delay(5000); // Stabilize sensor
        pms.passiveMode();
        Wire.begin();
        sht.begin();
        fetchPMS();
        fetchSHT3x();
        if (connectMQTT()) {
            // Publish sensor values
            mqtt.publish("okha/vbatt", String(vBatt).c_str());
            delay(100);
            mqtt.publish("okha/pm1_0", String(PM1_0).c_str());
            delay(100);
            mqtt.publish("okha/pm2_5", String(PM2_5).c_str());
            delay(100);
            mqtt.publish("okha/pm10_0", String(PM10_0).c_str());
            delay(100);
            mqtt.publish("okha/temperature", String(temperature).c_str());
            delay(100);
            mqtt.publish("okha/humidity", String(humidity).c_str());
            delay(100);
        }
        mqtt.disconnect();
        Serial.println("Going to deep sleep for 5 minutes...");
        esp_sleep_enable_timer_wakeup(100 * 1000000); // 5 minutes in microseconds
        esp_deep_sleep_start();
    }
}

void loop() {
    // Leave loop empty
}
