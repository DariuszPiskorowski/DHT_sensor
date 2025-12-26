#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <math.h>

// Definitions for DHT sensor (adjust type to your hardware)
#define DHTPIN D7          // GPIO pin where the DHT sensor is connected
#define DHTTYPE DHT11      // Change to DHT11 if you have DHT11 sensor

// WiFi configuration (hardcoded)
const char* candidateSSIDs[] = {"eir-JPP", "JPP2", "JPPTV", "JPP1"};
const int candidateCount = sizeof(candidateSSIDs) / sizeof(candidateSSIDs[0]);
const char* wifiPassword = "**************"; // wspólne hasło dla wszystkich sieci

// MQTT server details
const char* mqtt_server = "192.168.1.***";
const char* mqtt_user = "s****h";
const char* mqtt_password = "*****";

// Device name (hardcoded)
char deviceName[33];
// Default device name (hardcoded)
const char* default_deviceName = "SilverRoom";

// Initialize objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);
// Create two DHT instances (one for each type) and select at runtime
DHT dht11(DHTPIN, DHT11);
DHT dht22(DHTPIN, DHT22);
DHT* dht = nullptr; // pointer to selected sensor instance
int detectedDHTType = 0; // 11 or 22 when detected

// Timing variables
unsigned long lastReadTime = 0;
const unsigned long readInterval = 3000; // 3 seconds
bool sensorPresent = true;

void setup() {
  Serial.begin(115200);
  // EEPROM removed - use hardcoded credentials and device name
  strncpy(deviceName, default_deviceName, 32);
  deviceName[32] = 0;
  Serial.print("Device Name: ");
  Serial.println(deviceName);
  connectToWiFi();
  mqttClient.setServer(mqtt_server, 1883);
  // Start both DHT instances (they share the same pin)
  dht11.begin();
  dht22.begin();
  delay(1000);

  // Try to detect sensor type by sampling both handlers and applying heuristics
  float h11 = dht11.readHumidity();
  float t11 = dht11.readTemperature();
  float h22 = dht22.readHumidity();
  float t22 = dht22.readTemperature();

  bool valid11 = !isnan(h11) && !isnan(t11) && (h11 >= 0.0) && (h11 <= 100.0) && (t11 >= 0.0) && (t11 <= 60.0);
  bool valid22 = !isnan(h22) && !isnan(t22) && (h22 >= 0.0) && (h22 <= 100.0) && (t22 >= -40.0) && (t22 <= 80.0);

  if (valid22 && !valid11) {
    dht = &dht22;
    detectedDHTType = 22;
    sensorPresent = true;
    Serial.println("Sensor diagnostic: Detected DHT22");
  } else if (valid11 && !valid22) {
    dht = &dht11;
    detectedDHTType = 11;
    sensorPresent = true;
    Serial.println("Sensor diagnostic: Detected DHT11");
  } else if (valid22 && valid11) {
    // Both returned plausible values; prefer DHT22 if fractional precision observed
    bool frac22 = (fabs(t22 - round(t22)) > 0.05) || (fabs(h22 - round(h22)) > 0.05);
    if (frac22) {
      dht = &dht22;
      detectedDHTType = 22;
      Serial.println("Sensor diagnostic: Ambiguous — choosing DHT22 based on fractional data");
    } else {
      dht = &dht11;
      detectedDHTType = 11;
      Serial.println("Sensor diagnostic: Ambiguous — choosing DHT11");
    }
    sensorPresent = true;
  } else {
    // Neither valid
    sensorPresent = false;
    dht = &dht11; // default to DHT11 to keep code paths valid
    detectedDHTType = 0;
    Serial.println("Sensor diagnostic: ERROR (no valid readings)");
  }
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectToMQTT();
  }
  mqttClient.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastReadTime >= readInterval) {
    lastReadTime = currentMillis;
    readAndSendSensorData();
  }
}

// connectToWiFi() is implemented below with scanning logic

void reconnectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.println(mqtt_server);

    // Ensure WiFi is connected; if not, try to find & connect to best AP
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected, attempting scan/connect...");
      connectToWiFi();
      delay(1000);
      if (WiFi.status() != WL_CONNECTED) {
        delay(5000);
        continue;
      }
    }

    // Resolve MQTT server hostname
    IPAddress mqttServerIp;
    if (!WiFi.hostByName(mqtt_server, mqttServerIp)) {
      Serial.println("Failed to resolve MQTT server IP via WiFi.hostByName()");
      delay(5000);
      continue;
    }
    Serial.print("Resolved MQTT server IP: ");
    Serial.println(mqttServerIp.toString());
    mqttClient.setServer(mqttServerIp, 1883);

    if (mqttClient.connect(deviceName, mqtt_user, mqtt_password)) {
      Serial.println("connected");

      // Publish IP address
      String ipAddress = WiFi.localIP().toString();
      char ipTopic[64];
      snprintf(ipTopic, sizeof(ipTopic), "%s/IP/ipaddress", deviceName);
      Serial.print("Publishing IP to: ");
      Serial.println(ipTopic);
      mqttClient.publish(ipTopic, ipAddress.c_str());
      // Publish connected SSID and RSSI
      char ssidTopic[64];
      snprintf(ssidTopic, sizeof(ssidTopic), "%s/WiFi/SSID", deviceName);
      mqttClient.publish(ssidTopic, WiFi.SSID().c_str());

      char rssiTopic[64];
      snprintf(rssiTopic, sizeof(rssiTopic), "%s/WiFi/RSSI", deviceName);
      String rssiNow = String(WiFi.RSSI());
      mqttClient.publish(rssiTopic, rssiNow.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Scan available networks and connect to the strongest one from the candidate list
void connectToWiFi() {
  Serial.println("Scanning for candidate WiFi networks...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int n = WiFi.scanNetworks();
  if (n <= 0) {
    Serial.println("No networks found");
  } else {
    int bestRssi = -10000;
    String bestSSID = "";
    for (int i = 0; i < n; ++i) {
      String found = WiFi.SSID(i);
      int rssi = WiFi.RSSI(i);
      for (int j = 0; j < candidateCount; ++j) {
        if (found == String(candidateSSIDs[j])) {
          Serial.print("Found candidate: "); Serial.print(found); Serial.print(" rssi="); Serial.println(rssi);
          if (rssi > bestRssi) {
            bestRssi = rssi;
            bestSSID = found;
          }
        }
      }
    }

    if (bestSSID.length() > 0) {
      Serial.print("Attempting to connect to best SSID: "); Serial.println(bestSSID);
      WiFi.begin(bestSSID.c_str(), wifiPassword);
      int retry = 0;
      while (WiFi.status() != WL_CONNECTED && retry < 60) {
        delay(500);
        Serial.print('.');
        retry++;
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected");
        Serial.print("Connected SSID: "); Serial.println(WiFi.SSID());
        Serial.print("IP address: "); Serial.println(WiFi.localIP());
      } else {
        Serial.println("\nFailed to connect to best SSID");
      }
    } else {
      Serial.println("No candidate SSIDs found in scan");
    }
  }
  WiFi.scanDelete();
}

void readAndSendSensorData() {
  // Always publish current SSID and RSSI so you can monitor connectivity even when DHT fails
  char ssidTopic[64];
  snprintf(ssidTopic, sizeof(ssidTopic), "%s/WiFi/SSID", deviceName);
  mqttClient.publish(ssidTopic, WiFi.SSID().c_str());
  Serial.print("Published SSID: "); Serial.print(WiFi.SSID()); Serial.print(" -> "); Serial.println(ssidTopic);

  char rssiTopic[64];
  snprintf(rssiTopic, sizeof(rssiTopic), "%s/WiFi/RSSI", deviceName);
  String rssi = String(WiFi.RSSI());
  mqttClient.publish(rssiTopic, rssi.c_str());
  Serial.print("Published RSSI: "); Serial.print(rssi); Serial.print(" -> "); Serial.println(rssiTopic);

  // Read sensor values (read humidity first as recommended)
  float humidity = dht->readHumidity();
  float temp = dht->readTemperature();
  // Debug: print raw returned values and NaN status
  Serial.print("Raw humidity read: ");
  if (isnan(humidity)) Serial.println("NaN"); else Serial.println(humidity);
  Serial.print("Raw temp read: ");
  if (isnan(temp)) Serial.println("NaN"); else Serial.println(temp);
  if (isnan(temp) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor (first try), retrying in 3s...");
    delay(3000); // DHT sensors require a pause between reads
    temp = dht->readTemperature();
    humidity = dht->readHumidity();
    if (isnan(temp) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor (after retry). Publishing error placeholders.");
      // publish error placeholders so you notice missing sensor data
      char tempTopic[64];
      snprintf(tempTopic, sizeof(tempTopic), "%s/Temperature/temperature", deviceName);
      char humidityTopic[64];
      snprintf(humidityTopic, sizeof(humidityTopic), "%s/Humidity/humidity", deviceName);
      const char *errStr = "E";
      mqttClient.publish(tempTopic, errStr);
      mqttClient.publish(humidityTopic, errStr);
      Serial.print("Published E to "); Serial.println(tempTopic);
      Serial.print("Published E to "); Serial.println(humidityTopic);
      // Also publish diagnostic raw topics so it's clear sensor failed
      char diagTempTopic[64];
      snprintf(diagTempTopic, sizeof(diagTempTopic), "%s/Diagnostics/TemperatureRaw", deviceName);
      mqttClient.publish(diagTempTopic, errStr);
      char diagHumTopic[64];
      snprintf(diagHumTopic, sizeof(diagHumTopic), "%s/Diagnostics/HumidityRaw", deviceName);
      mqttClient.publish(diagHumTopic, errStr);
      Serial.print("Published diagnostics E to "); Serial.println(diagTempTopic);
      // Publish sensor presence status
      char sensorPresTopic[64];
      snprintf(sensorPresTopic, sizeof(sensorPresTopic), "%s/Diagnostics/SensorPresent", deviceName);
      mqttClient.publish(sensorPresTopic, errStr);
      Serial.print("Published SensorPresent E to "); Serial.println(sensorPresTopic);
      sensorPresent = false;
      return;
    }
  }

  // Format data
  char tempPayload[10];
  snprintf(tempPayload, sizeof(tempPayload), "%.1f", temp);
  char humidityPayload[10];
  snprintf(humidityPayload, sizeof(humidityPayload), "%.1f", humidity);

  // Publish diagnostic/raw values as well
  char diagTempTopic[64];
  snprintf(diagTempTopic, sizeof(diagTempTopic), "%s/Diagnostics/TemperatureRaw", deviceName);
  mqttClient.publish(diagTempTopic, tempPayload);
  char diagHumTopic[64];
  snprintf(diagHumTopic, sizeof(diagHumTopic), "%s/Diagnostics/HumidityRaw", deviceName);
  mqttClient.publish(diagHumTopic, humidityPayload);
  Serial.print("Published diagnostics temp: "); Serial.print(tempPayload); Serial.print(" -> "); Serial.println(diagTempTopic);
  Serial.print("Published diagnostics hum: "); Serial.print(humidityPayload); Serial.print(" -> "); Serial.println(diagHumTopic);

  // Publish sensor presence status (OK)
  char sensorPresTopic[64];
  snprintf(sensorPresTopic, sizeof(sensorPresTopic), "%s/Diagnostics/SensorPresent", deviceName);
  const char *okStr = "OK";
  mqttClient.publish(sensorPresTopic, okStr);
  Serial.print("Published SensorPresent OK to "); Serial.println(sensorPresTopic);

  // Create MQTT topics
  char tempTopic[64];
  snprintf(tempTopic, sizeof(tempTopic), "%s/Temperature/temperature", deviceName);
  char humidityTopic[64];
  snprintf(humidityTopic, sizeof(humidityTopic), "%s/Humidity/humidity", deviceName);

  // Publish data
  mqttClient.publish(tempTopic, tempPayload);
  mqttClient.publish(humidityTopic, humidityPayload);

  Serial.print("Published "); Serial.print(tempPayload); Serial.print(" to "); Serial.println(tempTopic);
  Serial.print("Published "); Serial.print(humidityPayload); Serial.print(" to "); Serial.println(humidityTopic);
}
