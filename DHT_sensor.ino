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

// Timing variables
unsigned long nextReadTime = 0;
const unsigned long readInterval = 60000; // 1 minute
const unsigned long retryDelay = 3000;
bool pendingRetry = false;
unsigned long retryAt = 0;
bool sensorPresent = true;
unsigned long lastMqttAttempt = 0;
const unsigned long mqttRetryInterval = 5000;
unsigned long lastWiFiCheck = 0;
const unsigned long wifiCheckInterval = 60000;
bool wifiConnecting = false;
unsigned long wifiConnectStart = 0;
const unsigned long wifiConnectTimeout = 30000;
bool wifiScanInProgress = false;
unsigned long wifiScanStart = 0;
const unsigned long wifiScanTimeout = 15000;

void handleWiFiScanResult(int networksFound);

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

  // Try to detect sensor type by sampling both handlers and applying heuristics
  delay(2000); // DHT sensors often need time after begin()
  float h11 = NAN, t11 = NAN, h22 = NAN, t22 = NAN;
  int valid11Count = 0;
  int valid22Count = 0;
  bool frac22 = false;
  const int detectSamples = 3;
  const unsigned long detectDelayMs = 400;

  for (int i = 0; i < detectSamples; ++i) {
    h11 = dht11.readHumidity();
    t11 = dht11.readTemperature();
    h22 = dht22.readHumidity();
    t22 = dht22.readTemperature();

    bool valid11 = !isnan(h11) && !isnan(t11) && (h11 >= 0.0) && (h11 <= 100.0) && (t11 >= 0.0) && (t11 <= 60.0);
    bool valid22 = !isnan(h22) && !isnan(t22) && (h22 >= 0.0) && (h22 <= 100.0) && (t22 >= -40.0) && (t22 <= 80.0);
    if (valid11) valid11Count++;
    if (valid22) {
      valid22Count++;
      if ((fabs(t22 - round(t22)) > 0.05) || (fabs(h22 - round(h22)) > 0.05)) {
        frac22 = true;
      }
    }
    delay(detectDelayMs);
  }

  if (valid22Count > 0 && valid11Count == 0) {
    dht = &dht22;
    sensorPresent = true;
    Serial.println("Sensor diagnostic: Detected DHT22");
  } else if (valid11Count > 0 && valid22Count == 0) {
    dht = &dht11;
    sensorPresent = true;
    Serial.println("Sensor diagnostic: Detected DHT11");
  } else if (valid22Count > 0 && valid11Count > 0) {
    // Both returned plausible values; prefer DHT22 if fractional precision observed
    if (frac22) {
      dht = &dht22;
      Serial.println("Sensor diagnostic: Ambiguous, choosing DHT22 based on fractional data");
    } else {
      dht = &dht11;
      Serial.println("Sensor diagnostic: Ambiguous, choosing DHT11");
    }
    sensorPresent = true;
  } else {
    // Neither valid
    sensorPresent = false;
    dht = &dht11; // default to DHT11 to keep code paths valid
    Serial.println("Sensor diagnostic: ERROR (no valid readings)");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastWiFiCheck >= wifiCheckInterval) {
    lastWiFiCheck = currentMillis;
    if (WiFi.status() != WL_CONNECTED && !wifiConnecting) {
      Serial.println("WiFi watchdog: not connected, attempting reconnect...");
      connectToWiFi();
    }
  }

  if (wifiConnecting) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnecting = false;
      Serial.println("\nWiFi connected");
      Serial.print("Connected SSID: "); Serial.println(WiFi.SSID());
      Serial.print("IP address: "); Serial.println(WiFi.localIP());
    } else if (currentMillis - wifiConnectStart >= wifiConnectTimeout) {
      Serial.println("\nWiFi connection timeout");
      WiFi.disconnect();
      wifiConnecting = false;
    }
  }

  if (wifiScanInProgress && (currentMillis - wifiScanStart >= wifiScanTimeout)) {
    Serial.println("WiFi scan timeout");
    WiFi.scanDelete();
    wifiScanInProgress = false;
  }

  if (!mqttClient.connected()) {
    if (currentMillis - lastMqttAttempt >= mqttRetryInterval) {
      lastMqttAttempt = currentMillis;
      reconnectToMQTT();
    }
  }
  mqttClient.loop();

  if (pendingRetry && currentMillis >= retryAt) {
    pendingRetry = false;
    readAndSendSensorData(true);
  } else if (!pendingRetry && currentMillis >= nextReadTime) {
    readAndSendSensorData(false);
  }
}

// connectToWiFi() is implemented below with scanning logic

void reconnectToMQTT() {
  Serial.print("Attempting MQTT connection to ");
  Serial.println(mqtt_server);

  // Ensure WiFi is connected; if not, try to find & connect to best AP
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiConnecting) {
      Serial.println("WiFi not connected, attempting scan/connect...");
      connectToWiFi();
    }
    return;
  }

  // Resolve MQTT server hostname
  IPAddress mqttServerIp;
  if (!WiFi.hostByName(mqtt_server, mqttServerIp)) {
    Serial.println("Failed to resolve MQTT server IP via WiFi.hostByName()");
    return;
  }
  Serial.print("Resolved MQTT server IP: ");
  Serial.println(mqttServerIp.toString());
  mqttClient.setServer(mqttServerIp, 1883);

  char willTopic[64];
  snprintf(willTopic, sizeof(willTopic), "%s/Status", deviceName);
  const char* willMessage = "OFFLINE";

  if (mqttClient.connect(deviceName, mqtt_user, mqtt_password, willTopic, 1, true, willMessage)) {
    Serial.println("connected");

    mqttClient.publish(willTopic, "ONLINE", true);

    // Publish IP address
    IPAddress localIp = WiFi.localIP();
    char ipAddress[16];
    snprintf(ipAddress, sizeof(ipAddress), "%u.%u.%u.%u", localIp[0], localIp[1], localIp[2], localIp[3]);
    char ipTopic[64];
    snprintf(ipTopic, sizeof(ipTopic), "%s/IP/ipaddress", deviceName);
    Serial.print("Publishing IP to: ");
    Serial.println(ipTopic);
    mqttClient.publish(ipTopic, ipAddress);
    // Publish connected SSID and RSSI
    char ssidTopic[64];
    snprintf(ssidTopic, sizeof(ssidTopic), "%s/WiFi/SSID", deviceName);
    mqttClient.publish(ssidTopic, WiFi.SSID().c_str());

    char rssiTopic[64];
    snprintf(rssiTopic, sizeof(rssiTopic), "%s/WiFi/RSSI", deviceName);
    char rssiNow[8];
    snprintf(rssiNow, sizeof(rssiNow), "%ld", WiFi.RSSI());
    mqttClient.publish(rssiTopic, rssiNow);
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// Scan available networks and connect to the strongest one from the candidate list
void connectToWiFi() {
  if (wifiScanInProgress) {
    return;
  }

  Serial.println("Scanning for candidate WiFi networks (async)...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifiScanInProgress = true;
  wifiScanStart = millis();
  WiFi.scanNetworksAsync(handleWiFiScanResult, true);
}

void handleWiFiScanResult(int networksFound) {
  wifiScanInProgress = false;

  if (networksFound <= 0) {
    Serial.println("No networks found");
    WiFi.scanDelete();
    return;
  }

  int bestRssi = -10000;
  String bestSSID = "";
  for (int i = 0; i < networksFound; ++i) {
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
    wifiConnecting = true;
    wifiConnectStart = millis();
  } else {
    Serial.println("No candidate SSIDs found in scan");
  }
  WiFi.scanDelete();
}

void readAndSendSensorData(bool isRetry) {
  if (!mqttClient.connected()) {
    // Avoid publishing when MQTT is offline; schedule next read instead.
    nextReadTime = millis() + readInterval;
    return;
  }

  // Always publish current SSID and RSSI so you can monitor connectivity even when DHT fails
  char ssidTopic[64];
  snprintf(ssidTopic, sizeof(ssidTopic), "%s/WiFi/SSID", deviceName);
  mqttClient.publish(ssidTopic, WiFi.SSID().c_str());
  Serial.print("Published SSID: "); Serial.print(WiFi.SSID()); Serial.print(" -> "); Serial.println(ssidTopic);

  char rssiTopic[64];
  snprintf(rssiTopic, sizeof(rssiTopic), "%s/WiFi/RSSI", deviceName);
  char rssi[8];
  snprintf(rssi, sizeof(rssi), "%ld", WiFi.RSSI());
  mqttClient.publish(rssiTopic, rssi);
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
    if (!isRetry) {
      Serial.println("Failed to read from DHT sensor (first try), scheduling retry...");
      pendingRetry = true;
      retryAt = millis() + retryDelay;
      return;
    }
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
    nextReadTime = millis() + readInterval;
    return;
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
  nextReadTime = millis() + readInterval;
}
