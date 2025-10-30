#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_PN532.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

// Struktur konfigurasi
struct NFCConfig {
    String nfcId;
    String deviceName;
    String wifi;
    String wifiPassword;
    String serverUrl;
    int serverPort;
    String sensorType;
    int interval;
    String deviceStatus;
    String timestamp;
    String version;
    String protocol;
    String mqttTopic;
    String mqttUsername;
    String mqttPassword;
    String apiKey;
    String endpoint;
};

NFCConfig currentConfig;
String lastRawData = "";
String lastUID = "";
bool tagPresent = false;
bool wifiConnected = false;
bool configurationValid = false;

// Protocol clients
WiFiClient wifiClient;
WiFiClientSecure secureClient;
PubSubClient mqttClient(wifiClient);

// Timing
unsigned long lastCheckTime = 0;
unsigned long lastReleaseTime = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastSensorSend = 0;
unsigned long lastReconnectMQTT = 0;
unsigned long lastHeartbeat = 0;

// Konfigurasi timing
const unsigned long CHECK_INTERVAL = 2000;
const unsigned long RELEASE_INTERVAL = 8000;
const unsigned long RELEASE_DURATION = 1500;
const unsigned long WIFI_CHECK_INTERVAL = 10000;
const unsigned long DEFAULT_SENSOR_INTERVAL = 5000;
const unsigned long WIFI_TIMEOUT = 20000;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 20000;

// ============ SENSOR FUNCTIONS ============

unsigned long getSensorInterval() {
    if (configurationValid && currentConfig.interval > 0) {
        return currentConfig.interval;
    }
    return DEFAULT_SENSOR_INTERVAL;
}

void readSensorData(float &temp, float &hum) {
    temp = random(250, 350) / 10.0;
    hum = random(600, 900) / 10.0;
}

String createSensorJSON(float temp, float hum) {
    JsonDocument jsonDoc;
    jsonDoc["temp"] = round(temp * 10) / 10.0;
    jsonDoc["hum"] = round(hum * 10) / 10.0;
    jsonDoc["deviceId"] = currentConfig.nfcId;
    jsonDoc["deviceName"] = currentConfig.deviceName;
    jsonDoc["sensorType"] = currentConfig.sensorType;
    jsonDoc["interval"] = currentConfig.interval;
    jsonDoc["protocol"] = currentConfig.protocol;
    jsonDoc["timestamp"] = millis() / 1000;
    jsonDoc["uptime"] = millis() / 1000;
    
    String json;
    serializeJson(jsonDoc, json);
    return json;
}

// ============ HTTP/HTTPS FUNCTIONS ============

bool sendDataViaHTTP(String jsonData, bool useHTTPS = false) {
    HTTPClient http;
    
    String fullUrl = currentConfig.serverUrl;
    
    // Add protocol if missing
    if (!fullUrl.startsWith("http://") && !fullUrl.startsWith("https://")) {
        fullUrl = String(useHTTPS ? "https://" : "http://") + fullUrl;
    }
    
    if (currentConfig.serverPort > 0) {
        // Only add port if not already in URL
        if (fullUrl.indexOf("://") > 0) {
            int colonPos = fullUrl.lastIndexOf(":");
            int slashPos = fullUrl.indexOf("/", fullUrl.indexOf("://") + 3);
            
            // Check if port is already specified
            if (colonPos < fullUrl.indexOf("://") || (slashPos > 0 && colonPos > slashPos)) {
                // No port specified, add it
                if (slashPos > 0) {
                    fullUrl = fullUrl.substring(0, slashPos) + ":" + String(currentConfig.serverPort) + fullUrl.substring(slashPos);
                } else {
                    fullUrl += ":" + String(currentConfig.serverPort);
                }
            }
        }
    }
    
    if (currentConfig.endpoint.length() > 0) {
        if (!fullUrl.endsWith("/") && !currentConfig.endpoint.startsWith("/")) {
            fullUrl += "/";
        }
        fullUrl += currentConfig.endpoint;
    } else {
        if (!fullUrl.endsWith("/")) fullUrl += "/";
        fullUrl += currentConfig.deviceName;
    }
    
    Serial.println("\n📡 Sending via " + String(useHTTPS ? "HTTPS" : "HTTP"));
    Serial.println("🔗 URL: " + fullUrl);
    
    bool success = false;
    
    if (useHTTPS) {
        secureClient.setInsecure();
        if (http.begin(secureClient, fullUrl)) {
            http.addHeader("Content-Type", "application/json");
            if (currentConfig.apiKey.length() > 0) {
                http.addHeader("Authorization", "Bearer " + currentConfig.apiKey);
            }
            http.setTimeout(10000);
            
            int httpResponseCode = http.POST(jsonData);
            
            if (httpResponseCode > 0) {
                Serial.println("✅ Response: " + String(httpResponseCode));
                Serial.println("📥 " + http.getString());
                success = true;
            } else {
                Serial.println("❌ Error: " + String(httpResponseCode));
            }
            http.end();
        }
    } else {
        if (http.begin(wifiClient, fullUrl)) {
            http.addHeader("Content-Type", "application/json");
            if (currentConfig.apiKey.length() > 0) {
                http.addHeader("Authorization", "Bearer " + currentConfig.apiKey);
            }
            http.setTimeout(10000);
            
            int httpResponseCode = http.POST(jsonData);
            
            if (httpResponseCode > 0) {
                Serial.println("✅ Response: " + String(httpResponseCode));
                Serial.println("📥 " + http.getString());
                success = true;
            } else {
                Serial.println("❌ Error: " + String(httpResponseCode));
            }
            http.end();
        }
    }
    
    return success;
}

// ============ MQTT FUNCTIONS ============

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("📨 MQTT [");
    Serial.print(topic);
    Serial.print("]: ");
    
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println(message);
}

bool reconnectMQTT() {
    unsigned long now = millis();
    
    // Throttle reconnection attempts
    if (now - lastReconnectMQTT < MQTT_RECONNECT_INTERVAL) {
        return false;
    }
    lastReconnectMQTT = now;
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("❌ WiFi not connected for MQTT");
        return false;
    }
    
    if (mqttClient.connected()) {
        return true;
    }
    
    // Clean broker URL
    String broker = currentConfig.serverUrl;
    broker.replace("mqtt://", "");
    broker.replace("mqtts://", "");
    broker.replace("http://", "");
    broker.replace("https://", "");
    
    // Remove any port from broker URL
    int colonPos = broker.indexOf(":");
    if (colonPos > 0) {
        broker = broker.substring(0, colonPos);
    }
    
    Serial.println("\n⚡ Connecting to MQTT...");
    Serial.println("🌐 Broker: " + broker + ":" + String(currentConfig.serverPort));
    
    // Set MQTT server - CRITICAL FIX!
    mqttClient.setServer(broker.c_str(), currentConfig.serverPort);
    
    // Generate unique client ID
    String clientId = "ESP32-" + currentConfig.nfcId + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.println("🆔 Client: " + clientId);
    
    // Setup MQTT topics
    String statusTopic = currentConfig.mqttTopic.length() > 0 ? 
                         currentConfig.mqttTopic + "/status" : 
                         "sensors/" + currentConfig.deviceName + "/status";
    
    String cmdTopic = currentConfig.mqttTopic.length() > 0 ? 
                      currentConfig.mqttTopic + "/cmd" : 
                      "sensors/" + currentConfig.deviceName + "/cmd";
    
    bool connected = false;
    
    // Connect with Last Will Testament
    if (currentConfig.mqttUsername.length() > 0 && currentConfig.mqttPassword.length() > 0) {
        Serial.println("👤 Auth: " + currentConfig.mqttUsername);
        connected = mqttClient.connect(
            clientId.c_str(),
            currentConfig.mqttUsername.c_str(),
            currentConfig.mqttPassword.c_str(),
            statusTopic.c_str(),
            1,
            true,
            "offline"
        );
    } else {
        Serial.println("🔓 No auth (Public broker)");
        connected = mqttClient.connect(
            clientId.c_str(),
            statusTopic.c_str(),
            1,
            true,
            "offline"
        );
    }
    
    if (connected) {
        Serial.println("✅ MQTT Connected!");
        
        // Publish online status
        mqttClient.publish(statusTopic.c_str(), "online", true);
        
        // Subscribe to command topic
        if (mqttClient.subscribe(cmdTopic.c_str())) {
            Serial.println("📬 Subscribed: " + cmdTopic);
        }
        
        // Publish initial info
        JsonDocument doc;
        doc["device"] = currentConfig.deviceName;
        doc["id"] = currentConfig.nfcId;
        doc["sensor"] = currentConfig.sensorType;
        doc["interval"] = currentConfig.interval;
        doc["ip"] = WiFi.localIP().toString();
        
        char buffer[256];
        serializeJson(doc, buffer);
        
        String infoTopic = currentConfig.mqttTopic.length() > 0 ? 
                          currentConfig.mqttTopic + "/info" : 
                          "sensors/" + currentConfig.deviceName + "/info";
        
        mqttClient.publish(infoTopic.c_str(), buffer, true);
        
        return true;
    } else {
        int state = mqttClient.state();
        Serial.print("❌ Failed, rc=");
        Serial.print(state);
        Serial.print(" (");
        
        switch(state) {
            case -4: Serial.print("TIMEOUT"); break;
            case -3: Serial.print("CONNECTION_LOST"); break;
            case -2: Serial.print("CONNECT_FAILED"); break;
            case -1: Serial.print("DISCONNECTED"); break;
            case 1: Serial.print("BAD_PROTOCOL"); break;
            case 2: Serial.print("BAD_CLIENT_ID"); break;
            case 3: Serial.print("UNAVAILABLE"); break;
            case 4: Serial.print("BAD_CREDENTIALS"); break;
            case 5: Serial.print("UNAUTHORIZED"); break;
            default: Serial.print("UNKNOWN");
        }
        Serial.println(")");
        
        return false;
    }
}

bool sendDataViaMQTT(String jsonData) {
    if (!mqttClient.connected()) {
        Serial.println("⚠️ MQTT disconnected, reconnecting...");
        if (!reconnectMQTT()) {
            return false;
        }
    }
    
    String topic = currentConfig.mqttTopic.length() > 0 ? 
                   currentConfig.mqttTopic : 
                   "sensors/" + currentConfig.deviceName;
    
    Serial.println("\n📡 MQTT Publish");
    Serial.println("📬 Topic: " + topic);
    
    bool published = mqttClient.publish(topic.c_str(), jsonData.c_str(), true);
    
    if (published) {
        Serial.println("✅ Published");
        return true;
    } else {
        Serial.println("❌ Publish failed");
        return false;
    }
}

// ============ SERIAL OUTPUT ============

bool sendDataViaSerial(String jsonData) {
    Serial.println("\n📡 Serial Output");
    Serial.println("📦 " + jsonData);
    return true;
}

// ============ UNIFIED SEND FUNCTION ============

bool sendSensorData() {
    if (!wifiConnected && currentConfig.protocol != "SERIAL") {
        Serial.println("❌ WiFi not connected");
        return false;
    }
    
    float temp, hum;
    readSensorData(temp, hum);
    String jsonData = createSensorJSON(temp, hum);
    
    Serial.println("\n📊 === SENSOR DATA ===");
    Serial.println("🌡️ Temp: " + String(temp, 1) + "°C");
    Serial.println("💧 Hum: " + String(hum, 1) + "%");
    Serial.println("📋 Protocol: " + currentConfig.protocol);
    
    bool success = false;
    String proto = currentConfig.protocol;
    proto.toUpperCase();
    
    if (proto == "HTTP") {
        success = sendDataViaHTTP(jsonData, false);
    } 
    else if (proto == "HTTPS") {
        success = sendDataViaHTTP(jsonData, true);
    }
    else if (proto == "MQTT" || proto == "MQTTS") {
        success = sendDataViaMQTT(jsonData);
    }
    else if (proto == "SERIAL") {
        success = sendDataViaSerial(jsonData);
    }
    else {
        Serial.println("⚠️ Unknown protocol: " + proto);
        Serial.println("💡 Valid: HTTP, HTTPS, MQTT, MQTTS, SERIAL");
        return false;
    }
    
    if (success) {
        Serial.println("✅ Sent via " + currentConfig.protocol);
    } else {
        Serial.println("❌ Failed via " + currentConfig.protocol);
    }
    
    return success;
}

// ============ MQTT HEARTBEAT ============

void publishHeartbeat() {
    if (!mqttClient.connected()) return;
    
    String statusTopic = currentConfig.mqttTopic.length() > 0 ? 
                         currentConfig.mqttTopic + "/status" : 
                         "sensors/" + currentConfig.deviceName + "/status";
    
    JsonDocument doc;
    doc["type"] = "heartbeat";
    doc["uptime"] = millis() / 1000;
    doc["rssi"] = WiFi.RSSI();
    doc["ip"] = WiFi.localIP().toString();
    doc["free_heap"] = ESP.getFreeHeap();
    
    char buffer[200];
    serializeJson(doc, buffer);
    
    if (mqttClient.publish(statusTopic.c_str(), buffer)) {
        Serial.println("💓 Heartbeat sent");
    }
}

// ============ NFC FUNCTIONS ============

bool parseJSONConfig(const char* jsonData) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
        Serial.print(F("❌ JSON parse error: "));
        Serial.println(error.f_str());
        return false;
    }

    currentConfig.nfcId = doc["nfcId"] | "UNKNOWN";
    currentConfig.deviceName = doc["deviceName"] | "sensor";
    currentConfig.wifi = doc["wifi"] | "";
    currentConfig.wifiPassword = doc["wifiPassword"] | "";
    currentConfig.serverUrl = doc["serverUrl"] | "";
    currentConfig.serverPort = doc["serverPort"] | 0;
    currentConfig.sensorType = doc["sensorType"] | "generic";
    currentConfig.interval = doc["interval"] | DEFAULT_SENSOR_INTERVAL;
    currentConfig.deviceStatus = doc["deviceStatus"] | "active";
    currentConfig.timestamp = doc["timestamp"] | "";
    currentConfig.version = doc["version"] | "1.0";
    currentConfig.protocol = doc["protocol"] | "HTTP";
    currentConfig.mqttTopic = doc["mqttTopic"] | "";
    currentConfig.mqttUsername = doc["mqttUsername"] | "";
    currentConfig.mqttPassword = doc["mqttPassword"] | "";
    currentConfig.apiKey = doc["apiKey"] | "";
    currentConfig.endpoint = doc["endpoint"] | "";

    // Validation
    if (currentConfig.protocol != "SERIAL") {
        if (currentConfig.wifi.length() == 0) {
            Serial.println("❌ WiFi SSID required for network protocols");
            return false;
        }
        if (currentConfig.serverUrl.length() == 0) {
            Serial.println("❌ Server URL required");
            return false;
        }
    }

    // Auto-set default ports
    if (currentConfig.serverPort == 0) {
        if (currentConfig.protocol == "HTTP") currentConfig.serverPort = 80;
        else if (currentConfig.protocol == "HTTPS") currentConfig.serverPort = 443;
        else if (currentConfig.protocol == "MQTT") currentConfig.serverPort = 1883;
        else if (currentConfig.protocol == "MQTTS") currentConfig.serverPort = 8883;
    }

    Serial.println("\n=== 📋 CONFIG LOADED ===");
    Serial.println("🆔 ID: " + currentConfig.nfcId);
    Serial.println("📱 Device: " + currentConfig.deviceName);
    Serial.println("📡 WiFi: " + currentConfig.wifi);
    Serial.println("🌐 Server: " + currentConfig.serverUrl + ":" + String(currentConfig.serverPort));
    Serial.println("📊 Sensor: " + currentConfig.sensorType);
    Serial.println("⏱️ Interval: " + String(currentConfig.interval) + "ms");
    Serial.println("📡 Protocol: " + currentConfig.protocol);
    
    if (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS") {
        Serial.println("📬 Topic: " + (currentConfig.mqttTopic.length() > 0 ? currentConfig.mqttTopic : "sensors/" + currentConfig.deviceName));
        if (currentConfig.mqttUsername.length() > 0) {
            Serial.println("👤 User: " + currentConfig.mqttUsername);
        } else {
            Serial.println("🔓 Public broker (no auth)");
        }
    }
    
    Serial.println("========================\n");

    return true;
}

String getUIDString(uint8_t uid[], uint8_t uidLength) {
    String uidStr = "";
    for (uint8_t i = 0; i < uidLength; i++) {
        if (uid[i] < 0x10) uidStr += "0";
        uidStr += String(uid[i], HEX);
        if (i < uidLength - 1) uidStr += ":";
    }
    uidStr.toUpperCase();
    return uidStr;
}

void releaseNFCField() {
    Serial.println("\n📱 RELEASE NFC FIELD");
    nfc.begin();
    delay(RELEASE_DURATION);
    nfc.SAMConfig();
    Serial.println("🔄 Field reactivated");
    lastReleaseTime = millis();
}

String readNTAGData() {
    String allData = "";
    uint8_t data[4];
    int consecutiveFailures = 0;
    int consecutiveZeros = 0;
    
    for (uint8_t page = 4; page < 231 && consecutiveFailures < 3; page++) {
        if (nfc.ntag2xx_ReadPage(page, data)) {
            consecutiveFailures = 0;
            
            bool allZero = true;
            for (int i = 0; i < 4; i++) {
                if (data[i] != 0x00) {
                    allZero = false;
                    break;
                }
            }
            
            if (allZero) {
                consecutiveZeros++;
                if (consecutiveZeros >= 3) break;
            } else {
                consecutiveZeros = 0;
                for (int i = 0; i < 4; i++) {
                    if (data[i] >= 0x20 && data[i] <= 0x7E) {
                        allData += (char)data[i];
                    } else if (data[i] == 0x00) {
                        break;
                    }
                }
            }
        } else {
            consecutiveFailures++;
        }
    }
    
    return allData;
}

String extractJSON(String rawData) {
    int jsonStart = rawData.indexOf('{');
    if (jsonStart == -1) return "";
    
    int braceCount = 0;
    int jsonEnd = -1;
    
    for (int i = jsonStart; i < rawData.length(); i++) {
        if (rawData[i] == '{') {
            braceCount++;
        } else if (rawData[i] == '}') {
            braceCount--;
            if (braceCount == 0) {
                jsonEnd = i;
                break;
            }
        }
    }
    
    if (jsonEnd > jsonStart) {
        return rawData.substring(jsonStart, jsonEnd + 1);
    }
    
    return "";
}

bool connectToWiFi() {
    if (currentConfig.wifi.length() == 0) {
        Serial.println("❌ No WiFi SSID");
        return false;
    }
    
    Serial.println("\n🔄 WiFi connecting...");
    Serial.println("📡 SSID: " + currentConfig.wifi);
    
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect();
        delay(1000);
    }
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    
    if (currentConfig.wifiPassword.length() > 0) {
        WiFi.begin(currentConfig.wifi.c_str(), currentConfig.wifiPassword.c_str());
    } else {
        WiFi.begin(currentConfig.wifi.c_str());
    }
    
    unsigned long startTime = millis();
    int dots = 0;
    
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
        dots++;
        if (dots % 10 == 0) Serial.println();
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected!");
        Serial.println("📶 IP: " + WiFi.localIP().toString());
        Serial.println("📡 RSSI: " + String(WiFi.RSSI()) + " dBm");
        wifiConnected = true;
        return true;
    } else {
        Serial.println("\n❌ WiFi failed!");
        wifiConnected = false;
        return false;
    }
}

void processNFCTag(String currentUID) {
    Serial.println("\n" + String('=', 50));
    Serial.println("🏷️ PROCESSING NFC TAG");
    Serial.println(String('=', 50));
    Serial.println("🆔 UID: " + currentUID);
    
    String rawData = readNTAGData();
    
    if (rawData.length() > 0) {
        Serial.println("📊 Data: " + String(rawData.length()) + " bytes");
        
        if (rawData != lastRawData) {
            Serial.println("🔄 New data!");
            lastRawData = rawData;
            
            String jsonData = extractJSON(rawData);
            
            if (jsonData.length() > 0) {
                Serial.println("🎉 JSON found!");
                Serial.println("📝 JSON: " + jsonData);
                
                if (parseJSONConfig(jsonData.c_str())) {
                    configurationValid = true;
                    
                    if (currentConfig.protocol != "SERIAL") {
                        if (connectToWiFi()) {
                            delay(1000);
                            
                            if (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS") {
                                Serial.println("\n🔌 Initializing MQTT...");
                                reconnectMQTT();
                            }
                        }
                    } else {
                        Serial.println("📡 Serial mode - no WiFi needed");
                    }
                    
                    lastSensorSend = millis();
                } else {
                    Serial.println("❌ Config parse failed");
                    configurationValid = false;
                }
            } else {
                Serial.println("❌ No valid JSON found in NFC data");
                Serial.println("📄 Raw data: " + rawData);
                configurationValid = false;
            }
        } else {
            Serial.println("ℹ️ Same data as before");
        }
    } else {
        Serial.println("❌ No data read from NFC");
    }
}

// ============ SETUP ============

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n🚀 ESP32 NFC Multi-Protocol v2.2");
    Serial.println("====================================");
    Serial.println("✨ Protocols:");
    Serial.println("   • HTTP / HTTPS");
    Serial.println("   • MQTT / MQTTS");
    Serial.println("   • Serial Output");
    Serial.println("====================================");
    
    nfc.begin();
    
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
        Serial.println("❌ PN532 not found!");
        while (1) delay(1000);
    }
    
    Serial.print("✅ Found PN5");
    Serial.println((versiondata>>24) & 0xFF, HEX);
    
    nfc.SAMConfig();
    WiFi.mode(WIFI_STA);
    
    // Setup MQTT client
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(15);
    
    Serial.println("\n⏳ Ready! Place NFC tag...");
    Serial.println("💡 Type 'help' for commands\n");
    
    lastReleaseTime = millis();
    lastWifiCheck = millis();
    lastSensorSend = millis();
    lastHeartbeat = millis();
}

// ============ LOOP ============

void loop() {
    unsigned long currentTime = millis();
    
    // Field release
    if (currentTime - lastReleaseTime >= RELEASE_INTERVAL) {
        releaseNFCField();
        return;
    }
    
    // WiFi check
    if (configurationValid && currentConfig.protocol != "SERIAL" && 
        currentTime - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
        if (WiFi.status() != WL_CONNECTED && wifiConnected) {
            Serial.println("\n⚠️ WiFi lost, reconnecting...");
            connectToWiFi();
            
            if (wifiConnected && (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS")) {
                reconnectMQTT();
            }
        }
        lastWifiCheck = currentTime;
    }
    
    // MQTT maintain
    if (configurationValid && wifiConnected && 
        (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS")) {
        if (!mqttClient.connected()) {
            reconnectMQTT();
        } else {
            mqttClient.loop();
        }
    }
    
    // Send sensor data
    if (configurationValid && currentTime - lastSensorSend >= getSensorInterval()) {
        if (currentConfig.protocol == "SERIAL" || wifiConnected) {
            sendSensorData();
        }
        lastSensorSend = currentTime;
    }
    
    // Heartbeat for MQTT
    if (configurationValid && mqttClient.connected() && 
        currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        publishHeartbeat();
        lastHeartbeat = currentTime;
    }
    
    // NFC scanning
    if (currentTime - lastCheckTime < CHECK_INTERVAL) {
        delay(10);
        return;
    }
    
    lastCheckTime = currentTime;
    
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;
    
    bool tagDetected = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100);
    
    if (tagDetected) {
        String currentUID = getUIDString(uid, uidLength);
        
        if (!tagPresent) {
            Serial.println("\n🆕 NEW TAG!");
            tagPresent = true;
            lastUID = currentUID;
            processNFCTag(currentUID);
        } else if (currentUID != lastUID) {
            Serial.println("\n🔄 DIFFERENT TAG!");
            lastUID = currentUID;
            lastRawData = "";
            processNFCTag(currentUID);
        }
    } else {
        if (tagPresent) {
            Serial.println("\n❌ TAG REMOVED");
            tagPresent = false;
            lastUID = "";
        }
    }
    
    // Serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();
        
        if (cmd == "status") {
            Serial.println("\n📊 === STATUS ===");
            Serial.println("Tag: " + String(tagPresent ? "YES" : "NO"));
            Serial.println("Config: " + String(configurationValid ? "VALID" : "INVALID"));
            Serial.println("WiFi: " + String(wifiConnected ? "CONNECTED" : "DISCONNECTED"));
            
            if (wifiConnected) {
                Serial.println("IP: " + WiFi.localIP().toString());
                Serial.println("RSSI: " + String(WiFi.RSSI()) + " dBm");
            }
            
            if (configurationValid) {
                Serial.println("\n📡 Protocol: " + currentConfig.protocol);
                Serial.println("🌐 Server: " + currentConfig.serverUrl + ":" + String(currentConfig.serverPort));
                Serial.println("⏱️ Interval: " + String(currentConfig.interval) + "ms");
                
                if (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS") {
                    Serial.println("📬 MQTT: " + String(mqttClient.connected() ? "CONNECTED ✅" : "DISCONNECTED ❌"));
                    if (!mqttClient.connected()) {
                        int state = mqttClient.state();
                        Serial.print("   State: ");
                        switch(state) {
                            case -4: Serial.println("TIMEOUT"); break;
                            case -3: Serial.println("CONNECTION_LOST"); break;
                            case -2: Serial.println("CONNECT_FAILED"); break;
                            case -1: Serial.println("DISCONNECTED"); break;
                            case 1: Serial.println("BAD_PROTOCOL"); break;
                            case 2: Serial.println("BAD_CLIENT_ID"); break;
                            case 3: Serial.println("UNAVAILABLE"); break;
                            case 4: Serial.println("BAD_CREDENTIALS"); break;
                            case 5: Serial.println("UNAUTHORIZED"); break;
                            default: Serial.println("UNKNOWN (" + String(state) + ")");
                        }
                    }
                }
            }
            Serial.println("=================");
            
        } else if (cmd == "send") {
            if (configurationValid) {
                Serial.println("\n📤 Manual send test...");
                sendSensorData();
            } else {
                Serial.println("\n❌ No valid configuration");
            }
            
        } else if (cmd == "wifi") {
            if (configurationValid && currentConfig.protocol != "SERIAL") {
                Serial.println("\n🔄 Reconnecting WiFi...");
                connectToWiFi();
            } else {
                Serial.println("\n❌ No config or Serial mode");
            }
            
        } else if (cmd == "mqtt") {
            if (configurationValid && (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS")) {
                Serial.println("\n🔄 Reconnecting MQTT...");
                if (wifiConnected) {
                    if (mqttClient.connected()) {
                        mqttClient.disconnect();
                        delay(1000);
                    }
                    reconnectMQTT();
                } else {
                    Serial.println("❌ WiFi not connected first");
                }
            } else {
                Serial.println("\n❌ Not using MQTT protocol");
            }
            
        } else if (cmd == "config") {
            if (configurationValid) {
                Serial.println("\n📋 === CONFIGURATION ===");
                Serial.println("🆔 NFC ID: " + currentConfig.nfcId);
                Serial.println("📱 Device: " + currentConfig.deviceName);
                Serial.println("📡 WiFi SSID: " + currentConfig.wifi);
                Serial.println("🌐 Server: " + currentConfig.serverUrl);
                Serial.println("🔌 Port: " + String(currentConfig.serverPort));
                Serial.println("📡 Protocol: " + currentConfig.protocol);
                Serial.println("📊 Sensor: " + currentConfig.sensorType);
                Serial.println("⏱️ Interval: " + String(currentConfig.interval) + "ms");
                
                if (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS") {
                    Serial.println("\n📬 MQTT Topic: " + (currentConfig.mqttTopic.length() > 0 ? currentConfig.mqttTopic : "sensors/" + currentConfig.deviceName));
                    Serial.println("👤 Username: " + (currentConfig.mqttUsername.length() > 0 ? currentConfig.mqttUsername : "(none - public)"));
                }
                
                if (currentConfig.endpoint.length() > 0) {
                    Serial.println("🎯 Endpoint: " + currentConfig.endpoint);
                }
                
                Serial.println("📦 Version: " + currentConfig.version);
                Serial.println("========================");
            } else {
                Serial.println("\n❌ No valid configuration loaded");
            }
            
        } else if (cmd == "release") {
            Serial.println("\n🎛️ Manual NFC field release");
            releaseNFCField();
            
        } else if (cmd == "test") {
            Serial.println("\n🧪 === DIAGNOSTIC TEST ===");
            Serial.println("1️⃣ WiFi Status: " + String(WiFi.status()));
            Serial.println("2️⃣ Config Valid: " + String(configurationValid));
            
            if (configurationValid) {
                Serial.println("3️⃣ Protocol: " + currentConfig.protocol);
                Serial.println("4️⃣ Server: " + currentConfig.serverUrl);
                Serial.println("5️⃣ Port: " + String(currentConfig.serverPort));
                
                if (currentConfig.protocol == "MQTT" || currentConfig.protocol == "MQTTS") {
                    Serial.println("6️⃣ MQTT Connected: " + String(mqttClient.connected()));
                    Serial.println("7️⃣ MQTT State: " + String(mqttClient.state()));
                }
            }
            Serial.println("========================");
            
        } else if (cmd == "help") {
            Serial.println("\n📖 === COMMANDS ===");
            Serial.println("status   - Show system status");
            Serial.println("send     - Send test sensor data");
            Serial.println("wifi     - Reconnect WiFi");
            Serial.println("mqtt     - Reconnect MQTT broker");
            Serial.println("config   - Display configuration");
            Serial.println("release  - Release NFC field");
            Serial.println("test     - Run diagnostic test");
            Serial.println("help     - Show this help");
            Serial.println("===================");
            
        } else if (cmd.length() > 0) {
            Serial.println("\n❓ Unknown command: '" + cmd + "'");
            Serial.println("💡 Type 'help' for available commands");
        }
    }
}
