#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Adafruit_PN532.h>
#include <ArduinoJson.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_PN532 nfc(SDA_PIN, SCL_PIN);

// Struktur untuk menyimpan konfigurasi
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
};

// Variabel global
NFCConfig currentConfig;
String lastRawData = "";
String lastUID = "";
bool tagPresent = false;
bool wifiConnected = false;
bool serverConnected = false;
bool configurationValid = false;

// Timing variabel
unsigned long lastCheckTime = 0;
unsigned long lastReleaseTime = 0;
unsigned long lastWifiCheck = 0;
unsigned long lastServerCheck = 0;
unsigned long lastSensorSend = 0;

// Konfigurasi timing
const unsigned long CHECK_INTERVAL = 2000;     // Cek NFC setiap 2 detik
const unsigned long RELEASE_INTERVAL = 8000;   // Release field setiap 8 detik
const unsigned long RELEASE_DURATION = 1500;   // Release selama 1.5 detik
const unsigned long WIFI_CHECK_INTERVAL = 10000;  // Cek WiFi setiap 10 detik
const unsigned long SERVER_CHECK_INTERVAL = 1000; // Cek server setiap 15 detik
const unsigned long DEFAULT_SENSOR_INTERVAL = 5000; // Default interval jika tidak ada config
const unsigned long WIFI_TIMEOUT = 20000;      // Timeout WiFi 20 detik

// Fungsi untuk mendapatkan interval sensor yang aktif
unsigned long getSensorInterval() {
    if (configurationValid && currentConfig.interval > 0) {
        return currentConfig.interval;
    }
    return DEFAULT_SENSOR_INTERVAL;
}

// Fungsi untuk memparsing data JSON dan mengisi struktur konfigurasi
bool parseJSONConfig(const char* jsonData) {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, jsonData);

    if (error) {
        Serial.print(F("❌ JSON parsing failed: "));
        Serial.println(error.f_str());
        return false;
    }

    // Simpan interval lama untuk perbandingan
    int oldInterval = currentConfig.interval;

    // Mengisi struktur konfigurasi
    currentConfig.nfcId = doc["nfcId"].as<String>();
    currentConfig.deviceName = doc["deviceName"].as<String>();
    currentConfig.wifi = doc["wifi"].as<String>();
    currentConfig.wifiPassword = doc["wifiPassword"].as<String>();
    currentConfig.serverUrl = doc["serverUrl"].as<String>();
    currentConfig.serverPort = doc["serverPort"].as<int>();
    currentConfig.sensorType = doc["sensorType"].as<String>();
    currentConfig.interval = doc["interval"].as<int>();
    currentConfig.deviceStatus = doc["deviceStatus"].as<String>();
    currentConfig.timestamp = doc["timestamp"].as<String>();
    currentConfig.version = doc["version"].as<String>();

    // Validasi data penting
    if (currentConfig.wifi.length() == 0 || currentConfig.serverUrl.length() == 0) {
        Serial.println("❌ Critical configuration missing (WiFi SSID or Server URL)");
        return false;
    }

    // Validasi interval
    if (currentConfig.interval <= 0) {
        Serial.println("⚠️ Invalid interval value, using default: " + String(DEFAULT_SENSOR_INTERVAL) + "ms");
        currentConfig.interval = DEFAULT_SENSOR_INTERVAL;
    }

    // Cek apakah interval berubah
    if (oldInterval != currentConfig.interval && oldInterval > 0) {
        Serial.println("🔄 Sensor interval changed from " + String(oldInterval) + "ms to " + String(currentConfig.interval) + "ms");
        // Reset timer untuk segera menerapkan interval baru
        lastSensorSend = 0;
    }

    // Tampilkan konfigurasi
    Serial.println("\n=== 📋 NFC Configuration Loaded ===");
    Serial.println("🆔 NFC ID: " + currentConfig.nfcId);
    Serial.println("📱 Device Name: " + currentConfig.deviceName);
    Serial.println("📡 WiFi SSID: " + currentConfig.wifi);
    Serial.print("🔐 WiFi Password: ");
    Serial.println(currentConfig.wifiPassword.length() > 0 ? "***" : "No password");
    Serial.println("🌐 Server URL: " + currentConfig.serverUrl);
    Serial.println("🔌 Server Port: " + String(currentConfig.serverPort));
    Serial.println("📊 Sensor Type: " + currentConfig.sensorType);
    Serial.println("⏱️ Sensor Interval: " + String(currentConfig.interval) + "ms (" + String(currentConfig.interval/1000.0, 1) + "s)");
    Serial.println("🔄 Device Status: " + currentConfig.deviceStatus);
    Serial.println("📅 Timestamp: " + currentConfig.timestamp);
    Serial.println("🏷️ Version: " + currentConfig.version);
    Serial.println("=====================================\n");

    return true;
}

// Fungsi untuk membuat UID string
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

// Fungsi untuk release NFC field
void releaseNFCField() {
    Serial.println("📱 RELEASING NFC FIELD - Smartphone access enabled!");
    
    // Reset dan reinitialize untuk release field
    nfc.begin();
    
    Serial.println("⏳ Field released for " + String(RELEASE_DURATION/1000) + " seconds...");
    delay(RELEASE_DURATION);
    
    // Reconfigure
    nfc.SAMConfig();
    
    Serial.println("🔄 NFC field reactivated");
    lastReleaseTime = millis();
}

// Fungsi untuk membaca data NTAG dengan metode yang lebih robust
String readNTAGData() {
    String allData = "";
    uint8_t data[4];
    int consecutiveFailures = 0;
    int consecutiveZeros = 0;
    
    // Mulai dari page 4 (user data area untuk NTAG)
    for (uint8_t page = 4; page < 231 && consecutiveFailures < 3; page++) {
        if (nfc.ntag2xx_ReadPage(page, data)) {
            consecutiveFailures = 0;
            
            // Cek apakah semua data adalah 0x00
            bool allZero = true;
            for (int i = 0; i < 4; i++) {
                if (data[i] != 0x00) {
                    allZero = false;
                    break;
                }
            }
            
            if (allZero) {
                consecutiveZeros++;
                if (consecutiveZeros >= 3) {
                    break; // Stop jika terlalu banyak page kosong berturut-turut
                }
            } else {
                consecutiveZeros = 0;
                
                // Tambahkan data yang valid
                for (int i = 0; i < 4; i++) {
                    if (data[i] >= 0x20 && data[i] <= 0x7E) {
                        // Karakter printable ASCII
                        allData += (char)data[i];
                    } else if (data[i] == 0x00) {
                        // Null terminator, berhenti
                        break;
                    } else {
                        // Non-printable character, tambahkan sebagai raw data
                        allData += (char)data[i];
                    }
                }
            }
        } else {
            consecutiveFailures++;
        }
    }
    
    return allData;
}

// Fungsi untuk mencari dan ekstrak JSON dari raw data
String extractJSON(String rawData) {
    // Cari posisi awal JSON
    int jsonStart = rawData.indexOf('{');
    if (jsonStart == -1) return "";
    
    // Cari posisi akhir JSON dengan menghitung bracket
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

// Fungsi untuk menghubungkan ke WiFi
bool connectToWiFi() {
    if (currentConfig.wifi.length() == 0) {
        Serial.println("❌ No WiFi SSID configured");
        return false;
    }
    
    Serial.println("\n🔄 Attempting WiFi connection...");
    Serial.println("📡 SSID: " + currentConfig.wifi);
    
    // Disconnect jika sudah terhubung
    if (WiFi.status() == WL_CONNECTED) {
        WiFi.disconnect();
        delay(1000);
    }
    
    // Mulai koneksi
    if (currentConfig.wifiPassword.length() > 0) {
        WiFi.begin(currentConfig.wifi.c_str(), currentConfig.wifiPassword.c_str());
    } else {
        WiFi.begin(currentConfig.wifi.c_str());
    }
    
    // Wait untuk koneksi dengan timeout
    unsigned long startTime = millis();
    int dots = 0;
    
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT) {
        delay(500);
        Serial.print(".");
        dots++;
        if (dots >= 10) {
            Serial.println();
            dots = 0;
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected successfully!");
        Serial.println("📶 IP Address: " + WiFi.localIP().toString());
        Serial.println("📡 Signal Strength: " + String(WiFi.RSSI()) + " dBm");
        wifiConnected = true;
        return true;
    } else {
        Serial.println("\n❌ WiFi connection failed!");
        Serial.println("🔍 Status: " + String(WiFi.status()));
        wifiConnected = false;
        return false;
    }
}

// Fungsi untuk test koneksi server
bool testServerConnection() {
    if (!wifiConnected || currentConfig.serverUrl.length() == 0) {
        return false;
    }
    
    Serial.println("\n🌐 Testing server connection...");
    Serial.println("🔗 URL: " + currentConfig.serverUrl + ":" + String(currentConfig.serverPort) +"/" + currentConfig.deviceName);
    
    HTTPClient http;
    String fullUrl = currentConfig.serverUrl;
    if (currentConfig.serverPort > 0) {
        fullUrl += ":" + String(currentConfig.serverPort) + "/" + currentConfig.deviceName;
    }
    
    // Test dengan endpoint sederhana
    http.begin(fullUrl + "/");
    http.setTimeout(10000);
    
    int httpResponseCode = http.GET();
    
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("✅ Server connection successful!");
        // Serial.println("📊 Response Code: " + String(httpResponseCode));
        // Serial.println("📄 Response: " + response.substring(0, 100) + (response.length() > 100 ? "..." : ""));
        serverConnected = true;
        http.end();
        return true;
    } else {
        Serial.println("❌ Server connection failed!");
        Serial.println("🔍 Error: " + String(httpResponseCode));
        Serial.println("📝 Error Description: " + http.errorToString(httpResponseCode));
        serverConnected = false;
        http.end();
        return false;
    }
}

// Fungsi untuk membaca sensor (simulasi)
void readSensorData(float &temp, float &hum) {
    // Simulasi pembacaan sensor suhu dan kelembaban
    temp = random(250, 350) / 10.0; // suhu antara 25.0 - 35.0°C
    hum = random(600, 900) / 10.0;  // kelembaban antara 60.0 - 90.0%
}

// Fungsi untuk mengirim data sensor ke Node-RED
bool sendSensorDataToNodeRED() {
    if (!wifiConnected || currentConfig.serverUrl.length() == 0) {
        Serial.println("❌ WiFi not connected or no server URL configured");
        return false;
    }
    
    // Baca data sensor
    float suhu, kelembaban;
    readSensorData(suhu, kelembaban);
    
    HTTPClient http;
    String fullUrl = currentConfig.serverUrl;
    if (currentConfig.serverPort > 0) {
        // fullUrl += ":" + String(currentConfig.serverPort);
        fullUrl += ":" + String(currentConfig.serverPort) + "/" + currentConfig.deviceName;
    }
    
    // Gunakan endpoint yang sama seperti contoh Node-RED (/esp32)
    http.begin(fullUrl);
    http.addHeader("Content-Type", "application/json");
    
    // Buat JSON payload sesuai format Node-RED dengan informasi tambahan
    DynamicJsonDocument jsonDoc(512);
    jsonDoc["temp"] = String(suhu, 1).toFloat();
    jsonDoc["hum"] = String(kelembaban, 1).toFloat();
    jsonDoc["deviceId"] = currentConfig.nfcId;
    jsonDoc["deviceName"] = currentConfig.deviceName;
    jsonDoc["sensorType"] = currentConfig.sensorType;
    jsonDoc["interval"] = currentConfig.interval;
    jsonDoc["timestamp"] = millis();
    
    String json;
    serializeJson(jsonDoc, json);
    
    Serial.print("📤 Kirim data (interval " + String(currentConfig.interval) + "ms): ");
    Serial.println(json);
    
    int httpResponseCode = http.POST(json);
    
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.print("✅ Response code: ");
        Serial.println(httpResponseCode);
        Serial.print("📥 Response: ");
        Serial.println(response);
        http.end();
        return true;
    } else {
        Serial.print("❌ HTTP error: ");
        Serial.println(httpResponseCode);
        http.end();
        return false;
    }
}

// Fungsi untuk mengirim data ke server (format lengkap)
bool sendDataToServer(String data) {
    if (!serverConnected || currentConfig.serverUrl.length() == 0) {
        return false;
    }
    
    HTTPClient http;
    String fullUrl = currentConfig.serverUrl;
    if (currentConfig.serverPort > 0) {
        fullUrl += ":" + String(currentConfig.serverPort);
    }
    
    http.begin(fullUrl + "/api/data");
    http.addHeader("Content-Type", "application/json");
    
    // Buat payload JSON
    DynamicJsonDocument payload(512);
    payload["deviceId"] = currentConfig.nfcId;
    payload["deviceName"] = currentConfig.deviceName;
    payload["sensorType"] = currentConfig.sensorType;
    payload["data"] = data;
    payload["interval"] = currentConfig.interval;
    payload["timestamp"] = millis();
    
    String jsonString;
    serializeJson(payload, jsonString);
    
    int httpResponseCode = http.POST(jsonString);
    
    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("📤 Data sent successfully! Response: " + String(httpResponseCode));
        http.end();
        return true;
    } else {
        Serial.println("❌ Failed to send data: " + String(httpResponseCode));
        http.end();
        return false;
    }
}

// Fungsi untuk memproses tag NFC yang terdeteksi
void processNFCTag(String currentUID) {
    Serial.println("\n" + String("=").substring(0,60));
    Serial.println("🏷️ PROCESSING NFC TAG");
    Serial.println(String("=").substring(0,60));
    Serial.println("🆔 UID: " + currentUID);
    
    // Baca data dari NTAG
    String rawData = readNTAGData();
    
    if (rawData.length() > 0) {
        Serial.println("📊 Raw data length: " + String(rawData.length()) + " bytes");
        
        // Cek apakah data berubah
        if (rawData != lastRawData) {
            Serial.println("🔄 New or changed data detected!");
            lastRawData = rawData;
            
            // Ekstrak JSON
            String jsonData = extractJSON(rawData);
            
            if (jsonData.length() > 0) {
                Serial.println("🎉 JSON extracted successfully!");
                
                // Parse konfigurasi
                if (parseJSONConfig(jsonData.c_str())) {
                    configurationValid = true;
                    
                    // Auto-connect ke WiFi
                    if (connectToWiFi()) {
                        // Test koneksi server
                        delay(2000); // Berikan waktu untuk stabilitas koneksi
                        testServerConnection();
                    }
                    
                    // Reset sensor timer untuk menggunakan interval baru
                    lastSensorSend = millis();
                    Serial.println("🎯 Sensor interval set to: " + String(currentConfig.interval) + "s");
                } else {
                    Serial.println("❌ Failed to parse configuration");
                    configurationValid = false;
                }
            } else {
                Serial.println("❌ No valid JSON found in tag data");
                configurationValid = false;
            }
        } else {
            Serial.println("ℹ️ Same data as before, skipping processing");
        }
    } else {
        Serial.println("❌ No data read from tag");
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n🚀 ESP32 NFC Auto-Configuration System");
    Serial.println("========================================");
    Serial.println("✨ Features:");
    Serial.println("   • Auto WiFi connection from NFC data");
    Serial.println("   • Server communication setup");
    Serial.println("   • Connection monitoring & recovery");
    Serial.println("   • Smartphone-compatible field release");
    Serial.println("   • Dynamic sensor interval from NFC config");
    Serial.println("⚙️ Configuration:");
    Serial.println("   • NFC check: " + String(CHECK_INTERVAL/1000) + "s");
    Serial.println("   • Field release: " + String(RELEASE_INTERVAL/1000) + "s");
    Serial.println("   • WiFi monitor: " + String(WIFI_CHECK_INTERVAL/1000) + "s");
    Serial.println("   • Server monitor: " + String(SERVER_CHECK_INTERVAL/1000) + "s");
    Serial.println("   • Default sensor interval: " + String(DEFAULT_SENSOR_INTERVAL/1000) + "s");
    Serial.println("   • Sensor interval will be updated from NFC config");
    
    // Initialize NFC
    nfc.begin();
    
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (!versiondata) {
        Serial.println("❌ PN532 not found! Check wiring.");
        while (1) delay(1000);
    }
    
    Serial.print("✅ Found PN5"); 
    Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("📋 Firmware v"); 
    Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); 
    Serial.println((versiondata>>8) & 0xFF, DEC);
    
    nfc.SAMConfig();
    
    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    
    Serial.println("\n⏳ Ready! Place NFC tag near reader...");
    Serial.println("💡 TIP: Use smartphone during 'RELEASING NFC FIELD' messages");
    Serial.println("🎯 Sensor interval will be configured from NFC tag data");
    
    lastReleaseTime = millis();
    lastWifiCheck = millis();
    lastServerCheck = millis();
    lastSensorSend = millis();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Periodic field release untuk smartphone compatibility
    if (currentTime - lastReleaseTime >= RELEASE_INTERVAL) {
        releaseNFCField();
        return;
    }
    
    // Check WiFi connection periodically
    if (configurationValid && currentTime - lastWifiCheck >= WIFI_CHECK_INTERVAL) {
        if (WiFi.status() != WL_CONNECTED && wifiConnected) {
            Serial.println("🔄 WiFi disconnected, attempting reconnection...");
            connectToWiFi();
        } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
            wifiConnected = true;
            Serial.println("✅ WiFi reconnected!");
        }
        lastWifiCheck = currentTime;
    }
    
    // Check server connection periodically
    if (configurationValid && wifiConnected && currentTime - lastServerCheck >= SERVER_CHECK_INTERVAL) {
        testServerConnection();
        lastServerCheck = currentTime;
    }
    
    // Kirim data sensor berdasarkan interval dari NFC configuration
    unsigned long sensorInterval = getSensorInterval();
    if (configurationValid && wifiConnected && currentTime - lastSensorSend >= sensorInterval) {
        Serial.println("\n📊 === SENDING SENSOR DATA ===");
        // Serial.println("⏱️ Using interval: " + String(sensorInterval) + "ms (" + String(sensorInterval/1000.0, 1) + "s)");
        Serial.println("⏱️ Using interval: " + String(sensorInterval) + "ms" + "("+String(sensorInterval/1000) + "s" + ")");
        Serial.println("⏱️ Using interval: " + String(sensorInterval));
        if (sendSensorDataToNodeRED()) {
            Serial.println("✅ Sensor data sent successfully!");
        } else {
            Serial.println("❌ Failed to send sensor data");
        }
        lastSensorSend = currentTime;
    }
    
    // NFC scanning dengan interval
    if (currentTime - lastCheckTime < CHECK_INTERVAL) {
        delay(10);
        return;
    }
    
    lastCheckTime = currentTime;
    
    // Scan untuk NFC tag
    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;
    
    bool tagDetected = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
    
    if (tagDetected) {
        String currentUID = getUIDString(uid, uidLength);
        
        if (!tagPresent) {
            // Tag baru terdeteksi
            Serial.println("\n🆕 NEW NFC TAG DETECTED!");
            tagPresent = true;
            lastUID = currentUID;
            processNFCTag(currentUID);
            
        } else if (currentUID != lastUID) {
            // Tag berbeda
            Serial.println("\n🔄 DIFFERENT TAG DETECTED!");
            Serial.println("Previous: " + lastUID);
            Serial.println("Current:  " + currentUID);
            lastUID = currentUID;
            lastRawData = ""; // Reset data untuk force re-read
            processNFCTag(currentUID);
            
        } else {
            // Tag sama, tampilkan heartbeat
            Serial.print("💓");
            
            // Opsional: Cek perubahan data pada tag yang sama
            String currentRawData = readNTAGData();
            if (currentRawData != lastRawData && currentRawData.length() > 0) {
                Serial.println("\n🔄 DATA UPDATE DETECTED ON SAME TAG!");
                processNFCTag(currentUID);
            }
        }
        
    } else {
        if (tagPresent) {
            Serial.println("\n❌ TAG REMOVED");
            tagPresent = false;
            lastUID = "";
            Serial.println("⏳ Waiting for next NFC tag...");
        } else {
            Serial.print("⏳");
        }
    }
}

// Serial commands untuk debugging
void serialEvent() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        
        if (command == "RELEASE") {
            Serial.println("\n🎛️ Manual field release triggered!");
            releaseNFCField();
            
        } else if (command == "STATUS") {
            Serial.println("\n📊 === SYSTEM STATUS ===");
            Serial.println("🏷️ Tag Present: " + String(tagPresent ? "YES" : "NO"));
            Serial.println("🆔 Last UID: " + (lastUID.length() > 0 ? lastUID : "None"));
            Serial.println("📊 Data Length: " + String(lastRawData.length()) + " bytes");
            Serial.println("🔧 Config Valid: " + String(configurationValid ? "YES" : "NO"));
            Serial.println("📡 WiFi Status: " + String(wifiConnected ? "CONNECTED" : "DISCONNECTED"));
            if (wifiConnected) {
                Serial.println("📶 WiFi IP: " + WiFi.localIP().toString());
                Serial.println("📶 WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
            }
            Serial.println("🌐 Server Status: " + String(serverConnected ? "CONNECTED" : "DISCONNECTED"));
            Serial.println("⏰ Next Release: " + String((RELEASE_INTERVAL - (millis() - lastReleaseTime))/1000) + "s");
            Serial.println("🎯 Active Sensor Interval: " + String(getSensorInterval()) + "ms (" + String(getSensorInterval()/1000.0, 1) + "s)");
            if (configurationValid) {
                Serial.println("📝 Config Interval: " + String(currentConfig.interval) + "ms");
            } else {
                Serial.println("📝 Using Default Interval: " + String(DEFAULT_SENSOR_INTERVAL) + "ms");
            }
            
        } else if (command == "WIFI") {
            if (configurationValid) {
                Serial.println("\n🔄 Forcing WiFi reconnection...");
                connectToWiFi();
            } else {
                Serial.println("\n❌ No valid configuration loaded");
            }
            
        } else if (command == "SERVER") {
            if (configurationValid && wifiConnected) {
                Serial.println("\n🔄 Testing server connection...");
                testServerConnection();
            } else {
                Serial.println("\n❌ WiFi not connected or no valid configuration");
            }
            
        } else if (command == "SEND") {
            if (wifiConnected && configurationValid) {
                Serial.println("\n📤 Sending test sensor data to Node-RED...");
                Serial.println("⏱️ Current interval: " + String(getSensorInterval()) + "ms");
                if (sendSensorDataToNodeRED()) {
                    Serial.println("✅ Test data sent successfully!");
                } else {
                    Serial.println("❌ Failed to send test data");
                }
            } else {
                Serial.println("\n❌ WiFi not connected or no valid configuration");
            }
            
        } else if (command == "INTERVAL") {
            Serial.println("\n⏱️ === INTERVAL STATUS ===");
            Serial.println("🎯 Active Interval: " + String(getSensorInterval()) + "ms (" + String(getSensorInterval()/1000.0, 1) + "s)");
            Serial.println("📋 Default Interval: " + String(DEFAULT_SENSOR_INTERVAL) + "ms");
            if (configurationValid) {
                Serial.println("📝 NFC Config Interval: " + String(currentConfig.interval) + "ms");
            } else {
                Serial.println("❌ No NFC configuration loaded");
            }
            Serial.println("⏰ Next sensor send in: " + String((getSensorInterval() - (millis() - lastSensorSend))/1000.0, 1) + "s");
            
        } else if (command == "HELP") {
            Serial.println("\n📖 === AVAILABLE COMMANDS ===");
            Serial.println("RELEASE  - Manually release NFC field");
            Serial.println("STATUS   - Show detailed system status");
            Serial.println("WIFI     - Force WiFi reconnection");
            Serial.println("SERVER   - Test server connection");
            Serial.println("SEND     - Send test sensor data to Node-RED");
            Serial.println("INTERVAL - Show sensor interval information");
            Serial.println("HELP     - Show this help menu");
        }
    }
}
