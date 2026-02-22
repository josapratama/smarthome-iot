#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <Update.h>

#ifndef LED_BUILTIN
  #define LED_BUILTIN 8
#endif

// ===== KONFIGURASI - GANTI SESUAI KEBUTUHAN =====
// PENTING: Pastikan WiFi menggunakan 2.4GHz (ESP32 tidak support 5GHz)
// Default WiFi credentials (akan di-override oleh Preferences jika sudah disimpan)
String WIFI_SSID = "";  // Ganti dengan hotspot HP untuk test
String WIFI_PASSWORD = "";  // Ganti dengan password hotspot

const char* MQTT_SERVER = "192.168.100.11";  // IP komputer (bukan router!)
const int MQTT_PORT = 1883;
const char* MQTT_USER = "";  // Kosongkan jika tidak pakai auth
const char* MQTT_PASSWORD = "";
// ================================================

const char* FIRMWARE_VERSION = "1.0.6";  // Enhanced WiFi connection stability
const char* DEVICE_TYPE = "BASE";  // Type: BASE (no sensor, OTA only)

// Backend URL untuk OTA
String BACKEND_OTA_URL = "http://192.168.100.11:3000";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;

unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 30000; // 30 detik

// MAC Address untuk identifikasi unik
String macAddress;

// Forward declarations
void connectWiFi();
void connectMQTT();
void sendHeartbeat();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void sendCommandAck(int commandId, const char* status, const char* message);
void setupOTA();
void performOTAUpdate(String firmwareUrl, String version, int otaJobId);
void sendOTAProgress(int otaJobId, const char* status, int progress, String error);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Get MAC Address
    macAddress = WiFi.macAddress();
    
    Serial.println("\n================================");
    Serial.println("ESP32-C3 BASE (OTA Only)");
    Serial.println("================================");
    Serial.println("MAC: " + macAddress);
    Serial.println("Type: " + String(DEVICE_TYPE));
    Serial.println("Firmware: " + String(FIRMWARE_VERSION));
    Serial.println("================================\n");
    
    // Init pins
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Init preferences
    preferences.begin("device", false);
    
    // Load saved WiFi credentials from Preferences
    if (preferences.isKey("wifiSsid") && preferences.isKey("wifiPassword")) {
        WIFI_SSID = preferences.getString("wifiSsid", "");
        WIFI_PASSWORD = preferences.getString("wifiPassword", "");
        if (WIFI_SSID.length() > 0) {
            Serial.println("✓ Loaded WiFi credentials from memory");
            Serial.println("SSID: " + WIFI_SSID);
        }
    }
    
    // Load saved device credentials
    if (preferences.isKey("deviceId")) {
        int savedId = preferences.getInt("deviceId", 0);
        String savedKey = preferences.getString("deviceKey", "");
        if (savedId > 0 && savedKey.length() > 0) {
            Serial.println("✓ Loaded device credentials from memory");
            Serial.println("Device ID: " + String(savedId));
        }
    }
    
    // Connect WiFi
    connectWiFi();
    
    // Setup MQTT
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(15);
    
    // Connect MQTT
    connectMQTT();
    
    // Setup OTA
    setupOTA();
    
    Serial.println("\n=== DEVICE READY ===");
    Serial.println("Untuk registrasi device:");
    Serial.println("1. Buka backend admin panel");
    Serial.println("2. Tambah device baru");
    Serial.println("3. Gunakan MAC: " + macAddress);
    Serial.println("4. Setelah dapat Device ID & Key,");
    Serial.println("   kirim command SET_CREDENTIALS");
    Serial.println("====================\n");
}

void loop() {
    // Check WiFi
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost!");
        connectWiFi();
    }
    
    // Check MQTT
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    
    // MQTT loop
    mqttClient.loop();
    
    // OTA handle
    ArduinoOTA.handle();
    
    // Send heartbeat setiap interval
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
    }
    
    // Blink LED (slow = waiting registration, fast = registered)
    int deviceId = preferences.getInt("deviceId", 0);
    int blinkInterval = (deviceId > 0) ? 1000 : 200;
    digitalWrite(LED_BUILTIN, (millis() / blinkInterval) % 2);
    
    delay(100);
}

void connectWiFi() {
    Serial.println("\n=== Connecting to WiFi ===");
    Serial.println("SSID: " + WIFI_SSID);
    
    // Disconnect first to clear any previous state
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    
    // Set WiFi mode
    WiFi.mode(WIFI_STA);
    
    // ESP32-C3 specific WiFi configurations
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);  // Disable WiFi sleep for stability
    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Max power for better connection
    
    // Set hostname
    WiFi.setHostname(("ESP32-" + macAddress).c_str());
    
    // Try to connect with specific config
    Serial.println("Attempting connection with optimized settings...");
    
    // Method 1: Standard connection
    WiFi.begin(WIFI_SSID.c_str(), WIFI_PASSWORD.c_str());
    
    Serial.print("Connecting");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        
        // Print detailed status every 5 seconds
        if (attempts % 10 == 0 && attempts > 0) {
            Serial.print("\nStatus: ");
            Serial.print(WiFi.status());
            Serial.print(" | RSSI: ");
            Serial.print(WiFi.RSSI());
            Serial.print(" dBm | ");
        }
        
        // Try reconnect after 15 attempts
        if (attempts == 15) {
            Serial.println("\nRetrying with different method...");
            WiFi.disconnect();
            delay(1000);
            WiFi.begin(WIFI_SSID.c_str(), WIFI_PASSWORD.c_str());
        }
        
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✓ WiFi Connected!");
        Serial.println("IP: " + WiFi.localIP().toString());
        Serial.println("Gateway: " + WiFi.gatewayIP().toString());
        Serial.println("DNS: " + WiFi.dnsIP().toString());
        Serial.println("Signal: " + String(WiFi.RSSI()) + " dBm");
        Serial.println("Channel: " + String(WiFi.channel()));
    } else {
        Serial.println("\n✗ WiFi Connection Failed!");
        Serial.println("Status Code: " + String(WiFi.status()));
        Serial.println("\nStatus meanings:");
        Serial.println("0 = WL_IDLE_STATUS");
        Serial.println("1 = WL_NO_SSID_AVAIL (SSID not found)");
        Serial.println("2 = WL_SCAN_COMPLETED");
        Serial.println("3 = WL_CONNECTED");
        Serial.println("4 = WL_CONNECT_FAILED");
        Serial.println("5 = WL_CONNECTION_LOST");
        Serial.println("6 = WL_DISCONNECTED");
        Serial.println("\nPossible solutions:");
        Serial.println("1. Check router security: Use WPA2-PSK (not WPA3)");
        Serial.println("2. Check password is correct");
        Serial.println("3. Move ESP32 closer to router");
        Serial.println("4. Check router 2.4GHz band is enabled");
        Serial.println("5. Disable MAC filtering on router");
        Serial.println("6. Change router channel to 1, 6, or 11");
        Serial.println("7. Set router channel width to 20MHz");
        Serial.println("\nWaiting 10 seconds before restart...");
        delay(10000);
        ESP.restart();
    }
}

void connectMQTT() {
    if (mqttClient.connected()) return;
    
    Serial.print("Connecting MQTT");
    
    // Use MAC address as client ID if not registered yet
    int deviceId = preferences.getInt("deviceId", 0);
    String clientId = (deviceId > 0) ? "ESP32_" + String(deviceId) : "ESP32_" + macAddress;
    clientId.replace(":", "");
    
    int attempts = 0;
    while (!mqttClient.connected() && attempts < 3) {
        if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
            Serial.println("\n✓ MQTT OK");
            
            if (deviceId > 0) {
                // Subscribe to device-specific commands
                String commandTopic = "devices/" + String(deviceId) + "/commands";
                mqttClient.subscribe(commandTopic.c_str(), 1);
                Serial.println("Subscribed: " + commandTopic);
            } else {
                // Subscribe to registration topic
                String regTopic = "devices/register/" + macAddress;
                regTopic.replace(":", "");
                mqttClient.subscribe(regTopic.c_str(), 1);
                Serial.println("Subscribed: " + regTopic + " (waiting registration)");
            }
            
        } else {
            Serial.print(".");
            attempts++;
            delay(2000);
        }
    }
    
    if (!mqttClient.connected()) {
        Serial.println("\n✗ MQTT failed!");
    }
}

void sendHeartbeat() {
    if (!mqttClient.connected()) return;
    
    int deviceId = preferences.getInt("deviceId", 0);
    String deviceKey = preferences.getString("deviceKey", "");
    
    if (deviceId == 0) {
        // Not registered yet, send registration request
        StaticJsonDocument<256> doc;
        doc["mac"] = macAddress;
        doc["type"] = DEVICE_TYPE;
        doc["firmware"] = FIRMWARE_VERSION;
        doc["ip"] = WiFi.localIP().toString();
        
        String payload;
        serializeJson(doc, payload);
        
        String topic = "devices/register/request";
        mqttClient.publish(topic.c_str(), payload.c_str());
        Serial.println("Sent registration request");
        return;
    }
    
    // Send heartbeat
    StaticJsonDocument<256> doc;
    doc["deviceKey"] = deviceKey;
    doc["firmware"] = FIRMWARE_VERSION;
    doc["ip"] = WiFi.localIP().toString();
    doc["uptime"] = millis() / 1000;
    doc["freeHeap"] = ESP.getFreeHeap();
    
    // Add mqttClientId
    String clientId = "ESP32_" + String(deviceId);
    doc["mqttClientId"] = clientId;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "devices/" + String(deviceId) + "/heartbeat";
    mqttClient.publish(topic.c_str(), payload.c_str());
    Serial.println("Heartbeat sent (ID=" + String(deviceId) + ")");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.println("\nCommand received:");
    Serial.println("Topic: " + String(topic));
    Serial.println("Payload: " + message);
    
    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        Serial.println("JSON parse error!");
        return;
    }
    
    // Handle command
    String type = doc["type"] | "";
    int commandId = doc["commandId"] | 0;  // Changed from "id" to "commandId"
    
    if (type == "RESTART") {
        Serial.println("Restarting...");
        sendCommandAck(commandId, "SUCCESS", "Device restarting");
        delay(1000);
        ESP.restart();
        
    } else if (type == "PING") {
        Serial.println("Ping received");
        sendCommandAck(commandId, "SUCCESS", "Pong");
        
    } else if (type == "UPDATE_CONFIG") {
        Serial.println("Config update");
        sendCommandAck(commandId, "SUCCESS", "Config updated");
        
    } else if (type == "OTA_UPDATE") {
        Serial.println("OTA Update command received");
        
        JsonObject payloadObj = doc["payload"];
        String firmwareUrl = payloadObj["firmwareUrl"] | "";
        String version = payloadObj["version"] | "";
        int otaJobId = payloadObj["otaJobId"] | 0;
        
        if (firmwareUrl.length() > 0 && otaJobId > 0) {
            sendCommandAck(commandId, "SUCCESS", "Starting OTA update");
            performOTAUpdate(firmwareUrl, version, otaJobId);
        } else {
            sendCommandAck(commandId, "FAILED", "Invalid OTA parameters");
        }
        
    } else if (type == "SET_CREDENTIALS") {
        Serial.println("Set credentials command received");
        
        JsonObject payloadObj = doc["payload"];
        int newDeviceId = payloadObj["deviceId"] | 0;
        String newDeviceKey = payloadObj["deviceKey"] | "";
        String wifiSsid = payloadObj["wifiSsid"] | "";
        String wifiPassword = payloadObj["wifiPassword"] | "";
        
        if (newDeviceId > 0 && newDeviceKey.length() > 0) {
            // Save device credentials to preferences
            preferences.putInt("deviceId", newDeviceId);
            preferences.putString("deviceKey", newDeviceKey);
            
            // Save WiFi credentials if provided
            if (wifiSsid.length() > 0 && wifiPassword.length() > 0) {
                preferences.putString("wifiSsid", wifiSsid);
                preferences.putString("wifiPassword", wifiPassword);
                Serial.println("✓ WiFi credentials saved!");
                Serial.println("SSID: " + wifiSsid);
            }
            
            Serial.println("✓ Device credentials saved!");
            Serial.println("Device ID: " + String(newDeviceId));
            Serial.println("Device Key: " + newDeviceKey);
            
            sendCommandAck(commandId, "SUCCESS", "Credentials saved, restarting...");
            delay(2000);
            ESP.restart();
        } else {
            sendCommandAck(commandId, "FAILED", "Invalid credentials");
        }
    }
}

void sendCommandAck(int commandId, const char* status, const char* message) {
    if (!mqttClient.connected() || commandId == 0) return;
    
    int deviceId = preferences.getInt("deviceId", 0);
    if (deviceId == 0) return;
    
    StaticJsonDocument<256> doc;
    doc["commandId"] = commandId;
    doc["status"] = status;
    doc["message"] = message;
    doc["ts"] = millis();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "devices/" + String(deviceId) + "/commands/ack";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("ACK sent: " + String(status));
}

void setupOTA() {
    // Hostname untuk OTA
    int deviceId = preferences.getInt("deviceId", 0);
    String hostname = (deviceId > 0) ? "ESP32-" + String(deviceId) : "ESP32-" + macAddress;
    hostname.replace(":", "");
    ArduinoOTA.setHostname(hostname.c_str());
    
    // Password OTA
    ArduinoOTA.setPassword("admin123");
    
    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
        Serial.println("OTA Start: " + type);
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
    ArduinoOTA.begin();
    Serial.println("OTA Ready");
}

void performOTAUpdate(String firmwareUrl, String version, int otaJobId) {
    Serial.println("\n=== Starting HTTP OTA Update ===");
    Serial.println("URL: " + firmwareUrl);
    Serial.println("Version: " + version);
    Serial.println("OTA Job ID: " + String(otaJobId));
    
    // Send progress: DOWNLOADING
    sendOTAProgress(otaJobId, "DOWNLOADING", 0, "");
    
    HTTPClient http;
    http.begin(firmwareUrl);
    http.setTimeout(30000); // 30 second timeout
    
    int httpCode = http.GET();
    
    if (httpCode != 200) {
        String error = "HTTP Error: " + String(httpCode);
        Serial.println(error);
        sendOTAProgress(otaJobId, "FAILED", 0, error);
        http.end();
        return;
    }
    
    int contentLength = http.getSize();
    if (contentLength <= 0) {
        sendOTAProgress(otaJobId, "FAILED", 0, "Invalid content length");
        http.end();
        return;
    }
    
    Serial.printf("Firmware size: %d bytes\n", contentLength);
    
    bool canBegin = Update.begin(contentLength);
    if (!canBegin) {
        sendOTAProgress(otaJobId, "FAILED", 0, "Not enough space");
        http.end();
        return;
    }
    
    // Get stream
    WiFiClient* stream = http.getStreamPtr();
    
    size_t written = 0;
    uint8_t buff[128];
    int lastProgress = 0;
    
    while (http.connected() && (written < contentLength)) {
        size_t available = stream->available();
        
        if (available) {
            int c = stream->readBytes(buff, min(available, sizeof(buff)));
            
            if (c > 0) {
                Update.write(buff, c);
                written += c;
                
                // Send progress every 10%
                int progress = (written * 100) / contentLength;
                if (progress >= lastProgress + 10) {
                    Serial.printf("Progress: %d%%\n", progress);
                    sendOTAProgress(otaJobId, "DOWNLOADING", progress, "");
                    lastProgress = progress;
                }
            }
        }
        delay(1);
    }
    
    if (written != contentLength) {
        sendOTAProgress(otaJobId, "FAILED", 0, "Download incomplete");
        Update.abort();
        http.end();
        return;
    }
    
    if (Update.end()) {
        Serial.println("OTA Update Success!");
        
        if (Update.isFinished()) {
            sendOTAProgress(otaJobId, "APPLIED", 100, "Update successful, rebooting...");
            Serial.println("Rebooting...");
            delay(2000);
            ESP.restart();
        } else {
            sendOTAProgress(otaJobId, "FAILED", 0, "Update not finished");
        }
    } else {
        String error = "Update Error: " + String(Update.getError());
        Serial.println(error);
        sendOTAProgress(otaJobId, "FAILED", 0, error);
    }
    
    http.end();
}

void sendOTAProgress(int otaJobId, const char* status, int progress, String error) {
    if (!mqttClient.connected()) return;
    
    int deviceId = preferences.getInt("deviceId", 0);
    if (deviceId == 0) return;
    
    StaticJsonDocument<256> doc;
    doc["otaJobId"] = otaJobId;
    doc["status"] = status;
    doc["progress"] = progress;
    doc["currentVersion"] = FIRMWARE_VERSION;
    
    if (error.length() > 0) {
        doc["error"] = error;
    }
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "devices/" + String(deviceId) + "/ota/progress";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("OTA Progress: " + String(status) + " " + String(progress) + "%");
}
