#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Preferences.h>
#include <esp_system.h>

// Pin definitions
#define WIFI_RESET_PIN 2
#define BUZZER_PIN 3
#define LED_BUILTIN 8

// Device configuration
const char* DEVICE_TYPE = "base_device";
const char* FIRMWARE_VERSION = "1.0.0";
String DEVICE_ID = "ESP32_C3_" + String((uint32_t)ESP.getEfuseMac(), HEX);

// WiFi and MQTT
WiFiClient espClient;
PubSubClient mqttClient(espClient);
AsyncWebServer server(80);
Preferences preferences;

// Configuration variables
String mqtt_server = "localhost";
String mqtt_port = "1883";
String mqtt_user = "";
String mqtt_password = "";
String home_id = "default_home";

// Timing variables
unsigned long lastMqttReconnect = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTelemetry = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 30000;
const unsigned long TELEMETRY_INTERVAL = 60000;

// Status variables
bool wifiConnected = false;
bool mqttConnected = false;
unsigned long bootTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-C3 Smart Home Base Device ===");
    Serial.println("Firmware Version: " + String(FIRMWARE_VERSION));
    Serial.println("Device ID: " + DEVICE_ID);
    
    bootTime = millis();
    
    // Initialize pins
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Initialize preferences
    preferences.begin("smarthome", false);
    
    // Startup beep
    buzzerBeep(1, 200);
    
    // Initialize WiFi
    setupWiFi();
    
    // Load MQTT configuration
    loadMqttConfig();
    
    // Setup MQTT
    mqttClient.setServer(mqtt_server.c_str(), mqtt_port.toInt());
    mqttClient.setCallback(mqttCallback);
    
    // Setup web server
    setupWebServer();
    
    // Setup OTA
    AsyncElegantOTA.begin(&server);
    server.begin();
    
    Serial.println("Setup completed!");
    Serial.println("Web interface: http://" + WiFi.localIP().toString());
    Serial.println("OTA update: http://" + WiFi.localIP().toString() + "/update");
    
    // Connected beeps
    buzzerBeep(3, 100);
}

void loop() {
    // Handle WiFi reset button
    handleWiFiReset();
    
    // Maintain WiFi connection
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) {
            Serial.println("WiFi disconnected!");
            wifiConnected = false;
            buzzerBeep(1, 1000); // Long beep for error
        }
        // Try to reconnect
        if (millis() % 10000 == 0) {
            WiFi.reconnect();
        }
    } else {
        if (!wifiConnected) {
            Serial.println("WiFi reconnected!");
            wifiConnected = true;
            buzzerBeep(2, 200);
        }
    }
    
    // Maintain MQTT connection
    if (wifiConnected) {
        if (!mqttClient.connected()) {
            if (mqttConnected) {
                Serial.println("MQTT disconnected!");
                mqttConnected = false;
            }
            if (millis() - lastMqttReconnect > MQTT_RECONNECT_INTERVAL) {
                reconnectMqtt();
                lastMqttReconnect = millis();
            }
        } else {
            if (!mqttConnected) {
                Serial.println("MQTT connected!");
                mqttConnected = true;
                buzzerBeep(3, 100);
                publishStatus();
            }
        }
        
        mqttClient.loop();
        
        // Send heartbeat
        if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
            publishHeartbeat();
            lastHeartbeat = millis();
        }
        
        // Send telemetry
        if (millis() - lastTelemetry > TELEMETRY_INTERVAL) {
            publishTelemetry();
            lastTelemetry = millis();
        }
    }
    
    // Blink LED to show device is alive
    digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
    
    delay(100);
}

void setupWiFi() {
    WiFiManager wm;
    
    // Add custom parameters for MQTT configuration
    WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqtt_server.c_str(), 40);
    WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port.c_str(), 6);
    WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user.c_str(), 32);
    WiFiManagerParameter custom_mqtt_password("password", "MQTT Password", mqtt_password.c_str(), 32);
    WiFiManagerParameter custom_home_id("home_id", "Home ID", home_id.c_str(), 32);
    
    wm.addParameter(&custom_mqtt_server);
    wm.addParameter(&custom_mqtt_port);
    wm.addParameter(&custom_mqtt_user);
    wm.addParameter(&custom_mqtt_password);
    wm.addParameter(&custom_home_id);
    
    // Set callback to save parameters
    wm.setSaveParamsCallback(saveConfigCallback);
    
    // Try to connect with saved credentials
    wm.setConfigPortalTimeout(300); // 5 minutes timeout
    
    if (!wm.autoConnect(("SmartHome-" + DEVICE_ID).c_str(), "smarthome123")) {
        Serial.println("Failed to connect and hit timeout");
        ESP.restart();
    }
    
    Serial.println("WiFi connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());
    Serial.println("MAC address: " + WiFi.macAddress());
    
    wifiConnected = true;
    buzzerBeep(2, 200);
}

void saveConfigCallback() {
    Serial.println("Saving MQTT configuration...");
    
    // Save MQTT configuration to preferences
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putString("mqtt_port", mqtt_port);
    preferences.putString("mqtt_user", mqtt_user);
    preferences.putString("mqtt_password", mqtt_password);
    preferences.putString("home_id", home_id);
    
    Serial.println("Configuration saved!");
}

void loadMqttConfig() {
    mqtt_server = preferences.getString("mqtt_server", "localhost");
    mqtt_port = preferences.getString("mqtt_port", "1883");
    mqtt_user = preferences.getString("mqtt_user", "");
    mqtt_password = preferences.getString("mqtt_password", "");
    home_id = preferences.getString("home_id", "default_home");
    
    Serial.println("Loaded MQTT config:");
    Serial.println("Server: " + mqtt_server + ":" + mqtt_port);
    Serial.println("User: " + mqtt_user);
    Serial.println("Home ID: " + home_id);
}

void reconnectMqtt() {
    if (mqttClient.connect(DEVICE_ID.c_str(), mqtt_user.c_str(), mqtt_password.c_str())) {
        Serial.println("MQTT connected!");
        
        // Subscribe to command topics
        String commandTopic = "smarthome/" + home_id + "/" + DEVICE_ID + "/commands";
        String configTopic = "smarthome/" + home_id + "/" + DEVICE_ID + "/config";
        String otaTopic = "smarthome/" + home_id + "/" + DEVICE_ID + "/ota";
        
        mqttClient.subscribe(commandTopic.c_str());
        mqttClient.subscribe(configTopic.c_str());
        mqttClient.subscribe(otaTopic.c_str());
        
        Serial.println("Subscribed to: " + commandTopic);
        Serial.println("Subscribed to: " + configTopic);
        Serial.println("Subscribed to: " + otaTopic);
        
    } else {
        Serial.print("MQTT connection failed, rc=");
        Serial.println(mqttClient.state());
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.println("MQTT message received:");
    Serial.println("Topic: " + String(topic));
    Serial.println("Message: " + message);
    
    // Parse JSON message
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, message);
    
    String topicStr = String(topic);
    
    if (topicStr.endsWith("/commands")) {
        handleCommand(doc);
    } else if (topicStr.endsWith("/config")) {
        handleConfig(doc);
    } else if (topicStr.endsWith("/ota")) {
        handleOTA(doc);
    }
}

void handleCommand(DynamicJsonDocument& doc) {
    String commandType = doc["type"];
    
    if (commandType == "PING") {
        publishStatus();
        buzzerBeep(1, 100);
    } else if (commandType == "RESTART") {
        Serial.println("Restart command received");
        buzzerBeep(2, 500);
        delay(1000);
        ESP.restart();
    } else if (commandType == "BUZZER_TEST") {
        int beeps = doc["payload"]["beeps"] | 3;
        int duration = doc["payload"]["duration"] | 200;
        buzzerBeep(beeps, duration);
    }
}

void handleConfig(DynamicJsonDocument& doc) {
    Serial.println("Configuration update received");
    
    if (doc.containsKey("telemetryInterval")) {
        // Update telemetry interval (not implemented in this base version)
        Serial.println("Telemetry interval update requested");
    }
    
    if (doc.containsKey("buzzerEnabled")) {
        bool enabled = doc["buzzerEnabled"];
        preferences.putBool("buzzer_enabled", enabled);
        Serial.println("Buzzer enabled: " + String(enabled));
    }
}

void handleOTA(DynamicJsonDocument& doc) {
    String otaUrl = doc["url"];
    if (otaUrl.length() > 0) {
        Serial.println("OTA update requested: " + otaUrl);
        // OTA update will be handled by AsyncElegantOTA web interface
        buzzerBeep(5, 100);
    }
}

void publishStatus() {
    DynamicJsonDocument doc(512);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["firmwareVersion"] = FIRMWARE_VERSION;
    doc["timestamp"] = getTimestamp();
    doc["status"]["online"] = true;
    doc["status"]["uptime"] = millis() - bootTime;
    doc["status"]["freeHeap"] = ESP.getFreeHeap();
    doc["status"]["rssi"] = WiFi.RSSI();
    doc["status"]["ip"] = WiFi.localIP().toString();
    doc["status"]["mac"] = WiFi.macAddress();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/status";
    mqttClient.publish(topic.c_str(), payload.c_str(), true); // Retained message
    
    Serial.println("Status published to: " + topic);
}

void publishHeartbeat() {
    DynamicJsonDocument doc(256);
    
    doc["deviceId"] = DEVICE_ID;
    doc["timestamp"] = getTimestamp();
    doc["uptime"] = millis() - bootTime;
    doc["rssi"] = WiFi.RSSI();
    doc["freeHeap"] = ESP.getFreeHeap();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/heartbeat";
    mqttClient.publish(topic.c_str(), payload.c_str());
}

void publishTelemetry() {
    DynamicJsonDocument doc(512);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    
    // Base device telemetry (system info)
    doc["data"]["uptime"] = millis() - bootTime;
    doc["data"]["freeHeap"] = ESP.getFreeHeap();
    doc["data"]["rssi"] = WiFi.RSSI();
    doc["data"]["temperature"] = temperatureRead(); // Internal temperature sensor
    
    doc["status"]["wifiConnected"] = wifiConnected;
    doc["status"]["mqttConnected"] = mqttConnected;
    doc["status"]["ip"] = WiFi.localIP().toString();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/telemetry";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Telemetry published");
}

void handleWiFiReset() {
    static unsigned long buttonPressStart = 0;
    static bool buttonPressed = false;
    
    bool currentState = digitalRead(WIFI_RESET_PIN) == LOW;
    
    if (currentState && !buttonPressed) {
        // Button just pressed
        buttonPressed = true;
        buttonPressStart = millis();
        Serial.println("WiFi reset button pressed");
    } else if (!currentState && buttonPressed) {
        // Button released
        buttonPressed = false;
        unsigned long pressDuration = millis() - buttonPressStart;
        
        if (pressDuration > 5000) {
            // Long press - reset WiFi
            Serial.println("WiFi reset triggered!");
            buzzerBeep(3, 200);
            
            // Clear WiFi credentials
            WiFi.disconnect(true);
            preferences.clear();
            
            delay(1000);
            ESP.restart();
        } else if (pressDuration > 1000) {
            // Short press - just beep
            buzzerBeep(1, 100);
        }
    }
}

void buzzerBeep(int count, int duration) {
    if (!preferences.getBool("buzzer_enabled", true)) {
        return; // Buzzer disabled
    }
    
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < count - 1) {
            delay(duration / 2);
        }
    }
}

void setupWebServer() {
    // Root page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<!DOCTYPE html><html><head><title>ESP32-C3 Smart Home Device</title>";
        html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
        html += "<style>body{font-family:Arial;margin:40px;background:#f0f0f0}";
        html += ".container{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
        html += ".status{padding:10px;margin:10px 0;border-radius:5px}";
        html += ".online{background:#d4edda;color:#155724;border:1px solid #c3e6cb}";
        html += ".offline{background:#f8d7da;color:#721c24;border:1px solid #f5c6cb}";
        html += "button{background:#007bff;color:white;border:none;padding:10px 20px;border-radius:5px;cursor:pointer;margin:5px}";
        html += "button:hover{background:#0056b3}</style></head><body>";
        
        html += "<div class='container'>";
        html += "<h1>🏠 ESP32-C3 Smart Home Device</h1>";
        html += "<h2>Device Information</h2>";
        html += "<p><strong>Device ID:</strong> " + DEVICE_ID + "</p>";
        html += "<p><strong>Device Type:</strong> " + String(DEVICE_TYPE) + "</p>";
        html += "<p><strong>Firmware Version:</strong> " + String(FIRMWARE_VERSION) + "</p>";
        html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
        html += "<p><strong>MAC Address:</strong> " + WiFi.macAddress() + "</p>";
        html += "<p><strong>Uptime:</strong> " + String((millis() - bootTime) / 1000) + " seconds</p>";
        
        html += "<h2>Connection Status</h2>";
        html += "<div class='status " + String(wifiConnected ? "online" : "offline") + "'>";
        html += "WiFi: " + String(wifiConnected ? "Connected" : "Disconnected");
        html += " (RSSI: " + String(WiFi.RSSI()) + " dBm)</div>";
        
        html += "<div class='status " + String(mqttConnected ? "online" : "offline") + "'>";
        html += "MQTT: " + String(mqttConnected ? "Connected" : "Disconnected") + "</div>";
        
        html += "<h2>Actions</h2>";
        html += "<button onclick='location.href=\"/restart\"'>Restart Device</button>";
        html += "<button onclick='location.href=\"/update\"'>OTA Update</button>";
        html += "<button onclick='buzzerTest()'>Test Buzzer</button>";
        
        html += "<script>";
        html += "function buzzerTest(){fetch('/buzzer-test').then(r=>alert('Buzzer test sent!'))}";
        html += "setTimeout(()=>location.reload(), 30000);"; // Auto refresh every 30 seconds
        html += "</script>";
        
        html += "</div></body></html>";
        
        request->send(200, "text/html", html);
    });
    
    // Restart endpoint
    server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Restarting device...");
        delay(1000);
        ESP.restart();
    });
    
    // Buzzer test endpoint
    server.on("/buzzer-test", HTTP_GET, [](AsyncWebServerRequest *request){
        buzzerBeep(3, 200);
        request->send(200, "text/plain", "Buzzer test completed");
    });
    
    // Device info API
    server.on("/api/info", HTTP_GET, [](AsyncWebServerRequest *request){
        DynamicJsonDocument doc(512);
        doc["deviceId"] = DEVICE_ID;
        doc["deviceType"] = DEVICE_TYPE;
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["uptime"] = millis() - bootTime;
        doc["freeHeap"] = ESP.getFreeHeap();
        doc["wifiConnected"] = wifiConnected;
        doc["mqttConnected"] = mqttConnected;
        doc["rssi"] = WiFi.RSSI();
        doc["ip"] = WiFi.localIP().toString();
        doc["mac"] = WiFi.macAddress();
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
}

String getTimestamp() {
    // Simple timestamp (milliseconds since boot)
    // In production, you might want to use NTP for real timestamps
    return String(millis());
}