#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Preferences.h>

// Pin definitions
#define WIFI_RESET_PIN 2
#define BUZZER_PIN 3
#define FLAME_ANALOG_PIN 8
#define FLAME_DIGITAL_PIN 9
#define LED_BUILTIN 10

// Device configuration
const char* DEVICE_TYPE = "flame_sensor";
const char* FIRMWARE_VERSION = "1.0.0";
String DEVICE_ID = "ESP32_C3_FLAME_" + String((uint32_t)ESP.getEfuseMac(), HEX);

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

// Sensor configuration
int flameThreshold = 1000;       // Flame detection threshold (0-4095)
int sensitivity = 2;             // Sensitivity level (1-5)
int reportInterval = 10;         // Report interval in seconds (faster for fire safety)
bool alarmEnabled = true;
bool falseAlarmPrevention = true;
int confirmationTime = 3;        // Seconds to confirm flame before alarm

// Timing variables
unsigned long lastMqttReconnect = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTelemetry = 0;
unsigned long lastSensorRead = 0;
unsigned long flameDetectedTime = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 30000;
const unsigned long SENSOR_READ_INTERVAL = 500; // Fast reading for fire detection

// Status variables
bool wifiConnected = false;
bool mqttConnected = false;
bool flameDetected = false;
bool flameConfirmed = false;
unsigned long bootTime = 0;

// Sensor data
struct SensorData {
    int analogValue = 0;
    bool digitalValue = false;
    float flameIntensity = 0.0;
    int flameLevel = 0;          // 0-5 scale
    bool flameAlarm = false;
    bool valid = true;
    unsigned long timestamp = 0;
};

SensorData currentReading;
SensorData lastReading;

// Moving average for noise reduction
const int SAMPLE_SIZE = 10;
int analogSamples[SAMPLE_SIZE];
int sampleIndex = 0;
bool samplesReady = false;

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-C3 Flame Sensor ===");
    Serial.println("Firmware Version: " + String(FIRMWARE_VERSION));
    Serial.println("Device ID: " + DEVICE_ID);
    
    bootTime = millis();
    
    // Initialize pins
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(FLAME_DIGITAL_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Initialize analog samples array
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        analogSamples[i] = 0;
    }
    
    // Initialize preferences
    preferences.begin("smarthome", false);
    loadConfiguration();
    
    // Startup beep
    buzzerBeep(1, 200);
    
    // Initialize WiFi
    setupWiFi();
    
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
    Serial.println("FLAME SENSOR ACTIVE - Fire detection enabled");
    
    // Connected beeps
    buzzerBeep(3, 100);
}

void loop() {
    // Handle WiFi reset button
    handleWiFiReset();
    
    // Maintain WiFi connection
    maintainWiFiConnection();
    
    // Maintain MQTT connection
    if (wifiConnected) {
        maintainMqttConnection();
        mqttClient.loop();
        
        // Send heartbeat
        if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
            publishHeartbeat();
            lastHeartbeat = millis();
        }
        
        // Read sensor frequently for fire safety
        if (millis() - lastSensorRead > SENSOR_READ_INTERVAL) {
            readSensor();
            lastSensorRead = millis();
        }
        
        // Send telemetry based on report interval
        if (millis() - lastTelemetry > (reportInterval * 1000)) {
            publishTelemetry();
            lastTelemetry = millis();
        }
    }
    
    // Handle flame confirmation logic
    handleFlameConfirmation();
    
    // Blink LED to show device is alive (faster blink if flame detected)
    int blinkRate = flameDetected ? 250 : 1000;
    digitalWrite(LED_BUILTIN, (millis() / blinkRate) % 2);
    
    delay(50); // Shorter delay for responsive fire detection
}

void readSensor() {
    SensorData newReading;
    newReading.timestamp = millis();
    
    // Read analog value (0-4095 for ESP32)
    int rawAnalog = analogRead(FLAME_ANALOG_PIN);
    
    // Add to moving average
    analogSamples[sampleIndex] = rawAnalog;
    sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
    
    if (sampleIndex == 0) samplesReady = true;
    
    // Calculate moving average if we have enough samples
    if (samplesReady) {
        long sum = 0;
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            sum += analogSamples[i];
        }
        newReading.analogValue = sum / SAMPLE_SIZE;
    } else {
        newReading.analogValue = rawAnalog;
    }
    
    // Read digital value (LOW = flame detected, HIGH = no flame)
    newReading.digitalValue = digitalRead(FLAME_DIGITAL_PIN) == LOW;
    
    // Calculate flame intensity (0.0 - 100.0%)
    // Lower analog values indicate stronger flame detection
    newReading.flameIntensity = map(newReading.analogValue, 4095, 0, 0, 10000) / 100.0;
    newReading.flameIntensity = constrain(newReading.flameIntensity, 0.0, 100.0);
    
    // Calculate flame level (0-5 scale)
    if (newReading.flameIntensity < 10) newReading.flameLevel = 0;
    else if (newReading.flameIntensity < 25) newReading.flameLevel = 1;
    else if (newReading.flameIntensity < 40) newReading.flameLevel = 2;
    else if (newReading.flameIntensity < 60) newReading.flameLevel = 3;
    else if (newReading.flameIntensity < 80) newReading.flameLevel = 4;
    else newReading.flameLevel = 5;
    
    // Determine flame alarm condition based on sensitivity
    bool analogAlarm = false;
    bool digitalAlarm = newReading.digitalValue;
    
    switch (sensitivity) {
        case 1: // Very Low - only digital trigger
            analogAlarm = false;
            break;
        case 2: // Low
            analogAlarm = newReading.analogValue < (flameThreshold * 0.8);
            break;
        case 3: // Medium
            analogAlarm = newReading.analogValue < flameThreshold;
            break;
        case 4: // High
            analogAlarm = newReading.analogValue < (flameThreshold * 1.2);
            break;
        case 5: // Very High
            analogAlarm = newReading.analogValue < (flameThreshold * 1.5);
            break;
    }
    
    newReading.flameAlarm = digitalAlarm || analogAlarm;
    
    // Update flame detection status
    bool previousFlameDetected = flameDetected;
    flameDetected = newReading.flameAlarm;
    
    if (flameDetected && !previousFlameDetected) {
        flameDetectedTime = millis();
        Serial.println("FLAME DETECTED - Confirming...");
        buzzerBeep(3, 100); // Initial detection beeps
    } else if (!flameDetected && previousFlameDetected) {
        Serial.println("Flame signal cleared");
        flameConfirmed = false;
        buzzerBeep(1, 200);
    }
    
    // Update readings
    lastReading = currentReading;
    currentReading = newReading;
    
    // Print readings to serial (less frequent to avoid spam)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000 || flameDetected) {
        Serial.println("=== Flame Sensor Readings ===");
        Serial.printf("Analog Value: %d\n", currentReading.analogValue);
        Serial.printf("Digital Value: %s\n", currentReading.digitalValue ? "FLAME" : "CLEAR");
        Serial.printf("Flame Intensity: %.1f%%\n", currentReading.flameIntensity);
        Serial.printf("Flame Level: %d/5\n", currentReading.flameLevel);
        Serial.printf("Flame Alarm: %s\n", currentReading.flameAlarm ? "ACTIVE" : "CLEAR");
        if (flameDetected) {
            Serial.printf("Detection Time: %lu ms\n", millis() - flameDetectedTime);
        }
        Serial.println("=============================");
        lastPrint = millis();
    }
}

void handleFlameConfirmation() {
    if (!falseAlarmPrevention) {
        // No confirmation needed - immediate alarm
        if (flameDetected && !flameConfirmed) {
            flameConfirmed = true;
            triggerFlameAlarm();
        }
        return;
    }
    
    // Confirmation logic for false alarm prevention
    if (flameDetected && !flameConfirmed) {
        unsigned long detectionDuration = millis() - flameDetectedTime;
        
        if (detectionDuration >= (confirmationTime * 1000)) {
            // Flame confirmed after confirmation time
            flameConfirmed = true;
            triggerFlameAlarm();
        }
    }
}

void triggerFlameAlarm() {
    Serial.println("🔥 FIRE ALARM TRIGGERED! 🔥");
    
    String alarmMessage = "FIRE DETECTED! ";
    alarmMessage += "Intensity: " + String(currentReading.flameIntensity, 1) + "%, ";
    alarmMessage += "Level: " + String(currentReading.flameLevel) + "/5";
    
    publishAlarm(alarmMessage);
    
    // Continuous alarm beeps
    buzzerBeep(20, 100);
}

void publishAlarm(String message) {
    DynamicJsonDocument doc(512);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    doc["alarmType"] = "FIRE_DETECTED";
    doc["message"] = message;
    doc["severity"] = "CRITICAL";
    doc["emergency"] = true;
    
    doc["data"]["flameIntensity"] = currentReading.flameIntensity;
    doc["data"]["flameLevel"] = currentReading.flameLevel;
    doc["data"]["analogValue"] = currentReading.analogValue;
    doc["data"]["digitalValue"] = currentReading.digitalValue;
    doc["data"]["detectionTime"] = millis() - flameDetectedTime;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/alarms";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    // Also publish to emergency topic
    String emergencyTopic = "smarthome/" + home_id + "/emergency/fire";
    mqttClient.publish(emergencyTopic.c_str(), payload.c_str());
    
    Serial.println("🚨 FIRE ALARM PUBLISHED: " + message);
}

void loadConfiguration() {
    flameThreshold = preferences.getInt("flame_thresh", 1000);
    sensitivity = preferences.getInt("sensitivity", 2);
    reportInterval = preferences.getInt("report_int", 10);
    alarmEnabled = preferences.getBool("alarm_en", true);
    falseAlarmPrevention = preferences.getBool("false_prev", true);
    confirmationTime = preferences.getInt("confirm_time", 3);
    
    // Load MQTT config
    mqtt_server = preferences.getString("mqtt_server", "localhost");
    mqtt_port = preferences.getString("mqtt_port", "1883");
    mqtt_user = preferences.getString("mqtt_user", "");
    mqtt_password = preferences.getString("mqtt_password", "");
    home_id = preferences.getString("home_id", "default_home");
    
    Serial.println("Configuration loaded:");
    Serial.println("Flame threshold: " + String(flameThreshold));
    Serial.println("Sensitivity: " + String(sensitivity) + "/5");
    Serial.println("Report interval: " + String(reportInterval) + "s");
    Serial.println("False alarm prevention: " + String(falseAlarmPrevention ? "ON" : "OFF"));
}

void saveConfiguration() {
    preferences.putInt("flame_thresh", flameThreshold);
    preferences.putInt("sensitivity", sensitivity);
    preferences.putInt("report_int", reportInterval);
    preferences.putBool("alarm_en", alarmEnabled);
    preferences.putBool("false_prev", falseAlarmPrevention);
    preferences.putInt("confirm_time", confirmationTime);
    
    Serial.println("Configuration saved!");
}

void setupWiFi() {
    WiFiManager wm;
    
    // Add custom parameters
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
    
    wm.setSaveParamsCallback(saveConfigCallback);
    wm.setConfigPortalTimeout(300);
    
    if (!wm.autoConnect(("SmartHome-FLAME-" + DEVICE_ID).c_str(), "smarthome123")) {
        Serial.println("Failed to connect and hit timeout");
        ESP.restart();
    }
    
    Serial.println("WiFi connected!");
    Serial.println("IP address: " + WiFi.localIP().toString());
    
    wifiConnected = true;
    buzzerBeep(2, 200);
}

void saveConfigCallback() {
    Serial.println("Saving configuration...");
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putString("mqtt_port", mqtt_port);
    preferences.putString("mqtt_user", mqtt_user);
    preferences.putString("mqtt_password", mqtt_password);
    preferences.putString("home_id", home_id);
}

void maintainWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnected) {
            Serial.println("WiFi disconnected!");
            wifiConnected = false;
            buzzerBeep(1, 1000);
        }
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
}

void maintainMqttConnection() {
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
}

void reconnectMqtt() {
    if (mqttClient.connect(DEVICE_ID.c_str(), mqtt_user.c_str(), mqtt_password.c_str())) {
        String commandTopic = "smarthome/" + home_id + "/" + DEVICE_ID + "/commands";
        String configTopic = "smarthome/" + home_id + "/" + DEVICE_ID + "/config";
        
        mqttClient.subscribe(commandTopic.c_str());
        mqttClient.subscribe(configTopic.c_str());
        
        Serial.println("MQTT subscriptions active");
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
    
    Serial.println("MQTT message: " + String(topic) + " -> " + message);
    
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, message);
    
    String topicStr = String(topic);
    
    if (topicStr.endsWith("/commands")) {
        handleCommand(doc);
    } else if (topicStr.endsWith("/config")) {
        handleConfig(doc);
    }
}

void handleCommand(DynamicJsonDocument& doc) {
    String commandType = doc["type"];
    
    if (commandType == "TEST_ALARM") {
        Serial.println("Alarm test requested");
        buzzerBeep(5, 200);
        publishTelemetry();
    } else if (commandType == "GET_READING") {
        publishTelemetry();
    } else if (commandType == "SILENCE_ALARM") {
        // Temporarily disable alarm (for testing)
        Serial.println("Alarm silenced temporarily");
        buzzerBeep(1, 100);
    }
}

void handleConfig(DynamicJsonDocument& doc) {
    bool configChanged = false;
    
    if (doc.containsKey("flameThreshold")) {
        flameThreshold = doc["flameThreshold"];
        configChanged = true;
    }
    
    if (doc.containsKey("sensitivity")) {
        sensitivity = constrain((int)doc["sensitivity"], 1, 5);
        configChanged = true;
    }
    
    if (doc.containsKey("reportInterval")) {
        reportInterval = doc["reportInterval"];
        configChanged = true;
    }
    
    if (doc.containsKey("alarmEnabled")) {
        alarmEnabled = doc["alarmEnabled"];
        configChanged = true;
    }
    
    if (doc.containsKey("falseAlarmPrevention")) {
        falseAlarmPrevention = doc["falseAlarmPrevention"];
        configChanged = true;
    }
    
    if (doc.containsKey("confirmationTime")) {
        confirmationTime = doc["confirmationTime"];
        configChanged = true;
    }
    
    if (configChanged) {
        saveConfiguration();
        Serial.println("Configuration updated via MQTT");
        buzzerBeep(1, 100);
    }
}

void publishStatus() {
    DynamicJsonDocument doc(1024);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["firmwareVersion"] = FIRMWARE_VERSION;
    doc["timestamp"] = getTimestamp();
    
    doc["status"]["online"] = true;
    doc["status"]["uptime"] = millis() - bootTime;
    doc["status"]["freeHeap"] = ESP.getFreeHeap();
    doc["status"]["rssi"] = WiFi.RSSI();
    doc["status"]["flameDetected"] = flameDetected;
    doc["status"]["flameConfirmed"] = flameConfirmed;
    
    doc["config"]["flameThreshold"] = flameThreshold;
    doc["config"]["sensitivity"] = sensitivity;
    doc["config"]["reportInterval"] = reportInterval;
    doc["config"]["alarmEnabled"] = alarmEnabled;
    doc["config"]["falseAlarmPrevention"] = falseAlarmPrevention;
    doc["config"]["confirmationTime"] = confirmationTime;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/status";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void publishHeartbeat() {
    DynamicJsonDocument doc(256);
    
    doc["deviceId"] = DEVICE_ID;
    doc["timestamp"] = getTimestamp();
    doc["uptime"] = millis() - bootTime;
    doc["rssi"] = WiFi.RSSI();
    doc["flameDetected"] = flameDetected;
    doc["flameConfirmed"] = flameConfirmed;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/heartbeat";
    mqttClient.publish(topic.c_str(), payload.c_str());
}

void publishTelemetry() {
    DynamicJsonDocument doc(1024);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    
    // Flame sensor data
    doc["data"]["analogValue"] = currentReading.analogValue;
    doc["data"]["digitalValue"] = currentReading.digitalValue;
    doc["data"]["flameIntensity"] = round(currentReading.flameIntensity * 10) / 10.0;
    doc["data"]["flameLevel"] = currentReading.flameLevel;
    doc["data"]["flameAlarm"] = currentReading.flameAlarm;
    doc["data"]["flameDetected"] = flameDetected;
    doc["data"]["flameConfirmed"] = flameConfirmed;
    
    if (flameDetected) {
        doc["data"]["detectionDuration"] = millis() - flameDetectedTime;
    }
    
    // Fire risk assessment
    String riskLevel = "Low";
    if (currentReading.flameLevel >= 4) riskLevel = "Critical";
    else if (currentReading.flameLevel >= 3) riskLevel = "High";
    else if (currentReading.flameLevel >= 2) riskLevel = "Medium";
    else if (currentReading.flameLevel >= 1) riskLevel = "Low";
    
    doc["data"]["fireRiskLevel"] = riskLevel;
    
    // System status
    doc["status"]["uptime"] = millis() - bootTime;
    doc["status"]["rssi"] = WiFi.RSSI();
    doc["status"]["freeHeap"] = ESP.getFreeHeap();
    doc["status"]["samplesReady"] = samplesReady;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/telemetry";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Telemetry published - Flame: " + String(currentReading.flameIntensity, 1) + "%, Level: " + String(currentReading.flameLevel));
}

void handleWiFiReset() {
    static unsigned long buttonPressStart = 0;
    static bool buttonPressed = false;
    
    bool currentState = digitalRead(WIFI_RESET_PIN) == LOW;
    
    if (currentState && !buttonPressed) {
        buttonPressed = true;
        buttonPressStart = millis();
    } else if (!currentState && buttonPressed) {
        buttonPressed = false;
        unsigned long pressDuration = millis() - buttonPressStart;
        
        if (pressDuration > 5000) {
            Serial.println("WiFi reset triggered!");
            buzzerBeep(3, 200);
            WiFi.disconnect(true);
            preferences.clear();
            delay(1000);
            ESP.restart();
        }
    }
}

void buzzerBeep(int count, int duration) {
    if (!preferences.getBool("buzzer_enabled", true)) return;
    
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration);
        digitalWrite(BUZZER_PIN, LOW);
        if (i < count - 1) delay(duration / 2);
    }
}

void setupWebServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateWebPage();
        request->send(200, "text/html", html);
    });
    
    server.on("/api/readings", HTTP_GET, [](AsyncWebServerRequest *request){
        DynamicJsonDocument doc(512);
        
        doc["analogValue"] = currentReading.analogValue;
        doc["digitalValue"] = currentReading.digitalValue;
        doc["flameIntensity"] = currentReading.flameIntensity;
        doc["flameLevel"] = currentReading.flameLevel;
        doc["flameAlarm"] = currentReading.flameAlarm;
        doc["flameDetected"] = flameDetected;
        doc["flameConfirmed"] = flameConfirmed;
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
}

String generateWebPage() {
    String html = "<!DOCTYPE html><html><head><title>Flame Sensor</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;margin:20px;background:#f0f0f0}";
    html += ".container{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
    html += ".reading{display:flex;justify-content:space-between;padding:10px;margin:5px 0;background:#f8f9fa;border-radius:5px}";
    html += ".value{font-weight:bold;color:#007bff}";
    html += ".alarm{color:#dc3545;font-weight:bold;animation:blink 1s infinite}";
    html += ".status{padding:10px;margin:10px 0;border-radius:5px}";
    html += ".online{background:#d4edda;color:#155724}";
    html += ".offline{background:#f8d7da;color:#721c24}";
    html += ".critical{background:#dc3545;color:white;animation:blink 1s infinite}";
    html += "@keyframes blink{0%,50%{opacity:1}51%,100%{opacity:0.3}}";
    html += "</style></head><body>";
    
    html += "<div class='container'>";
    html += "<h1>🔥 Flame Sensor</h1>";
    html += "<p><strong>Device ID:</strong> " + DEVICE_ID + "</p>";
    
    String statusClass = flameConfirmed ? "critical" : (flameDetected ? "offline" : "online");
    String statusText = flameConfirmed ? "🚨 FIRE CONFIRMED!" : (flameDetected ? "⚠️ Flame Detected" : "✅ All Clear");
    
    html += "<div class='status " + statusClass + "'>";
    html += "Status: " + statusText + "</div>";
    
    html += "<h2>Current Readings</h2>";
    html += "<div class='reading'><span>Analog Value:</span><span class='value'>" + String(currentReading.analogValue) + "</span></div>";
    html += "<div class='reading'><span>Digital Status:</span><span class='" + String(currentReading.digitalValue ? "alarm" : "value") + "'>" + String(currentReading.digitalValue ? "FLAME" : "CLEAR") + "</span></div>";
    html += "<div class='reading'><span>Flame Intensity:</span><span class='value'>" + String(currentReading.flameIntensity, 1) + "%</span></div>";
    html += "<div class='reading'><span>Flame Level:</span><span class='value'>" + String(currentReading.flameLevel) + "/5</span></div>";
    html += "<div class='reading'><span>Fire Alarm:</span><span class='" + String(currentReading.flameAlarm ? "alarm" : "value") + "'>" + String(currentReading.flameAlarm ? "ACTIVE" : "CLEAR") + "</span></div>";
    
    if (flameDetected) {
        html += "<div class='reading'><span>Detection Time:</span><span class='alarm'>" + String((millis() - flameDetectedTime) / 1000) + "s</span></div>";
    }
    
    html += "<script>setTimeout(()=>location.reload(), 2000);</script>"; // Fast refresh for fire safety
    html += "</div></body></html>";
    
    return html;
}

String getTimestamp() {
    return String(millis());
}