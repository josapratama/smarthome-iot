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
#define TRIG_PIN 10
#define ECHO_PIN 9
#define LED_BUILTIN 8

// Device configuration
const char* DEVICE_TYPE = "ultrasonic_sensor";
const char* FIRMWARE_VERSION = "1.0.0";
String DEVICE_ID = "ESP32_C3_ULTRA_" + String((uint32_t)ESP.getEfuseMac(), HEX);

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
float minDistance = 5.0;         // Minimum distance threshold (cm)
float maxDistance = 300.0;       // Maximum distance threshold (cm)
int reportInterval = 30;         // Report interval in seconds
bool alarmEnabled = true;
bool motionDetection = true;
float motionThreshold = 5.0;     // Motion detection threshold (cm change)
String applicationMode = "distance"; // distance, water_level, parking, motion

// Timing variables
unsigned long lastMqttReconnect = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTelemetry = 0;
unsigned long lastSensorRead = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 30000;
const unsigned long SENSOR_READ_INTERVAL = 1000;

// Status variables
bool wifiConnected = false;
bool mqttConnected = false;
bool objectDetected = false;
bool motionDetected = false;
unsigned long bootTime = 0;

// Sensor data
struct SensorData {
    float distance = 0.0;
    float temperature = 20.0;       // For sound speed compensation
    bool validReading = false;
    bool proximityAlarm = false;
    bool motionAlarm = false;
    unsigned long timestamp = 0;
};

SensorData currentReading;
SensorData lastReading;

// Moving average for noise reduction
const int SAMPLE_SIZE = 5;
float distanceSamples[SAMPLE_SIZE];
int sampleIndex = 0;
bool samplesReady = false;

// Motion detection
float previousDistance = 0.0;
unsigned long lastMotionTime = 0;
const unsigned long MOTION_TIMEOUT = 10000; // 10 seconds

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-C3 Ultrasonic Distance Sensor ===");
    Serial.println("Firmware Version: " + String(FIRMWARE_VERSION));
    Serial.println("Device ID: " + DEVICE_ID);
    
    bootTime = millis();
    
    // Initialize pins
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Initialize distance samples array
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        distanceSamples[i] = 0.0;
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
    Serial.println("Application Mode: " + applicationMode);
    
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
        
        // Read sensor
        if (millis() - lastSensorRead > SENSOR_READ_INTERVAL) {
            readSensor();
            lastSensorRead = millis();
        }
        
        // Send telemetry based on report interval
        if (millis() - lastTelemetry > (reportInterval * 1000)) {
            if (currentReading.validReading) {
                publishTelemetry();
                checkAlarms();
            }
            lastTelemetry = millis();
        }
    }
    
    // Handle motion detection timeout
    if (motionDetected && (millis() - lastMotionTime > MOTION_TIMEOUT)) {
        motionDetected = false;
        Serial.println("Motion detection timeout - no motion");
    }
    
    // Blink LED to show device is alive
    digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
    
    delay(100);
}

void readSensor() {
    SensorData newReading;
    newReading.timestamp = millis();
    
    // Trigger ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read echo pulse duration
    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
    
    if (duration > 0) {
        // Calculate distance in cm
        // Speed of sound = 343 m/s at 20°C
        // Distance = (duration * speed) / 2 (round trip)
        float soundSpeed = 343.0 + (0.6 * currentReading.temperature); // Temperature compensation
        float rawDistance = (duration * soundSpeed * 0.0001) / 2.0; // Convert to cm
        
        // Validate reading range (HC-SR04: 2cm - 400cm)
        if (rawDistance >= 2.0 && rawDistance <= 400.0) {
            // Add to moving average
            distanceSamples[sampleIndex] = rawDistance;
            sampleIndex = (sampleIndex + 1) % SAMPLE_SIZE;
            
            if (sampleIndex == 0) samplesReady = true;
            
            // Calculate moving average if we have enough samples
            if (samplesReady) {
                float sum = 0.0;
                for (int i = 0; i < SAMPLE_SIZE; i++) {
                    sum += distanceSamples[i];
                }
                newReading.distance = sum / SAMPLE_SIZE;
            } else {
                newReading.distance = rawDistance;
            }
            
            newReading.validReading = true;
            
            // Check proximity alarm
            newReading.proximityAlarm = (newReading.distance < minDistance) || (newReading.distance > maxDistance);
            
            // Motion detection
            if (motionDetection && previousDistance > 0) {
                float distanceChange = abs(newReading.distance - previousDistance);
                if (distanceChange > motionThreshold) {
                    newReading.motionAlarm = true;
                    if (!motionDetected) {
                        motionDetected = true;
                        lastMotionTime = millis();
                        Serial.println("Motion detected! Distance change: " + String(distanceChange, 2) + " cm");
                        buzzerBeep(2, 100);
                    } else {
                        lastMotionTime = millis(); // Reset timeout
                    }
                } else {
                    newReading.motionAlarm = false;
                }
            }
            
            previousDistance = newReading.distance;
            
        } else {
            newReading.validReading = false;
            Serial.println("Invalid distance reading: " + String(rawDistance) + " cm");
        }
    } else {
        newReading.validReading = false;
        Serial.println("Ultrasonic sensor timeout - no echo received");
    }
    
    // Update object detection status
    bool previousObjectDetected = objectDetected;
    objectDetected = newReading.proximityAlarm;
    
    if (objectDetected && !previousObjectDetected) {
        Serial.println("Object detected in range!");
        buzzerBeep(1, 200);
    } else if (!objectDetected && previousObjectDetected) {
        Serial.println("Object cleared from range");
        buzzerBeep(1, 100);
    }
    
    // Update readings
    lastReading = currentReading;
    currentReading = newReading;
    
    // Print readings to serial
    if (currentReading.validReading) {
        Serial.println("=== Ultrasonic Readings ===");
        Serial.printf("Distance: %.2f cm\n", currentReading.distance);
        Serial.printf("Proximity Alarm: %s\n", currentReading.proximityAlarm ? "ACTIVE" : "CLEAR");
        Serial.printf("Motion Alarm: %s\n", currentReading.motionAlarm ? "ACTIVE" : "CLEAR");
        Serial.printf("Motion Detected: %s\n", motionDetected ? "YES" : "NO");
        
        // Application-specific output
        if (applicationMode == "water_level") {
            float waterLevel = maxDistance - currentReading.distance;
            Serial.printf("Water Level: %.2f cm\n", waterLevel);
        } else if (applicationMode == "parking") {
            if (currentReading.distance < 50) {
                Serial.println("Parking Status: OCCUPIED");
            } else {
                Serial.println("Parking Status: AVAILABLE");
            }
        }
        
        Serial.println("==========================");
    }
}

void checkAlarms() {
    if (!alarmEnabled || !currentReading.validReading) return;
    
    String alarmMessage = "";
    bool alarmTriggered = false;
    
    // Proximity alarm
    if (currentReading.proximityAlarm) {
        if (currentReading.distance < minDistance) {
            alarmMessage += "Object too close: " + String(currentReading.distance, 2) + " cm ";
        } else {
            alarmMessage += "Object too far: " + String(currentReading.distance, 2) + " cm ";
        }
        alarmTriggered = true;
    }
    
    // Motion alarm
    if (currentReading.motionAlarm) {
        alarmMessage += "Motion detected ";
        alarmTriggered = true;
    }
    
    if (alarmTriggered) {
        Serial.println("ALARM: " + alarmMessage);
        publishAlarm(alarmMessage);
        buzzerBeep(3, 200);
    }
}

void publishAlarm(String message) {
    DynamicJsonDocument doc(512);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    doc["alarmType"] = currentReading.motionAlarm ? "MOTION_DETECTED" : "PROXIMITY_ALARM";
    doc["message"] = message;
    doc["severity"] = currentReading.motionAlarm ? "MEDIUM" : "LOW";
    
    doc["data"]["distance"] = currentReading.distance;
    doc["data"]["proximityAlarm"] = currentReading.proximityAlarm;
    doc["data"]["motionAlarm"] = currentReading.motionAlarm;
    doc["data"]["applicationMode"] = applicationMode;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/alarms";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Alarm published: " + message);
}

void loadConfiguration() {
    minDistance = preferences.getFloat("min_dist", 5.0);
    maxDistance = preferences.getFloat("max_dist", 300.0);
    reportInterval = preferences.getInt("report_int", 30);
    alarmEnabled = preferences.getBool("alarm_en", true);
    motionDetection = preferences.getBool("motion_det", true);
    motionThreshold = preferences.getFloat("motion_thresh", 5.0);
    applicationMode = preferences.getString("app_mode", "distance");
    
    // Load MQTT config
    mqtt_server = preferences.getString("mqtt_server", "localhost");
    mqtt_port = preferences.getString("mqtt_port", "1883");
    mqtt_user = preferences.getString("mqtt_user", "");
    mqtt_password = preferences.getString("mqtt_password", "");
    home_id = preferences.getString("home_id", "default_home");
    
    Serial.println("Configuration loaded:");
    Serial.println("Distance range: " + String(minDistance) + " - " + String(maxDistance) + " cm");
    Serial.println("Motion threshold: " + String(motionThreshold) + " cm");
    Serial.println("Application mode: " + applicationMode);
    Serial.println("Report interval: " + String(reportInterval) + "s");
}

void saveConfiguration() {
    preferences.putFloat("min_dist", minDistance);
    preferences.putFloat("max_dist", maxDistance);
    preferences.putInt("report_int", reportInterval);
    preferences.putBool("alarm_en", alarmEnabled);
    preferences.putBool("motion_det", motionDetection);
    preferences.putFloat("motion_thresh", motionThreshold);
    preferences.putString("app_mode", applicationMode);
    
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
    
    if (!wm.autoConnect(("SmartHome-ULTRA-" + DEVICE_ID).c_str(), "smarthome123")) {
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
    
    if (commandType == "CALIBRATE") {
        Serial.println("Calibration requested - measuring baseline");
        // Take several readings for calibration
        float sum = 0;
        int validReadings = 0;
        
        for (int i = 0; i < 10; i++) {
            readSensor();
            if (currentReading.validReading) {
                sum += currentReading.distance;
                validReadings++;
            }
            delay(100);
        }
        
        if (validReadings > 0) {
            float baseline = sum / validReadings;
            Serial.println("Calibration complete - baseline: " + String(baseline, 2) + " cm");
            buzzerBeep(3, 200);
        } else {
            Serial.println("Calibration failed - no valid readings");
            buzzerBeep(5, 100);
        }
        
    } else if (commandType == "GET_READING") {
        publishTelemetry();
    } else if (commandType == "SET_MODE") {
        String newMode = doc["payload"]["mode"];
        if (newMode == "distance" || newMode == "water_level" || newMode == "parking" || newMode == "motion") {
            applicationMode = newMode;
            preferences.putString("app_mode", applicationMode);
            Serial.println("Application mode changed to: " + applicationMode);
            buzzerBeep(2, 100);
        }
    }
}

void handleConfig(DynamicJsonDocument& doc) {
    bool configChanged = false;
    
    if (doc.containsKey("minDistance")) {
        minDistance = doc["minDistance"];
        configChanged = true;
    }
    
    if (doc.containsKey("maxDistance")) {
        maxDistance = doc["maxDistance"];
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
    
    if (doc.containsKey("motionDetection")) {
        motionDetection = doc["motionDetection"];
        configChanged = true;
    }
    
    if (doc.containsKey("motionThreshold")) {
        motionThreshold = doc["motionThreshold"];
        configChanged = true;
    }
    
    if (doc.containsKey("applicationMode")) {
        applicationMode = doc["applicationMode"];
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
    doc["status"]["objectDetected"] = objectDetected;
    doc["status"]["motionDetected"] = motionDetected;
    
    doc["config"]["minDistance"] = minDistance;
    doc["config"]["maxDistance"] = maxDistance;
    doc["config"]["reportInterval"] = reportInterval;
    doc["config"]["alarmEnabled"] = alarmEnabled;
    doc["config"]["motionDetection"] = motionDetection;
    doc["config"]["motionThreshold"] = motionThreshold;
    doc["config"]["applicationMode"] = applicationMode;
    
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
    doc["objectDetected"] = objectDetected;
    doc["motionDetected"] = motionDetected;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/heartbeat";
    mqttClient.publish(topic.c_str(), payload.c_str());
}

void publishTelemetry() {
    if (!currentReading.validReading) return;
    
    DynamicJsonDocument doc(1024);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    
    // Ultrasonic sensor data
    doc["data"]["distance"] = round(currentReading.distance * 100) / 100.0;
    doc["data"]["proximityAlarm"] = currentReading.proximityAlarm;
    doc["data"]["motionAlarm"] = currentReading.motionAlarm;
    doc["data"]["objectDetected"] = objectDetected;
    doc["data"]["motionDetected"] = motionDetected;
    doc["data"]["applicationMode"] = applicationMode;
    
    // Application-specific data
    if (applicationMode == "water_level") {
        float waterLevel = maxDistance - currentReading.distance;
        doc["data"]["waterLevel"] = round(waterLevel * 100) / 100.0;
        doc["data"]["waterPercentage"] = round((waterLevel / maxDistance) * 10000) / 100.0;
    } else if (applicationMode == "parking") {
        doc["data"]["parkingStatus"] = currentReading.distance < 50 ? "OCCUPIED" : "AVAILABLE";
        doc["data"]["parkingDistance"] = currentReading.distance;
    }
    
    // Distance classification
    String distanceClass = "Normal";
    if (currentReading.distance < 10) distanceClass = "Very Close";
    else if (currentReading.distance < 30) distanceClass = "Close";
    else if (currentReading.distance < 100) distanceClass = "Medium";
    else if (currentReading.distance < 200) distanceClass = "Far";
    else distanceClass = "Very Far";
    
    doc["data"]["distanceClass"] = distanceClass;
    
    // System status
    doc["status"]["uptime"] = millis() - bootTime;
    doc["status"]["rssi"] = WiFi.RSSI();
    doc["status"]["freeHeap"] = ESP.getFreeHeap();
    doc["status"]["samplesReady"] = samplesReady;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/telemetry";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Telemetry published - Distance: " + String(currentReading.distance, 2) + " cm");
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
        
        if (currentReading.validReading) {
            doc["distance"] = currentReading.distance;
            doc["proximityAlarm"] = currentReading.proximityAlarm;
            doc["motionAlarm"] = currentReading.motionAlarm;
            doc["objectDetected"] = objectDetected;
            doc["motionDetected"] = motionDetected;
            doc["applicationMode"] = applicationMode;
            
            if (applicationMode == "water_level") {
                doc["waterLevel"] = maxDistance - currentReading.distance;
            } else if (applicationMode == "parking") {
                doc["parkingStatus"] = currentReading.distance < 50 ? "OCCUPIED" : "AVAILABLE";
            }
        } else {
            doc["error"] = "No valid readings available";
        }
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
}

String generateWebPage() {
    String html = "<!DOCTYPE html><html><head><title>Ultrasonic Sensor</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;margin:20px;background:#f0f0f0}";
    html += ".container{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
    html += ".reading{display:flex;justify-content:space-between;padding:10px;margin:5px 0;background:#f8f9fa;border-radius:5px}";
    html += ".value{font-weight:bold;color:#007bff}";
    html += ".alarm{color:#dc3545;font-weight:bold}";
    html += ".status{padding:10px;margin:10px 0;border-radius:5px}";
    html += ".online{background:#d4edda;color:#155724}";
    html += ".offline{background:#f8d7da;color:#721c24}";
    html += ".warning{background:#fff3cd;color:#856404}";
    html += "</style></head><body>";
    
    html += "<div class='container'>";
    html += "<h1>📏 Ultrasonic Distance Sensor</h1>";
    html += "<p><strong>Device ID:</strong> " + DEVICE_ID + "</p>";
    html += "<p><strong>Application Mode:</strong> " + applicationMode + "</p>";
    
    html += "<div class='status " + String(objectDetected ? "warning" : "online") + "'>";
    html += "Object Detection: " + String(objectDetected ? "DETECTED" : "Clear") + "</div>";
    
    html += "<div class='status " + String(motionDetected ? "warning" : "online") + "'>";
    html += "Motion Detection: " + String(motionDetected ? "ACTIVE" : "Inactive") + "</div>";
    
    if (currentReading.validReading) {
        html += "<h2>Current Readings</h2>";
        html += "<div class='reading'><span>Distance:</span><span class='value'>" + String(currentReading.distance, 2) + " cm</span></div>";
        html += "<div class='reading'><span>Proximity Alarm:</span><span class='" + String(currentReading.proximityAlarm ? "alarm" : "value") + "'>" + String(currentReading.proximityAlarm ? "ACTIVE" : "CLEAR") + "</span></div>";
        html += "<div class='reading'><span>Motion Alarm:</span><span class='" + String(currentReading.motionAlarm ? "alarm" : "value") + "'>" + String(currentReading.motionAlarm ? "ACTIVE" : "CLEAR") + "</span></div>";
        
        // Application-specific display
        if (applicationMode == "water_level") {
            float waterLevel = maxDistance - currentReading.distance;
            html += "<div class='reading'><span>Water Level:</span><span class='value'>" + String(waterLevel, 2) + " cm</span></div>";
            html += "<div class='reading'><span>Water Percentage:</span><span class='value'>" + String((waterLevel / maxDistance) * 100, 1) + "%</span></div>";
        } else if (applicationMode == "parking") {
            String parkingStatus = currentReading.distance < 50 ? "OCCUPIED" : "AVAILABLE";
            html += "<div class='reading'><span>Parking Status:</span><span class='" + String(parkingStatus == "OCCUPIED" ? "alarm" : "value") + "'>" + parkingStatus + "</span></div>";
        }
        
    } else {
        html += "<p>No valid sensor readings available</p>";
    }
    
    html += "<script>setTimeout(()=>location.reload(), 5000);</script>";
    html += "</div></body></html>";
    
    return html;
}

String getTimestamp() {
    return String(millis());
}