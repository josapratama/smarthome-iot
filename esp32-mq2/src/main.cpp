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
#define MQ2_ANALOG_PIN 6
#define MQ2_DIGITAL_PIN 7
#define LED_BUILTIN 8

// Device configuration
const char* DEVICE_TYPE = "gas_sensor";
const char* FIRMWARE_VERSION = "1.0.0";
String DEVICE_ID = "ESP32_C3_MQ2_" + String((uint32_t)ESP.getEfuseMac(), HEX);

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
int gasThreshold = 300;          // Gas concentration threshold (0-4095)
int calibrationTime = 30;        // Calibration time in seconds
int reportInterval = 30;         // Report interval in seconds
bool alarmEnabled = true;
bool autoCalibration = true;

// Timing variables
unsigned long lastMqttReconnect = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTelemetry = 0;
unsigned long lastSensorRead = 0;
unsigned long calibrationStart = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 30000;
const unsigned long SENSOR_READ_INTERVAL = 2000;

// Status variables
bool wifiConnected = false;
bool mqttConnected = false;
bool sensorCalibrated = false;
bool gasDetected = false;
unsigned long bootTime = 0;

// Sensor data
struct SensorData {
    int analogValue = 0;
    bool digitalValue = false;
    float gasPpm = 0.0;
    int airQualityIndex = 0;
    bool gasAlarm = false;
    bool valid = false;
    unsigned long timestamp = 0;
};

SensorData currentReading;
SensorData lastReading;

// Calibration data
int baselineValue = 0;
float ro = 10.0;  // Sensor resistance in clean air
const float RL = 5.0;  // Load resistance (5kΩ)

// Gas concentration curves (approximate values for MQ-2)
const float LPG_CURVE[3] = {2.3, 0.21, -0.47};
const float CO_CURVE[3] = {2.3, 0.72, -0.34};
const float SMOKE_CURVE[3] = {2.3, 0.53, -0.44};

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-C3 MQ-2 Gas Sensor ===");
    Serial.println("Firmware Version: " + String(FIRMWARE_VERSION));
    Serial.println("Device ID: " + DEVICE_ID);
    
    bootTime = millis();
    
    // Initialize pins
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(MQ2_DIGITAL_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
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
    
    // Start sensor calibration
    startCalibration();
    
    // Connected beeps
    buzzerBeep(3, 100);
}

void loop() {
    // Handle WiFi reset button
    handleWiFiReset();
    
    // Maintain WiFi connection
    maintainWiFiConnection();
    
    // Handle sensor calibration
    handleCalibration();
    
    // Maintain MQTT connection
    if (wifiConnected) {
        maintainMqttConnection();
        mqttClient.loop();
        
        // Send heartbeat
        if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
            publishHeartbeat();
            lastHeartbeat = millis();
        }
        
        // Read sensor and send telemetry
        if (millis() - lastSensorRead > SENSOR_READ_INTERVAL) {
            readSensor();
            lastSensorRead = millis();
        }
        
        // Send telemetry based on report interval
        if (millis() - lastTelemetry > (reportInterval * 1000)) {
            if (currentReading.valid) {
                publishTelemetry();
                checkAlarms();
            }
            lastTelemetry = millis();
        }
    }
    
    // Blink LED to show device is alive
    digitalWrite(LED_BUILTIN, (millis() / 1000) % 2);
    
    delay(100);
}

void startCalibration() {
    Serial.println("Starting MQ-2 sensor calibration...");
    Serial.println("Ensure sensor is in clean air for " + String(calibrationTime) + " seconds");
    
    calibrationStart = millis();
    sensorCalibrated = false;
    
    // Load previous calibration if available
    baselineValue = preferences.getInt("baseline", 0);
    ro = preferences.getFloat("ro", 10.0);
    
    if (baselineValue > 0) {
        Serial.println("Using previous calibration: baseline=" + String(baselineValue) + ", Ro=" + String(ro));
        sensorCalibrated = true;
    }
    
    buzzerBeep(2, 300);
}

void handleCalibration() {
    if (sensorCalibrated || calibrationStart == 0) return;
    
    unsigned long calibrationElapsed = millis() - calibrationStart;
    
    if (calibrationElapsed < (calibrationTime * 1000)) {
        // Still calibrating - read baseline values
        int reading = analogRead(MQ2_ANALOG_PIN);
        if (reading > baselineValue) {
            baselineValue = reading;
        }
        
        // Blink LED during calibration
        digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
        
    } else {
        // Calibration complete
        if (baselineValue > 0) {
            // Calculate Ro (sensor resistance in clean air)
            float rs = ((4095.0 / baselineValue) - 1.0) * RL;
            ro = rs / 9.83; // Rs/Ro ratio in clean air for MQ-2
            
            // Save calibration data
            preferences.putInt("baseline", baselineValue);
            preferences.putFloat("ro", ro);
            
            sensorCalibrated = true;
            
            Serial.println("Calibration completed!");
            Serial.println("Baseline value: " + String(baselineValue));
            Serial.println("Ro: " + String(ro) + " kΩ");
            
            buzzerBeep(3, 200);
        } else {
            Serial.println("Calibration failed - no readings obtained");
            buzzerBeep(5, 100);
            
            // Retry calibration
            startCalibration();
        }
    }
}

void readSensor() {
    SensorData newReading;
    newReading.timestamp = millis();
    
    // Read analog value (0-4095 for ESP32)
    newReading.analogValue = analogRead(MQ2_ANALOG_PIN);
    
    // Read digital value (HIGH = no gas, LOW = gas detected)
    newReading.digitalValue = digitalRead(MQ2_DIGITAL_PIN) == LOW;
    
    // Calculate gas concentration if calibrated
    if (sensorCalibrated && baselineValue > 0) {
        // Calculate sensor resistance
        float rs = ((4095.0 / newReading.analogValue) - 1.0) * RL;
        float ratio = rs / ro;
        
        // Calculate LPG concentration using curve fitting
        newReading.gasPpm = calculateGasPpm(ratio, LPG_CURVE);
        
        // Calculate air quality index (0-500 scale)
        newReading.airQualityIndex = map(newReading.analogValue, baselineValue, 4095, 0, 500);
        newReading.airQualityIndex = constrain(newReading.airQualityIndex, 0, 500);
        
        // Check gas alarm condition
        newReading.gasAlarm = (newReading.analogValue > gasThreshold) || newReading.digitalValue;
        
        newReading.valid = true;
    } else {
        newReading.valid = false;
    }
    
    // Update gas detection status
    if (newReading.gasAlarm != gasDetected) {
        gasDetected = newReading.gasAlarm;
        if (gasDetected) {
            Serial.println("GAS DETECTED!");
            buzzerBeep(10, 100); // Rapid beeps for gas alarm
        } else {
            Serial.println("Gas cleared");
            buzzerBeep(2, 200);
        }
    }
    
    // Update readings
    lastReading = currentReading;
    currentReading = newReading;
    
    // Print readings to serial
    if (currentReading.valid) {
        Serial.println("=== MQ-2 Readings ===");
        Serial.printf("Analog Value: %d\n", currentReading.analogValue);
        Serial.printf("Digital Value: %s\n", currentReading.digitalValue ? "GAS" : "CLEAR");
        Serial.printf("Gas PPM: %.2f\n", currentReading.gasPpm);
        Serial.printf("Air Quality Index: %d\n", currentReading.airQualityIndex);
        Serial.printf("Gas Alarm: %s\n", currentReading.gasAlarm ? "ACTIVE" : "CLEAR");
        Serial.println("====================");
    }
}

float calculateGasPpm(float ratio, const float* curve) {
    // Use logarithmic curve fitting: log(ppm) = a * log(ratio) + b
    float a = curve[1];
    float b = curve[2];
    
    if (ratio <= 0) return 0;
    
    float logPpm = a * log10(ratio) + b;
    return pow(10, logPpm);
}

void checkAlarms() {
    if (!alarmEnabled || !currentReading.valid) return;
    
    if (currentReading.gasAlarm) {
        String alarmMessage = "Gas detected! ";
        alarmMessage += "Level: " + String(currentReading.gasPpm, 2) + " PPM, ";
        alarmMessage += "AQI: " + String(currentReading.airQualityIndex);
        
        Serial.println("ALARM: " + alarmMessage);
        publishAlarm(alarmMessage);
    }
}

void publishAlarm(String message) {
    DynamicJsonDocument doc(512);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    doc["alarmType"] = "GAS_DETECTED";
    doc["message"] = message;
    doc["severity"] = "CRITICAL";
    
    doc["data"]["gasPpm"] = currentReading.gasPpm;
    doc["data"]["airQualityIndex"] = currentReading.airQualityIndex;
    doc["data"]["analogValue"] = currentReading.analogValue;
    doc["data"]["digitalValue"] = currentReading.digitalValue;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/alarms";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Gas alarm published: " + message);
}

void loadConfiguration() {
    gasThreshold = preferences.getInt("gas_thresh", 300);
    calibrationTime = preferences.getInt("cal_time", 30);
    reportInterval = preferences.getInt("report_int", 30);
    alarmEnabled = preferences.getBool("alarm_en", true);
    autoCalibration = preferences.getBool("auto_cal", true);
    
    // Load MQTT config
    mqtt_server = preferences.getString("mqtt_server", "localhost");
    mqtt_port = preferences.getString("mqtt_port", "1883");
    mqtt_user = preferences.getString("mqtt_user", "");
    mqtt_password = preferences.getString("mqtt_password", "");
    home_id = preferences.getString("home_id", "default_home");
    
    Serial.println("Configuration loaded:");
    Serial.println("Gas threshold: " + String(gasThreshold));
    Serial.println("Calibration time: " + String(calibrationTime) + "s");
    Serial.println("Report interval: " + String(reportInterval) + "s");
}

void saveConfiguration() {
    preferences.putInt("gas_thresh", gasThreshold);
    preferences.putInt("cal_time", calibrationTime);
    preferences.putInt("report_int", reportInterval);
    preferences.putBool("alarm_en", alarmEnabled);
    preferences.putBool("auto_cal", autoCalibration);
    
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
    
    if (!wm.autoConnect(("SmartHome-MQ2-" + DEVICE_ID).c_str(), "smarthome123")) {
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
        Serial.println("Manual calibration requested");
        startCalibration();
        buzzerBeep(2, 200);
    } else if (commandType == "GET_READING") {
        publishTelemetry();
    } else if (commandType == "RESET_CALIBRATION") {
        preferences.remove("baseline");
        preferences.remove("ro");
        sensorCalibrated = false;
        Serial.println("Calibration data cleared");
        buzzerBeep(3, 300);
    }
}

void handleConfig(DynamicJsonDocument& doc) {
    bool configChanged = false;
    
    if (doc.containsKey("gasThreshold")) {
        gasThreshold = doc["gasThreshold"];
        configChanged = true;
    }
    
    if (doc.containsKey("calibrationTime")) {
        calibrationTime = doc["calibrationTime"];
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
    
    if (doc.containsKey("autoCalibration")) {
        autoCalibration = doc["autoCalibration"];
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
    doc["status"]["sensorCalibrated"] = sensorCalibrated;
    doc["status"]["gasDetected"] = gasDetected;
    
    doc["config"]["gasThreshold"] = gasThreshold;
    doc["config"]["calibrationTime"] = calibrationTime;
    doc["config"]["reportInterval"] = reportInterval;
    doc["config"]["alarmEnabled"] = alarmEnabled;
    doc["config"]["autoCalibration"] = autoCalibration;
    
    if (sensorCalibrated) {
        doc["calibration"]["baseline"] = baselineValue;
        doc["calibration"]["ro"] = ro;
    }
    
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
    doc["sensorCalibrated"] = sensorCalibrated;
    doc["gasDetected"] = gasDetected;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/heartbeat";
    mqttClient.publish(topic.c_str(), payload.c_str());
}

void publishTelemetry() {
    if (!currentReading.valid) return;
    
    DynamicJsonDocument doc(1024);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    
    // MQ-2 sensor data
    doc["data"]["analogValue"] = currentReading.analogValue;
    doc["data"]["digitalValue"] = currentReading.digitalValue;
    doc["data"]["gasPpm"] = round(currentReading.gasPpm * 100) / 100.0;
    doc["data"]["airQualityIndex"] = currentReading.airQualityIndex;
    doc["data"]["gasAlarm"] = currentReading.gasAlarm;
    
    // Air quality classification
    String aqiLevel = "Good";
    if (currentReading.airQualityIndex > 300) aqiLevel = "Hazardous";
    else if (currentReading.airQualityIndex > 200) aqiLevel = "Very Unhealthy";
    else if (currentReading.airQualityIndex > 150) aqiLevel = "Unhealthy";
    else if (currentReading.airQualityIndex > 100) aqiLevel = "Unhealthy for Sensitive";
    else if (currentReading.airQualityIndex > 50) aqiLevel = "Moderate";
    
    doc["data"]["airQualityLevel"] = aqiLevel;
    
    // System status
    doc["status"]["sensorCalibrated"] = sensorCalibrated;
    doc["status"]["gasDetected"] = gasDetected;
    doc["status"]["uptime"] = millis() - bootTime;
    doc["status"]["rssi"] = WiFi.RSSI();
    doc["status"]["freeHeap"] = ESP.getFreeHeap();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/telemetry";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Telemetry published - Gas: " + String(currentReading.gasPpm, 2) + " PPM, AQI: " + String(currentReading.airQualityIndex));
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
        
        if (currentReading.valid) {
            doc["analogValue"] = currentReading.analogValue;
            doc["digitalValue"] = currentReading.digitalValue;
            doc["gasPpm"] = currentReading.gasPpm;
            doc["airQualityIndex"] = currentReading.airQualityIndex;
            doc["gasAlarm"] = currentReading.gasAlarm;
            doc["sensorCalibrated"] = sensorCalibrated;
        } else {
            doc["error"] = "No valid readings available";
        }
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
}

String generateWebPage() {
    String html = "<!DOCTYPE html><html><head><title>MQ-2 Gas Sensor</title>";
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
    html += "<h1>🔥 MQ-2 Gas Sensor</h1>";
    html += "<p><strong>Device ID:</strong> " + DEVICE_ID + "</p>";
    
    html += "<div class='status " + String(sensorCalibrated ? "online" : "warning") + "'>";
    html += "Calibration: " + String(sensorCalibrated ? "Complete" : "In Progress") + "</div>";
    
    html += "<div class='status " + String(gasDetected ? "offline" : "online") + "'>";
    html += "Gas Status: " + String(gasDetected ? "DETECTED" : "Clear") + "</div>";
    
    if (currentReading.valid) {
        html += "<h2>Current Readings</h2>";
        html += "<div class='reading'><span>Analog Value:</span><span class='value'>" + String(currentReading.analogValue) + "</span></div>";
        html += "<div class='reading'><span>Digital Status:</span><span class='" + String(currentReading.digitalValue ? "alarm" : "value") + "'>" + String(currentReading.digitalValue ? "GAS" : "CLEAR") + "</span></div>";
        html += "<div class='reading'><span>Gas Concentration:</span><span class='value'>" + String(currentReading.gasPpm, 2) + " PPM</span></div>";
        html += "<div class='reading'><span>Air Quality Index:</span><span class='value'>" + String(currentReading.airQualityIndex) + "</span></div>";
        html += "<div class='reading'><span>Gas Alarm:</span><span class='" + String(currentReading.gasAlarm ? "alarm" : "value") + "'>" + String(currentReading.gasAlarm ? "ACTIVE" : "CLEAR") + "</span></div>";
    } else {
        html += "<p>Sensor calibrating or no valid readings available</p>";
    }
    
    html += "<script>setTimeout(()=>location.reload(), 5000);</script>";
    html += "</div></body></html>";
    
    return html;
}

String getTimestamp() {
    return String(millis());
}