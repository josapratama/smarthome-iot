#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Preferences.h>
#include <PZEM004Tv30.h>

// Pin definitions
#define WIFI_RESET_PIN 2
#define BUZZER_PIN 3
#define PZEM_RX_PIN 4
#define PZEM_TX_PIN 5
#define LED_BUILTIN 8

// Device configuration
const char* DEVICE_TYPE = "energy_monitor";
const char* FIRMWARE_VERSION = "1.0.0";
String DEVICE_ID = "ESP32_C3_PZEM_" + String((uint32_t)ESP.getEfuseMac(), HEX);

// PZEM-004T sensor
PZEM004Tv30 pzem(Serial1, PZEM_RX_PIN, PZEM_TX_PIN);

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
float voltageThreshold = 250.0;  // Maximum voltage threshold
float currentThreshold = 20.0;   // Maximum current threshold
float powerThreshold = 3000.0;   // Maximum power threshold
int reportInterval = 30;         // Report interval in seconds
bool alarmEnabled = true;

// Timing variables
unsigned long lastMqttReconnect = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTelemetry = 0;
unsigned long lastSensorRead = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;
const unsigned long HEARTBEAT_INTERVAL = 30000;
const unsigned long SENSOR_READ_INTERVAL = 5000;

// Status variables
bool wifiConnected = false;
bool mqttConnected = false;
bool sensorConnected = false;
unsigned long bootTime = 0;

// Sensor data
struct SensorData {
    float voltage = 0.0;
    float current = 0.0;
    float power = 0.0;
    float energy = 0.0;
    float frequency = 0.0;
    float powerFactor = 0.0;
    bool valid = false;
    unsigned long timestamp = 0;
};

SensorData currentReading;
SensorData lastReading;

// Energy accumulation
float totalEnergy = 0.0;
unsigned long energyStartTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32-C3 PZEM-004T Energy Monitor ===");
    Serial.println("Firmware Version: " + String(FIRMWARE_VERSION));
    Serial.println("Device ID: " + DEVICE_ID);
    
    bootTime = millis();
    energyStartTime = bootTime;
    
    // Initialize pins
    pinMode(WIFI_RESET_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Initialize preferences
    preferences.begin("smarthome", false);
    loadConfiguration();
    
    // Initialize PZEM sensor
    Serial.println("Initializing PZEM-004T sensor...");
    // Reset energy counter (optional)
    // pzem.resetEnergy();
    
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
    
    // Test sensor connection
    testSensorConnection();
    
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

void readSensor() {
    SensorData newReading;
    newReading.timestamp = millis();
    
    // Read all parameters from PZEM
    newReading.voltage = pzem.voltage();
    newReading.current = pzem.current();
    newReading.power = pzem.power();
    newReading.energy = pzem.energy();
    newReading.frequency = pzem.frequency();
    newReading.powerFactor = pzem.pf();
    
    // Check if readings are valid (PZEM returns NaN for invalid readings)
    newReading.valid = !isnan(newReading.voltage) && 
                      !isnan(newReading.current) && 
                      !isnan(newReading.power) &&
                      newReading.voltage > 0;
    
    if (newReading.valid) {
        if (!sensorConnected) {
            Serial.println("PZEM sensor connected!");
            sensorConnected = true;
            buzzerBeep(2, 100);
        }
        
        // Update current reading
        lastReading = currentReading;
        currentReading = newReading;
        
        // Calculate energy consumption since boot
        if (lastReading.valid && currentReading.power > 0) {
            float timeDiff = (currentReading.timestamp - lastReading.timestamp) / 1000.0 / 3600.0; // hours
            totalEnergy += (currentReading.power * timeDiff) / 1000.0; // kWh
        }
        
        // Print readings to serial
        Serial.println("=== PZEM Readings ===");
        Serial.printf("Voltage: %.2f V\n", currentReading.voltage);
        Serial.printf("Current: %.3f A\n", currentReading.current);
        Serial.printf("Power: %.2f W\n", currentReading.power);
        Serial.printf("Energy: %.3f kWh\n", currentReading.energy);
        Serial.printf("Frequency: %.1f Hz\n", currentReading.frequency);
        Serial.printf("Power Factor: %.2f\n", currentReading.powerFactor);
        Serial.printf("Total Energy (calculated): %.3f kWh\n", totalEnergy);
        Serial.println("====================");
        
    } else {
        if (sensorConnected) {
            Serial.println("PZEM sensor disconnected!");
            sensorConnected = false;
            buzzerBeep(1, 1000); // Long beep for error
        }
    }
}

void checkAlarms() {
    if (!alarmEnabled || !currentReading.valid) return;
    
    bool alarmTriggered = false;
    String alarmMessage = "";
    
    // Check voltage threshold
    if (currentReading.voltage > voltageThreshold) {
        alarmTriggered = true;
        alarmMessage += "High voltage: " + String(currentReading.voltage) + "V ";
    }
    
    // Check current threshold
    if (currentReading.current > currentThreshold) {
        alarmTriggered = true;
        alarmMessage += "High current: " + String(currentReading.current) + "A ";
    }
    
    // Check power threshold
    if (currentReading.power > powerThreshold) {
        alarmTriggered = true;
        alarmMessage += "High power: " + String(currentReading.power) + "W ";
    }
    
    if (alarmTriggered) {
        Serial.println("ALARM: " + alarmMessage);
        publishAlarm(alarmMessage);
        buzzerBeep(5, 200); // Alarm beeps
    }
}

void publishAlarm(String message) {
    DynamicJsonDocument doc(512);
    
    doc["deviceId"] = DEVICE_ID;
    doc["deviceType"] = DEVICE_TYPE;
    doc["timestamp"] = getTimestamp();
    doc["alarmType"] = "THRESHOLD_EXCEEDED";
    doc["message"] = message;
    doc["severity"] = "HIGH";
    
    doc["data"]["voltage"] = currentReading.voltage;
    doc["data"]["current"] = currentReading.current;
    doc["data"]["power"] = currentReading.power;
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/alarms";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Alarm published: " + message);
}

void testSensorConnection() {
    Serial.println("Testing PZEM sensor connection...");
    
    float voltage = pzem.voltage();
    if (!isnan(voltage) && voltage > 0) {
        Serial.println("PZEM sensor connected successfully!");
        Serial.println("Voltage: " + String(voltage) + "V");
        sensorConnected = true;
        buzzerBeep(2, 100);
    } else {
        Serial.println("PZEM sensor not detected!");
        Serial.println("Check wiring and power supply");
        sensorConnected = false;
        buzzerBeep(3, 500);
    }
}

void loadConfiguration() {
    voltageThreshold = preferences.getFloat("volt_thresh", 250.0);
    currentThreshold = preferences.getFloat("curr_thresh", 20.0);
    powerThreshold = preferences.getFloat("pow_thresh", 3000.0);
    reportInterval = preferences.getInt("report_int", 30);
    alarmEnabled = preferences.getBool("alarm_en", true);
    
    // Load MQTT config
    mqtt_server = preferences.getString("mqtt_server", "localhost");
    mqtt_port = preferences.getString("mqtt_port", "1883");
    mqtt_user = preferences.getString("mqtt_user", "");
    mqtt_password = preferences.getString("mqtt_password", "");
    home_id = preferences.getString("home_id", "default_home");
    
    Serial.println("Configuration loaded:");
    Serial.println("Voltage threshold: " + String(voltageThreshold) + "V");
    Serial.println("Current threshold: " + String(currentThreshold) + "A");
    Serial.println("Power threshold: " + String(powerThreshold) + "W");
    Serial.println("Report interval: " + String(reportInterval) + "s");
}

void saveConfiguration() {
    preferences.putFloat("volt_thresh", voltageThreshold);
    preferences.putFloat("curr_thresh", currentThreshold);
    preferences.putFloat("pow_thresh", powerThreshold);
    preferences.putInt("report_int", reportInterval);
    preferences.putBool("alarm_en", alarmEnabled);
    
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
    
    if (!wm.autoConnect(("SmartHome-PZEM-" + DEVICE_ID).c_str(), "smarthome123")) {
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
    
    if (commandType == "RESET_ENERGY") {
        pzem.resetEnergy();
        totalEnergy = 0.0;
        energyStartTime = millis();
        Serial.println("Energy counter reset");
        buzzerBeep(2, 200);
        publishStatus();
    } else if (commandType == "GET_READING") {
        publishTelemetry();
    } else if (commandType == "CALIBRATE") {
        // PZEM doesn't support calibration via software
        Serial.println("Calibration not supported by PZEM-004T");
    }
}

void handleConfig(DynamicJsonDocument& doc) {
    bool configChanged = false;
    
    if (doc.containsKey("voltageThreshold")) {
        voltageThreshold = doc["voltageThreshold"];
        configChanged = true;
    }
    
    if (doc.containsKey("currentThreshold")) {
        currentThreshold = doc["currentThreshold"];
        configChanged = true;
    }
    
    if (doc.containsKey("powerThreshold")) {
        powerThreshold = doc["powerThreshold"];
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
    doc["status"]["sensorConnected"] = sensorConnected;
    
    doc["config"]["voltageThreshold"] = voltageThreshold;
    doc["config"]["currentThreshold"] = currentThreshold;
    doc["config"]["powerThreshold"] = powerThreshold;
    doc["config"]["reportInterval"] = reportInterval;
    doc["config"]["alarmEnabled"] = alarmEnabled;
    
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
    doc["sensorConnected"] = sensorConnected;
    
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
    
    // PZEM sensor data
    doc["data"]["voltage"] = round(currentReading.voltage * 100) / 100.0;
    doc["data"]["current"] = round(currentReading.current * 1000) / 1000.0;
    doc["data"]["power"] = round(currentReading.power * 100) / 100.0;
    doc["data"]["energy"] = round(currentReading.energy * 1000) / 1000.0;
    doc["data"]["frequency"] = round(currentReading.frequency * 10) / 10.0;
    doc["data"]["powerFactor"] = round(currentReading.powerFactor * 100) / 100.0;
    doc["data"]["totalEnergy"] = round(totalEnergy * 1000) / 1000.0;
    
    // System status
    doc["status"]["sensorConnected"] = sensorConnected;
    doc["status"]["uptime"] = millis() - bootTime;
    doc["status"]["rssi"] = WiFi.RSSI();
    doc["status"]["freeHeap"] = ESP.getFreeHeap();
    
    String payload;
    serializeJson(doc, payload);
    
    String topic = "smarthome/" + home_id + "/" + DEVICE_ID + "/telemetry";
    mqttClient.publish(topic.c_str(), payload.c_str());
    
    Serial.println("Telemetry published - Power: " + String(currentReading.power) + "W");
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
            doc["voltage"] = currentReading.voltage;
            doc["current"] = currentReading.current;
            doc["power"] = currentReading.power;
            doc["energy"] = currentReading.energy;
            doc["frequency"] = currentReading.frequency;
            doc["powerFactor"] = currentReading.powerFactor;
            doc["totalEnergy"] = totalEnergy;
            doc["sensorConnected"] = sensorConnected;
        } else {
            doc["error"] = "No valid readings available";
        }
        
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });
}

String generateWebPage() {
    String html = "<!DOCTYPE html><html><head><title>PZEM Energy Monitor</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;margin:20px;background:#f0f0f0}";
    html += ".container{background:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}";
    html += ".reading{display:flex;justify-content:space-between;padding:10px;margin:5px 0;background:#f8f9fa;border-radius:5px}";
    html += ".value{font-weight:bold;color:#007bff}";
    html += ".status{padding:10px;margin:10px 0;border-radius:5px}";
    html += ".online{background:#d4edda;color:#155724}";
    html += ".offline{background:#f8d7da;color:#721c24}";
    html += "</style></head><body>";
    
    html += "<div class='container'>";
    html += "<h1>⚡ PZEM Energy Monitor</h1>";
    html += "<p><strong>Device ID:</strong> " + DEVICE_ID + "</p>";
    
    html += "<div class='status " + String(sensorConnected ? "online" : "offline") + "'>";
    html += "Sensor: " + String(sensorConnected ? "Connected" : "Disconnected") + "</div>";
    
    if (currentReading.valid) {
        html += "<h2>Current Readings</h2>";
        html += "<div class='reading'><span>Voltage:</span><span class='value'>" + String(currentReading.voltage, 2) + " V</span></div>";
        html += "<div class='reading'><span>Current:</span><span class='value'>" + String(currentReading.current, 3) + " A</span></div>";
        html += "<div class='reading'><span>Power:</span><span class='value'>" + String(currentReading.power, 2) + " W</span></div>";
        html += "<div class='reading'><span>Energy:</span><span class='value'>" + String(currentReading.energy, 3) + " kWh</span></div>";
        html += "<div class='reading'><span>Frequency:</span><span class='value'>" + String(currentReading.frequency, 1) + " Hz</span></div>";
        html += "<div class='reading'><span>Power Factor:</span><span class='value'>" + String(currentReading.powerFactor, 2) + "</span></div>";
        html += "<div class='reading'><span>Total Energy:</span><span class='value'>" + String(totalEnergy, 3) + " kWh</span></div>";
    } else {
        html += "<p>No valid sensor readings available</p>";
    }
    
    html += "<script>setTimeout(()=>location.reload(), 10000);</script>";
    html += "</div></body></html>";
    
    return html;
}

String getTimestamp() {
    return String(millis());
}