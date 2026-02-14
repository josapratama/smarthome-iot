# Smart Home IoT Devices

[![PlatformIO](https://img.shields.io/badge/PlatformIO-FF7F00?style=flat&logo=platformio&logoColor=white)](https://platformio.org/)
[![ESP32](https://img.shields.io/badge/ESP32-000000?style=flat&logo=espressif&logoColor=white)](https://www.espressif.com/)
[![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat&logo=Arduino&logoColor=white)](https://www.arduino.cc/)
[![MQTT](https://img.shields.io/badge/MQTT-660066?style=flat&logo=mqtt&logoColor=white)](https://mqtt.org/)

Firmware collection for ESP32-C3 SuperMini based IoT devices with sensor integration, OTA updates, and MQTT communication.

## 🔌 Supported Hardware

### **ESP32-C3 SuperMini**

- **MCU**: ESP32-C3 (RISC-V single-core, 160MHz)
- **Memory**: 400KB SRAM, 4MB Flash
- **Connectivity**: WiFi 802.11 b/g/n, Bluetooth 5.0 LE
- **GPIO**: 13 digital I/O pins
- **ADC**: 6 channels, 12-bit
- **Size**: Ultra-compact form factor

## 📦 Device Firmware Collection

### 🔧 [Base Firmware](./esp32-base/)

**Initial setup firmware for first-time cable upload**

- WiFi configuration and connection
- MQTT client setup
- OTA update capability
- Basic device registration
- WiFi reset button functionality
- Buzzer support for notifications

### ⚡ [PZEM-004T Energy Monitor](./esp32-pzem004t/)

**Power consumption monitoring device**

- Real-time voltage, current, power measurement
- Energy consumption tracking
- Power factor calculation
- Frequency monitoring
- Data logging and transmission

### 🔥 [MQ-2 Gas Sensor](./esp32-mq2/)

**Gas leak detection and air quality monitoring**

- LPG, propane, methane detection
- Smoke and combustible gas monitoring
- Analog and digital output reading
- Threshold-based alarm system
- Air quality index calculation

### 🔥 [Flame Sensor](./esp32-flame/)

**Fire detection and safety monitoring**

- Infrared flame detection
- Digital and analog flame sensing
- Multi-level sensitivity adjustment
- Emergency alert system
- False alarm prevention

### 📏 [Ultrasonic Distance Sensor](./esp32-ultrasonic/)

**Distance measurement and proximity detection**

- HC-SR04 ultrasonic sensor integration
- Accurate distance measurement (2cm - 400cm)
- Water level monitoring
- Parking assistance
- Motion detection applications

## 🚀 Quick Start

### Prerequisites

- **PlatformIO IDE** or **PlatformIO Core**
- **ESP32-C3 SuperMini** development board
- **USB-C cable** for initial programming
- **Sensors** (PZEM-004T, MQ-2, Flame sensor, HC-SR04)

### Initial Setup

1. **Install PlatformIO:**

```bash
# Using pip
pip install platformio

# Or install PlatformIO IDE extension in VS Code
```

2. **Clone and setup:**

```bash
git clone <repository-url>
cd smarthome-iot
```

3. **First-time upload (Cable):**

```bash
cd esp32-base
pio run --target upload --target monitor
```

4. **Configure WiFi and MQTT:**

- Connect to ESP32 hotspot: `SmartHome-Setup`
- Open browser: `http://192.168.4.1`
- Enter WiFi credentials and MQTT settings
- Device will restart and connect

5. **Subsequent uploads (OTA):**

```bash
# Update firmware over-the-air
pio run --target upload --upload-port <device-ip>
```

## 📡 Communication Protocol

### **MQTT Topics Structure**

```
smarthome/{homeId}/{deviceId}/telemetry    # Sensor data
smarthome/{homeId}/{deviceId}/commands     # Control commands
smarthome/{homeId}/{deviceId}/status       # Device status
smarthome/{homeId}/{deviceId}/config       # Configuration
smarthome/{homeId}/{deviceId}/ota          # OTA updates
```

### **Telemetry Data Format**

```json
{
  "timestamp": "2026-02-13T12:00:00.000Z",
  "deviceId": "ESP32_C3_001",
  "deviceType": "energy_monitor",
  "data": {
    "voltage": 220.5,
    "current": 0.75,
    "power": 165.4,
    "energy": 1.25,
    "frequency": 50.0,
    "powerFactor": 0.95
  },
  "status": {
    "rssi": -45,
    "uptime": 3600,
    "freeHeap": 280000,
    "temperature": 35.2
  }
}
```

### **Command Format**

```json
{
  "commandId": "cmd_123456789",
  "timestamp": "2026-02-13T12:00:00.000Z",
  "type": "SET_CONFIG",
  "payload": {
    "reportInterval": 30,
    "threshold": 100.0,
    "buzzerEnabled": true
  }
}
```

## 🔧 Hardware Connections

### **ESP32-C3 SuperMini Pinout**

```
                    ESP32-C3 SuperMini
                    ┌─────────────────┐
                    │  [ ]         [ ]│
                    │  [ ]         [ ]│
               3V3  │  [ ]         [ ]│  GND
               EN   │  [ ]         [ ]│  3V3
              GPIO4 │  [ ]         [ ]│  GPIO10
              GPIO5 │  [ ]         [ ]│  GPIO9
              GPIO6 │  [ ]         [ ]│  GPIO8
              GPIO7 │  [ ]         [ ]│  GPIO7
                    │  [ ]         [ ]│
                    │     USB-C       │
                    └─────────────────┘
```

### **Sensor Connections**

**PZEM-004T Energy Monitor:**

```
ESP32-C3    PZEM-004T
GPIO4   →   TX
GPIO5   →   RX
3V3     →   VCC
GND     →   GND
```

**MQ-2 Gas Sensor:**

```
ESP32-C3    MQ-2
GPIO6   →   A0 (Analog)
GPIO7   →   D0 (Digital)
3V3     →   VCC
GND     →   GND
```

**Flame Sensor:**

```
ESP32-C3    Flame Sensor
GPIO8   →   A0 (Analog)
GPIO9   →   D0 (Digital)
3V3     →   VCC
GND     →   GND
```

**HC-SR04 Ultrasonic:**

```
ESP32-C3    HC-SR04
GPIO10  →   Trig
GPIO9   →   Echo
5V      →   VCC (use level shifter)
GND     →   GND
```

**Common Components:**

```
ESP32-C3    Component
GPIO2   →   WiFi Reset Button (Pull-up)
GPIO3   →   Buzzer (+)
GND     →   Buzzer (-)
```

## ⚙️ Configuration

### **platformio.ini Example**

```ini
[env:esp32-c3-devkitm-1]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
upload_speed = 921600

lib_deps =
    knolleary/PubSubClient@^2.8
    bblanchon/ArduinoJson@^6.21.3
    tzapu/WiFiManager@^2.0.16-rc.2
    ayushsharma82/AsyncElegantOTA@^2.2.7

build_flags =
    -DCORE_DEBUG_LEVEL=3
    -DBOARD_HAS_PSRAM
```

### **WiFi Configuration**

- **Setup Mode**: Device creates hotspot `SmartHome-Setup`
- **Configuration Portal**: `http://192.168.4.1`
- **Reset**: Hold WiFi reset button for 5 seconds
- **Fallback**: Auto-reconnect with saved credentials

### **MQTT Configuration**

```cpp
// MQTT Settings (configurable via web portal)
const char* mqtt_server = "your-mqtt-broker.com";
const int mqtt_port = 1883;
const char* mqtt_user = "your-username";
const char* mqtt_password = "your-password";
```

## 🔄 OTA Updates

### **Enable OTA in Code**

```cpp
#include <AsyncElegantOTA.h>

void setup() {
    // ... WiFi setup

    AsyncElegantOTA.begin(&server);
    server.begin();
}
```

### **Upload via OTA**

```bash
# Method 1: PlatformIO
pio run --target upload --upload-port 192.168.1.100

# Method 2: Web Interface
# Open http://device-ip/update
# Upload .bin file through web interface

# Method 3: Backend API
curl -X POST http://device-ip/update \
  -F "firmware=@.pio/build/esp32-c3-devkitm-1/firmware.bin"
```

## 🚨 Safety Features

### **WiFi Reset Button**

- **Function**: Reset WiFi credentials and restart setup mode
- **Usage**: Hold button for 5+ seconds
- **Indicator**: Buzzer beeps 3 times, LED blinks rapidly
- **Recovery**: Device creates setup hotspot

### **Buzzer Notifications**

- **Startup**: 1 short beep
- **WiFi Connected**: 2 short beeps
- **MQTT Connected**: 3 short beeps
- **Error**: Long continuous beep
- **Reset**: 3 rapid beeps

### **Watchdog Timer**

- **Function**: Auto-restart if device hangs
- **Timeout**: 30 seconds
- **Reset**: Hardware and software reset capability

### **Error Handling**

- **WiFi Disconnection**: Auto-reconnect with exponential backoff
- **MQTT Disconnection**: Automatic reconnection attempts
- **Sensor Errors**: Error reporting and fallback values
- **Memory Management**: Heap monitoring and cleanup

## 📊 Monitoring & Debugging

### **Serial Monitor**

```bash
pio device monitor --baud 115200
```

### **Debug Output Example**

```
[INFO] ESP32-C3 Smart Home Device Starting...
[INFO] MAC Address: 34:85:18:12:34:56
[INFO] Connecting to WiFi: MyHomeWiFi
[INFO] WiFi Connected! IP: 192.168.1.100
[INFO] MQTT Connecting to: mqtt.smarthome.local:1883
[INFO] MQTT Connected! Client ID: ESP32_C3_001
[INFO] OTA Server Started: http://192.168.1.100/update
[INFO] Device Ready - Type: energy_monitor
[DATA] Voltage: 220.5V, Current: 0.75A, Power: 165.4W
```

### **Web Dashboard**

Access device status at: `http://device-ip/`

- Real-time sensor readings
- System information
- Configuration options
- OTA update interface

## 🔧 Development Guidelines

### **Code Structure**

```
esp32-device/
├── src/
│   ├── main.cpp              # Main application
│   ├── config.h              # Configuration constants
│   ├── wifi_manager.cpp      # WiFi management
│   ├── mqtt_client.cpp       # MQTT communication
│   ├── sensor_handler.cpp    # Sensor-specific code
│   └── ota_updater.cpp       # OTA functionality
├── include/
│   └── *.h                   # Header files
├── lib/
│   └── custom_libs/          # Custom libraries
└── platformio.ini            # PlatformIO configuration
```

### **Coding Standards**

- Use meaningful variable names
- Comment complex logic
- Handle errors gracefully
- Implement proper logging
- Follow Arduino/ESP32 conventions

### **Testing**

- Test WiFi connection scenarios
- Verify MQTT communication
- Validate sensor readings
- Test OTA update process
- Check error recovery

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](../LICENSE) file for details.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Test on actual hardware
4. Submit a pull request

## 📞 Support

- **Documentation**: [Smart Home Backend](../smarthome-backend/docs/)
- **Hardware Issues**: Check connections and power supply
- **Software Issues**: Enable debug logging and check serial output
- **MQTT Issues**: Verify broker settings and network connectivity

---

**Ready to build your IoT smart home network?**

Start with the [Base Firmware](./esp32-base/) for initial setup, then choose the appropriate sensor firmware for your specific use case.
