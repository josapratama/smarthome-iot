# ESP32 IoT Devices - Firmware Collection

Koleksi firmware untuk berbagai jenis ESP32 IoT devices dengan fungsi spesifik.

## Konsep: One Device, One Task

Setiap folder berisi firmware untuk satu jenis device dengan satu tugas spesifik:

- **esp32-base** - Firmware minimal (OTA only, no sensor)
- **esp32-flame** - Flame detection + OTA
- **esp32-mq2** - Gas detection (MQ-2) + OTA
- **esp32-ultrasonic** - Distance measurement + OTA
- **esp32-current** - Current monitoring + OTA
- **esp32-relay** - Relay control + OTA
- dll

## Struktur Folder

```
smarthome-iot/
├── esp32-base/              # Base firmware (OTA only)
│   ├── src/
│   │   └── main.cpp
│   ├── platformio.ini
│   ├── OTA_GUIDE.md
│   └── README.md
│
├── esp32-flame/             # Flame sensor
│   ├── src/
│   │   └── main.cpp
│   ├── platformio.ini
│   └── README.md
│
├── esp32-mq2/               # Gas sensor (MQ-2)
│   ├── src/
│   │   └── main.cpp
│   ├── platformio.ini
│   └── README.md
│
├── esp32-ultrasonic/        # Ultrasonic sensor (HC-SR04)
│   ├── src/
│   │   └── main.cpp
│   ├── platformio.ini
│   └── README.md
│
├── esp32-current/           # Current sensor (ACS712)
│   ├── src/
│   │   └── main.cpp
│   ├── platformio.ini
│   └── README.md
│
├── esp32-relay/             # Relay control
│   ├── src/
│   │   └── main.cpp
│   ├── platformio.ini
│   └── README.md
│
├── DEVICE_REGISTRATION_GUIDE.md  # Panduan registrasi device
└── README.md                      # This file
```

## Quick Start

### 1. Pilih Firmware Sesuai Kebutuhan

Contoh: Anda ingin membuat flame detector

```bash
cd esp32-flame
```

### 2. Konfigurasi WiFi & MQTT

Edit `src/main.cpp`:

```cpp
const char* WIFI_SSID = "YourWiFi";
const char* WIFI_PASSWORD = "YourPassword";
const char* MQTT_SERVER = "192.168.100.1";
```

### 3. Upload Firmware

```bash
pio run -t upload
pio device monitor
```

### 4. Catat MAC Address

Dari serial monitor:

```
MAC: AA:BB:CC:DD:EE:FF
```

### 5. Registrasi di Backend

Lihat [DEVICE_REGISTRATION_GUIDE.md](./DEVICE_REGISTRATION_GUIDE.md)

### 6. Kirim Credentials

Via API atau MQTT, kirim Device ID & Key ke ESP32.

### 7. Done!

Device siap digunakan.

## Firmware Features

Semua firmware include:

✅ **WiFi Connection** - Auto-reconnect  
✅ **MQTT Client** - Pub/sub messaging  
✅ **OTA Update** - Wireless firmware update  
✅ **Device Registration** - Auto-registration flow  
✅ **Heartbeat** - Keep-alive monitoring  
✅ **Command Handler** - PING, RESTART, OTA_UPDATE, dll  
✅ **Credentials Storage** - NVS persistent storage

Plus fitur spesifik per device type.

## Device Types

### esp32-base

- **Purpose:** Base firmware untuk testing OTA
- **Sensors:** None
- **Use Case:** Testing, development, OTA verification

### esp32-flame

- **Purpose:** Fire detection
- **Sensors:** Flame sensor (digital)
- **Telemetry:** `flame` (boolean)
- **Use Case:** Fire alarm, safety monitoring

### esp32-mq2

- **Purpose:** Gas leak detection
- **Sensors:** MQ-2 gas sensor (analog)
- **Telemetry:** `gasPpm` (float)
- **Use Case:** Gas leak alarm, kitchen safety

### esp32-ultrasonic

- **Purpose:** Distance measurement
- **Sensors:** HC-SR04 ultrasonic (digital)
- **Telemetry:** `distanceCm` (float), `binLevel` (%)
- **Use Case:** Trash bin monitoring, parking sensor

### esp32-current

- **Purpose:** Current monitoring
- **Sensors:** ACS712 current sensor (analog)
- **Telemetry:** `current` (float), `powerW` (float)
- **Use Case:** Power monitoring, energy management

### esp32-relay

- **Purpose:** Device control
- **Actuators:** Relay module (digital)
- **Commands:** `TURN_ON`, `TURN_OFF`, `TOGGLE`
- **Use Case:** Light control, appliance control

## Development Workflow

### 1. Create New Device Type

```bash
# Copy base firmware
cp -r esp32-base esp32-newtype

cd esp32-newtype
```

### 2. Modify Firmware

Edit `src/main.cpp`:

```cpp
// Change device type
const char* DEVICE_TYPE = "NEWTYPE";

// Add sensor pins
#define SENSOR_PIN 34

// Add sensor reading function
float readSensor() {
    int raw = analogRead(SENSOR_PIN);
    return (raw / 4095.0) * 100.0;
}

// Modify telemetry
void sendTelemetry() {
    // ... existing code ...

    float value = readSensor();
    data["sensorValue"] = value;

    // ... rest of code ...
}
```

### 3. Test Locally

```bash
pio run -t upload
pio device monitor
```

### 4. Register Device

Follow [DEVICE_REGISTRATION_GUIDE.md](./DEVICE_REGISTRATION_GUIDE.md)

### 5. Deploy

Upload firmware to backend and trigger OTA to production devices.

## OTA Update Workflow

### Development (ArduinoOTA)

```bash
# First upload via USB
pio run -t upload

# Get ESP32 IP from serial monitor
# Edit platformio.ini with IP

# Upload via WiFi
pio run -t upload
```

### Production (HTTP OTA)

```bash
# Build firmware
pio run

# Upload to backend
curl -X POST http://backend/api/v1/firmware/releases \
  -F "file=@.pio/build/esp32-c3-devkitm-1/firmware.bin" \
  -F "platform=esp32-c3" \
  -F "version=1.0.4"

# Trigger OTA
curl -X POST http://backend/api/v1/ota/devices/5/trigger \
  -d '{"releaseId": 1}'
```

## Hardware Requirements

### Minimum (All Devices)

- ESP32-C3 DevKitM-1 (or compatible)
- USB-C cable
- 5V power supply

### Per Device Type

**esp32-flame:**

- Flame sensor module (digital output)
- Jumper wires

**esp32-mq2:**

- MQ-2 gas sensor module
- Jumper wires

**esp32-ultrasonic:**

- HC-SR04 ultrasonic sensor
- Jumper wires

**esp32-current:**

- ACS712 current sensor (5A/20A/30A)
- Jumper wires

**esp32-relay:**

- Relay module (1/2/4/8 channel)
- Jumper wires
- External power for relay (if needed)

## Pin Configuration

Default pins (dapat diubah di firmware):

```cpp
// Analog sensors
#define SENSOR_PIN 34  // ADC1_CH6

// Digital sensors
#define SENSOR_PIN 32  // GPIO32

// Ultrasonic
#define TRIG_PIN 25
#define ECHO_PIN 26

// Relay
#define RELAY_PIN 33

// Built-in LED
#define LED_BUILTIN 8  // ESP32-C3
```

## MQTT Topics

### Device → Backend

```
devices/{deviceId}/telemetry      # Sensor data
devices/{deviceId}/heartbeat      # Keep-alive
devices/{deviceId}/commands/ack   # Command acknowledgment
devices/{deviceId}/ota/progress   # OTA progress
devices/register/request          # Registration request
```

### Backend → Device

```
devices/{deviceId}/commands       # Commands (PING, RESTART, OTA, etc)
devices/register/{MAC}            # Registration response (SET_CREDENTIALS)
```

## Telemetry Schema

Setiap device kirim telemetry dengan format:

```json
{
  "deviceKey": "sk_abc123",
  "ts": 1234567890,
  "data": {
    // Device-specific sensor data
    "flame": false,
    "gasPpm": 150.5,
    "distanceCm": 45.2,
    "current": 2.5
    // etc
  }
}
```

## Commands

Semua device support commands:

### PING

```json
{
  "commandId": 123,
  "type": "PING",
  "payload": {}
}
```

### RESTART

```json
{
  "commandId": 124,
  "type": "RESTART",
  "payload": {}
}
```

### OTA_UPDATE

```json
{
  "commandId": 125,
  "type": "OTA_UPDATE",
  "payload": {
    "firmwareUrl": "http://backend/firmware.bin",
    "version": "1.0.4",
    "otaJobId": 456
  }
}
```

### SET_CREDENTIALS (Registration only)

```json
{
  "commandId": 126,
  "type": "SET_CREDENTIALS",
  "payload": {
    "deviceId": 5,
    "deviceKey": "sk_abc123"
  }
}
```

## Troubleshooting

### Device tidak connect WiFi

- Check SSID dan password
- Check WiFi 2.4GHz (ESP32 tidak support 5GHz)
- Check signal strength

### MQTT tidak connect

- Check MQTT broker running
- Check MQTT_SERVER IP benar
- Check firewall tidak block port 1883

### Device tidak kirim telemetry

- Check device registered (Device ID > 0)
- Check MQTT connected
- Check sensor wiring

### OTA gagal

- Check firmware size < available flash
- Check WiFi stable
- Check backend URL accessible

## Best Practices

1. **One Task Per Device** - Jangan gabungkan banyak sensor dalam satu device
2. **Descriptive Names** - Beri nama device yang jelas (e.g., "Kitchen Gas Sensor")
3. **Version Control** - Increment version number setiap update
4. **Test First** - Test di 1 device sebelum mass deployment
5. **Monitor Logs** - Selalu monitor serial output saat development
6. **Backup Firmware** - Simpan firmware lama sebelum update
7. **Document Changes** - Catat perubahan di README setiap folder

## Contributing

Untuk menambah device type baru:

1. Copy `esp32-base` sebagai template
2. Modify firmware sesuai kebutuhan
3. Test thoroughly
4. Update README dengan specs
5. Commit dengan descriptive message

## Support

- **Documentation:** Lihat README.md di setiap folder
- **Registration:** [DEVICE_REGISTRATION_GUIDE.md](./DEVICE_REGISTRATION_GUIDE.md)
- **OTA Guide:** [esp32-base/OTA_GUIDE.md](./esp32-base/OTA_GUIDE.md)
- **Issues:** Report via GitHub issues

## License

MIT License - See LICENSE file for details
