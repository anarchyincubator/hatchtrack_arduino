#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif

#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>

// macro definitions
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

SensirionI2cSht4x sensor;

static char errorMessage[64];
static int16_t error;

constexpr char WIFI_SSID[] = "";
constexpr char WIFI_PASSWORD[] = "";
constexpr char TOKEN[] = "";

// Thingsboard server connection details
constexpr char THINGSBOARD_SERVER[] = "hatch.hatchtrack.com";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
// Maximum size packets for MQTT
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
// Initialize WiFi and MQTT client
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
// Initialize ThingsBoard instance
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, Default_Max_Stack_Size);

// Sleep time in microseconds (10 seconds)
constexpr uint64_t SLEEP_DURATION_US = 300 * 1000000ULL;

/// @brief Initializes WiFi connection,
// will endlessly delay until a connection has been successfully established
void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

/// @brief Reconnects the WiFi uses InitWiFi if the connection has been removed
/// @return Returns true as soon as a connection has been established again
const bool reconnect() {
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  InitWiFi();
  return true;
}

void setup() {
  // Initialize serial connection for debugging
    Serial.begin(SERIAL_DEBUG_BAUD);

  delay(1000);

  // Read sensor data
  Wire.begin(7, 6);
  sensor.begin(Wire, SHT40_I2C_ADDR_44);

  sensor.softReset();
  delay(10);
  uint32_t serialNumber = 0;
  error = sensor.serialNumber(serialNumber);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute serialNumber(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
  Serial.print("serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();
}

void loop() {
 
  delay(5000);

  // Get real sensor data
  float aTemperature = 0.0;
  float aHumidity = 0.0;
  error = sensor.measureLowestPrecision(aTemperature, aHumidity);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute measureLowestPrecision(): ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
  double dTemperature = (double)aTemperature;
  double dHumidity = (double)aHumidity;

  Serial.print("aTemperature: ");
  Serial.print(aTemperature);
  Serial.print("\t");
  Serial.print("aHumidity: ");
  Serial.print(aHumidity);
  Serial.println();

   if (!reconnect()) {
    return;
  }

  // Connect to ThingsBoard if not already connected
  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
  }

  // Send telemetry data
  tb.sendTelemetryData("temperature", dTemperature);
  tb.sendTelemetryData("humidity", dHumidity);
  tb.sendAttributeData("rssi", WiFi.RSSI());
  tb.sendAttributeData("channel", WiFi.channel());
  tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
  tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
  tb.sendAttributeData("ssid", WiFi.SSID().c_str());

  tb.loop();  // Make sure MQTT communication happens

  // Wait for a short time to ensure all data is sent
  delay(1000); // Adjust delay if needed based on your network speed

  // Put ESP32 into deep sleep for 15 min
  Serial.println("Going to sleep for 15 min...");
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  esp_deep_sleep_start();
}
