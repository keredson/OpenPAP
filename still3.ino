/*
 * ESP32 Dual DS18B20 Temperature Monitor with Web Server, OTA, and Pump Control
 * * This sketch reads temperature from two DS18B20 sensors, displays it on an 
 * SSD1306 OLED display, and hosts a web page to show a graph of the 
 * temperature history and control a PWM pump. It also allows downloading the data as a CSV file.
 *
 * Features:
 * - WiFiManager for easy WiFi configuration.
 * - Over-the-Air (OTA) updates for wireless flashing.
 * - SSD1306 128x64 OLED display support.
 * - Reads from two DS18B20 temperature sensors on a single pin.
 * - PWM pump control via a web UI slider.
 * - Stores temperature readings in memory.
 * - Web server with a graphical chart and CSV download functionality.
 *
 * Hardware:
 * - ESP32 development board
 * - SSD1306 OLED Display (I2C)
 * - 2 x DS18B20 temperature sensors
 * - 4.7k Ohm pull-up resistor for the DS18B20 data line
 * - A logic-level MOSFET or motor driver to control the pump.
 *
 * Wiring:
 * - SSD1306 SDA -> GPIO 21 (I2C SDA)
 * - SSD1306 SCL -> GPIO 22 (I2C SCL)
 * - DS18B20 Data -> GPIO 14
 * - DS18B20 VCC -> 3.3V
 * - DS18B20 GND -> GND
 * - Connect a 4.7k Ohm resistor between DS18B20 Data (GPIO 14) and 3.3V.
 * - Pump Control (e.g., MOSFET Gate) -> GPIO 13
 *
 * Libraries to install via Arduino Library Manager:
 * - WiFiManager by tzapu
 * - OneWire by Paul Stoffregen
 * - DallasTemperature by Miles Burton
 * - Adafruit SSD1306 by Adafruit
 * - Adafruit GFX Library by Adafruit
 */

// --- Library Includes ---
#include <Arduino.h> // Explicitly include for ledc functions
#include <WiFi.h>
#include <WebServer.h> // Using the standard WebServer library to avoid conflicts
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- Pin Definitions ---
#define ONE_WIRE_BUS 14 // GPIO for DS18B20 sensors
#define PUMP_PIN 13     // GPIO for PWM Pump Control

// --- PWM Configuration ---
// const int PUMP_PWM_CHANNEL = 0; // Channel is no longer managed manually in ESP32 Core v3+
const int PUMP_PWM_FREQ = 5000;
const int PUMP_PWM_RESOLUTION = 8; // 8-bit resolution (0-255)
int pumpSpeed = 0; // Current pump speed (0-100%)

// --- Display Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- Sensor Configuration ---
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1Address, sensor2Address;

// --- Web Server and WiFi ---
WebServer server(80); // Standard WebServer instance
WiFiManager wm;

// --- Data Storage ---
struct TempReading {
  unsigned long time;
  float temp1;
  float temp2;
};

#define MAX_READINGS 6000 // Store up to 6000 readings
TempReading data[MAX_READINGS];
int readingCount = 0;

// --- Timing ---
unsigned long lastTempRead = 0;
const long tempReadInterval = 5000; // 5 seconds

// --- Function Prototypes ---
void setupOTA();
void setupWebServer();
void readTemperatures();
void updateDisplay();
void handleRoot();
void handlePumpControl();
void handleDataJson();
void handleDownloadCsv();
void handleNotFound();
String getSensorAddressString(DeviceAddress deviceAddress);


// --- Setup Function ---
void setup() {
  Serial.begin(115200);

  // --- Setup Pump PWM for ESP32 Core v3.x+ ---
  // ledcSetup is removed. Configuration is now done in ledcAttach.
  // The channel is managed internally by the pin number.
  ledcAttach(PUMP_PIN, PUMP_PWM_FREQ, PUMP_PWM_RESOLUTION);
  ledcWrite(PUMP_PIN, 0); // Start with pump off. Use Pin number directly.

  // Initialize display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Starting up...");
  display.display();

  // Initialize sensors and print their addresses
  sensors.begin();
  Serial.println("Locating sensors...");
  if (sensors.getAddress(sensor1Address, 0)) {
    Serial.print("Sensor 1 Address: ");
    Serial.println(getSensorAddressString(sensor1Address));
  } else {
    Serial.println("Unable to find address for Sensor 1");
  }
  
  if (sensors.getAddress(sensor2Address, 1)) {
    Serial.print("Sensor 2 Address: ");
    Serial.println(getSensorAddressString(sensor2Address));
  } else {
    Serial.println("Unable to find address for Sensor 2");
  }

  // WiFiManager setup
  WiFi.mode(WIFI_STA);
  wm.setConnectTimeout(20);
  bool res = wm.autoConnect("ESP32-TempMon-AP");

  if(!res) {
      Serial.println("Failed to connect");
      display.clearDisplay();
      display.println("Connection Failed");
      display.println("Restarting...");
      display.display();
      ESP.restart();
  } 
  else {
      Serial.println("Connected to WiFi!");
      display.clearDisplay();
      display.println("Connected!");
      display.println("IP Address:");
      display.println(WiFi.localIP());
      display.display();
  }

  // Setup OTA, Web Server, and initial temp reading
  setupOTA();
  setupWebServer();
  readTemperatures();
  updateDisplay();
}

// --- Main Loop ---
void loop() {
  ArduinoOTA.handle();
  server.handleClient(); // Handle incoming web server requests

  unsigned long currentMillis = millis();
  if (currentMillis - lastTempRead >= tempReadInterval) {
    lastTempRead = currentMillis;
    readTemperatures();
    updateDisplay();
  }
}

// --- Function Implementations ---

/**
 * @brief Sets up Over-The-Air programming.
 */
void setupOTA() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    Serial.println("Start updating " + type);
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("OTA Update...");
    display.display();
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("OTA Update...");
    display.printf("Progress: %u%%", (progress / (total / 100)));
    display.display();
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
}

/**
 * @brief Reads temperatures from both DS18B20 sensors and stores them.
 */
void readTemperatures() {
  sensors.requestTemperatures(); 
  float tempC1 = sensors.getTempC(sensor1Address);
  float tempC2 = sensors.getTempC(sensor2Address);

  if (tempC1 == DEVICE_DISCONNECTED_C || tempC2 == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    return;
  }

  if (readingCount < MAX_READINGS) {
    data[readingCount].time = millis() / 1000;
    data[readingCount].temp1 = tempC1;
    data[readingCount].temp2 = tempC2;
    readingCount++;
  } else {
    // Shift all data left to make space for the new reading
    for (int i = 0; i < MAX_READINGS - 1; i++) {
      data[i] = data[i+1];
    }
    data[MAX_READINGS - 1].time = millis() / 1000;
    data[MAX_READINGS - 1].temp1 = tempC1;
    data[MAX_READINGS - 1].temp2 = tempC2;
  }
  
  Serial.print("Sensor 1: "); Serial.print(tempC1); Serial.println(" *C");
  Serial.print("Sensor 2: "); Serial.print(tempC2); Serial.println(" *C");
}

/**
 * @brief Updates the OLED display with the latest temperature readings.
 */
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Temp Monitor");
  display.println("------------");
  
  if (readingCount > 0) {
    display.print("S1: ");
    display.print(data[readingCount-1].temp1);
    display.println(" C");

    display.print("S2: ");
    display.print(data[readingCount-1].temp2);
    display.println(" C");
  } else {
    display.println("No readings yet.");
  }
  
  display.println("------------");
  display.println(WiFi.localIP());
  display.display();
}

/**
 * @brief Sets up the web server and its routes.
 */
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/pump", HTTP_POST, handlePumpControl);
  server.on("/data.json", HTTP_GET, handleDataJson);
  server.on("/download.csv", HTTP_GET, handleDownloadCsv);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

/**
 * @brief Handles the root ("/") web request, serving the main HTML page.
 */
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Temp Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    html, body {
      height: 100%;
      margin: 0;
      padding: 0;
      font-family: Arial, sans-serif;
      background-color: #f4f4f4;
    }
    .container {
      display: flex;
      flex-direction: column;
      height: 100%;
      max-width: 900px;
      margin: 0 auto;
      background: white;
      box-shadow: 0 0 10px rgba(0,0,0,0.1);
    }
    h1 {
      color: #333;
      text-align: center;
      padding: 15px 0;
      margin: 0;
    }
    #chart-container {
      flex-grow: 1; /* Allows the chart container to fill available space */
      position: relative;
      padding: 10px;
    }
    .controls-container {
      text-align: center;
      padding: 15px 0;
      border-top: 1px solid #eee;
    }
    .pump-control {
      margin-bottom: 15px;
    }
    .pump-control label {
      font-size: 18px;
      margin-right: 10px;
    }
    .pump-control input[type="range"] {
      width: 50%;
      max-width: 300px;
      vertical-align: middle;
    }
    .button {
      background-color: #4CAF50;
      color: white;
      padding: 10px 20px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      margin: 4px 2px;
      cursor: pointer;
      border: none;
      border-radius: 4px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>ESP32 Temperature & Pump Control</h1>
    <div id="chart-container">
      <canvas id="tempChart"></canvas>
    </div>
    <div class="controls-container">
      <div class="pump-control">
        <label for="pumpSlider">Pump Speed: <span id="pumpSpeedLabel">0</span>%</label>
        <input type="range" min="0" max="100" value="0" id="pumpSlider">
      </div>
      <a href="/download.csv" class="button">Download Temp CSV</a>
    </div>
  </div>
  <script>
    let myChart; // Variable to hold the chart instance

    const fetchData = () => {
      return fetch('/data.json').then(response => response.json());
    };

    const createOrUpdateChart = () => {
      fetchData().then(data => {
        const ctx = document.getElementById('tempChart').getContext('2d');
        
        const labels = data.map(d => {
          const date = new Date(d.time * 1000);
          return date.toLocaleTimeString();
        });

        if (myChart) {
          // If chart exists, update data and redraw
          myChart.data.labels = labels;
          myChart.data.datasets[0].data = data.map(d => d.temp1);
          myChart.data.datasets[1].data = data.map(d => d.temp2);
          myChart.update();
        } else {
          // If chart doesn't exist, create it
          myChart = new Chart(ctx, {
            type: 'line',
            data: {
              labels: labels,
              datasets: [{
                label: 'Sensor 1 (°C)',
                data: data.map(d => d.temp1),
                borderColor: 'rgba(255, 99, 132, 1)',
                backgroundColor: 'rgba(255, 99, 132, 0.2)',
                fill: false,
                borderWidth: 2
              }, {
                label: 'Sensor 2 (°C)',
                data: data.map(d => d.temp2),
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(54, 162, 235, 0.2)',
                fill: false,
                borderWidth: 2
              }]
            },
            options: {
              responsive: true,
              maintainAspectRatio: false,
              scales: {
                x: {
                  display: true,
                  title: {
                    display: true,
                    text: 'Time'
                  }
                },
                y: {
                  display: true,
                  title: {
                    display: true,
                    text: 'Temperature (°C)'
                  },
                  //min: 70,  // Set minimum Y-axis value
                  //max: 100  // Set maximum Y-axis value
                }
              }
            }
          });
        }
      }).catch(error => console.error('Chart update error:', error));
    };

    // --- Pump Control Logic ---
    const pumpSlider = document.getElementById('pumpSlider');
    const pumpSpeedLabel = document.getElementById('pumpSpeedLabel');

    pumpSlider.addEventListener('input', (event) => {
      const speed = event.target.value;
      pumpSpeedLabel.textContent = speed;
    });

    pumpSlider.addEventListener('change', (event) => {
      const speed = event.target.value;
      const formData = new FormData();
      formData.append('speed', speed);

      fetch('/pump', {
        method: 'POST',
        body: new URLSearchParams(formData)
      });
    });

    // --- Initial Load ---
    createOrUpdateChart();
    setInterval(createOrUpdateChart, 5000); // Refresh chart every 5 seconds
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

/**
 * @brief Handles POST requests to control the pump speed.
 */
void handlePumpControl() {
  if (server.hasArg("speed")) {
    String speedStr = server.arg("speed");
    pumpSpeed = speedStr.toInt();
    
    // Map the 0-100% speed to the 8-bit PWM resolution (0-255)
    int dutyCycle = map(pumpSpeed, 0, 100, 0, 255);
    
    // Use the pin number directly for ledcWrite in ESP32 Core v3.x+
    ledcWrite(PUMP_PIN, dutyCycle);
    
    Serial.print("Pump speed set to: ");
    Serial.print(pumpSpeed);
    Serial.print("% (Duty Cycle: ");
    Serial.print(dutyCycle);
    Serial.println(")");
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request: 'speed' parameter missing");
  }
}


/**
 * @brief Serves the temperature data as a JSON object by streaming it.
 */
void handleDataJson() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", ""); // Send headers

  server.sendContent("[");
  for (int i = 0; i < readingCount; i++) {
    String json_item = "{";
    json_item += "\"time\":" + String(data[i].time);
    json_item += ",\"temp1\":" + String(data[i].temp1);
    json_item += ",\"temp2\":" + String(data[i].temp2);
    json_item += "}";
    if (i < readingCount - 1) {
      json_item += ",";
    }
    server.sendContent(json_item);
  }
  server.sendContent("]");
  
  server.sendContent(""); // End of stream
}

/**
 * @brief Handles the request to download data as a CSV file by streaming it.
 */
void handleDownloadCsv() {
  server.sendHeader("Content-Disposition", "attachment; filename=temp_data.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", ""); // Send headers

  server.sendContent("Time,Sensor1_C,Sensor2_C\n");
  for (int i = 0; i < readingCount; i++) {
    String row = String(data[i].time) + "," + String(data[i].temp1) + "," + String(data[i].temp2) + "\n";
    server.sendContent(row);
  }

  server.sendContent(""); // End of stream
}

/**
 * @brief Handles requests to non-existent pages.
 */
void handleNotFound(){
  server.send(404, "text/plain", "Not found");
}

/**
 * @brief Converts a sensor's 8-byte address to a printable hex string.
 */
String getSensorAddressString(DeviceAddress deviceAddress) {
  String address = "";
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) address += "0";
    address += String(deviceAddress[i], HEX);
  }
  address.toUpperCase();
  return address;
}
