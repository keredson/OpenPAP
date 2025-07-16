/*
 * OpenStill Controller
 * * This sketch reads temperature from two DS18B20 sensors, pressure from an
 * HX711, displays it on an SSD1306 OLED display, and hosts a web page to 
 * show a graph of the history and control a pump via a PID controller.
 *
 * Features:
 * - WiFiManager for easy WiFi configuration.
 * - Over-the-Air (OTA) updates for wireless flashing.
 * - PID controller to automatically manage pump power based on a target temperature.
 * - Time-based pump control for better low-end performance.
 * - Web UI to set target temperature, PID tunings, and pump mode (Off/Auto/Manual).
 * - Reads from DS18B20 temperature sensors and an HX711 pressure sensor.
 * - Compensates for known pressure sensor drift.
 * - Stores sensor data in memory.
 * - Web server with a graphical chart and CSV download functionality.
 *
 * Hardware:
 * - ESP32 development board
 * - SSD1306 OLED Display (I2C)
 * - 2 x DS18B20 temperature sensors
 * - HX711 Load Cell Amplifier
 * - 4.7k Ohm pull-up resistor for the DS18B20 data line
 * - A logic-level MOSFET or motor driver to control the pump with a flyback diode.
 *
 * Wiring:
 * - SSD1306 SDA -> GPIO 21 (I2C SDA)
 * - SSD1306 SCL -> GPIO 22 (I2C SCL)
 * - DS18B20 Data -> GPIO 14
 * - DS18B20 VCC -> 3.3V
 * - DS18B20 GND -> GND
 * - Connect a 4.7k Ohm resistor between DS18B20 Data (GPIO 14) and 3.3V.
 * - Pump Control (e.g., MOSFET Gate) -> GPIO 27
 * - HX711 DOUT -> GPIO 25
 * - HX711 SCK  -> GPIO 26
 *
 * Libraries to install via Arduino Library Manager:
 * - WiFiManager by tzapu
 * - OneWire by Paul Stoffregen
 * - DallasTemperature by Miles Burton
 * - Adafruit SSD1306 by Adafruit
 * - Adafruit GFX Library by Adafruit
 * - PID by Brett Beauregard (PID_v1)
 * - HX711 by bodge
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
#include <PID_v1.h>
#include "HX711.h"

// --- Pin Definitions ---
#define ONE_WIRE_BUS 14 // GPIO for DS18B20 sensors
#define PUMP_PIN 27     // GPIO for PWM Pump Control
#define HX711_DOUT 25
#define HX711_SCK  26

// --- PWM Configuration ---
const int PUMP_PWM_FREQ = 5000;
const int PUMP_PWM_RESOLUTION = 8; // 8-bit resolution (0-255)
const int PUMP_FIXED_DUTY_CYCLE = 150; // Fixed power level for the pump when on

// --- Pump Control State ---
enum PumpMode { PUMP_OFF, PUMP_AUTO, PUMP_MANUAL };
PumpMode currentPumpMode = PUMP_AUTO; // Default to automatic PID control
int manualPumpPower = 0; // Manual power setting (0-100%)

// --- PID Controller Configuration ---
double Setpoint, Input, Output;
// Tuning parameters - these may need to be adjusted for your specific setup
double Kp=5, Ki=0.1, Kd=1; 
// Use REVERSE for cooling applications. Output increases as Input rises above Setpoint.
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

// --- Sensor Configuration ---
HX711 scale;
const float PRESSURE_DRIFT_PER_MINUTE = 0.0;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress sensor1Address, sensor2Address;

// --- Display Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- Web Server and WiFi ---
WebServer server(80); // Standard WebServer instance
WiFiManager wm;

// --- Data Storage ---
struct SensorReading {
  unsigned long time;
  float temp1;
  float temp2;
  float pumpPower;
  float pressure;
};

#define MAX_READINGS 3000 
SensorReading data[MAX_READINGS];
int readingCount = 0;

// --- Timing ---
unsigned long lastSensorRead = 0;
const long sensorReadInterval = 5000; // 5 seconds
unsigned long pumpCycleStartTime = 0;
const long pumpCycleInterval = 1000; // 1 second cycle for time-based control
long pumpOnDuration = 0; // Calculated duration in ms for the pump to be on

// --- Function Prototypes ---
void setupOTA();
void setupWebServer();
void readSensors();
void updateDisplay();
void handleRoot();
void handleSetpointControl();
void handlePidControl();
void handleGetPid();
void handlePumpModeControl();
void handleGetPumpMode();
void handleManualPumpControl();
void handleDataJson();
void handleDownloadCsv();
void handleNotFound();
String getSensorAddressString(DeviceAddress deviceAddress);


// --- Setup Function ---
void setup() {
  Serial.begin(115200);

  // --- Setup Pump PWM for ESP32 Core v3.x+ ---
  ledcAttach(PUMP_PIN, PUMP_PWM_FREQ, PUMP_PWM_RESOLUTION);
  ledcWrite(PUMP_PIN, 0); // Start with pump off

  // --- Initialize PID Controller ---
  Setpoint = 78.2; // Default target temperature
  myPID.SetOutputLimits(0, pumpCycleInterval); // PID output is mapped to time (0-1000ms)
  myPID.SetMode(AUTOMATIC);

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
  Serial.println("Locating temp sensors...");
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
  
  // Initialize HX711
  Serial.println("Initializing pressure sensor...");
  scale.begin(HX711_DOUT, HX711_SCK);
  // You need to calibrate your HX711 and set the scale factor.
  // This value is arbitrary and must be replaced.
  // See HX711 library examples for calibration sketches.
  scale.set_scale(2280.f); 
  scale.tare(); // Assume the initial reading is 0 pressure.

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

  // Setup OTA, Web Server, and initial sensor reading
  setupOTA();
  setupWebServer();
  readSensors(); // Get initial reading
  updateDisplay();
}

// --- Main Loop ---
void loop() {
  ArduinoOTA.handle();
  server.handleClient(); // Handle incoming web server requests

  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorRead >= sensorReadInterval) {
    lastSensorRead = currentMillis;
    readSensors();
    updateDisplay();
  }

  // --- Main Pump Control Logic ---
  switch(currentPumpMode) {
    case PUMP_AUTO:
      if (readingCount > 0) {
        Input = data[readingCount-1].temp1; // PID input is the latest temp reading
        myPID.Compute();
        pumpOnDuration = Output;
      }
      break;
    case PUMP_MANUAL:
      pumpOnDuration = map(manualPumpPower, 0, 100, 0, pumpCycleInterval);
      break;
    case PUMP_OFF:
    default:
      pumpOnDuration = 0; // Off
      break;
  }
  
  // Time-based pump actuation
  if (currentMillis - pumpCycleStartTime >= pumpCycleInterval) {
    pumpCycleStartTime = currentMillis; // Start new cycle
  }

  if (pumpOnDuration > 0 && (currentMillis - pumpCycleStartTime < pumpOnDuration)) {
    // We are within the 'on' portion of the cycle
    ledcWrite(PUMP_PIN, PUMP_FIXED_DUTY_CYCLE); // Run at fixed power
  } else {
    // We are in the 'off' portion of the cycle
    ledcWrite(PUMP_PIN, 0);
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
 * @brief Reads all sensors and stores the data.
 */
void readSensors() {
  // Read Temperatures
  sensors.requestTemperatures(); 
  float tempC1 = sensors.getTempC(sensor1Address);
  float tempC2 = sensors.getTempC(sensor2Address);

  if (tempC1 == DEVICE_DISCONNECTED_C || tempC2 == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  
  // Read Pressure and apply drift compensation
  float pressure = 0;
  if (scale.is_ready()) {
    float rawPressure = scale.get_units(5); // Get an average of 5 readings
    float elapsedMinutes = millis() / 60000.0;
    float driftCorrection = elapsedMinutes * PRESSURE_DRIFT_PER_MINUTE;
    pressure = rawPressure - driftCorrection;
  } else {
    Serial.println("HX711 not found.");
  }

  // Calculate the actual pump power percentage based on on-time
  float pumpPowerPercent = (pumpOnDuration / (float)pumpCycleInterval) * 100.0;
  if (pumpPowerPercent < 0) pumpPowerPercent = 0;
  if (pumpPowerPercent > 100) pumpPowerPercent = 100;

  if (readingCount < MAX_READINGS) {
    data[readingCount].time = millis() / 1000;
    data[readingCount].temp1 = tempC1;
    data[readingCount].temp2 = tempC2;
    data[readingCount].pressure = pressure;
    data[readingCount].pumpPower = pumpPowerPercent;
    readingCount++;
  } else {
    // Shift all data left to make space for the new reading
    for (int i = 0; i < MAX_READINGS - 1; i++) {
      data[i] = data[i+1];
    }
    data[MAX_READINGS - 1].time = millis() / 1000;
    data[MAX_READINGS - 1].temp1 = tempC1;
    data[MAX_READINGS - 1].temp2 = tempC2;
    data[MAX_READINGS - 1].pressure = pressure;
    data[MAX_READINGS - 1].pumpPower = pumpPowerPercent;
  }
  
  Serial.print("S1: "); Serial.print(tempC1); Serial.print("C, S2: "); Serial.print(tempC2); Serial.print("C, Pressure: "); Serial.print(pressure);
  Serial.print(", Pump: "); Serial.print(data[readingCount-1].pumpPower); Serial.println("%");
}

/**
 * @brief Updates the OLED display with the latest sensor readings.
 */
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0,0);
  
  switch(currentPumpMode) {
    case PUMP_AUTO: display.println("Mode: Auto"); break;
    case PUMP_MANUAL: display.println("Mode: Manual"); break;
    case PUMP_OFF:  display.println("Mode: Off");  break;
  }
  
  if (readingCount > 0) {
    display.print("S1: ");
    display.print(data[readingCount-1].temp1, 1);
    display.print("/");
    display.println(Setpoint, 1);

    display.print("Pump: ");
    display.print(data[readingCount-1].pumpPower, 0);
    display.println("%");
    
    display.print("Pres: ");
    display.println(data[readingCount-1].pressure, 1);

  } else {
    display.println("No readings yet.");
  }
  
  display.println(WiFi.localIP());
  display.display();
}

/**
 * @brief Sets up the web server and its routes.
 */
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/setpoint", HTTP_POST, handleSetpointControl);
  server.on("/pid", HTTP_POST, handlePidControl);
  server.on("/pid/get", HTTP_GET, handleGetPid);
  server.on("/pump/mode", HTTP_POST, handlePumpModeControl);
  server.on("/pump/mode/get", HTTP_GET, handleGetPumpMode);
  server.on("/pump/manual", HTTP_POST, handleManualPumpControl);
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
  <title>OpenStill</title>
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
      background: white;
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
    .control-group {
      margin-bottom: 15px;
    }
    .control-group label {
      font-size: 18px;
      margin: 0 10px;
    }
    .control-group input[type="number"] {
      width: 80px;
      font-size: 16px;
      padding: 5px;
      text-align: center;
    }
    .button {
      background-color: #4CAF50;
      color: white;
      padding: 10px 20px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      margin-top: 10px;
      cursor: pointer;
      border: none;
      border-radius: 4px;
    }
    .pid-controls, .manual-controls {
        border: 1px solid #ccc;
        border-radius: 5px;
        padding: 10px;
        margin: 10px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>OpenStill</h1>
    <div id="chart-container">
      <canvas id="tempChart"></canvas>
    </div>
    <div class="controls-container">
      <div class="control-group">
        <label>Pump Mode:</label>
        <input type="radio" id="pumpOff" name="pumpMode" value="0"><label for="pumpOff">Off</label>
        <input type="radio" id="pumpAuto" name="pumpMode" value="1" checked><label for="pumpAuto">Auto</label>
        <input type="radio" id="pumpManual" name="pumpMode" value="2"><label for="pumpManual">Manual</label>
      </div>
      <div class="pid-controls">
        <div class="control-group">
          <label for="setpointInput">Target Temp (&deg;C):</label>
          <input type="number" min="20" max="100" step="0.1" id="setpointInput">
        </div>
        <div class="control-group">
          <label>Kp:</label>
          <input type="number" step="0.1" id="kpInput">
          <label>Ki:</label>
          <input type="number" step="0.1" id="kiInput">
          <label>Kd:</label>
          <input type="number" step="0.1" id="kdInput">
        </div>
      </div>
      <div class="manual-controls">
        <div class="control-group">
          <label for="manualPowerInput">Manual Power (%):</label>
          <input type="number" min="0" max="100" step="1" id="manualPowerInput" value="0">
        </div>
      </div>
      <a href="/download.csv" class="button">Download Data CSV</a>
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
          myChart.data.datasets[2].data = data.map(d => d.pumpPower);
          myChart.data.datasets[3].data = data.map(d => d.pressure);
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
                yAxisID: 'y-temp',
                fill: false
              }, {
                label: 'Sensor 2 (°C)',
                data: data.map(d => d.temp2),
                borderColor: 'rgba(54, 162, 235, 1)',
                yAxisID: 'y-temp',
                fill: false
              }, {
                label: 'Pump Power (%)',
                data: data.map(d => d.pumpPower),
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(75, 192, 192, 0.2)',
                yAxisID: 'y-power',
                fill: true
              }, {
                label: 'Pressure',
                data: data.map(d => d.pressure),
                borderColor: 'rgba(255, 159, 64, 1)',
                yAxisID: 'y-pressure',
                fill: false
              }]
            },
            options: {
              responsive: true,
              maintainAspectRatio: false,
              scales: {
                x: { display: true, title: { display: true, text: 'Time' } },
                'y-temp': {
                  type: 'linear',
                  display: true,
                  position: 'left',
                  title: { display: true, text: 'Temperature (°C)' }
                },
                'y-power': {
                  type: 'linear',
                  display: true,
                  position: 'right',
                  min: 0,
                  max: 100,
                  title: { display: true, text: 'Pump Power (%)' },
                  grid: { drawOnChartArea: false }
                },
                'y-pressure': {
                  type: 'linear',
                  display: true,
                  position: 'right',
                  title: { display: true, text: 'Pressure' },
                  grid: { drawOnChartArea: false }
                }
              }
            }
          });
        }
      }).catch(error => console.error('Chart update error:', error));
    };

    // --- Control Logic ---
    const setpointInput = document.getElementById('setpointInput');
    const kpInput = document.getElementById('kpInput');
    const kiInput = document.getElementById('kiInput');
    const kdInput = document.getElementById('kdInput');
    const manualPowerInput = document.getElementById('manualPowerInput');
    const pumpModeRadios = document.querySelectorAll('input[name="pumpMode"]');
    const pidControlsDiv = document.querySelector('.pid-controls');
    const manualControlsDiv = document.querySelector('.manual-controls');

    const fetchAndUpdatePidInputs = () => {
        return fetch('/pid/get')
            .then(response => response.json())
            .then(data => {
                kpInput.value = data.kp;
                kiInput.value = data.ki;
                kdInput.value = data.kd;
            })
            .catch(error => console.error('Error fetching PID values:', error));
    };
    
    const fetchAndUpdatePumpMode = () => {
        return fetch('/pump/mode/get')
            .then(response => response.json())
            .then(data => {
                document.getElementById('pump' + (data.mode === 0 ? 'Off' : data.mode === 1 ? 'Auto' : 'Manual')).checked = true;
                toggleControlsVisibility();
            })
            .catch(error => console.error('Error fetching pump mode:', error));
    };

    const toggleControlsVisibility = () => {
        pidControlsDiv.style.display = document.getElementById('pumpAuto').checked ? 'block' : 'none';
        manualControlsDiv.style.display = document.getElementById('pumpManual').checked ? 'block' : 'none';
    };

    setpointInput.addEventListener('change', (event) => {
      const temp = event.target.value;
      const formData = new FormData();
      formData.append('target', temp);
      fetch('/setpoint', { method: 'POST', body: new URLSearchParams(formData) });
    });

    manualPowerInput.addEventListener('change', (event) => {
      const power = event.target.value;
      const formData = new FormData();
      formData.append('power', power);
      fetch('/pump/manual', { method: 'POST', body: new URLSearchParams(formData) });
    });

    const updatePidTunings = () => {
      const formData = new FormData();
      formData.append('kp', kpInput.value);
      formData.append('ki', kiInput.value);
      formData.append('kd', kdInput.value);
      fetch('/pid', { method: 'POST', body: new URLSearchParams(formData) });
    };

    kpInput.addEventListener('change', updatePidTunings);
    kiInput.addEventListener('change', updatePidTunings);
    kdInput.addEventListener('change', updatePidTunings);

    pumpModeRadios.forEach(radio => {
        radio.addEventListener('change', (event) => {
            const formData = new FormData();
            formData.append('mode', event.target.value);
            fetch('/pump/mode', { method: 'POST', body: new URLSearchParams(formData) });
            toggleControlsVisibility();
        });
    });

    // --- Initial Load ---
    document.addEventListener('DOMContentLoaded', (event) => {
        fetchAndUpdatePidInputs();
        fetchAndUpdatePumpMode();
        setpointInput.value = 78.2; // Set default temp target on page load
        createOrUpdateChart();
        setInterval(createOrUpdateChart, 5000);
    });
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

/**
 * @brief Handles POST requests to update the PID setpoint.
 */
void handleSetpointControl() {
  if (server.hasArg("target")) {
    String targetStr = server.arg("target");
    Setpoint = targetStr.toDouble();
    
    Serial.print("New Setpoint received: ");
    Serial.println(Setpoint);
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request: 'target' parameter missing");
  }
}

/**
 * @brief Handles POST requests to update the PID tuning parameters.
 */
void handlePidControl() {
  bool updated = false;
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd")) {
    Kp = server.arg("kp").toDouble();
    Ki = server.arg("ki").toDouble();
    Kd = server.arg("kd").toDouble();
    myPID.SetTunings(Kp, Ki, Kd);
    
    myPID.SetMode(MANUAL);
    myPID.SetMode(AUTOMATIC);

    updated = true;
    Serial.println("PID Tunings Updated:");
    Serial.print("Kp: "); Serial.println(Kp);
    Serial.print("Ki: "); Serial.println(Ki);
    Serial.print("Kd: "); Serial.println(Kd);
  }
  
  if (updated) {
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request: Missing PID parameters");
  }
}

/**
 * @brief Handles GET requests for the current PID parameters.
 */
void handleGetPid() {
  String json = "{";
  json += "\"kp\":" + String(Kp);
  json += ",\"ki\":" + String(Ki);
  json += ",\"kd\":" + String(Kd);
  json += "}";
  server.send(200, "application/json", json);
}

/**
 * @brief Handles POST requests to update the pump mode.
 */
void handlePumpModeControl() {
  if (server.hasArg("mode")) {
    int mode = server.arg("mode").toInt();
    switch(mode) {
      case 0: currentPumpMode = PUMP_OFF; Serial.println("Pump mode set to OFF"); break;
      case 1: currentPumpMode = PUMP_AUTO; Serial.println("Pump mode set to AUTO"); break;
      case 2: currentPumpMode = PUMP_MANUAL; Serial.println("Pump mode set to MANUAL"); break;
    }
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request: 'mode' parameter missing");
  }
}

/**
 * @brief Handles GET requests for the current pump mode.
 */
void handleGetPumpMode() {
  String json = "{\"mode\":" + String(currentPumpMode) + "}";
  server.send(200, "application/json", json);
}

/**
 * @brief Handles POST requests to update the manual pump power.
 */
void handleManualPumpControl() {
  if (server.hasArg("power")) {
    manualPumpPower = server.arg("power").toInt();
    if (manualPumpPower < 0) manualPumpPower = 0;
    if (manualPumpPower > 100) manualPumpPower = 100;
    Serial.print("Manual pump power set to: ");
    Serial.println(manualPumpPower);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request: 'power' parameter missing");
  }
}


/**
 * @brief Serves the temperature and pump data as a JSON object by streaming it.
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
    json_item += ",\"pumpPower\":" + String(data[i].pumpPower);
    json_item += ",\"pressure\":" + String(data[i].pressure);
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

  server.sendContent("Time,Sensor1_C,Sensor2_C,PumpPower_%,Pressure\n");
  for (int i = 0; i < readingCount; i++) {
    String row = String(data[i].time) + "," + String(data[i].temp1) + "," + String(data[i].temp2) + "," + String(data[i].pumpPower) + "," + String(data[i].pressure) + "\n";
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
