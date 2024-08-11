#include "HX711.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <qrcode.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

#include "peer.h"


// Define the pins for KY-040 rotary encoder
#define CLK_PIN 5
#define DT_PIN 4
#define SW_PIN 18

#define CONFIG_FREERTOS_UNICORE 1 // Add this line for FreeRTOS Unicore
#define CONFIG_FREERTOS_HZ 1000


#define ESC_PIN 15
#define PWM_PIN 15
#define TACH_PIN 2
#define SCK_PIN 16
#define OUT_PIN 17

#define ESC_MIN 900
#define ESC_MAX 2000

#define CALIBRATION 6283.0

#define USE_PWM false
#define USE_ESC true

#define NAME "OpenPAP"
#define TITLE " --==[ OpenPAP ]==--"
#define COPY "(C) Derek Anderson"
#define LINK " https://openpap.org"

Servo ESC;  
HX711 scale;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

WiFiManager wm;

enum State {
  STATE_INIT,
  STATE_READY,
  STATE_RUNNING,
  STATE_ERROR
};

State state = STATE_INIT;

class PID {

  private:

    double kp;
    double ki;
    double kd;
    double *input;
    double *output; 
    double *setpoint;
    double output_sum, last_input;
    double out_min, out_max;

  public:

    PID(double* input, double* output, double* setpoint, double Kp, double Ki, double Kd, double out_min, double out_max) {
      this->output = output;
      this->input = input;
      this->setpoint = setpoint;
      this->out_min = out_min;
      this->out_max = out_max;
      this->kp = Kp;
      this->ki = Ki;
      this->kd = Kd;
      output_sum = *output;
      last_input = *input;
      if(output_sum > out_max) output_sum = out_max;
      else if(output_sum < out_min) output_sum = out_min;
    }

    void compute() {
      double input = *(this->input);
      double error = *setpoint - input;
      double delta_input = (input - last_input);
      output_sum+= (ki * error);
      if(output_sum > out_max) output_sum = out_max;
      else if(output_sum < out_min) output_sum = out_min;
      double output = kp * error;
      output += output_sum - kd * delta_input;
      if(output > out_max) output = out_max;
      else if(output < out_min) output = out_min;
      *(this->output) = output;
    }

};

double pressure = 90; // mmH2O

void esc_setup() {
  display.println("Init ESC...");
  display.display();
  ESC.attach(ESC_PIN);
  ESC.writeMicroseconds(ESC_MIN);
  /*for (int i=ESC_MIN; i<ESC_MIN+200; ++i) {
    Serial.println(i);
    ESC.writeMicroseconds(i);
    delay(20);
  }
  for (int i=ESC_MIN+200; i<ESC_MIN; --i) {
    Serial.println(i);
    ESC.writeMicroseconds(i);
    delay(20);
  }//*/
  delay(500);
}

int tach = 0;
float tach_ma = 0;
float tach_ma_weight = 1.0 / 20.0;
void IRAM_ATTR handle_tach() {
  ++tach;
}

void pwm_setup() {
  attachInterrupt(TACH_PIN, handle_tach, RISING);
  ledcAttach(PWM_PIN, 4000, 9);
  ledcWrite(PWM_PIN, 0);
}

void setup_display() {
  Wire.begin();
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.setRotation(2);

  // Clear the display buffer
  display.clearDisplay();

  // Set text size and color
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Display initial text
  display.setCursor(0, 0);
  display.println(TITLE);
  display.println(LINK);
  display.println();
  display.display(); // Display text
  delay(1000);
}

void setup_scale() {
  scale.begin(OUT_PIN, SCK_PIN);
  display.println("Taring sensor...");
  display.display();
  Serial.println("Before setting up the scale:");
  Serial.print("read average: ");
  Serial.println(scale.read_average(10));              // print the average of 20 readings from the ADC
  scale.tare(); 
}

bool button_pressed = false;
void IRAM_ATTR handle_button() {
  button_pressed = true;
}


void setup_button() {
  pinMode(SW_PIN, INPUT_PULLDOWN);
  attachInterrupt(SW_PIN, handle_button, HIGH);
}

void cpap(void *params) {
  Serial.println("Starting...");
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t delay_ms = 100 / portTICK_PERIOD_MS;
  double input, output;
  PID pid(&input, &output, &pressure, 8, 1, 0, 0, USE_ESC ? ESC_MAX-ESC_MIN : 512);

  while (!button_pressed) {
    int now = millis();
    input = scale.get_value(1) / CALIBRATION;
    pid.compute();
    if (USE_PWM) ledcWrite(PWM_PIN, (int)output);
    if (USE_ESC) ESC.writeMicroseconds(ESC_MIN+(int)output);
    double fan_pct = output*100/(USE_ESC ? ESC_MAX-ESC_MIN : 512);
    Serial.print("pressure:");
    Serial.print(input/10);
    Serial.print(",fan:");
    Serial.print(fan_pct);
    Serial.print(",now:");
    Serial.print(now);
    Serial.println();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(TITLE);
    display.println("Pressure (cm H2O)");
    display.print("");
    display.print(pressure/10);
    display.print(" => ");
    display.println(input/10);
    display.println();
    display.println("Fan (%)");
    display.print("");
    display.print(fan_pct);
    display.display();
    vTaskDelayUntil(&last_wake_time, delay_ms);
  }
  button_pressed = false;
  wind_down(output);
  ESP.restart();
}

void drawQRCode(esp_qrcode_handle_t qrcode) {
    int qr_size = esp_qrcode_get_size(qrcode);
    int scale = min(SCREEN_WIDTH, SCREEN_HEIGHT) / qr_size;
    int qr_x = 64;
    int qr_y = 0;

    for (int y = 0; y < qr_size; y++) {
        for (int x = 0; x < qr_size; x++) {
            if (esp_qrcode_get_module(qrcode, x, y)) {
                display.fillRect(qr_x + x * scale, qr_y + y * scale, scale, scale, 1);
            }
        }
    }
}

void setup_wifi() {
  char ssid[28];
  char password[10];
  char login_msg[100];
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  snprintf(ssid,25, "OpenPAP-%08X",chipId);
  int randNumber = random(10000000, 99999999);
  itoa(randNumber, password, 10);
  Serial.print("WIFI Password:");
  Serial.println(password);
  sprintf(login_msg, "WIFI:T:WPA;S:%s;P:%i;;", ssid, randNumber);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println();
  display.println("Scan for");
  display.println(NAME);
  display.println("setup or");
  display.println("press");
  display.println("button.");
  esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
  cfg.display_func = drawQRCode;
  cfg.max_qrcode_version = 10;  // Adjust as needed
  cfg.qrcode_ecc_level = ESP_QRCODE_ECC_LOW;
  esp_err_t ret = esp_qrcode_generate(&cfg, login_msg);
  if (ret != ESP_OK) {
      Serial.println("Failed to generate QR code");
      return;
  }
  display.display();

  wm.setConfigPortalBlocking(false);
  wm.setConfigPortalTimeout(320);
  if (wm.autoConnect(ssid, password)) {
    Serial.println("Wifi connected!");
    setup_ota();
  } else {
    Serial.println("No wifi connection.");
    for (int i=0; i<60; ++i) {
      if (button_pressed) break;
      delay(1000);
    }
    button_pressed = false;
  }
  display.clearDisplay();
  display.setCursor(0,0);

}

void setup_ota() {
  Serial.println("Setting up OTA...");
  //ArduinoOTA.setPassword("admin");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();
}

PeerConnection *g_pc;
PeerConnectionState eState = PEER_CONNECTION_CLOSED;
int gDataChannelOpened = 0;
static TaskHandle_t xPcTaskHandle = NULL;
static TaskHandle_t xCameraTaskHandle = NULL;
static TaskHandle_t xPsTaskHandle = NULL;


int64_t get_timestamp() {

  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

static void oniceconnectionstatechange(PeerConnectionState state, void *user_data) {

  ESP_LOGI(TAG, "PeerConnectionState: %d", state);
  eState = state;
  // not support datachannel close event
  if (eState != PEER_CONNECTION_COMPLETED) {
    gDataChannelOpened = 0;
  }
}

static void onmessage(char *msg, size_t len, void *userdata, uint16_t sid) {

  ESP_LOGI(TAG, "Datachannel message: %.*s, size", len, msg);
}

void onopen(void *userdata) {
 
  ESP_LOGI(TAG, "Datachannel opened");
  gDataChannelOpened = 1;
}

static void onclose(void *userdata) {
}

void peer_signaling_task(void *arg) {

  ESP_LOGI(TAG, "peer_signaling_task started");

  for(;;) {

    peer_signaling_loop();

    vTaskDelay(pdMS_TO_TICKS(10));
  }

}

void peer_connection_task(void *arg) {

  ESP_LOGI(TAG, "peer_connection_task started");

  for(;;) {

    peer_connection_loop(g_pc);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup_webrtc() {
  static char deviceid[32] = {0};
  uint32_t chipId = 0;
  for (int i = 0; i < 17; i = i + 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  snprintf(deviceid,32, "OpenPAP-%08X",chipId);

  PeerConfiguration config = {
   .ice_servers = {
    { .urls = "stun:stun.l.google.com:19302" }
   },
   .datachannel = DATA_CHANNEL_BINARY,
  };

  ESP_LOGI(TAG, "[APP] Startup..");
  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
  esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
  esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
  esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
  esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
  esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

//  ESP_ERROR_CHECK(nvs_flash_init());
//  ESP_ERROR_CHECK(esp_netif_init());
//  ESP_ERROR_CHECK(esp_event_loop_create_default());
//  ESP_ERROR_CHECK(mdns_init());
//  ESP_ERROR_CHECK(example_connect());

  ESP_LOGI(TAG, "Device ID: %s", deviceid);

  peer_init();

  g_pc = peer_connection_create(&config);
  peer_connection_oniceconnectionstatechange(g_pc, oniceconnectionstatechange);
  peer_connection_ondatachannel(g_pc, onmessage, onopen, onclose);

  peer_signaling_join_channel(deviceid, g_pc);

  xTaskCreatePinnedToCore(peer_connection_task, "peer_connection", 8192, NULL, 10, &xPcTaskHandle, 1);

  xTaskCreatePinnedToCore(peer_signaling_task, "peer_signaling", 8192, NULL, 10, &xPsTaskHandle, 0);

  ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
  ESP_LOGI(TAG, "open https://sepfy.github.io/webrtc?deviceId=%s", deviceid);

}


void setup() {
  Serial.begin(115200);

  setup_display();

  if (USE_ESC) esc_setup();
  if (USE_PWM) pwm_setup();

  setup_scale();

  setup_button();

  setup_wifi();

  setup_webrtc();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(TITLE);
  display.println(LINK);
  display.println();
  display.print("CPAP: ");
  display.print(pressure/10);
  display.println(" cm H2O");
  display.println("Press to start...");
  display.display();
  Serial.println("READY");
  state = STATE_READY;
  delay(1000);
}

void wind_down(double current_speed) {
      if (USE_PWM) {
        for (int i=(int)current_speed; i>0; --i) {
          ledcWrite(PWM_PIN, i);
          delay(1);
        }
      }
      if (USE_ESC) {
        for (int i=ESC_MIN+(int)current_speed; i>ESC_MIN; --i) {
          ESC.writeMicroseconds(i);
          delay(1);
        }
      }
}

void loop() {
  wm.process();
  ArduinoOTA.handle();

  if (state==STATE_RUNNING) {
    delay(100);
  } else
  if (state==STATE_READY) {
    if (button_pressed) {
      button_pressed = false;
      state = STATE_RUNNING;
    xTaskCreatePinnedToCore(cpap, "cpap", 4096, (void *)1, 1, NULL, 1);
    }
    delay(100);
  } else
  if (state==STATE_ERROR) {
    if (button_pressed) {
      button_pressed = false;
      ESP.restart();
    }
  } else {
    delay(100);
  }
}

