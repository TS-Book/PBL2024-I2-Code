#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// Select camera model
#define CAMERA_MODEL_WROVER_KIT
#include "camera_pins.h"

// WiFi credentials
const char* ssid = "YourWIFI";
const char* password = "YourPassword";

// UART configuration for Arduino communication
#define RX_PIN 14
#define TX_PIN 13

// UDP configuration
WiFiUDP udp;
const unsigned int localPort = 12345;
char incomingPacket[255];

// Task Handles
TaskHandle_t cameraTaskHandle;
TaskHandle_t udpTaskHandle;

// Function Declarations
void startCameraServer();
void configureCamera(camera_config_t &config);
void initializeWiFi();
void handleCameraStream(void *pvParameters);
void handleUdpCommands(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nStarting ESP32 Camera...");
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Camera configuration
  camera_config_t config;
  configureCamera(config);

  // Initialize the camera
  esp_err_t err;
  do {
    err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera initialization failed! Error: 0x%x. Retrying...\n", err);
      delay(1000);
    }
  } while (err != ESP_OK);

  Serial.println("Camera initialized successfully!");

  // WiFi setup
  initializeWiFi();

  // Start the camera streaming server
  startCameraServer();

  Serial.printf("Camera Ready! Use 'http://%s' to connect\n", WiFi.localIP().toString().c_str());

  // Start listening for UDP packets
  udp.begin(localPort);
  Serial.printf("Listening for UDP packets on port %d\n", localPort);

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(handleCameraStream, "Camera Stream Task", 8192, NULL, 1, &cameraTaskHandle, 0); // Camera stream task
  xTaskCreatePinnedToCore(handleUdpCommands, "UDP Commands Task", 4096, NULL, 1, &udpTaskHandle, 1);       // UDP commands task
}

void loop() {
  // The loop can remain empty or handle light tasks if needed
  delay(100); // Just a placeholder
}

// Task: Camera Stream
void handleCameraStream(void *pvParameters) {
  while (true) {
    // Camera server runs continuously, so no extra code is needed here
    vTaskDelay(10 / portTICK_PERIOD_MS); // Yield task to allow other processes
  }
}

// Task: Handle UDP Commands
void handleUdpCommands(void *pvParameters) {
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
      if (len > 0) {
        incomingPacket[len] = 0; // Null-terminate the packet
      }

      Serial.print("Received command: ");
      Serial.println(incomingPacket);

      if (strcmp(incomingPacket, "l") == 0) {
        Serial2.write("l\n");
      } else if (strcmp(incomingPacket, "r") == 0) {
        Serial2.write("r\n");
      } else if (strcmp(incomingPacket, "c") == 0) {
        Serial2.write("c\n");
      } else {
        Serial.println("Unknown command received");
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Small delay to yield the task
  }
}

void configureCamera(camera_config_t &config) {
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_HVGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_HVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
}

void initializeWiFi() {
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  Serial.println("Connecting to WiFi...");

  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Check credentials or signal strength.");
  }
}
