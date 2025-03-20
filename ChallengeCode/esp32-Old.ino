/*
  IoT Challenge #1 - Parking Occupancy Detection Node
  
  11075199 - XU XUELI
  11072044 - SUN YILIN
  
  Optimizations can be enabled/disabled using the flags below
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_timer.h>

  
// Record the boot timestamp in advance (unit: microseconds)
// Actually, this is not accurate 
unsigned long bootTimeEarly = 0;

__attribute__((constructor))
void initBootTime() {
  bootTimeEarly = esp_timer_get_time();
}

// ========== OPTIMIZATION FLAGS ==========
// Set to true to enable optimizations, false for baseline behavior
#define OPTIMIZATION_EARLY_ECHO_RETURN  false   // Optimize ultrasonic sensor to return early when object detected
#define OPTIMIZATION_LOW_WIFI_POWER     false   // Use lower WiFi transmission power
#define OPTIMIZATION_SINGLE_BYTE_MSG    false  // Use single byte instead of string message

// HC-SR04 sensor pins
#define TRIG_PIN 5
#define ECHO_PIN 18

// Parking occupancy threshold
#define MAX_DISTANCE 50  // cm
// For 50cm distance, round trip is 100cm, taking ~2915μs
#define ECHO_TIMEOUT_US 3000  // Timeout for 50cm + margin
#define ECHO_FULL_TIMEOUT_US 30000 // Timeout for full range

// Deep sleep
#define TIME_TO_SLEEP 54  // seconds (personcode: 11075199)
#define uS_TO_S_FACTOR 1000000ULL

// Debug mode enables detailed timing information via Serial
#define DEBUG true

uint8_t broadcastAddress[] = {0x8C, 0xAA, 0xB5, 0x84, 0xFB, 0x90};
esp_now_peer_info_t peerInfo;


unsigned long bootStartTime;
unsigned long bootEndTime;
unsigned long sensorIdleStartTime;
unsigned long sensorIdleEndTime;
unsigned long sensorActiveStartTime;
unsigned long sensorActiveEndTime;
unsigned long wifiSetupStartTime;
unsigned long wifiSetupEndTime;
unsigned long txStartTime;
unsigned long txEndTime;
unsigned long wifiOffStartTime;
unsigned long wifiOffEndTime;


void printTimestamp(String message);
void setupESPNOW();
bool checkParking();
void sendParkingStatus(bool occupied);
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status);
void onDataReceived(const uint8_t *macAddr, const uint8_t *data, int dataLen);

void setup() {

  Serial.begin(115200);
  while (!Serial && millis() < 500); // Wait for serial, timeout after 500ms
  
  // Boot completed
  bootEndTime = esp_timer_get_time();
  unsigned long bootDuration = bootEndTime - bootStartTime;
  Serial.println("Boot time: " + String(bootDuration / 1000.0) + " ms");
  printTimestamp("BOOT COMPLETED");
  
  // SENSOR IDLE PERIOD - Configure HC-SR04 sensor
  sensorIdleStartTime = micros();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  sensorIdleEndTime = micros();
  printTimestamp("SENSOR IDLE COMPLETE");
  
  // SENSOR ACTIVE PERIOD - Check parking status
  sensorActiveStartTime = micros();
  bool occupied = checkParking();
  sensorActiveEndTime = micros();
  printTimestamp("SENSOR ACTIVE COMPLETE - " + String(occupied ? "OCCUPIED" : "FREE"));
  
  // WIFI SETUP PERIOD - Initialize ESP-NOW
  wifiSetupStartTime = micros();
  setupESPNOW();
  wifiSetupEndTime = micros();
  printTimestamp("WIFI SETUP COMPLETE");
  
  // TX PERIOD - Send parking status
  txStartTime = micros();
  sendParkingStatus(occupied);
  // Small delay to ensure transmission completes
  delay(20);
  txEndTime = micros();
  printTimestamp("TX COMPLETE");
  
  // WIFI OFF PERIOD - Disable WiFi to save power
  wifiOffStartTime = micros();
  WiFi.mode(WIFI_OFF);

  
  wifiOffEndTime = micros();
  printTimestamp("WIFI OFF COMPLETE");
  
  // Print timing summary for energy analysis
  printEnergyTimingSummary();
  
  // Configure deep sleep wakeup timer
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  printTimestamp("Going to deep sleep for " + String(TIME_TO_SLEEP) + " seconds");
  
  Serial.println("\n=== OPTIMIZATIONS STATUS ===");
  Serial.println("Early echo return: " + String(OPTIMIZATION_EARLY_ECHO_RETURN ? "ENABLED" : "DISABLED"));
  Serial.println("Low WiFi power: " + String(OPTIMIZATION_LOW_WIFI_POWER ? "ENABLED" : "DISABLED"));
  Serial.println("Single byte message: " + String(OPTIMIZATION_SINGLE_BYTE_MSG ? "ENABLED" : "DISABLED"));
  Serial.println("===========================\n");
  
 
  Serial.flush();
  
  
  esp_deep_sleep_start();
}

void loop() {
  
}

// Print timestamp and message for debugging
void printTimestamp(String message) {
  if (DEBUG) {
    unsigned long elapsed = micros() - bootStartTime;
    Serial.print(elapsed);
    Serial.print(" us: ");
    Serial.println(message);
  }
}

// Print summary of timing measurements for energy analysis
void printEnergyTimingSummary() {
  Serial.println("\n--- ENERGY TIMING SUMMARY ---");
  // Serial.println("Boot period: " + String((bootEndTime - bootStartTime) / 1000.0) + " ms");
  Serial.println("Sensor idle period: " + String((sensorIdleEndTime - sensorIdleStartTime) / 1000.0) + " ms");
  Serial.println("Sensor active period: " + String((sensorActiveEndTime - sensorActiveStartTime) / 1000.0) + " ms");
  Serial.println("WiFi setup period: " + String((wifiSetupEndTime - wifiSetupStartTime) / 1000.0) + " ms");
  Serial.println("Transmission period: " + String((txEndTime - txStartTime) / 1000.0) + " ms");
  Serial.println("WiFi off period: " + String((wifiOffEndTime - wifiOffStartTime) / 1000.0) + " ms");
  Serial.println("Total active time: " + String((wifiOffEndTime - bootStartTime) / 1000.0) + " ms");
  Serial.println("Deep sleep period: " + String(TIME_TO_SLEEP * 1000) + " ms");
  Serial.println("------------------------------\n");
}

// Setup ESP-NOW communication
void setupESPNOW() {
  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
  
  // Set WiFi power based on optimization setting
  if (OPTIMIZATION_LOW_WIFI_POWER) {
    WiFi.setTxPower(WIFI_POWER_2dBm);  // Lower power setting
    printTimestamp("WiFi power set to 2dBm");
  } else {
    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Default high power
    printTimestamp("WiFi power set to 19.5dBm (default)");
  }
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    printTimestamp("Error initializing ESP-NOW");
    return;
  }
  
  // Register callbacks
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
  // Register peer device
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    printTimestamp("Failed to add peer");
    return;
  }
}

// Check if the parking spot is occupied
bool checkParking() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long duration;
  
  if (OPTIMIZATION_EARLY_ECHO_RETURN) {
    // Optimized version: use shorter timeout for early return
    duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
    
    if (duration == 0) {
      // No echo received within timeout, parking spot is free
      printTimestamp("No object detected within 50cm");
      return false;
    } else {
      // Echo received before timeout, something is within 50cm
      float distance = duration * 0.0343 / 2;
      printTimestamp("Object detected at approximately " + String(distance) + " cm");
      return true;
    }
  } else {
    // Unoptimized version: measure full distance
    duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.0343 / 2;
    printTimestamp("Distance measured: " + String(distance) + " cm");
    return (distance <= MAX_DISTANCE && distance > 0);
  }
}

// Send parking status via ESP-NOW
void sendParkingStatus(bool occupied) {
  if (OPTIMIZATION_SINGLE_BYTE_MSG) {
    // Optimized version: send single byte
    uint8_t status = occupied ? '1' : '0';
    printTimestamp("Sending optimized message: " + String((char)status));
    esp_err_t result = esp_now_send(broadcastAddress, &status, 1);
    
    if (result != ESP_OK) {
      printTimestamp("Error sending the message");
    }
  } else {
    // Unoptimized version: send full string
    const char* message = occupied ? "OCCUPIED" : "FREE";
    printTimestamp("Sending message: " + String(message));
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)message, strlen(message) + 1);
    
    if (result != ESP_OK) {
      printTimestamp("Error sending the message");
    }
  }
}

// Callback function when data is sent
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  printTimestamp("Message send status: " + String(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed"));
}

// Callback function when data is received
void onDataReceived(const uint8_t *macAddr, const uint8_t *data, int dataLen) {
  if (dataLen == 1) {
    // Single byte message
    printTimestamp("Received one-byte message: " + String((char)data[0]));
  } else {
    // String message
    char message[dataLen + 1];
    memcpy(message, data, dataLen);
    message[dataLen] = '\0';
    printTimestamp("Received message: " + String(message));
  }
}