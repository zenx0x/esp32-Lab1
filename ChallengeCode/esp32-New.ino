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
#include <HCSR04.h>

#include "esp_timer.h"  


unsigned long bootTimeEarly = 0;

__attribute__((constructor))
void initBootTime() {
  bootTimeEarly = esp_timer_get_time();
}

// HC-SR04 sensor pins
#define TRIG_PIN 5
#define ECHO_PIN 18

// Parking occupancy threshold
#define MAX_DISTANCE 50  // cm
// For 50cm distance, round trip is 100cm, taking ~2915μs
#define ECHO_TIMEOUT_US 3000  // Timeout for 50cm + margin


#define TIME_TO_SLEEP 54  // seconds (personcode：11075199)
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
bool isOccupied();
void sendParkingStatus(const char* status);
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status);

void setup() {

  Serial.begin(115200);

  bootEndTime = esp_timer_get_time();
  unsigned long bootDuration = bootEndTime - bootTimeEarly;
  
  Serial.println("Boot time: " + String(bootDuration / 1000.0) + " ms");
  printTimestamp("BOOT START (after bootloader)");

  
  // SENSOR IDLE PERIOD - Configure HC-SR04 sensor
  sensorIdleStartTime = micros();
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  printTimestamp("SENSOR IDLE COMPLETE");
  sensorIdleEndTime = micros();
  
  // SENSOR ACTIVE PERIOD - Check for occupancy using optimized detection
  sensorActiveStartTime = micros();
  bool occupied = isOccupied();
  printTimestamp("SENSOR ACTIVE COMPLETE - Occupancy check done");
  sensorActiveEndTime = micros();
  
  // Determine parking status based on occupancy
  const char* parkingStatus = occupied ? "OCCUPIED" : "FREE";
  printTimestamp("Parking status: " + String(parkingStatus));
  
  // WIFI SETUP PERIOD - Initialize ESP-NOW
  wifiSetupStartTime = micros();
  setupESPNOW();
  printTimestamp("WIFI SETUP COMPLETE");
  wifiSetupEndTime = micros();
  
  // TX PERIOD - Send parking status
  txStartTime = micros();
  sendParkingStatus(parkingStatus);
  printTimestamp("TX COMPLETE");
  txEndTime = micros();
  // Small delay to ensure transmission completes
  delay(20);
  
  // WIFI OFF PERIOD - Disable WiFi to save power
  wifiOffStartTime = micros();
  WiFi.mode(WIFI_OFF);
  printTimestamp("WIFI OFF COMPLETE");
  wifiOffEndTime = micros();

  // Print timing summary for energy analysis
  printEnergyTimingSummary();
  
  // Configure deep sleep wakeup timer
  printTimestamp("Going to deep sleep for " + String(TIME_TO_SLEEP) + " seconds");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
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
  Serial.println("Total active time: " + String((txEndTime - sensorActiveStartTime) / 1000.0) + " ms");
  Serial.println("Deep sleep period: " + String(TIME_TO_SLEEP * 1000) + " ms");
  Serial.println("------------------------------\n");
}

// Setup ESP-NOW communication
void setupESPNOW() {
  // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
  
  // Set WiFi power to minimum required level
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    printTimestamp("Error initializing ESP-NOW");
    return;
  }

  esp_now_init();
  // Register callback for when data is sent
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
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

// Check if the parking spot is occupied (optimized for 50cm threshold)
bool isOccupied() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse duration with optimized timeout
  // If pulseIn returns 0, it means timeout occurred (no echo within ECHO_TIMEOUT_US)
  // which indicates no object detected within MAX_DISTANCE
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  
  if (duration == 0) {
    // No echo received within timeout, parking spot is free
    printTimestamp("No object detected within 50cm");
    return false;
  } else {
    // Echo received before timeout, something is within 50cm
    // Calculate approximate distance for debugging
    float distance = duration * 0.0343 / 2;
    printTimestamp("Object detected at approximately " + String(distance) + " cm");
    return true;
  }
}

// Send parking status via ESP-NOW
void sendParkingStatus(const char* status) {
  printTimestamp("Sending message: " + String(status));
  
  // Send message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)status, strlen(status) + 1);
  
  if (result != ESP_OK) {
    printTimestamp("Error sending the message");
  }
}

// Callback function when data is sent
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  printTimestamp("Message send status: " + String(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed"));
}

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
  char receivedString[len];
  memcpy(receivedString, data, len);
  //print the received message
  Serial.println("Message received: " + String(receivedString));
}