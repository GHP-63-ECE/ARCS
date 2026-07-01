#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Replace with the receiver's MAC address
uint8_t receiverAddress[] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

// Data structure (must match receiver)
typedef struct {
int value;
float voltage;
char message[32];
} DataPacket;

DataPacket data;

// Callback after sending
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
Serial.print("Send Status: ");
Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void setup() {
Serial.begin(115200);

// Set WiFi to Station mode
WiFi.mode(WIFI_STA);

// Initialize ESP-NOW
if (esp_now_init() != ESP_OK) {
Serial.println("Error initializing ESP-NOW");
return;
}

// Register send callback
esp_now_register_send_cb(onDataSent);

// Add receiver
esp_now_peer_info_t peerInfo = {};
memcpy(peerInfo.peer_addr, receiverAddress, 6);
peerInfo.channel = 0;
peerInfo.encrypt = false;

if (esp_now_add_peer(&peerInfo) != ESP_OK) {
Serial.println("Failed to add peer");
return;
}
}

void loop() {
data.value = random(0, 100);
data.voltage = analogRead(34) * (3.3 / 4095.0);
strcpy(data.message, "Hello!");

esp_err_t result = esp_now_send(
receiverAddress,
(uint8_t *)&data,
sizeof(data)
);

if (result == ESP_OK) {
Serial.println("Packet queued");
} else {
Serial.println("Send error");
}

delay(1000);
}