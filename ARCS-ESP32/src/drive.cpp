
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Replace with the receiver's MAC address
uint8_t broadcastAddress[] = {0x70, 0x4b, 0xca, 0x4e, 0x03, 0x98}; //  70:4b:ca:4e:03:98


const int vy1Pin = 32;
const int vy2Pin = 35;
const int vy3Pin = 34;
const int button1Pin = 5;
const int button2Pin = 18;
const int button3Pin = 19;

int vy1Value = 0;
int vy2Value = 0;
int vy3Value = 0;
bool button1Value = false;
bool button2Value = false;
bool button3Value = false;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
int vy1;
int vy2;
int vy3;
bool button1;
bool button2;
bool button3;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  pinMode(vy1Pin, INPUT);
  pinMode(vy2Pin, INPUT);
  pinMode(vy3Pin, INPUT);
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  pinMode(button3Pin, INPUT_PULLUP);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {

  vy1Value = analogRead(vy1Pin);
  vy2Value = analogRead(vy2Pin);
  vy3Value = analogRead(vy3Pin);
  Serial.print("VY1: ");
  Serial.print(vy1Value);
  Serial.print(" VY2: ");
  Serial.print(vy2Value);
  Serial.print(" VY3: ");
  Serial.println(vy3Value);

  button1Value = digitalRead(button1Pin);
  button2Value = digitalRead(button2Pin);
  button3Value = digitalRead(button3Pin);
  // Set values to send
  myData.vy1 = vy1Value;
  myData.vy2 = vy2Value;
  myData.vy3 = vy3Value;
  myData.button1 = button1Value;
  myData.button2 = button2Value;
  myData.button3 = button3Value;
 
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
delay(500); // Send data every 500 milliseconds
}
/*




#include <esp_now.h>
#include <WiFi.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
int vy1;
int vy2;
int vy3;
bool button1;
bool button2;
bool button3;
} struct_message;


// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("VY1 ");
  Serial.println(myData.vy1);
  Serial.print("VY2 ");
  Serial.println(myData.vy2);
  Serial.print("VY3 ");
  Serial.println(myData.vy3);
  Serial.print("Button1: ");
  Serial.println(myData.button1);
  Serial.print("Button2: ");
  Serial.println(myData.button2);
  Serial.print("Button3: ");
  Serial.println(myData.button3);
  Serial.println();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {

}




*/