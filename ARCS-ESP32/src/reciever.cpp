#include <Arduino.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>
#include <Wifi.h>

BluetoothSerial BT;

// Initialize configuration parameters
const long SERIAL_BAUD_RATE = 115200;

void setup() {
BT.begin("ARCS-Vision");
  // Open the primary hardware serial channel for Mac communication
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {
    delay(10); // Wait for terminal configuration to stabilize
  }
  
  Serial.println("\n=============================================");
  Serial.println("ESP32 Autonomous Tracking Receiver Online (PIO)");
  Serial.println("Awaiting JSON data from Mac M5 Engine...");
  Serial.println("=============================================");
}

void loop() {


  // Check if a telemetry packet string has crossed the serial bus
  if (Serial.available() > 0) {
    // Read the incoming byte stream until the newline character is reached
    String jsonPayload = Serial.readStringUntil('\n');
    jsonPayload.trim();

    // Skip processing if the line is blank noise
    if (jsonPayload.length() == 0) return;\

    // Allocate an optimization block memory document for parsing
    JsonDocument doc;

    // Unpack the JSON string
    DeserializationError error = deserializeJson(doc, jsonPayload);

    // Guard rail: If the packet was fragmented over the wire, catch the error cleanly
    if (error) {
      Serial.print("Data Stream Sync Error: ");
      Serial.println(error.f_str());
      return;
    }

    // Extract core tracking headers
    long frameId = doc["f"];
    int targetCount = doc["n"];
    JsonArray targets = doc["d"];

    // Print diagnostic breakdown back to the system console
    Serial.print("[Frame #");
    Serial.print(frameId);
    Serial.print("] Detected Units: ");
    Serial.println(targetCount);

    // Loop through all parsed target objects in the payload data stream
    for (JsonObject target : targets) {
      float cx = target["cx"];
      float cy = target["cy"];
      float confidence = target["conf"];

      // If the Python script sends -1, it means the field of view is completely clear
      if (cx != -1.0) {
        Serial.print("  --> Target Locked | Center X: ");
        Serial.print(cx, 4);
        Serial.print(" | Center Y: ");
        Serial.print(cy, 4);
        Serial.print(" | Conf: ");
        Serial.println(confidence, 2);

        // -------------------------------------------------------------
        // YOUR HARDWARE CONTROL GOES HERE:
        // 'cx' and 'cy' are normalized values between 0.0000 and 1.0000.
        // -------------------------------------------------------------
        
      } else {
        Serial.println("  --> No tracking signatures found.");
      }
    }
  }
}