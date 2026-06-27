// /*
//   ESP32 Serial JSON Receiver
//   --------------------------
//   This sketch runs on the ESP32. It listens to incoming serial data from the 
//   Raspberry Pi on Hardware Serial 2 (RX2/TX2 pins), parses the custom JSON payload, 
//   and extracts the crack coordinates.
  
//   Dependencies:
//   - Install "ArduinoJson" library (by Benoit Blanchon) via the Arduino IDE Library Manager.
  
//   Wiring:
//   - RPi Pin 8 (GPIO 14 TXD)  -> ESP32 RX2 Pin (Typically GPIO 16)
//   - RPi Pin 10 (GPIO 15 RXD) -> ESP32 TX2 Pin (Typically GPIO 17)
//   - RPi Pin 6 (GND)          -> ESP32 GND (Crucial: Common ground is required!)
// */

// #include <ArduinoJson.h>

// // --- CONFIGURATION ---
// // Most ESP32 dev boards use GPIO 16 (RX2) and GPIO 17 (TX2) for Serial2
// #define RX2_PIN 16
// #define TX2_PIN 17
// #define SERIAL_BAUD 115200

// // We use a clean look-ahead buffer for incoming strings
// String inputString = "";
// bool stringComplete = false;

// void setup() {
//   // Initialize the USB Serial port for debugging output to your computer
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("--- ESP32 Crack Receiver Initialized ---");

//   // Initialize Hardware Serial 2 (comms with Raspberry Pi)
//   // PARAMETERS: Baud rate, config, RX pin, TX pin
//   Serial2.begin(SERIAL_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  
//   // Reserve 2048 bytes for the incoming JSON string to avoid RAM fragmentation
//   inputString.reserve(2048);
// }

// void loop() {
//   // 1. Read serial bytes from the Pi
//   while (Serial2.available()) {
//     char inChar = (char)Serial2.read();
    
//     // The Pi script ends every JSON packet with a newline character (\n)
//     if (inChar == '\n') {
//       stringComplete = true;
//     } else {
//       inputString += inChar;
//     }
//   }

//   // 2. Parse the JSON packet when a complete line is received
//   if (stringComplete) {
//     parsePiData(inputString);
    
//     // Reset the buffer for the next incoming packet
//     inputString = "";
//     stringComplete = false;
//   }
// }

// // Function to safely deserialize and extract variables
// void parsePiData(String jsonString) {
//   // DynamicJsonDocument allocates memory on the stack (v6/v7 compatible layout)
//   // 2048 bytes is plenty of room for several detected cracks in one frame
//   StaticJsonDocument<2048> doc;

//   DeserializationError error = deserializeJson(doc, jsonString);

//   if (error) {
//     Serial.print("JSON Deserialization failed: ");
//     Serial.println(error.f_str());
//     return;
//   }

//   // Extract core envelope variables
//   long frameId = doc["f"];       // Frame ID
//   int numDetections = doc["n"];  // Number of detections in this frame

//   // Print frame info to ESP32 console
//   Serial.print("Frame: ");
//   Serial.print(frameId);
//   Serial.print(" | Found: ");
//   Serial.print(numDetections);
//   Serial.println(" cracks");

//   // If there are detections, loop through and pull relative coordinates
//   if (numDetections > 0) {
//     JsonArray detections = doc["d"];
    
//     for (int i = 0; i < detections.size(); i++) {
//       JsonObject d = detections[i];
      
//       // Pull coordinates (0.0000 to 1.0000 relative to camera width/height)
//       float cx = d["cx"];     // Centroid X
//       float cy = d["cy"];     // Centroid Y
//       float conf = d["conf"]; // Confidence score (0.0 to 1.0)

//       // Print parsed coordinates out
//       Serial.print("  -> Crack #");
//       Serial.print(i);
//       Serial.print(" | Center: (");
//       Serial.print(cx, 4);
//       Serial.print(", ");
//       Serial.print(cy, 4);
//       Serial.print(") | Confidence: ");
//       Serial.println(conf, 3);

//       // --- YOUR ESP32 CONTROL LOGIC GOES HERE ---
//       // For example:
//       // if (cx < 0.45) { Turn Left; }
//       // else if (cx > 0.55) { Turn Right; }
//       // else { Stay Centered; }
//     }
//   }
// }