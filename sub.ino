#define START_BYTE '<'
#define END_BYTE '>'
#define MAX_PACKET_LEN 64

#include <Arduino.h>

// -------------------------------
// Global Variables & Constants
// -------------------------------
uint32_t lastSerialSend = 0;            // Timestamp for last serial transmission
const uint16_t SERIAL_SEND_INTERVAL = 15; // ~15ms interval (~66Hz update rate)

const float LINE_FOLLOW_SPEED_FACTOR = 0.6; // Limit for line following speed (60% of max)

char packetBuffer[MAX_PACKET_LEN];
uint8_t packetIndex = 0;
bool packetInProgress = false;
uint8_t currentChecksum = 0;

bool lineFollowingMode = false; // Flag for line following mode

// -------------------------------
// Function Prototypes
// -------------------------------
void handleSerialComm();
void processPacket(char* data, uint8_t length);
void sendXYRData(int x, int y, int r);
void readLineSensors();
void calculateXYR(int* x, int* y, int* r);
void setMotorSpeeds(int16_t speeds[4]);

// -------------------------------
// Arduino Setup Function
// -------------------------------
void setup() {
  Serial.begin(115200);
  // Initialize sensors and other devices here
}

// -------------------------------
// Arduino Main Loop
// -------------------------------
void loop() {
  handleSerialComm(); // Process incoming serial data

  int x = 0, y = 0, r = 0; // Variables for sensor data

  if (lineFollowingMode) {
    // Update sensor readings and compute control values
    readLineSensors();
    calculateXYR(&x, &y, &r);

    // Ensure we send data at a controlled rate
    uint32_t currentMillis = millis();
    if (currentMillis - lastSerialSend >= SERIAL_SEND_INTERVAL) {
      lastSerialSend = currentMillis;

      // Apply speed limitation for line following
      x = round(x * LINE_FOLLOW_SPEED_FACTOR);
      y = round(y * LINE_FOLLOW_SPEED_FACTOR);
      r = round(r * LINE_FOLLOW_SPEED_FACTOR);

      sendXYRData(x, y, r); // Transmit computed data to master
    }
  }
}

// -------------------------------
// Serial Communication Handler
// -------------------------------
void handleSerialComm() {
  while (Serial.available()) {
    char c = Serial.read();

    // Detect start of packet
    if (c == START_BYTE) {
      packetIndex = 0;
      currentChecksum = 0;
      packetInProgress = true;
      continue;
    }

    // Skip characters if no packet is in progress
    if (!packetInProgress) continue;

    // Detect end of packet and process it
    if (c == END_BYTE) {
      processPacket(packetBuffer, packetIndex);
      packetInProgress = false;
      return;
    }

    // Append character to buffer and update checksum if space allows
    if (packetIndex < MAX_PACKET_LEN - 1) {
      currentChecksum ^= c;
      packetBuffer[packetIndex++] = c;
    }
  }
}

// -------------------------------
// Packet Processing Function
// -------------------------------
void processPacket(char* data, uint8_t length) {
  // Validate checksum
  uint8_t checksum = (uint8_t)data[length - 1];
  uint8_t calcChecksum = currentChecksum ^ data[length - 1];
  
  if (calcChecksum != 0) return; // Discard invalid packet

  // Tokenize command from packet data
  char* command = strtok(data, ",:");
  if (!command) return;

  // Process commands from master
  if (strcmp(command, "M") == 0) { // Motor command
    int16_t speeds[4];
    for (uint8_t i = 0; i < 4; i++) {
      char* val = strtok(NULL, ",");
      if (!val) return;
      speeds[i] = atoi(val);
    }
    setMotorSpeeds(speeds);
  }
  else if (strcmp(command, "L") == 0) { // Line following mode command
    char* val = strtok(NULL, ",:");
    if (!val) return;
    lineFollowingMode = (atoi(val) == 1);
  }
  // Additional commands can be handled here
}

// -------------------------------
// Data Transmission Function
// -------------------------------
void sendXYRData(int x, int y, int r) {
  char payload[20];
  snprintf(payload, sizeof(payload), "%d:%d:%d", x, y, r);
  
  // Calculate checksum for payload
  byte checksum = 0;
  for (int i = 0; payload[i] != '\0'; i++) {
    checksum ^= payload[i];
  }
  
  // Construct and send the packet
  char packet[30];
  snprintf(packet, sizeof(packet), "%c%s:%02X%c", START_BYTE, payload, checksum, END_BYTE);
  Serial.println(packet);
}

// -------------------------------
// Stub Functions (Implement as needed)
// -------------------------------
void readLineSensors() {
  // Implement sensor reading logic here
}

void calculateXYR(int* x, int* y, int* r) {
  // Compute XYR values from sensor data
  *x = 0;   // Default value (update based on actual algorithm)
  *y = 100; // Forward movement with moderate speed
  *r = 0;   // No rotation by default
}

void setMotorSpeeds(int16_t speeds[4]) {
  // Set motor speeds based on received command values
}
