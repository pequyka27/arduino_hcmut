#define START_BYTE '<'
#define END_BYTE '>'
#define MAX_PACKET_LEN 64

char packetBuffer[MAX_PACKET_LEN];
uint8_t packetIndex = 0;
bool packetInProgress = false;
uint8_t currentChecksum = 0;

void handleSerialComm() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == START_BYTE) {
      packetIndex = 0;
      currentChecksum = 0;
      packetInProgress = true;
      continue;
    }

    if (!packetInProgress) continue;

    if (c == END_BYTE) {
      processPacket(packetBuffer, packetIndex);
      packetInProgress = false;
      return;
    }

    if (packetIndex < MAX_PACKET_LEN - 1) {
      // Update checksum while receiving
      currentChecksum ^= c; 
      packetBuffer[packetIndex++] = c;
    }
  }
}

void processPacket(char* data, uint8_t length) {
  // Tách command và checksum
  uint8_t checksum = (uint8_t)data[length-1];
  uint8_t calcChecksum = currentChecksum ^ data[length-1]; // Loại bỏ checksum khỏi phép XOR
  
  if (calcChecksum != 0) return; // Bỏ qua packet lỗi

  // Xử lý lệnh (ví dụ: "M,100,200,50")
  char* command = strtok(data, ",");
  if (!command) return;

  if (strcmp(command, "M") == 0) { // Motor command
    int16_t speeds[4];
    for (uint8_t i=0; i<4; i++) {
      char* val = strtok(NULL, ",");
      if (!val) return;
      speeds[i] = atoi(val);
    }
    setMotorSpeeds(speeds);
  }
  // ... các lệnh khác
}
