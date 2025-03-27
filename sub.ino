#include <Arduino.h>

// Cấu hình protocol
const char START_BYTE = '<';
const char END_BYTE = '>';
const unsigned int MAX_PACKET_LENGTH = 32;

// Biến xử lý
char packetBuffer[MAX_PACKET_LENGTH];
unsigned int packetIndex = 0;
bool inPacket = false;

void setup() {
  Serial.begin(115200); // Phải khớp baudrate với master
  while (!Serial);
}

void loop() {
  processSerial();
}

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == START_BYTE) {
      inPacket = true;
      packetIndex = 0;
      continue;
    }
    
    if (inPacket) {
      if (c == END_BYTE) {
        parsePacket(packetBuffer);
        inPacket = false;
        return;
      }
      
      if (packetIndex < MAX_PACKET_LENGTH-1) {
        packetBuffer[packetIndex++] = c;
      }
      else {
        // Buffer overflow
        inPacket = false;
      }
    }
  }
}

void parsePacket(char* packet) {
  packet[packetIndex] = '\0'; // Thêm kết thúc chuỗi
  
  // Tách phần checksum (ví dụ: "M:100,-200,150,-50*C8")
  char* checksumPos = strchr(packet, '*');
  if (!checksumPos) {
    Serial.println("ERROR: Missing checksum");
    return;
  }
  
  // Verify checksum
  uint8_t receivedChecksum = strtoul(checksumPos+1, NULL, 16);
  *checksumPos = '\0'; // Tách phần data
  
  if (calculateChecksum(packet) != receivedChecksum) {
    Serial.println("ERROR: Checksum mismatch");
    return;
  }
  
  // Tách command và data
  char* colonPos = strchr(packet, ':');
  if (!colonPos) {
    Serial.println("ERROR: Invalid format");
    return;
  }
  
  char command = packet[0];
  char* data = colonPos + 1;
  
  // Xử lý lệnh
  switch (command) {
    case 'M': 
      break;
    case 'L': 
      break;
    // Thêm các lệnh khác...
    default:
      Serial.print("ERROR: Unknown command ");
      Serial.println(command);
  }
}

uint8_t calculateChecksum(const char* data) {
  uint8_t checksum = 0;
  while (*data) {
    checksum ^= *data++;
  }
  return checksum;
}

