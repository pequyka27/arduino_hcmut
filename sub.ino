// SLAVE CODE - Phiên bản tối ưu từ hàm của bạn
void handleSerialComm() {
  static String inputBuffer = "";
  static bool packetInProgress = false;

  while (Serial.available()) {
    char c = Serial.read();

    // Bắt đầu packet
    if (c == START_BYTE) {
      inputBuffer = "";
      packetInProgress = true;
      continue;
    }

    // Kết thúc packet
    if (c == END_BYTE && packetInProgress) {
      processSerialCommand(inputBuffer);
      inputBuffer = "";
      packetInProgress = false;
      return;
    }

    // Thu thập dữ liệu
    if (packetInProgress && (inputBuffer.length() < 64)) {
      inputBuffer += c;
    }
  }
}

void processSerialCommand(String &cmd) {
  // Thêm phần xử lý checksum và validate
  int lastColon = cmd.lastIndexOf(':');
  if (lastColon == -1) {
    Serial.println("<ERR:NO_CHECKSUM>");
    return;
  }

  // Tách checksum (2 ký tự cuối)
  String receivedChecksum = cmd.substring(lastColon + 1);
  String payload = cmd.substring(0, lastColon);

  // Verify checksum
  if (calculateChecksum(payload) != strtol(receivedChecksum.c_str(), NULL, 16)) {
    Serial.print("<ERR:CHECKSUM:");
    Serial.print(calculateChecksum(payload), HEX);
    Serial.println(">");
    return;
  }

  // Xử lý lệnh (giữ nguyên logic từ master)
  int firstColon = payload.indexOf(':');
  if (firstColon == -1) return;

  String command = payload.substring(0, firstColon);
  String value = payload.substring(firstColon + 1);

  if (command == "M") {
    // Xử lý motor command
    int16_t speeds[4];
    parseMotorValues(value, speeds);
    setMotorSpeeds(speeds);
    Serial.println("<ACK:M>");
  }
  // ... thêm các lệnh khác
}

// Hàm tính checksum (thêm vào)
uint8_t calculateChecksum(const String &data) {
  uint8_t crc = 0;
  for (unsigned int i = 0; i < data.length(); i++) {
    crc ^= (uint8_t)data.charAt(i);
  }
  return crc;
}
