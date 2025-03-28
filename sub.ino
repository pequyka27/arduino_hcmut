#define START_BYTE '<'
#define END_BYTE '>'
#define MAX_PACKET_LEN 64

// Thêm biến để kiểm soát tần suất gửi dữ liệu
uint32_t lastSerialSend = 0;
const uint16_t SERIAL_SEND_INTERVAL = 15; // 15ms ≈ 66Hz update rate

// Thêm biến giới hạn tốc độ cho chế độ dò line
const float LINE_FOLLOW_SPEED_FACTOR = 0.6; // Giới hạn ở 60% tốc độ tối đa

char packetBuffer[MAX_PACKET_LEN];
uint8_t packetIndex = 0;
bool packetInProgress = false;
uint8_t currentChecksum = 0;

// Thêm biến để theo dõi chế độ dò line
bool lineFollowingMode = false;

void setup() {
  Serial.begin(115200);
  // Thêm khởi tạo cho cảm biến dò line và các thiết bị khác tại đây
}

void loop() {
  // Xử lý dữ liệu nhận từ master
  handleSerialComm();
  
  // Đọc cảm biến dò line và tính toán giá trị x, y, r
  // Giả sử các hàm readLineSensors() và calculateXYR() đã được định nghĩa ở nơi khác
  int x = 0, y = 0, r = 0;
  
  if (lineFollowingMode) {
    // Đọc cảm biến và tính toán giá trị điều khiển
    readLineSensors();
    calculateXYR(&x, &y, &r);
    
    // Gửi dữ liệu về master với tần suất được kiểm soát
    uint32_t currentMillis = millis();
    if (currentMillis - lastSerialSend >= SERIAL_SEND_INTERVAL) {
      lastSerialSend = currentMillis;
      
      // Áp dụng giới hạn tốc độ khi dò line
      x = round(x * LINE_FOLLOW_SPEED_FACTOR);
      y = round(y * LINE_FOLLOW_SPEED_FACTOR);
      r = round(r * LINE_FOLLOW_SPEED_FACTOR);
      
      // Gửi dữ liệu về master
      sendXYRData(x, y, r);
    }
  }
  
  // Thêm xử lý khác nếu cần
}

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
  char* command = strtok(data, ",:");
  if (!command) return;

  // Xử lý các lệnh từ master
  if (strcmp(command, "M") == 0) { // Motor command
    int16_t speeds[4];
    for (uint8_t i=0; i<4; i++) {
      char* val = strtok(NULL, ",");
      if (!val) return;
      speeds[i] = atoi(val);
    }
    setMotorSpeeds(speeds);
  } else if (strcmp(command, "L") == 0) { // Line following mode
    char* val = strtok(NULL, ",:");
    if (!val) return;
    lineFollowingMode = (atoi(val) == 1);
  }
  // ... các lệnh khác
}

void sendXYRData(int x, int y, int r) {
  // Tính checksum
  char payload[20];
  snprintf(payload, sizeof(payload), "%d:%d:%d", x, y, r);
  
  byte checksum = 0;
  for (int i = 0; payload[i] != '\0'; i++) {
    checksum ^= payload[i];
  }
  
  // Gửi dữ liệu
  char packet[30];
  snprintf(packet, sizeof(packet), "%c%s:%02X%c", START_BYTE, payload, checksum, END_BYTE);
  Serial.println(packet);
}

// Giả định các hàm này được triển khai ở nơi khác trong code
void readLineSensors() {
  // Đọc dữ liệu từ cảm biến dò line
}

void calculateXYR(int* x, int* y, int* r) {
  // Tính toán giá trị x, y, r từ dữ liệu cảm biến
  // Giả sử một thuật toán đơn giản:
  *x = 0;  // Giá trị mặc định, thay đổi theo thuật toán thực tế
  *y = 100; // Tiến về phía trước với tốc độ trung bình
  *r = 0;  // Không xoay
}

void setMotorSpeeds(int16_t speeds[4]) {
  // Đặt tốc độ cho động cơ (nếu có)
}
