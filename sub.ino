// ================================================================
// SLAVE ARDUINO - LINE FOLLOWING & SENSOR CONTROL
// ================================================================

// Cảm biến dò line
const byte LINE_SENSORS[] = {A0, A1, A2, A3, A4};
const byte SENSOR_COUNT = 5;

// Hiệu chỉnh cảm biến
int sensorMin[SENSOR_COUNT] = {1023, 1023, 1023, 1023, 1023};
int sensorMax[SENSOR_COUNT] = {0, 0, 0, 0, 0};
const int CALIBRATION_TIME = 2000;  // Thời gian hiệu chỉnh
bool isCalibrated = false;

// Điều khiển PID
float Kp = 18.0;    // Hệ số P
float Ki =.08;    // Hệ số I 
float Kd = 22.0;    // Hệ số D
float integral = 0;
float lastError = 0;

// Tốc độ động cơ
const int BASE_SPEED = 170;
const int MAX_TURN_RATE = 150;
const int MIN_SPEED = 80;

// Ngưỡng phát hiện
const int LINE_THRESHOLD = 600;     // Ngưỡng nhận biết line
const int CROSS_DETECTION = 3;      // Số cảm biến cần kích hoạt để nhận biết ngã tư
const unsigned long LINE_TIMEOUT = 500; // Thời gian mất line trước khi dừng

// Biến trạng thái
int xSpeed = 0;
int ySpeed = 0;
int rSpeed = 0;
byte currentMode = 0; // 0:Idle, 1:Line, 2:Clamp, 3:Pull, 4:Stepper
bool lineLost = false;
unsigned long lastLineSeen = 0;

// Giao tiếp Serial
String inputBuffer = "";
const int SEND_INTERVAL = 50;
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);
  calibrateSensors();
}

void loop() {
  handleSerialComm();
  
  switch(currentMode) {
    case 1: // Chế độ dò line
      processLineFollowing();
      break;
      
    case 2: // Chế độ kẹp
      controlClamp();
      break;
      
    case 3: // Chế độ kéo
      controlPull();
      break;
      
    case 4: // Chế độ stepper
      controlStepper();
      break;
      
    default: // Chế độ không hoạt động
      xSpeed = ySpeed = rSpeed = 0;
      break;
  }
  
  sendControlData();
}

void processLineFollowing() {
  int sensorValues[SENSOR_COUNT];
  bool lineDetected[SENSOR_COUNT];
  bool anyLineDetected = false;
  int activeSensors = 0;

  // Đọc và chuẩn hóa giá trị cảm biến
  for(int i=0; i<SENSOR_COUNT; i++) {
    int rawValue = analogRead(LINE_SENSORS[i]);
    sensorValues[i] = map(rawValue, sensorMin[i], sensorMax[i], 1000, 0);
    lineDetected[i] = (sensorValues[i] > LINE_THRESHOLD);
    
    if(lineDetected[i]) {
      anyLineDetected = true;
      activeSensors++;
    }
  }

  // Xử lý ngã tư
  if(activeSensors >= CROSS_DETECTION) {
    handleCrossIntersection();
    return;
  }

  // Phát hiện line
  if(anyLineDetected) {
    lineLost = false;
    lastLineSeen = millis();
    float linePos = calculateLinePosition(sensorValues, lineDetected);
    calculatePID(linePos);
  } 
  else {
    handleLineLoss();
  }
}

float calculateLinePosition(int values[], bool detected[]) {
  float weightedSum = 0;
  float sumValues = 0;
  const float weights[] = {-2.0, -1.0, 0.0, 1.0, 2.0};

  for(int i=0; i<SENSOR_COUNT; i++) {
    if(detected[i]) {
      weightedSum += weights[i] * values[i];
      sumValues += values[i];
    }
  }

  return (sumValues == 0) ? lastError : weightedSum / sumValues;
}

void calculatePID(float position) {
  float error = position;
  integral = constrain(integral + error, -100, 100);
  float derivative = error - lastError;
  
  float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  rSpeed = constrain(-pidOutput, -MAX_TURN_RATE, MAX_TURN_RATE);
  ySpeed = BASE_SPEED - abs(error) * 15;
  ySpeed = constrain(ySpeed, MIN_SPEED, BASE_SPEED);
}

void handleLineLoss() {
  if(!lineLost) {
    lineLost = true;
    lastLineSeen = millis();
  }

  if(millis() - lastLineSeen > LINE_TIMEOUT) {
    currentMode = 0;
    ySpeed = rSpeed = 0;
  }
}

void handleCrossIntersection() {
  // Đi thẳng khi gặp ngã tư
  rSpeed = 0;
  ySpeed = BASE_SPEED;
}

void controlClamp() {
  // Triển khai logic điều khiển kẹp
}

void controlPull() {
  // Triển khai logic điều khiển cơ cấu kéo
}

void controlStepper() {
  // Triển khai logic điều khiển stepper
}

void handleSerialComm() {
  while(Serial.available() > 0) {
    char c = Serial.read();
    
    if(c == '<') {
      inputBuffer = "";
    }
    else if(c == '>') {
      processCommand(inputBuffer);
      inputBuffer = "";
    }
    else {
      inputBuffer += c;
    }
  }
}

void processCommand(String &cmd) {
  int colonIndex = cmd.indexOf(':');
  if(colonIndex == -1) return;

  String cmdType = cmd.substring(0, colonIndex);
  int value = cmd.substring(colonIndex+1).toInt();

  if(cmdType == "L") setMode(value ? 1 : 0);
  else if(cmdType == "C") setMode(value ? 2 : 0);
  else if(cmdType == "T") setMode(value ? 3 : 0);
  else if(cmdType == "S") setMode(value ? 4 : 0);
}

void setMode(byte newMode) {
  currentMode = newMode;
  if(newMode != 1) integral = lastError = 0;
}

void calibrateSensors() {
  unsigned long startTime = millis();
  while(millis() - startTime < CALIBRATION_TIME) {
    for(int i=0; i<SENSOR_COUNT; i++) {
      int val = analogRead(LINE_SENSORS[i]);
      if(val < sensorMin[i]) sensorMin[i] = val;
      if(val > sensorMax[i]) sensorMax[i] = val;
    }
  }
  isCalibrated = true;
}

void sendControlData() {
  if(millis() - lastSendTime < SEND_INTERVAL) return;
  
  String data = String(xSpeed) + ":" + String(ySpeed) + ":" + String(rSpeed);
  Serial.println("<" + data + ">");
  lastSendTime = millis();
}
