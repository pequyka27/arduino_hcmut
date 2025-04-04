const byte LINE_SENSORS[] = {A0, A1, A2, A3, A4};
const byte SENSOR_COUNT = 5;

int sensorMin[SENSOR_COUNT] = {1023, 1023, 1023, 1023, 1023};
int sensorMax[SENSOR_COUNT] = {0, 0, 0, 0, 0};
const int THRESHOLD = 500;

bool isCalibrated = false;
const unsigned long CALIBRATION_TIME = 2000;

float Kp = 30;
float Ki = 0;
float Kd = 15;

const int BASE_SPEED = 150;
const int MIN_SPEED = 80;
const int MAX_TURN_RATE = 100;

int xSpeed = 0;
int ySpeed = 0;
int rSpeed = 0;

float lastError = 0;
float integral = 0;

// Các biến quản lý mode hoạt động
byte currentMode = 0;  // 0: Không hoạt động, 1: Line (L), 2: Clamp (C), 3: Pull (T), 4: Stepper (S)
bool lineLost = false;
unsigned long lastLineSeen = 0;
unsigned long lastSendTime = 0;
const int SEND_INTERVAL = 50;

String inputBuffer = "";

const unsigned long LINE_TIMEOUT = 500;
int lastDirection = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  readCommandsFromMain();
  
  // Xử lý các chế độ khác nhau
  switch (currentMode) {
    case 0: // Chế độ không hoạt động
      xSpeed = 0;
      ySpeed = 0;
      rSpeed = 0;
      break;
      
    case 1: // Chế độ dò line (L)
      if (!isCalibrated) {
        calibrateSensors();
      } else {
        processLineSensors();
      }
      break;
      
    case 2: // Chế độ kẹp (C - Clamp)
      // Xử lý chế độ kẹp ở đây
      break;
      
    case 3: // Chế độ kéo (T - Pull) 
      // Xử lý chế độ kéo ở đây
      break;
    
    case 4: // Chế độ stepper (S)
      // Xử lý chế độ stepper ở đây
      break;
  }
  
  // Gửi giá trị điều khiển về Arduino chính định kỳ
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    sendControlValues();
    lastSendTime = millis();
  }
}

void calibrateSensors() {
  static unsigned long startCalibration = 0;
  static bool calibrationStarted = false;
  
  if (!calibrationStarted) {
    startCalibration = millis();
    calibrationStarted = true;
  }
  
  if (millis() - startCalibration < CALIBRATION_TIME) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      int sensorValue = analogRead(LINE_SENSORS[i]);
      
      if (sensorValue < sensorMin[i]) {
        sensorMin[i] = sensorValue;
      }
      if (sensorValue > sensorMax[i]) {
        sensorMax[i] = sensorValue;
      }
    }
  } 
  else {
    isCalibrated = true;
    calibrationStarted = false;
  }
}

void readCommandsFromMain() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '<') {
      inputBuffer = "";
    } 
    else if (c == '>') {
      processCommand(inputBuffer);
      inputBuffer = "";
    }
    else if (inputBuffer.length() < 32) {
      inputBuffer += c;
    }
  }
}

void processCommand(String &command) {
  int colonIndex = command.indexOf(':');
  
  if (colonIndex != -1) {
    String cmd = command.substring(0, colonIndex);
    int value = command.substring(colonIndex + 1).toInt();
    
    // Xử lý các lệnh mode khác nhau
    if (cmd.equals("L")) {  // Line following mode
      if (value == 1) {
        setMode(1); // Chuyển sang chế độ dò line
      } else {
        setMode(0); // Tắt hoạt động
      }
    }
    else if (cmd.equals("C")) {  // Clamp mode
      if (value == 1) {
        setMode(2); // Chuyển sang chế độ kẹp
      } else {
        setMode(0);
      }
    }
    else if (cmd.equals("T")) {  // Pull mode
      if (value == 1) {
        setMode(3); // Chuyển sang chế độ kéo
      } else {
        setMode(0);
      }
    }
    else if (cmd.equals("S")) {  // Stepper mode
      if (value == 1) {
        setMode(4); // Chuyển sang chế độ stepper
      } else {
        setMode(0);
      }
    }
  }
}

void setMode(byte newMode) {
  // Làm sạch trạng thái của chế độ cũ
  switch (currentMode) {
    case 1: // Thoát chế độ dò line
      break;
    case 2: // Thoát chế độ kẹp
      break;
    case 3: // Thoát chế độ kéo
      break;
    case 4: // Thoát chế độ stepper
      break;
  }
  
  // Thiết lập chế độ mới
  currentMode = newMode;
  
  // Khởi tạo cho chế độ mới
  switch (currentMode) {
    case 0: // Chế độ không hoạt động
      xSpeed = 0;
      ySpeed = 0;
      rSpeed = 0;
      break;
      
    case 1: // Chế độ dò line (L)
      lineLost = false;
      lastError = 0;
      integral = 0;
      isCalibrated = false;
      break;
      
    case 2: // Chế độ kẹp (C)
      // Khởi tạo các biến cho chế độ kẹp
      xSpeed = 0;
      ySpeed = 0;
      rSpeed = 0;
      break;
      
    case 3: // Chế độ kéo (T)
      // Khởi tạo các biến cho chế độ kéo
      xSpeed = 0;
      ySpeed = 0;
      rSpeed = 0;
      break;
      
    case 4: // Chế độ stepper (S)
      // Khởi tạo các biến cho chế độ stepper
      xSpeed = 0;
      ySpeed = 0;
      rSpeed = 0;
      break;
  }
  
  // Phản hồi xác nhận chế độ mới (phản hồi theo chữ cái của chế độ)
  String modeChar;
  switch (currentMode) {
    case 0: modeChar = "N"; break; // None
    case 1: modeChar = "L"; break; // Line
    case 2: modeChar = "C"; break; // Clamp
    case 3: modeChar = "T"; break; // Pull (T)
    case 4: modeChar = "S"; break; // Stepper
  }
  
  String ackMsg = "<A:" + modeChar + ":" + String(currentMode == 0 ? 0 : 1) + ">";
  Serial.println(ackMsg);
}

void processLineSensors() {
  int normalizedValues[SENSOR_COUNT];
  bool lineDetected[SENSOR_COUNT];
  bool anyLineDetected = false;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    int rawValue = analogRead(LINE_SENSORS[i]);
    
    normalizedValues[i] = map(constrain(rawValue, sensorMin[i], sensorMax[i]), 
                             sensorMin[i], sensorMax[i], 1000, 0);
    
    lineDetected[i] = (normalizedValues[i] > THRESHOLD);
    
    if (lineDetected[i]) {
      anyLineDetected = true;
    }
  }
  
  if (anyLineDetected) {
    lastLineSeen = millis();
    lineLost = false;
    
    float position = calculateLinePosition(normalizedValues, lineDetected);
    
    followLine(position);
  } else {
    handleLineLoss();
  }
}

float calculateLinePosition(int values[], bool lineDetected[]) {
  float weightedSum = 0;
  int sumValues = 0;
  
  const float weights[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (lineDetected[i]) {
      weightedSum += weights[i] * values[i];
      sumValues += values[i];
    }
  }
  
  if (sumValues == 0) {
    return lastError;
  }
  
  return weightedSum / sumValues;
}

void followLine(float position) {
  float error = position;
  
  integral += error;
  integral = constrain(integral, -50, 50);
  
  float derivative = error - lastError;
  lastError = error;
  
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  
  rSpeed = -int(pidOutput);
  rSpeed = constrain(rSpeed, -MAX_TURN_RATE, MAX_TURN_RATE);
  
  float reduction = abs(error) / 2.0;
  ySpeed = BASE_SPEED - int(BASE_SPEED * reduction * 0.5);
  ySpeed = constrain(ySpeed, MIN_SPEED, BASE_SPEED);
  
  xSpeed = 0;
}

void handleLineLoss() {
  if (!lineLost) {
    lineLost = true;
    
    if (lastError > 0) {
      lastDirection = 1;
    } else {
      lastDirection = -1;
    }
  }
  
  unsigned long lostTime = millis() - lastLineSeen;
  
  if (lostTime < LINE_TIMEOUT) {
    xSpeed = 0;
    ySpeed = BASE_SPEED / 2;
    rSpeed = lastDirection * MAX_TURN_RATE;
  } else {
    xSpeed = 0;
    ySpeed = 0;
    rSpeed = 0;
  }
}

void sendControlValues() {
  String controlData = String(xSpeed) + ":" + String(ySpeed) + ":" + String(rSpeed);
  String message = "<" + controlData + ">";
  Serial.println(message);
}
