#include <Arduino.h>

// ================================================================
// HARDWARE CONFIGURATION
// ================================================================

// Line Sensor Pins (IR sensors)
#define NUM_SENSORS 5
const uint8_t LINE_SENSOR_PINS[NUM_SENSORS] = {A0, A1, A2, A3, A4};

// Mechanism control pins
// Tow mechanism
const uint8_t TOW_PIN_1 = 8;
const uint8_t TOW_PIN_2 = 9;

// Clamp servo
const uint8_t CLAMP_PIN = 10;

// Stepper motor pins
const uint8_t STEPPER_DIR_PIN = 2;
const uint8_t STEPPER_STEP_PIN = 3;
const uint8_t STEPPER_ENABLE_PIN = 4;

// ================================================================
// SYSTEM PARAMETERS
// ================================================================

// Line following parameters
const uint16_t LINE_THRESHOLD = 500;    // Threshold for line detection (0-1023)
const int16_t MAX_SPEED = 255;          // Maximum output speed (matches master)
const int16_t BASE_SPEED = 150;         // Base forward speed for line following
const float KP = 2.0;                   // PID Proportional gain
const float KI = 0.0;                   // PID Integral gain
const float KD = 5.0;                   // PID Derivative gain

// Serial communication parameters
const char START_BYTE = '<';
const char END_BYTE = '>';
const uint32_t SERIAL_BAUD = 115200;
const uint16_t SERIAL_TIMEOUT = 500;    // Milliseconds
const uint16_t SENSOR_INTERVAL = 20;    // Line sensor reading interval (50Hz)

// Mechanism parameters
const uint16_t STEPPER_SPEED = 800;     // Steps per second
const uint16_t STEPPER_MAX_STEPS = 400; // Maximum steps in one direction

// ================================================================
// GLOBAL VARIABLES
// ================================================================

// System state
struct {
  bool lineFollowing : 1;    // Line following mode active
  bool towActive : 1;        // Tow mechanism active
  bool clampClosed : 1;      // Clamp mechanism active
  bool stepperMoving : 1;    // Stepper direction (1=forward, 0=reverse)
} systemStatus;

// Line following variables
uint16_t sensorValues[NUM_SENSORS];
int16_t lastError = 0;
int16_t integral = 0;

// Command processing
String inputBuffer = "";
bool commandComplete = false;

// Timing variables
uint32_t lastSensorRead = 0;
uint32_t lastCommandCheck = 0;
uint32_t lastStepperStep = 0;

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================

// Setup functions
void setupSensorPins();
void setupMechanismPins();
void setupSerial();

// Line following
void readLineSensors();
void calculateLinePosition();
void sendMovementCommand();
int16_t calculatePID();

// Command processing
void processSerialInput();
void executeCommand(String cmd);
void sendCommand(String cmd);

// Mechanism control
void controlTow(int state);
void controlClamp(int state);
void controlStepper(int direction);
void updateStepperPosition();

// Utility
void resetSystem();
uint8_t calculateChecksum(String data);

// ================================================================
// SETUP AND MAIN LOOP
// ================================================================

void setup() {
  setupSerial();
  setupSensorPins();
  setupMechanismPins();
  resetSystem();
}

void loop() {
  // Process any incoming commands
  processSerialInput();
  
  // Handle line following if active
  if (systemStatus.lineFollowing) {
    uint32_t currentMillis = millis();
    if (currentMillis - lastSensorRead >= SENSOR_INTERVAL) {
      lastSensorRead = currentMillis;
      readLineSensors();
      calculateLinePosition();
      sendMovementCommand();
    }
  }
  
  // Update stepper position if it's moving
  if (systemStatus.stepperMoving) {
    updateStepperPosition();
  }
}

// ================================================================
// SETUP FUNCTIONS
// ================================================================

void setupSerial() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial); // Wait for serial port to connect
}

void setupSensorPins() {
  // Set up line sensor pins
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(LINE_SENSOR_PINS[i], INPUT);
  }
}

void setupMechanismPins() {
  // Set up tow mechanism
  pinMode(TOW_PIN_1, OUTPUT);
  pinMode(TOW_PIN_2, OUTPUT);
  digitalWrite(TOW_PIN_1, LOW);
  digitalWrite(TOW_PIN_2, LOW);
  
  // Set up clamp
  pinMode(CLAMP_PIN, OUTPUT);
  digitalWrite(CLAMP_PIN, LOW);
  
  // Set up stepper
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH); // Disable stepper initially
}

// ================================================================
// LINE FOLLOWING FUNCTIONS
// ================================================================

void readLineSensors() {
  // Read raw values from all sensors
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(LINE_SENSOR_PINS[i]);
  }
}

void calculateLinePosition() {
  // Convert sensor readings to position
  int16_t weightedSum = 0;
  int16_t sum = 0;
  int16_t error = 0;
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    bool isOnLine = (sensorValues[i] > LINE_THRESHOLD);
    
    // Use weighted position - center is 0, left negative, right positive
    if (isOnLine) {
      int16_t weight = (i - (NUM_SENSORS - 1) / 2) * 1000;
      weightedSum += weight;
      sum++;
    }
  }
  
  // Calculate error (-2000 to +2000 for 5 sensors)
  if (sum > 0) {
    error = weightedSum / sum;
  } else {
    // All sensors off the line - use last error but amplified
    error = lastError * 2;
  }
  
  // Calculate PID control
  int16_t pidOutput = calculatePID(error);
  
  // Convert PID output to X, Y, R speeds
  // We primarily want Y (forward) and R (rotation)
  // Keep X (sideways) at 0 or slight correction
  
  int16_t speedX = 0;
  int16_t speedY = BASE_SPEED;
  int16_t speedR = -pidOutput;  // Negative because robot needs to turn opposite to error
  
  // If completely lost or sharp turn needed, modify speeds
  if (abs(error) > 1500) {
    // Slow down forward speed to make sharper turns
    speedY = BASE_SPEED / 2;
    
    // Add slight sideways movement to help recover
    speedX = (error > 0) ? 30 : -30;
  }
  
  // Constrain values to MAX_SPEED
  speedX = constrain(speedX, -MAX_SPEED, MAX_SPEED);
  speedY = constrain(speedY, 0, MAX_SPEED);  // Always forward
  speedR = constrain(speedR, -MAX_SPEED/2, MAX_SPEED/2);  // Limit rotation speed
  
  // Send these values to the master Arduino
  String command = String(speedX) + ":" + String(speedY) + ":" + String(speedR);
  sendCommand(command);
}

int16_t calculatePID(int16_t error) {
  // Calculate PID terms
  int16_t P = error;
  integral += error;
  integral = constrain(integral, -10000, 10000);  // Prevent windup
  int16_t I = integral;
  int16_t D = error - lastError;
  lastError = error;
  
  // Calculate PID output
  int16_t output = (KP * P) + (KI * I) + (KD * D);
  return output;
}

void sendMovementCommand() {
  // Format and send X:Y:R command to master
  String command = String(0) + ":" + String(BASE_SPEED) + ":" + String(0);
  
  // If we have valid PID output, use it instead
  if (lastError != 0) {
    // Calculate values based on PID
    int16_t speedR = -map(lastError, -2000, 2000, -MAX_SPEED/2, MAX_SPEED/2);
    command = String(0) + ":" + String(BASE_SPEED) + ":" + String(speedR);
  }
  
  sendCommand(command);
}

// ================================================================
// COMMAND PROCESSING
// ================================================================

void processSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == START_BYTE) {
      inputBuffer = "";
    } else if (c == END_BYTE) {
      executeCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void executeCommand(String cmd) {
  // Parse command format: CMD:DATA:CHECKSUM
  int firstColon = cmd.indexOf(':');
  int secondColon = cmd.indexOf(':', firstColon + 1);
  
  if (firstColon > 0 && secondColon > firstColon) {
    String command = cmd.substring(0, firstColon);
    String data = cmd.substring(firstColon + 1, secondColon);
    String checksumStr = cmd.substring(secondColon + 1);
    
    // Verify checksum
    String payloadToCheck = command + ":" + data;
    uint8_t expectedChecksum = calculateChecksum(payloadToCheck);
    uint8_t receivedChecksum = strtol(checksumStr.c_str(), NULL, 16);
    
    if (expectedChecksum == receivedChecksum) {
      // Valid command - process
      char cmdType = command.charAt(0);
      int value = data.toInt();
      
      switch (cmdType) {
        case 'L':  // Line following
          systemStatus.lineFollowing = (value == 1);
          if (!systemStatus.lineFollowing) {
            // Reset PID values when turning off
            integral = 0;
            lastError = 0;
          }
          break;
          
        case 'T':  // Tow mechanism
          controlTow(value);
          break;
          
        case 'C':  // Clamp
          controlClamp(value);
          break;
          
        case 'S':  // Stepper
          controlStepper(value);
          break;
      }
    }
  }
}

void sendCommand(String cmd) {
  // Calculate checksum
  uint8_t checksum = calculateChecksum(cmd);
  
  // Format and send command
  String packet = String(START_BYTE) + cmd + ":" + String(checksum, HEX) + String(END_BYTE);
  Serial.println(packet);
}

// ================================================================
// MECHANISM CONTROL
// ================================================================

void controlTow(int state) {
  systemStatus.towActive = (state == 1);
  
  if (systemStatus.towActive) {
    digitalWrite(TOW_PIN_1, HIGH);
    digitalWrite(TOW_PIN_2, LOW);
  } else {
    digitalWrite(TOW_PIN_1, LOW);
    digitalWrite(TOW_PIN_2, HIGH);
  }
}

void controlClamp(int state) {
  systemStatus.clampClosed = (state == 1);
  
  if (systemStatus.clampClosed) {
    // Servo or solenoid activation for clamp
    digitalWrite(CLAMP_PIN, HIGH);
  } else {
    digitalWrite(CLAMP_PIN, LOW);
  }
}

void controlStepper(int direction) {
  // Set direction (1=forward, 0=reverse)
  systemStatus.stepperMoving = true;
  digitalWrite(STEPPER_DIR_PIN, direction ? HIGH : LOW);
  digitalWrite(STEPPER_ENABLE_PIN, LOW);  // Enable stepper
}

void updateStepperPosition() {
  static uint16_t stepCount = 0;
  uint32_t currentMicros = micros();
  
  // Calculate step interval based on desired speed
  uint32_t stepInterval = 1000000 / STEPPER_SPEED;
  
  // Step the motor at the correct interval
  static uint32_t lastStepTime = 0;
  if (currentMicros - lastStepTime >= stepInterval) {
    lastStepTime = currentMicros;
    
    // Generate step pulse
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    
    stepCount++;
    
    // Stop after reaching max steps
    if (stepCount >= STEPPER_MAX_STEPS) {
      systemStatus.stepperMoving = false;
      digitalWrite(STEPPER_ENABLE_PIN, HIGH);  // Disable stepper
      stepCount = 0;
    }
  }
}

// ================================================================
// UTILITY FUNCTIONS
// ================================================================

void resetSystem() {
  // Initialize all status flags
  systemStatus.lineFollowing = false;
  systemStatus.towActive = false;
  systemStatus.clampClosed = false;
  systemStatus.stepperMoving = false;
  
  // Reset movement parameters
  lastError = 0;
  integral = 0;
  
  // Disable all mechanisms
  digitalWrite(TOW_PIN_1, LOW);
  digitalWrite(TOW_PIN_2, LOW);
  digitalWrite(CLAMP_PIN, LOW);
  digitalWrite(STEPPER_ENABLE_PIN, HIGH);
}

uint8_t calculateChecksum(String data) {
  // Calculate XOR checksum of the data
  uint8_t checksum = 0;
  for (size_t i = 0; i < data.length(); i++) {
    checksum ^= data.charAt(i);
  }
  return checksum;
}

/*
-----------------------------------------------------------------------------------------
| Param | Code Location      | Purpose                | Default Value    | Adjustment Range |
-----------------------------------------------------------------------------------------
| [P1]  | Line Following     | Sensor Threshold       | 500              | 300 - 700        |
| [P2]  | Line Following     | Base Speed             | 150              | 100 - 200        |
| [P3]  | Line Following     | PID Constants (KP,KI,KD) | 2.0, 0.0, 5.0 | As needed        |
| [P4]  | Mechanism Control  | Stepper Speed          | 800 steps/sec    | 400 - 1600       |
| [P5]  | Mechanism Control  | Stepper Max Steps      | 400              | As needed        |
| [P6]  | Hardware Config    | Sensor Pins            | A0-A4            | Change as needed |
| [P7]  | Hardware Config    | Mechanism Pins         | Defined above    | Change as needed |
| [P8]  | Communication      | Serial Baud Rate       | 115200           | Match master     |
-----------------------------------------------------------------------------------------
*/