#include <PS2X_lib.h>
#include <math.h>
// ================================================================
// FUNCTION PROTOTYPES
// ================================================================

// Setup functions
void setupMotorPins();
void setupPS2Controller();

// Main loop helpers
void updatePS2Controller();
void processControlInput();
void processAnalogSticks();
void processDigitalButtons();
void updateMotorOutput();
void calculateMecanumSpeeds(int16_t* speeds);
void setMotorSpeed(byte motorIndex, int16_t speed);
void handleSystemModes();
void handleModeSelection();
void checkSafety();

// Serial communication
void handleSerialComm();
void processSerialCommand(String &cmd);
void sendAck(char cmd);
void sendCommand(char cmd, int data);

// Utility functions
int16_t applyDeadzone(int16_t value);
int16_t applyCurve(int16_t value);
int16_t applyRamping(int16_t current, int16_t target);
void emergencyStop();

// Placeholder functions for additional controls
void controlTowMechanism();
void controlClamp();
void controlStepper();
void lineFollowingMode();

// ================================================================
// HARDWARE CONFIGURATION
// ================================================================

// Motor configuration (IN1, IN2, EN)
#define MOTOR_COUNT 4
const struct {
  byte in1, in2, en;
} MOTORS[MOTOR_COUNT] = {
  {4, 5, 3},    // Back Right (BR) - Timer2
  {7, 8, 9},    // Back Left (BL) - Timer1
  {2, 12, 10},  // Front Right (FR) - Timer1
  {13, A0, 11}  // Front Left (FL) - Timer2
};

// PS2 controller pins: DAT, CMD, SEL, CLK
const byte PS2_PINS[] = {A2, A3, A4, A5};

// ================================================================
// SYSTEM PARAMETERS
// ================================================================

// Motor control parameters
const int16_t MAX_SPEED = 255;         // Maximum motor speed
const int16_t RAMP_STEP = 15;          // Speed change per cycle (acceleration)
const int16_t DEADZONE = 12;           // Joystick deadzone to ignore noise
const float SPEED_CURVE_GAIN = 1.5;    // Exponential response curve
#define BTN_SPEED 200                  // Digital control button speed

// Serial communication parameters
const char START_BYTE = '<';
const char END_BYTE = '>';
const uint16_t SERIAL_TIMEOUT = 500;   // Milliseconds before emergency stop

// ================================================================
// GLOBAL VARIABLES
// ================================================================

// PS2 Controller instance
PS2X ps2Controller;

// Control modes: Analog (joystick) or Digital (buttons)
enum ControlMode { MODE_ANALOG, MODE_DIGITAL };
ControlMode currentControlMode = MODE_ANALOG;

// System status flags
struct {
  bool lineFollowing : 1;  // Line following mode active
  bool towActive : 1;      // Tow mechanism active
  bool clampClosed : 1;    // Clamp mechanism active
  bool stepperMoving : 1;  // Stepper motor active
} systemStatus;

// Speed variables
int16_t targetSpeed[3] = {0};   // Desired [X, Y, R] movement
int16_t currentSpeed[3] = {0};  // Current [X, Y, R] movement after ramping

// Timing variables
uint32_t lastPS2Check = 0;
uint32_t lastControlUpdate = 0;
uint32_t lastValidPacket = 0;

// ================================================================
// INITIALIZATION & SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial port
  
  setupMotorPins();
  setupPS2Controller();

  // Set PWM frequency to ~31 kHz for smoother motor control
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
  
  lastValidPacket = millis();  // Prevent immediate emergency stop
}

void setupMotorPins() {
  for (byte i = 0; i < MOTOR_COUNT; i++) {
    pinMode(MOTORS[i].in1, OUTPUT);
    pinMode(MOTORS[i].in2, OUTPUT);
    pinMode(MOTORS[i].en, OUTPUT);
    digitalWrite(MOTORS[i].en, LOW);  // Motors off initially
  }
}

void setupPS2Controller() {
  byte retryCount = 0;
  int error = 1;
  
  while (error && retryCount < 5) {
// Add a controller type parameter (try 0, 1, or 2)
  error = ps2Controller.config_gamepad(
    PS2_PINS[3], PS2_PINS[1], 
    PS2_PINS[2], PS2_PINS[0],
    false, false
  );
    if (error) {
      delay(500);
      retryCount++;
    }
  }
}

// ================================================================
// MAIN CONTROL LOOP
// ================================================================

void loop() {
  const uint16_t CONTROL_INTERVAL = 10; // 10ms = 100Hz update rate
  uint32_t currentMillis = millis();

  if (currentMillis - lastControlUpdate >= CONTROL_INTERVAL) {
    lastControlUpdate = currentMillis;
    updatePS2Controller();

    if (systemStatus.lineFollowing) {
      // Line Following Mode: only process START button and square to exit
      if (ps2Controller.ButtonPressed(PSB_SQUARE)) {
        systemStatus.lineFollowing = false;
        sendCommand('L', 0);
      }
      lineFollowingMode();
      updateMotorOutput();
    } else {
      // Manual Control Mode: process inputs and handle additional modes
      processControlInput();
      updateMotorOutput();
      handleSystemModes();
    }

    checkSafety();
    handleSerialComm();  // Process Serial data once per cycle
  }
}

// ================================================================
// PS2 CONTROLLER PROCESSING
// ================================================================

void updatePS2Controller() {
  ps2Controller.read_gamepad();
}

void processControlInput() {
  if (systemStatus.lineFollowing) return;

  // Switch between Analog/Digital control modes
  if (ps2Controller.ButtonPressed(PSB_SELECT)) {
    currentControlMode = (currentControlMode == MODE_ANALOG) ? MODE_DIGITAL : MODE_ANALOG;
    memset(targetSpeed, 0, sizeof(targetSpeed));
  }

  // Process input based on current control mode
  if (currentControlMode == MODE_ANALOG) {
    processAnalogSticks();
  } else {
    processDigitalButtons();
  }
}

void processAnalogSticks() {
  int16_t rawX = map(ps2Controller.Analog(PSS_RX), 0, 255, -MAX_SPEED, MAX_SPEED);
  int16_t rawY = map(ps2Controller.Analog(PSS_RY), 0, 255, MAX_SPEED, -MAX_SPEED);
  int16_t rawR = map(ps2Controller.Analog(PSS_LX), 0, 255, -MAX_SPEED, MAX_SPEED);

  targetSpeed[0] = applyCurve(applyDeadzone(rawX));
  targetSpeed[1] = applyCurve(applyDeadzone(rawY));
  targetSpeed[2] = applyCurve(applyDeadzone(rawR));
}

void processDigitalButtons() {
  targetSpeed[0] = (ps2Controller.Button(PSB_PAD_RIGHT) ? BTN_SPEED : 0) -
                   (ps2Controller.Button(PSB_PAD_LEFT)  ? BTN_SPEED : 0);
  targetSpeed[1] = (ps2Controller.Button(PSB_PAD_UP)    ? BTN_SPEED : 0) -
                   (ps2Controller.Button(PSB_PAD_DOWN)  ? BTN_SPEED : 0);
  targetSpeed[2] = (ps2Controller.Button(PSB_R1)        ? BTN_SPEED : 0) -
                   (ps2Controller.Button(PSB_L1)        ? BTN_SPEED : 0);
}

// ================================================================
// MOTOR CONTROL
// ================================================================

void updateMotorOutput() {
  // Apply acceleration/deceleration ramping to all axes
  for (byte i = 0; i < 3; i++) {
    currentSpeed[i] = applyRamping(currentSpeed[i], targetSpeed[i]);
  }

  // Calculate and apply speeds to all 4 wheels
  int16_t wheelSpeeds[MOTOR_COUNT];
  calculateMecanumSpeeds(wheelSpeeds);
  
  for (byte i = 0; i < MOTOR_COUNT; i++) {
    setMotorSpeed(i, wheelSpeeds[i]);
  }
}

void calculateMecanumSpeeds(int16_t* speeds) {
  // Mecanum wheel mixing algorithm
  speeds[0] = currentSpeed[1] - currentSpeed[0] - currentSpeed[2]; // BR
  speeds[1] = currentSpeed[1] + currentSpeed[0] + currentSpeed[2]; // BL
  speeds[2] = currentSpeed[1] + currentSpeed[0] - currentSpeed[2]; // FR
  speeds[3] = currentSpeed[1] - currentSpeed[0] + currentSpeed[2]; // FL

  // Normalize speeds if any exceeds maximum
  int16_t maxSpeed = 0;
  for (byte i = 0; i < MOTOR_COUNT; i++) {
    maxSpeed = max(maxSpeed, abs(speeds[i]));
  }
  
  if (maxSpeed > MAX_SPEED) {
    float ratio = (float)MAX_SPEED / maxSpeed;
    for (byte i = 0; i < MOTOR_COUNT; i++) {
      speeds[i] = round(speeds[i] * ratio);
    }
  }
}

void setMotorSpeed(byte motorIndex, int16_t speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  bool forward = (speed > 0);
  
  digitalWrite(MOTORS[motorIndex].in1, forward);
  digitalWrite(MOTORS[motorIndex].in2, !forward);
  analogWrite(MOTORS[motorIndex].en, abs(speed));
}

// ================================================================
// SYSTEM MODES & MECHANISMS CONTROL
// ================================================================

void handleSystemModes() {
  handleModeSelection();

  // Process active mechanisms 
  if (systemStatus.towActive)     controlTowMechanism();
  if (systemStatus.clampClosed)   controlClamp();
  if (systemStatus.stepperMoving) controlStepper();
}

void handleModeSelection() {
  static uint32_t lastModeChange = 0;
  if (millis() - lastModeChange < 250) return;  // Debounce

  // PS2 geometric buttons control different modes
  
  // SQUARE (□): Line Following Mode
  if (ps2Controller.ButtonPressed(PSB_SQUARE)) {
    systemStatus.lineFollowing = true;
    lastModeChange = millis();
  }
  
  // TRIANGLE (△): Tow Mode (exclusive)
  if (ps2Controller.ButtonPressed(PSB_TRIANGLE)) {
    if (!systemStatus.towActive) {
      systemStatus.clampClosed = false;
      systemStatus.stepperMoving = false;
    }
    systemStatus.towActive = !systemStatus.towActive;
    lastModeChange = millis();
  }
  
  // CIRCLE (○): Clamp Mode (disables tow)
  if (ps2Controller.ButtonPressed(PSB_CIRCLE)) {
    if (!systemStatus.clampClosed) {
      systemStatus.towActive = false;
    }
    systemStatus.clampClosed = !systemStatus.clampClosed;
    lastModeChange = millis();
  }
  
  // CROSS (×): Stepper Mode (disables tow)
  if (ps2Controller.ButtonPressed(PSB_CROSS)) {
    if (!systemStatus.stepperMoving) {
      systemStatus.towActive = false;
    }
    systemStatus.stepperMoving = !systemStatus.stepperMoving;
    lastModeChange = millis();
  }
}

void lineFollowingMode() {
  if (ps2Controller.ButtonPressed(PSB_START)) {
    delay(5000);
    sendCommand('L', 1);
    lastValidPacket = millis();  // Prevent timeout
  }
}

void controlTowMechanism() {
  if (ps2Controller.ButtonPressed(PSB_L2)) sendCommand('T', 1);
  if (ps2Controller.ButtonPressed(PSB_R2)) sendCommand('T', 0);
}

void controlClamp() {
  if (ps2Controller.ButtonPressed(PSB_L2)) sendCommand('C', 1);
  if (ps2Controller.ButtonPressed(PSB_R2)) sendCommand('C', 0);
}

void controlStepper() {
  if (ps2Controller.ButtonPressed(PSB_L3)) sendCommand('S', 1);
  if (ps2Controller.ButtonPressed(PSB_R3)) sendCommand('S', 0);
}

// ================================================================
// SERIAL COMMUNICATION
// ================================================================

void handleSerialComm() {
  static String inputBuffer = "";
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == START_BYTE) {
      inputBuffer = "";
    } else if (c == END_BYTE) {
      processSerialCommand(inputBuffer);
      inputBuffer = "";
    } else if (inputBuffer.length() < 64) {
      inputBuffer += c;
    }
  }
}

void processSerialCommand(String &cmd) { // Process X:Y:R commands from sensor data on slave Arduino
  int firstColon = cmd.indexOf(':');
  int secondColon = cmd.indexOf(':', firstColon + 1);

  // Process X:Y:R format commands from slave Arduino
  if (firstColon != -1 && secondColon != -1) {
    int x = cmd.substring(0, firstColon).toInt();
    int y = cmd.substring(firstColon + 1, secondColon).toInt();
    int r = cmd.substring(secondColon + 1).toInt();
    
    targetSpeed[0] = constrain(x, -MAX_SPEED, MAX_SPEED);
    targetSpeed[1] = constrain(y, -MAX_SPEED, MAX_SPEED);
    targetSpeed[2] = constrain(r, -MAX_SPEED, MAX_SPEED);
    
    lastValidPacket = millis();  // Reset timeout
  }
}

void sendCommand(char cmd, int data) {
  // Format: <CMD:DATA:CHECKSUM>
  char payload[20];
  snprintf(payload, sizeof(payload), "%c:%d", cmd, data);
  
  // Calculate XOR checksum
  byte checksum = 0;
  for (int i = 0; payload[i] != '\0'; i++) {
    checksum ^= payload[i];
  }
  
  char packet[30];
  snprintf(packet, sizeof(packet), "%c%s:%02X%c", START_BYTE, payload, checksum, END_BYTE);
  Serial.println(packet);
}

// ================================================================
// UTILITY FUNCTIONS
// ================================================================

int16_t applyDeadzone(int16_t value) {
  return (abs(value) < DEADZONE) ? 0 : value;
}

int16_t applyCurve(int16_t value) {
  if (value == 0) return 0;
  bool negative = (value < 0);
  float ratio = pow(abs(value) / (float)MAX_SPEED, SPEED_CURVE_GAIN);
  int16_t curved = round(ratio * MAX_SPEED);
  return negative ? -curved : curved;
}

int16_t applyRamping(int16_t current, int16_t target) {
  if (target > current) return min(current + RAMP_STEP, target);
  if (target < current) return max(current - RAMP_STEP, target);
  return target;
}

void emergencyStop() {
  memset(targetSpeed, 0, sizeof(targetSpeed));
  updateMotorOutput();
}

void checkSafety() {    
  if (millis() - lastValidPacket > SERIAL_TIMEOUT) {
    emergencyStop();  // Stop if no valid data received within timeout period
  }
}

/*
--------------------------------------------------------------------------------------------------------
| Param | Code Location      | Purpose                   | Default Value          | Adjustment Range   |
--------------------------------------------------------------------------------------------------------
| [P1]  | Motor Performance  | Maximum Speed             | MAX_SPEED = 255        | 150 - 255          |
| [P2]  | Motor Performance  | Ramping Step              | RAMP_STEP = 15         | 5 - 30             |
| [P3]  | Motor Performance  | Speed Curve Gain          | SPEED_CURVE_GAIN = 1.5 | Changed as needed  |
| [P4]  | PS2 Controls       | Deadzone                  | DEADZONE = 12          | 10 - 50            |
| [P5]  | PS2 Controls       | Button Speed              | BTN_SPEED = 200        | 100 - 255          |
| [P6]  | Hardware Config    | Motor Pins (MOTORS[])     | Defined above          | Change as needed   |
| [P7]  | Hardware Config    | PS2 Controller Pins       | PS2_PINS[]             | Change as needed   |
| [P8]  | Communication      | Serial Timeout            | SERIAL_TIMEOUT = 500   | 250 - 2000         |
--------------------------------------------------------------------------------------------------------
*/
