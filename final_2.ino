#include <PS2X_lib.h>

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

// Utility functions
int16_t applyDeadzone(int16_t value);
int16_t applyCurve(int16_t value);
int16_t applyRamping(int16_t current, int16_t target);
void emergencyStop();
void lineFollowingMode();

// ================================================================
// HARDWARE CONFIGURATION
// ================================================================

// Motor configuration (IN1, IN2, EN)
#define MOTOR_COUNT 4
const struct {
  byte in1, in2, en;
} MOTORS[MOTOR_COUNT] = {
  {4, 5, 3},    // Back Right (BR)
  {7, 8, 9},    // Back Left (BL)
  {2, 12, 10},  // Front Right (FR)
  {13, A0, 11}  // Front Left (FL)
};

// PS2 controller pins: DAT, CMD, SEL, CLK
const byte PS2_PINS[] = {A2, A3, A4, A5};

// ================================================================
// SYSTEM PARAMETERS
// ================================================================

// Motor control parameters
const int16_t MAX_SPEED = 255;
const int16_t RAMP_STEP = 15;
const int16_t DEADZONE = 12;
const float SPEED_CURVE_GAIN = 1.5;
#define BTN_SPEED 200
#define LINE_RUN_TIME 5000  // Thời gian chạy thẳng 5 giây

// ================================================================
// GLOBAL VARIABLES
// ================================================================

PS2X ps2Controller;

enum ControlMode { MODE_ANALOG, MODE_DIGITAL };
ControlMode currentControlMode = MODE_ANALOG;

struct {
  bool lineFollowing : 1;
  bool clampClosed : 1;
  uint32_t lineStartTime = 0;
} systemStatus;

int16_t targetSpeed[3] = {0};
int16_t currentSpeed[3] = {0};

uint32_t lastControlUpdate = 0;

// ================================================================
// INITIALIZATION & SETUP
// ================================================================

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupPS2Controller();

  // Cấu hình PWM
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
}

void setupMotorPins() {
  for (byte i = 0; i < MOTOR_COUNT; i++) {
    pinMode(MOTORS[i].in1, OUTPUT);
    pinMode(MOTORS[i].in2, OUTPUT);
    pinMode(MOTORS[i].en, OUTPUT);
    digitalWrite(MOTORS[i].en, LOW);
  }
}

void setupPS2Controller() {
  byte retryCount = 0;
  int error = 1;
  
  while (error && retryCount < 5) {
    error = ps2Controller.config_gamepad(PS2_PINS[3], PS2_PINS[1], PS2_PINS[2], PS2_PINS[0], false, false);
    if (error) delay(500);
    retryCount++;
  }
}

// ================================================================
// MAIN CONTROL LOOP
// ================================================================

void loop() {
  const uint16_t CONTROL_INTERVAL = 10;
  uint32_t currentMillis = millis();

  if (currentMillis - lastControlUpdate >= CONTROL_INTERVAL) {
    lastControlUpdate = currentMillis;
    updatePS2Controller();

    if (systemStatus.lineFollowing) {
      lineFollowingMode();
      updateMotorOutput();
    } else {
      processControlInput();
      updateMotorOutput();
      handleSystemModes();
    }
  }
}

// ================================================================
// PS2 CONTROLLER PROCESSING
// ================================================================

void updatePS2Controller() {
  ps2Controller.read_gamepad();
}

void processControlInput() {
  if (ps2Controller.ButtonPressed(PSB_SELECT)) {
    currentControlMode = (currentControlMode == MODE_ANALOG) ? MODE_DIGITAL : MODE_ANALOG;
    memset(targetSpeed, 0, sizeof(targetSpeed));
  }

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
  for (byte i = 0; i < 3; i++) {
    currentSpeed[i] = applyRamping(currentSpeed[i], targetSpeed[i]);
  }

  int16_t wheelSpeeds[MOTOR_COUNT];
  calculateMecanumSpeeds(wheelSpeeds);
  
  for (byte i = 0; i < MOTOR_COUNT; i++) {
    setMotorSpeed(i, wheelSpeeds[i]);
  }
}

void calculateMecanumSpeeds(int16_t* speeds) {
  speeds[0] = currentSpeed[1] - currentSpeed[0] - currentSpeed[2];
  speeds[1] = currentSpeed[1] + currentSpeed[0] + currentSpeed[2];
  speeds[2] = currentSpeed[1] + currentSpeed[0] - currentSpeed[2];
  speeds[3] = currentSpeed[1] - currentSpeed[0] + currentSpeed[2];

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
  digitalWrite(MOTORS[motorIndex].in1, speed > 0);
  digitalWrite(MOTORS[motorIndex].in2, speed <= 0);
  analogWrite(MOTORS[motorIndex].en, abs(speed));
}

// ================================================================
// SYSTEM MODES
// ================================================================

void handleSystemModes() {
  handleModeSelection();
  if (systemStatus.clampClosed) controlClamp();
}

void handleModeSelection() {
  static uint32_t lastModeChange = 0;
  if (millis() - lastModeChange < 250) return;

  // Kích hoạt chế độ line following
  if (ps2Controller.ButtonPressed(PSB_SQUARE)) {
    systemStatus.lineFollowing = true;
    systemStatus.lineStartTime = millis();
    targetSpeed[1] = BTN_SPEED; // Chạy thẳng về phía trước
    lastModeChange = millis();
  }

  // Điều khiển kẹp
  if (ps2Controller.ButtonPressed(PSB_CIRCLE)) {
    systemStatus.clampClosed = !systemStatus.clampClosed;
    lastModeChange = millis();
  }
}

void lineFollowingMode() {
    const float LINE_SPEED_FACTOR = 0.6;
    int16_t targetLineSpeed = BTN_SPEED * LINE_SPEED_FACTOR;
    
    // Áp dụng ramping cho trục Y (tiến/lùi)
    targetSpeed[1] = applyRamping(targetSpeed[1], targetLineSpeed);
    
    // Giữ nguyên logic dừng
    if (millis() - systemStatus.lineStartTime >= LINE_RUN_TIME) {
        systemStatus.lineFollowing = false;
        memset(targetSpeed, 0, sizeof(targetSpeed));
    }
}

// ================================================================
// CLAMP CONTROL
// ================================================================

void controlClamp() {
  // Thêm code điều khiển servo kẹp ở đây
  // Ví dụ:
  // if (systemStatus.clampClosed) closeClamp();
  // else openClamp();
}

// ================================================================
// UTILITY FUNCTIONS
// ================================================================

int16_t applyDeadzone(int16_t value) {
  return (abs(value) < DEADZONE) ? 0 : value;
}

int16_t applyCurve(int16_t value) {
  if (value == 0) return 0;
  bool negative = value < 0;
  float ratio = pow(abs(value) / (float)MAX_SPEED, SPEED_CURVE_GAIN);
  return negative ? -round(ratio * MAX_SPEED) : round(ratio * MAX_SPEED);
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
