#include <PS2X_lib.h>
#include <Servo.h>

/*
--------------------------------------------------------------------------------------------------------
| Param | Code Location       | Purpose                   | Default Value          | Adjustment Range   |
--------------------------------------------------------------------------------------------------------
| [P1]  | Motor Performance   | Maximum Speed             | MAX_SPEED = 255        | 150 - 255          |
| [P2]  | Motor Performance   | Ramping Step              | RAMP_STEP = 15         | 5 - 30             |
| [P3]  | Motor Performance   | Speed Curve Gain          | SPEED_CURVE_GAIN = 1.5 | 1.0 - 3.0          |
| [P4]  | PS2 Controls        | Deadzone                  | DEADZONE = 12          | 8 - 20             |
| [P5]  | PS2 Controls        | Button Speed              | BTN_SPEED = 200        | 100 - 255          |
| [P6]  | Line Following      | Run Duration              | LINE_RUN_TIME = 5000   | 1000 - 10000       |
| [P7]  | System Timing       | Control Update Interval   | CONTROL_INTERVAL = 10  | 5 - 20             |
| [P8]  | System Timing       | Button Debounce Time      | DEBOUNCE = 250         | 150 - 500          |
| [P9]  | Servo Configuration | Open Angle                | OPEN_ANGLE = 30        | 0 - 90             |
| [P10] | Servo Configuration | Close Angle               | CLOSE_ANGLE = 180      | 90 - 180           |
| [P11] | Hardware Config     | Servo Pin                 | SERVO_PIN = 13         | Change as needed   |
| [P12] | Hardware Config     | Motor Pins (MOTORS[])     | See motors array       | Change as needed   |
| [P13] | Hardware Config     | PS2 Controller Pins       | PS2_PINS[]             | Change as needed   |
| [P14] | System Behavior     | Countdown Time            | 5000 (in lineFollowing)| 0 - 10000          |
--------------------------------------------------------------------------------------------------------
*/

// ==============================
// Hardware & System Config
// ==============================
#define MOTOR_COUNT 4
const struct {
  byte in1, in2, en;
} MOTORS[MOTOR_COUNT] = {
  {4, 7, 3},     // Back Right (BR)
  {8, 9, 5},     // Back Left (BL)
  {2, 12, 6},   // Front Right (FR)
  {A1, A0, 11}   // Front Left (FL) - Thay đổi IN1 từ 6 sang A1
};

const byte PS2_PINS[] = {A2, A3, A4, A5}; // DAT, CMD, SEL, CLK

#define SERVO_PIN 13
const int OPEN_ANGLE = 30;
const int CLOSE_ANGLE = 180;

// ------------------------------
// System Parameters & Defines
// ------------------------------
const int16_t MAX_SPEED = 255;
const int16_t RAMP_STEP = 15;
const int16_t DEADZONE = 12;
const float SPEED_CURVE_GAIN = 1.5;
#define BTN_SPEED 200
#define LINE_RUN_TIME 5000
#define SERIAL_TIMEOUT 500

// ==============================
// Global Variables & Objects
// ==============================
PS2X ps2Controller;
Servo clampServo;

enum ControlMode { MODE_ANALOG, MODE_DIGITAL };
ControlMode currentControlMode = MODE_ANALOG;

struct {
  bool lineFollowing = false;
  bool clampMode = false;
  bool clampClosed = false;
  bool lineActivated = false;
  uint32_t lineStartTime = 0;
} systemStatus;

int16_t targetSpeed[3] = {0};
int16_t currentSpeed[3] = {0};
uint32_t lastControlUpdate = 0;

// ==============================
// Function Prototypes
// ==============================
void setupMotorPins();
void setupPS2Controller();
void setupServo();
void updatePS2Controller();
void processControlInput();
void processAnalogSticks();
void processDigitalButtons();
void updateMotorOutput();
void calculateMecanumSpeeds(int16_t* speeds);
void setMotorSpeed(byte motorIndex, int16_t speed);
void handleModes();
void lineFollowingMode();
int16_t applyDeadzone(int16_t value);
int16_t applyCurve(int16_t value);
int16_t applyRamping(int16_t current, int16_t target);
void emergencyStop();

// ==============================
// Setup & Main Loop
// ==============================
void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupPS2Controller();
  setupServo();
  
  // Configure PWM frequency - tránh điều chỉnh cho Timer2 để không ảnh hưởng Servo
  TCCR0B = (TCCR0B & 0b11111000) | 0x01; // PWM 62.5kHz
  TCCR2B = (TCCR2B & 0b11111000) | 0x01; // PWM 31.4kHz
}

void loop() {
  const uint16_t CONTROL_INTERVAL = 10;
  if (millis() - lastControlUpdate >= CONTROL_INTERVAL) {
    lastControlUpdate = millis();
    updatePS2Controller();
    handleModes();
    processControlInput();
    updateMotorOutput();
  }
}

// ==============================
// Setup Functions
// ==============================
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
    error = ps2Controller.config_gamepad(PS2_PINS[3], PS2_PINS[1],
           PS2_PINS[2], PS2_PINS[0], false, false);
    if (error) delay(500);
    retryCount++;
  }
}

void setupServo() {
  clampServo.attach(SERVO_PIN);
  clampServo.write(OPEN_ANGLE);
}

// ==============================
// PS2 Controller Functions
// ==============================
void updatePS2Controller() {
  ps2Controller.read_gamepad();
}

void processControlInput() {
  if (systemStatus.lineFollowing) return;
  
  // Toggle control mode with SELECT button
  if (ps2Controller.ButtonPressed(PSB_SELECT)) {
    currentControlMode = (currentControlMode == MODE_ANALOG) ? MODE_DIGITAL : MODE_ANALOG;
    memset(targetSpeed, 0, sizeof(targetSpeed));
    Serial.print("Control Mode: ");
    Serial.println(currentControlMode == MODE_ANALOG ? "Analog" : "Digital");
  }
  
  (currentControlMode == MODE_ANALOG) ? processAnalogSticks() : processDigitalButtons();
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
  targetSpeed[2] = (ps2Controller.Button(PSB_R1)       ? BTN_SPEED : 0) -
                   (ps2Controller.Button(PSB_L1)      ? BTN_SPEED : 0);
}

// ==============================
// Motor Control Functions
// ==============================
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
  speeds[0] = currentSpeed[1] - currentSpeed[0] - currentSpeed[2]; // BR
  speeds[1] = currentSpeed[1] + currentSpeed[0] + currentSpeed[2]; // BL
  speeds[2] = currentSpeed[1] + currentSpeed[0] - currentSpeed[2]; // FR
  speeds[3] = currentSpeed[1] - currentSpeed[0] + currentSpeed[2]; // FL

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

// ==============================
// Mode & Special Functions
// ==============================
void handleModes() {
  static uint32_t lastModeChange = 0;
  const uint16_t DEBOUNCE = 250;
  if (millis() - lastModeChange < DEBOUNCE) return;

  // Toggle clamp mode with TRIANGLE button
  if (ps2Controller.ButtonPressed(PSB_TRIANGLE)) {
    systemStatus.clampMode = !systemStatus.clampMode;
    Serial.print("Clamp Mode: ");
    Serial.println(systemStatus.clampMode ? "ON" : "OFF");
    lastModeChange = millis();
  }
  
  // Toggle line following with SQUARE button
  if (ps2Controller.ButtonPressed(PSB_SQUARE)) {
    systemStatus.lineFollowing = !systemStatus.lineFollowing;
    systemStatus.lineActivated = false;
    Serial.print("Line Mode: ");
    Serial.println(systemStatus.lineFollowing ? "ON" : "OFF");
    lastModeChange = millis();
  }
  
  // Control clamp with START button in clamp mode
  if (systemStatus.clampMode && ps2Controller.ButtonPressed(PSB_START)) {
    systemStatus.clampClosed = !systemStatus.clampClosed;
    clampServo.write(systemStatus.clampClosed ? CLOSE_ANGLE : OPEN_ANGLE);
    Serial.print("Clamp: ");
    Serial.println(systemStatus.clampClosed ? "CLOSED" : "OPEN");
    lastModeChange = millis();
  }
  
  // Activate line following with START button in line mode
  if (systemStatus.lineFollowing && ps2Controller.ButtonPressed(PSB_START)) {
    systemStatus.lineActivated = true;
    systemStatus.lineStartTime = millis();
    Serial.println("Line mode ACTIVATED");
    lastModeChange = millis();
  }
  
  // Process line following if activated
  if (systemStatus.lineFollowing && systemStatus.lineActivated) {
    lineFollowingMode();
  }
}

void lineFollowingMode() {
  if (millis() - systemStatus.lineStartTime < 5000) {
    Serial.print("Countdown: ");
    Serial.println(5000 - (millis() - systemStatus.lineStartTime));
    targetSpeed[1] = 0;
    return;
  }
  if (millis() - systemStatus.lineStartTime < (5000 + LINE_RUN_TIME)) {
    targetSpeed[1] = BTN_SPEED;
  } else {
    systemStatus.lineFollowing = false;
    systemStatus.lineActivated = false;
    memset(targetSpeed, 0, sizeof(targetSpeed));
    Serial.println("Line mode COMPLETED");
  }
}

// ==============================
// Utility Functions
// ==============================
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
