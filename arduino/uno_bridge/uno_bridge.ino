/*
  Arduino Uno bridge sketch for the robot-field-ui project.

  Current responsibilities:
  - receive front/back motor commands over serial
  - clamp and store the command values safely
  - expose a clean place to wire in the real motor driver later
  - leave room for future IR sensor / encoder reporting back to the Pi

  Serial protocol from Python:
    M,<front_output>,<back_output>

  Example:
    M,0.5000,-0.2000
*/

struct MotorCommand {
  float front;
  float back;
};

MotorCommand currentCommand = {0.0f, 0.0f};

// Replace these once the motor driver wiring is finalized.
const int FRONT_PWM_PIN = -1;
const int FRONT_DIR_PIN = -1;
const int BACK_PWM_PIN = -1;
const int BACK_DIR_PIN = -1;

// Future sensor pins can go here once the hardware is wired.
const int LEFT_IR_PIN = -1;
const int RIGHT_IR_PIN = -1;

float clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

int toPwm(float value) {
  value = clampUnit(value);
  return (int)(fabs(value) * 255.0f + 0.5f);
}

bool isForward(float value) {
  return value >= 0.0f;
}

void applySingleMotor(float value, int pwmPin, int dirPin) {
  if (pwmPin < 0 || dirPin < 0) {
    return;
  }

  digitalWrite(dirPin, isForward(value) ? HIGH : LOW);
  analogWrite(pwmPin, toPwm(value));
}

void applyMotorOutputs(const MotorCommand &command) {
  applySingleMotor(command.front, FRONT_PWM_PIN, FRONT_DIR_PIN);
  applySingleMotor(command.back, BACK_PWM_PIN, BACK_DIR_PIN);
}

void stopMotors() {
  currentCommand.front = 0.0f;
  currentCommand.back = 0.0f;
  applyMotorOutputs(currentCommand);
}

bool parseMotorCommand(const String &line, MotorCommand &outCommand) {
  if (!line.startsWith("M,")) {
    return false;
  }

  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  if (secondComma <= firstComma) {
    return false;
  }

  String frontStr = line.substring(firstComma + 1, secondComma);
  String backStr = line.substring(secondComma + 1);

  outCommand.front = clampUnit(frontStr.toFloat());
  outCommand.back = clampUnit(backStr.toFloat());
  return true;
}

void printCurrentCommand() {
  Serial.print("OK front=");
  Serial.print(currentCommand.front, 4);
  Serial.print(" back=");
  Serial.println(currentCommand.back, 4);
}

void setupMotorPins() {
  if (FRONT_PWM_PIN >= 0) pinMode(FRONT_PWM_PIN, OUTPUT);
  if (FRONT_DIR_PIN >= 0) pinMode(FRONT_DIR_PIN, OUTPUT);
  if (BACK_PWM_PIN >= 0) pinMode(BACK_PWM_PIN, OUTPUT);
  if (BACK_DIR_PIN >= 0) pinMode(BACK_DIR_PIN, OUTPUT);
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  stopMotors();

  Serial.println("Arduino robot interface ready");
  Serial.println("Expected format: M,<front>,<back>");
}

void loop() {
  if (Serial.available() <= 0) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) {
    return;
  }

  MotorCommand incoming;
  if (parseMotorCommand(line, incoming)) {
    currentCommand = incoming;
    applyMotorOutputs(currentCommand);
    printCurrentCommand();
    return;
  }

  if (line == "STOP") {
    stopMotors();
    printCurrentCommand();
    return;
  }

  Serial.print("ERR unknown command: ");
  Serial.println(line);
}
