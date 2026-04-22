/*
  Arduino Uno bridge sketch for the waypoint-ui robot.

  This version is meant for the real hardware layout:
  - Raspberry Pi <-> Arduino Uno over USB serial
  - Arduino Uno <-> L298N motor driver
  - two encoder-equipped motors
  - optional left/right IR edge sensors
  - optional front/back temperature analog sense
  - optional main-battery and Pi-battery analog sense

  Serial protocol from the Pi:
    M,<front_output>,<back_output>
    STOP

  Sensor packet back to the Pi:
    S,<left_edge>,<right_edge>,<heading_deg>,<front_count>,<back_count>,<front_rpm>,<back_rpm>,<front_temp_c>,<back_temp_c>,<battery_voltage>,<pi_battery_voltage>

  Important:
  - Outputs are clamped to [-1.0, 1.0]
  - The Uno hard-stops both motors if commands stop arriving
  - A zero command immediately cuts PWM and both H-bridge direction pins
*/

#include <math.h>

struct MotorCommand {
  float front;
  float back;
};

struct EncoderState {
  volatile long count;
  long lastReportedCount;
  float rpm;
  unsigned long lastSampleMs;
};

MotorCommand currentCommand = {0.0f, 0.0f};
EncoderState frontEncoder = {0, 0, 0.0f, 0};
EncoderState backEncoder = {0, 0, 0.0f, 0};

// L298N wiring.
// Front motor channel: ENA, IN1, IN2, OUT1, OUT2
const int FRONT_PWM_PIN = 9;   // ENA
const int FRONT_IN1_PIN = 7;   // IN1
const int FRONT_IN2_PIN = 6;   // IN2

// Back motor channel: ENB, IN3, IN4, OUT3, OUT4
const int BACK_PWM_PIN = 10;   // ENB
const int BACK_IN1_PIN = 8;    // IN3
const int BACK_IN2_PIN = 11;   // IN4

// Encoder wiring.
// Uno external interrupts are on pins 2 and 3, so use them for channel A.
const int FRONT_ENCODER_A_PIN = 2;
const int FRONT_ENCODER_B_PIN = 4;
const int BACK_ENCODER_A_PIN = 3;
const int BACK_ENCODER_B_PIN = 5;

const float FRONT_ENCODER_COUNTS_PER_REV = 360.0f;
const float BACK_ENCODER_COUNTS_PER_REV = 360.0f;

// Optional sensors. Leave as -1 if not wired yet.
const int LEFT_IR_PIN = -1;
const int RIGHT_IR_PIN = -1;
const int LEFT_IR_ANALOG_PIN = -1;
const int RIGHT_IR_ANALOG_PIN = -1;
const int FRONT_TEMP_SENSOR_PIN = -1;
const int BACK_TEMP_SENSOR_PIN = -1;
const int BATTERY_VOLTAGE_PIN = -1;
const int PI_BATTERY_VOLTAGE_PIN = -1;
const float BATTERY_VOLTAGE_DIVIDER_RATIO = 1.0f;
const float PI_BATTERY_VOLTAGE_DIVIDER_RATIO = 1.0f;
const float ADC_REFERENCE_VOLTAGE = 5.0f;
const float TEMP_SENSOR_VOLTS_PER_C = 0.01f;
const float TEMP_SENSOR_OFFSET_C = 0.0f;
const float IR_EDGE_DROP_THRESHOLD_MM = 2.0f;
const float IR_MM_PER_ADC_COUNT = 0.1f;  // Tune this for your specific IR sensor.
const unsigned int IR_BASELINE_SAMPLE_COUNT = 8;
const unsigned long IR_BASELINE_SETTLE_MS = 120;

const unsigned long SENSOR_REPORT_INTERVAL_MS = 50;
const unsigned long COMMAND_WATCHDOG_TIMEOUT_MS = 250;

unsigned long lastSensorReportMs = 0;
unsigned long lastCommandMs = 0;
unsigned long irRunStartMs = 0;
bool irRunArmed = false;

struct IRSensorTracker {
  long baselineAccumulator;
  unsigned int baselineSampleCount;
  int baselineRaw;
  bool baselineReady;
};

IRSensorTracker leftIrTracker = {0, 0, 0, false};
IRSensorTracker rightIrTracker = {0, 0, 0, false};

float clampUnit(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

int toPwm(float value) {
  return (int)(fabs(clampUnit(value)) * 255.0f + 0.5f);
}

bool commandIsActive(const MotorCommand &command) {
  return fabs(command.front) > 0.0005f || fabs(command.back) > 0.0005f;
}

bool encoderPinsConfigured(int aPin, int bPin) {
  return aPin >= 0 && bPin >= 0;
}

void updateEncoderCount(volatile long &countRef, int aPin, int bPin) {
  if (!encoderPinsConfigured(aPin, bPin)) {
    return;
  }

  const int aState = digitalRead(aPin);
  const int bState = digitalRead(bPin);
  countRef += (aState == bState) ? 1 : -1;
}

void onFrontEncoderA() {
  updateEncoderCount(frontEncoder.count, FRONT_ENCODER_A_PIN, FRONT_ENCODER_B_PIN);
}

void onBackEncoderA() {
  updateEncoderCount(backEncoder.count, BACK_ENCODER_A_PIN, BACK_ENCODER_B_PIN);
}

void setMotorPinsLow(int in1Pin, int in2Pin, int pwmPin) {
  if (in1Pin >= 0) digitalWrite(in1Pin, LOW);
  if (in2Pin >= 0) digitalWrite(in2Pin, LOW);
  if (pwmPin >= 0) analogWrite(pwmPin, 0);
}

void applySingleMotor(float value, int pwmPin, int in1Pin, int in2Pin) {
  if (pwmPin < 0 || in1Pin < 0 || in2Pin < 0) {
    return;
  }

  value = clampUnit(value);
  if (fabs(value) < 0.0005f) {
    setMotorPinsLow(in1Pin, in2Pin, pwmPin);
    return;
  }

  if (value > 0.0f) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }

  analogWrite(pwmPin, toPwm(value));
}

void applyMotorOutputs(const MotorCommand &command) {
  applySingleMotor(command.front, FRONT_PWM_PIN, FRONT_IN1_PIN, FRONT_IN2_PIN);
  applySingleMotor(command.back, BACK_PWM_PIN, BACK_IN1_PIN, BACK_IN2_PIN);
}

void stopMotors() {
  currentCommand.front = 0.0f;
  currentCommand.back = 0.0f;
  setMotorPinsLow(FRONT_IN1_PIN, FRONT_IN2_PIN, FRONT_PWM_PIN);
  setMotorPinsLow(BACK_IN1_PIN, BACK_IN2_PIN, BACK_PWM_PIN);
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

float computeRpm(EncoderState &encoder, float countsPerRev) {
  if (countsPerRev <= 0.0f) {
    encoder.lastReportedCount = encoder.count;
    encoder.lastSampleMs = millis();
    encoder.rpm = 0.0f;
    return 0.0f;
  }

  const unsigned long nowMs = millis();
  const unsigned long elapsedMs = nowMs - encoder.lastSampleMs;
  if (elapsedMs == 0) {
    return encoder.rpm;
  }

  const long deltaCounts = encoder.count - encoder.lastReportedCount;
  const float revolutions = deltaCounts / countsPerRev;
  const float elapsedMinutes = elapsedMs / 60000.0f;
  encoder.rpm = revolutions / elapsedMinutes;
  encoder.lastReportedCount = encoder.count;
  encoder.lastSampleMs = nowMs;
  return encoder.rpm;
}

int readEdgePin(int pin) {
  if (pin < 0) {
    return 0;
  }
  return digitalRead(pin) == LOW ? 1 : 0;
}

void resetIrTracker(IRSensorTracker &tracker) {
  tracker.baselineAccumulator = 0;
  tracker.baselineSampleCount = 0;
  tracker.baselineRaw = 0;
  tracker.baselineReady = false;
}

void armIrBaselineIfNeeded() {
  const bool active = commandIsActive(currentCommand);
  if (active && !irRunArmed) {
    irRunArmed = true;
    irRunStartMs = millis();
    resetIrTracker(leftIrTracker);
    resetIrTracker(rightIrTracker);
    return;
  }

  if (!active && irRunArmed) {
    irRunArmed = false;
    resetIrTracker(leftIrTracker);
    resetIrTracker(rightIrTracker);
  }
}

bool analogIrConfigured(int pin) {
  return pin >= 0;
}

int readIrAnalogRaw(int pin) {
  if (!analogIrConfigured(pin)) {
    return -1;
  }
  return analogRead(pin);
}

bool updateAnalogIrEdgeState(int pin, IRSensorTracker &tracker) {
  if (!irRunArmed) {
    return false;
  }

  const int raw = readIrAnalogRaw(pin);
  if (raw < 0) {
    return false;
  }

  if (millis() - irRunStartMs < IR_BASELINE_SETTLE_MS) {
    return false;
  }

  if (!tracker.baselineReady) {
    tracker.baselineAccumulator += raw;
    tracker.baselineSampleCount += 1;
    if (tracker.baselineSampleCount >= IR_BASELINE_SAMPLE_COUNT) {
      tracker.baselineRaw = (int)(tracker.baselineAccumulator / (long)tracker.baselineSampleCount);
      tracker.baselineReady = true;
    }
    return false;
  }

  const float deltaMm = fabs((float)(raw - tracker.baselineRaw)) * IR_MM_PER_ADC_COUNT;
  return deltaMm > IR_EDGE_DROP_THRESHOLD_MM;
}

int readLeftEdgeState() {
  if (analogIrConfigured(LEFT_IR_ANALOG_PIN)) {
    return updateAnalogIrEdgeState(LEFT_IR_ANALOG_PIN, leftIrTracker) ? 1 : 0;
  }
  return readEdgePin(LEFT_IR_PIN);
}

int readRightEdgeState() {
  if (analogIrConfigured(RIGHT_IR_ANALOG_PIN)) {
    return updateAnalogIrEdgeState(RIGHT_IR_ANALOG_PIN, rightIrTracker) ? 1 : 0;
  }
  return readEdgePin(RIGHT_IR_PIN);
}

float readScaledVoltage(int pin, float dividerRatio) {
  if (pin < 0) {
    return NAN;
  }

  const int raw = analogRead(pin);
  const float sensedVoltage = (raw / 1023.0f) * ADC_REFERENCE_VOLTAGE;
  return sensedVoltage * dividerRatio;
}

float readBatteryVoltage() {
  return readScaledVoltage(BATTERY_VOLTAGE_PIN, BATTERY_VOLTAGE_DIVIDER_RATIO);
}

float readPiBatteryVoltage() {
  return readScaledVoltage(PI_BATTERY_VOLTAGE_PIN, PI_BATTERY_VOLTAGE_DIVIDER_RATIO);
}

float readTemperatureC(int pin) {
  if (pin < 0) {
    return NAN;
  }

  const int raw = analogRead(pin);
  const float sensedVoltage = (raw / 1023.0f) * ADC_REFERENCE_VOLTAGE;
  return (sensedVoltage / TEMP_SENSOR_VOLTS_PER_C) + TEMP_SENSOR_OFFSET_C;
}

void printOptionalFloat(float value) {
  if (isnan(value)) {
    return;
  }
  Serial.print(value, 3);
}

void printSensorPacket() {
  computeRpm(frontEncoder, FRONT_ENCODER_COUNTS_PER_REV);
  computeRpm(backEncoder, BACK_ENCODER_COUNTS_PER_REV);
  const float frontTempC = readTemperatureC(FRONT_TEMP_SENSOR_PIN);
  const float backTempC = readTemperatureC(BACK_TEMP_SENSOR_PIN);

  Serial.print("S,");
  Serial.print(readLeftEdgeState());
  Serial.print(",");
  Serial.print(readRightEdgeState());
  Serial.print(",");
  Serial.print("");
  Serial.print(",");
  Serial.print(frontEncoder.count);
  Serial.print(",");
  Serial.print(backEncoder.count);
  Serial.print(",");
  Serial.print(frontEncoder.rpm, 3);
  Serial.print(",");
  Serial.print(backEncoder.rpm, 3);
  Serial.print(",");
  printOptionalFloat(frontTempC);
  Serial.print(",");
  printOptionalFloat(backTempC);
  Serial.print(",");
  printOptionalFloat(readBatteryVoltage());
  Serial.print(",");
  printOptionalFloat(readPiBatteryVoltage());
  Serial.println();
}

void setupMotorPins() {
  if (FRONT_PWM_PIN >= 0) pinMode(FRONT_PWM_PIN, OUTPUT);
  if (FRONT_IN1_PIN >= 0) pinMode(FRONT_IN1_PIN, OUTPUT);
  if (FRONT_IN2_PIN >= 0) pinMode(FRONT_IN2_PIN, OUTPUT);
  if (BACK_PWM_PIN >= 0) pinMode(BACK_PWM_PIN, OUTPUT);
  if (BACK_IN1_PIN >= 0) pinMode(BACK_IN1_PIN, OUTPUT);
  if (BACK_IN2_PIN >= 0) pinMode(BACK_IN2_PIN, OUTPUT);
}

void setupSensorPins() {
  if (LEFT_IR_PIN >= 0) pinMode(LEFT_IR_PIN, INPUT_PULLUP);
  if (RIGHT_IR_PIN >= 0) pinMode(RIGHT_IR_PIN, INPUT_PULLUP);
  if (LEFT_IR_ANALOG_PIN >= 0) pinMode(LEFT_IR_ANALOG_PIN, INPUT);
  if (RIGHT_IR_ANALOG_PIN >= 0) pinMode(RIGHT_IR_ANALOG_PIN, INPUT);
  if (FRONT_TEMP_SENSOR_PIN >= 0) pinMode(FRONT_TEMP_SENSOR_PIN, INPUT);
  if (BACK_TEMP_SENSOR_PIN >= 0) pinMode(BACK_TEMP_SENSOR_PIN, INPUT);
  if (BATTERY_VOLTAGE_PIN >= 0) pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  if (PI_BATTERY_VOLTAGE_PIN >= 0) pinMode(PI_BATTERY_VOLTAGE_PIN, INPUT);

  if (encoderPinsConfigured(FRONT_ENCODER_A_PIN, FRONT_ENCODER_B_PIN)) {
    pinMode(FRONT_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(FRONT_ENCODER_B_PIN, INPUT_PULLUP);
    const int interruptNumber = digitalPinToInterrupt(FRONT_ENCODER_A_PIN);
    if (interruptNumber != NOT_AN_INTERRUPT) {
      attachInterrupt(interruptNumber, onFrontEncoderA, CHANGE);
    }
  }

  if (encoderPinsConfigured(BACK_ENCODER_A_PIN, BACK_ENCODER_B_PIN)) {
    pinMode(BACK_ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(BACK_ENCODER_B_PIN, INPUT_PULLUP);
    const int interruptNumber = digitalPinToInterrupt(BACK_ENCODER_A_PIN);
    if (interruptNumber != NOT_AN_INTERRUPT) {
      attachInterrupt(interruptNumber, onBackEncoderA, CHANGE);
    }
  }

  frontEncoder.lastSampleMs = millis();
  backEncoder.lastSampleMs = millis();
}

void setup() {
  Serial.begin(115200);
  setupMotorPins();
  setupSensorPins();
  stopMotors();
  lastSensorReportMs = millis();
  lastCommandMs = millis();

  Serial.println("Arduino robot interface ready");
  Serial.println("Expected format: M,<front>,<back>");
}

void handleCommandLine(const String &line) {
  MotorCommand incoming;
  if (parseMotorCommand(line, incoming)) {
    currentCommand = incoming;
    applyMotorOutputs(currentCommand);
    lastCommandMs = millis();
    printCurrentCommand();
    return;
  }

  if (line == "STOP") {
    stopMotors();
    lastCommandMs = millis();
    printCurrentCommand();
    return;
  }

  if (line == "PING") {
    Serial.println("PONG");
    return;
  }

  Serial.print("ERR unknown command: ");
  Serial.println(line);
}

void loop() {
  const unsigned long nowMs = millis();

  if (nowMs - lastCommandMs >= COMMAND_WATCHDOG_TIMEOUT_MS) {
    stopMotors();
  }

  armIrBaselineIfNeeded();

  if (nowMs - lastSensorReportMs >= SENSOR_REPORT_INTERVAL_MS) {
    printSensorPacket();
    lastSensorReportMs = nowMs;
  }

  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      continue;
    }
    handleCommandLine(line);
  }
}
