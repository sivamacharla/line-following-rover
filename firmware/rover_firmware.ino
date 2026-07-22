/*
  Autonomous Line-Following Rover — Firmware
  Platform: Arduino (Uno/Nano/Mega compatible)
  Motor driver: L298N (or equivalent H-bridge)

  Sensors:
    - 5x IR reflectance sensors (line position)
    - 1x HC-SR04 ultrasonic sensor (obstacle / terrain edge detection)

  Control:
    - PID loop on IR-derived line error -> differential motor speed
    - Sensor fusion: ultrasonic distance gates/limits speed near obstacles
    - Exponential moving average (EMA) filters on both sensor types to
      reject noise and prevent PID overcorrection
*/

#include <Arduino.h>

// ---------- Pin configuration ----------
const uint8_t IR_PINS[5]   = {A0, A1, A2, A3, A4};   // left -> right
const uint8_t TRIG_PIN     = 8;
const uint8_t ECHO_PIN     = 9;

const uint8_t ENA_PIN      = 5;   // left motor PWM
const uint8_t IN1_PIN      = 4;
const uint8_t IN2_PIN      = 3;
const uint8_t ENB_PIN      = 6;   // right motor PWM
const uint8_t IN3_PIN      = 7;
const uint8_t IN4_PIN      = 2;

// ---------- Tuning constants (derived from MATLAB/Python sim) ----------
float KP = 34.0f;
float KI = 0.6f;
float KD = 9.5f;

const int   BASE_SPEED     = 150;   // 0-255 PWM
const int   MAX_SPEED      = 255;
const int   MIN_SPEED      = 0;

const float IR_EMA_ALPHA   = 0.35f; // higher = less smoothing, faster response
const float US_EMA_ALPHA   = 0.25f;
const float D_FILTER_ALPHA = 0.25f; // low-pass on the derivative term, rejects noise-driven "derivative kick"
const float PID_OUTPUT_LIMIT = 120.0f; // clamp steering correction to prevent single-step overcorrection

const float OBSTACLE_SLOW_CM  = 25.0f; // start slowing below this distance
const float OBSTACLE_STOP_CM  = 8.0f;  // stop below this distance

// ---------- State ----------
int   irRaw[5];
float irFiltered[5] = {0, 0, 0, 0, 0};
float lineError     = 0.0f;
float lastError      = 0.0f;
float integral        = 0.0f;
float filteredDerivative = 0.0f;

float ultrasonicFiltered = 100.0f; // start "far away"

unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 20; // 50 Hz control loop

// Sensor weights for weighted-average line position (-2 .. +2)
const float SENSOR_WEIGHTS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 5; i++) pinMode(IR_PINS[i], INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  stopMotors();
  lastLoopTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_INTERVAL_MS) return;
  float dt = (now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;

  readAndFilterIR();
  float distanceCm = readUltrasonicCm();
  ultrasonicFiltered = emaFilter(ultrasonicFiltered, distanceCm, US_EMA_ALPHA);

  lineError = computeLineError();
  float correction = computePID(lineError, dt);

  int speedCap = speedCapFromDistance(ultrasonicFiltered);
  driveMotors(correction, speedCap);

  logTelemetry(distanceCm, speedCap, correction);
}

// ---------- Sensing ----------

void readAndFilterIR() {
  for (uint8_t i = 0; i < 5; i++) {
    irRaw[i] = analogRead(IR_PINS[i]);
    irFiltered[i] = emaFilter(irFiltered[i], irRaw[i], IR_EMA_ALPHA);
  }
}

// Weighted-average line position from filtered IR array.
// Returns 0 when centered, negative when line is left of center, positive when right.
float computeLineError() {
  float weightedSum = 0.0f;
  float total = 0.0f;

  for (uint8_t i = 0; i < 5; i++) {
    // Higher analog reading = darker surface = line detected (adjust if sensors are inverted)
    float activation = irFiltered[i];
    weightedSum += activation * SENSOR_WEIGHTS[i];
    total += activation;
  }

  if (total < 50.0f) {
    // No line detected under any sensor: reuse last known error direction
    // so the rover keeps turning toward where it last saw the line.
    return lastError >= 0 ? 2.0f : -2.0f;
  }

  return weightedSum / total;
}

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout ~5m range
  if (duration == 0) return 400.0f; // timeout -> treat as "no obstacle"

  return duration * 0.0343f / 2.0f; // speed of sound cm/us
}

float emaFilter(float previous, float sample, float alpha) {
  return alpha * sample + (1.0f - alpha) * previous;
}

// ---------- Control ----------

float computePID(float error, float dt) {
  integral += error * dt;
  integral = constrain(integral, -50.0f, 50.0f); // anti-windup clamp

  float rawDerivative = (dt > 0) ? (error - lastError) / dt : 0.0f;
  filteredDerivative = emaFilter(filteredDerivative, rawDerivative, D_FILTER_ALPHA);
  lastError = error;

  float output = (KP * error) + (KI * integral) + (KD * filteredDerivative);
  return constrain(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
}

// Sensor fusion: ultrasonic reading gates the maximum allowed speed,
// independent of the PID steering correction.
int speedCapFromDistance(float distanceCm) {
  if (distanceCm <= OBSTACLE_STOP_CM) return 0;
  if (distanceCm >= OBSTACLE_SLOW_CM) return BASE_SPEED;

  float ratio = (distanceCm - OBSTACLE_STOP_CM) / (OBSTACLE_SLOW_CM - OBSTACLE_STOP_CM);
  return (int)(BASE_SPEED * ratio);
}

void driveMotors(float correction, int speedCap) {
  int leftSpeed  = speedCap - (int)correction;
  int rightSpeed = speedCap + (int)correction;

  leftSpeed  = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  setMotor(ENA_PIN, IN1_PIN, IN2_PIN, leftSpeed);
  setMotor(ENB_PIN, IN3_PIN, IN4_PIN, rightSpeed);
}

void setMotor(uint8_t enPin, uint8_t in1, uint8_t in2, int speed) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enPin, speed);
}

void stopMotors() {
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

// ---------- Telemetry ----------
// CSV over serial, consumed by dashboard/serial bridge: time,error,correction,distance,speedCap
void logTelemetry(float distanceCm, int speedCap, float correction) {
  Serial.print(millis());   Serial.print(",");
  Serial.print(lineError, 3); Serial.print(",");
  Serial.print(correction, 2); Serial.print(",");
  Serial.print(distanceCm, 1); Serial.print(",");
  Serial.println(speedCap);
}
