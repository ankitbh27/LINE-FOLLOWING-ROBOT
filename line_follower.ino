/*
  Line Following Robot - Arduino Uno + L298N + 5 IR sensors, PID control

  WIRING
  ------
  L298N:  ENA -> D5 (PWM)   IN1 -> D7   IN2 -> D8     (left motor)
          ENB -> D6 (PWM)   IN3 -> D9   IN4 -> D10    (right motor)
          Remove the ENA/ENB jumpers on the L298N so PWM works.
          Motor battery -> L298N 12V, battery GND -> L298N GND AND Arduino GND.

  IR sensor modules (digital output), left to right:
          S1 -> A0   S2 -> A1   S3 -> A2   S4 -> A3   S5 -> A4
          VCC -> 5V, GND -> GND

  TUNING
  ------
  1. Set LINE_IS_HIGH to match your sensors (see the sensor test below).
  2. Start with Ki = 0, Kd = 0, raise Kp until the robot follows but wobbles.
  3. Raise Kd until the wobble settles. Add a tiny Ki only if it drifts.
  4. Raise BASE_SPEED gradually.

  Set DEBUG to true and open Serial Monitor at 115200 to see sensor readings.
*/

// ---------------- Pins ----------------
const uint8_t ENA = 5, IN1 = 7, IN2 = 8;    // left motor
const uint8_t ENB = 6, IN3 = 9, IN4 = 10;   // right motor
const uint8_t SENSOR_PINS[5] = { A0, A1, A2, A3, A4 };

// ---------------- Settings ----------------
const bool LINE_IS_HIGH = true;   // true: sensor reads HIGH over the line, false: LOW
const bool DEBUG        = false;  // print sensor values to Serial

int BASE_SPEED = 150;             // 0-255 forward speed on a straight line
int MAX_SPEED  = 230;             // upper limit for either motor
int TURN_SPEED = 140;             // speed used when searching for a lost line

float Kp = 0.045;
float Ki = 0.0;
float Kd = 0.35;

// Sensor weights: leftmost = -2000, rightmost = +2000
const int WEIGHTS[5] = { -2000, -1000, 0, 1000, 2000 };

// ---------------- State ----------------
float integralTerm = 0;
int lastError = 0;
int lastSeenDir = 0;   // -1 = line was last on the left, +1 = on the right

// ---------------- Motor control ----------------
// speed: -255 (full reverse) .. +255 (full forward)
void setMotor(uint8_t pinA, uint8_t pinB, uint8_t pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
  analogWrite(pwmPin, abs(speed));
}

void drive(int left, int right) {
  setMotor(IN1, IN2, ENA, left);
  setMotor(IN3, IN4, ENB, right);
}

void stopMotors() {
  drive(0, 0);
}

// ---------------- Sensors ----------------
// Fills s[] with 1 when the sensor is over the line. Returns how many see it.
int readSensors(uint8_t s[5]) {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    bool raw = digitalRead(SENSOR_PINS[i]);
    s[i] = (raw == LINE_IS_HIGH) ? 1 : 0;
    count += s[i];
  }
  return count;
}

// ---------------- Setup ----------------
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);

  if (DEBUG) Serial.begin(115200);

  stopMotors();
  delay(2000);   // time to place the robot on the line
}

// ---------------- Main loop ----------------
void loop() {
  uint8_t s[5];
  int count = readSensors(s);

  if (DEBUG) {
    for (int i = 0; i < 5; i++) Serial.print(s[i]);
    Serial.println();
  }

  // Line lost: spin toward the side where it was last seen
  if (count == 0) {
    integralTerm = 0;
    if (lastSeenDir <= 0) drive(-TURN_SPEED, TURN_SPEED);   // turn left
    else                  drive(TURN_SPEED, -TURN_SPEED);   // turn right
    return;
  }

  // All sensors on the line (junction / thick line): go straight through
  if (count == 5) {
    drive(BASE_SPEED, BASE_SPEED);
    return;
  }

  // Weighted average position of the line
  long sum = 0;
  for (int i = 0; i < 5; i++) {
    if (s[i]) sum += WEIGHTS[i];
  }
  int error = sum / count;   // negative = line on left, positive = line on right

  if (error < 0) lastSeenDir = -1;
  else if (error > 0) lastSeenDir = 1;

  // PID
  integralTerm += error;
  integralTerm = constrain(integralTerm, -5000, 5000);   // anti-windup
  float derivative = error - lastError;
  lastError = error;

  float correction = Kp * error + Ki * integralTerm + Kd * derivative;

  // Line on the right (error > 0) -> speed up left motor, slow right motor
  int leftSpeed  = constrain(BASE_SPEED + (int)correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - (int)correction, -MAX_SPEED, MAX_SPEED);

  drive(leftSpeed, rightSpeed);
}
