// ==========================================================
// AGRICULTURAL AUTONOMOUS ROVER - FINAL VERSION
// ==========================================================
// Features:
//   - Autonomous obstacle avoidance (3x HC-SR04)
//   - Compass-assisted straight driving (QMC5883L)
//   - GPS tracking (NEO-6M)
//   - Soil moisture + EC sampling with auto-watering
//   - Bluetooth telemetry to custom farmer app
//   - Soft-start motor control (BTS7960)
//   - Hardware watchdog for reliability
// ==========================================================

#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

// ==========================================================
// PIN DEFINITIONS
// ==========================================================
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define MOISTURE_PIN 34
#define EC_POWER_PIN 15
#define EC_ANALOG_PIN 36

#define FRONT_TRIG 4
#define FRONT_ECHO 18
#define LEFT_TRIG 19
#define LEFT_ECHO 35
#define RIGHT_TRIG 23
#define RIGHT_ECHO 39

#define PUMP_PIN 5
#define L_RPWM 33
#define L_LPWM 25
#define R_RPWM 26
#define R_LPWM 27

#define SERVO_PIN 13
#define SERVO_UP 0
#define SERVO_DOWN 90
#define QMC5883L_ADDR 0x0D

// ==========================================================
// SETTINGS
// ==========================================================
int FORWARD_SPEED = 180;
int TURN_SPEED = 170;
int REVERSE_SPEED = 170;
int OBSTACLE_DISTANCE = 30;       // cm — obstacle detection threshold
int USER_CLEAR_DISTANCE = 45;     // cm — distance to resume auto after dead-end
int MOISTURE_DRY_THRESHOLD = 40;  // % — below this = dry, trigger pump
unsigned long SOIL_CHECK_INTERVAL = 15000; // 15 s between soil checks

#define COMPASS_KP 2.0
#define COMPASS_DEADBAND 3
#define TURN_TOLERANCE 8
#define TURN_TIMEOUT 1800
#define FALLBACK_TURN_TIME 800

#define ACCEL_STEP 5              // motor ramp per loop tick (soft start)
#define SENSOR_STAGGER_MS 40      // gap between ultrasonic triggers

// ==========================================================
// OBJECTS
// ==========================================================
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
BluetoothSerial SerialBT;
Servo probeServo;

// ==========================================================
// GLOBAL VARIABLES
// ==========================================================
unsigned long lastTransmit = 0;
unsigned long lastTurnEndTime = 0;
unsigned long reverseDuration = 0;
unsigned long actionStartTime = 0;
unsigned long agStartTime = 0;

double smoothedLat = 0.0;
double smoothedLng = 0.0;

int distFront = 400;
int distLeft = 400;
int distRight = 400;
int frontPercent = 100;
int leftPercent = 100;
int rightPercent = 100;

int moisturePercent = 0;
int ecPercent = 0;
bool isWatering = false;

int lastValidHeading = 0;
bool compassValid = false;
bool headingLocked = false;
int targetHeading = 0;
int turnTargetHeading = 0;

bool hasTurnedBefore = false;
bool turningRight = true;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;

uint8_t ultrasonicCycle = 0;
unsigned long lastUltrasonicTick = 0;

// ==========================================================
// STATE MACHINE
// ==========================================================
enum RoverState {
  DRIVING,
  TURNING,
  REVERSING,
  WAITING_FOR_USER,
  LOWERING,
  READING,
  WATERING,
  RAISING
};
RoverState currentState = DRIVING;

// ==========================================================
// MOTOR CONTROL (Soft-Start Acceleration)
// ==========================================================
void setMotorTargets(int left, int right) {
  targetLeftSpeed = constrain(left, -255, 255);
  targetRightSpeed = constrain(right, -255, 255);
}

void updateMotors() {
  // Ramp left motor
  if (currentLeftSpeed < targetLeftSpeed) {
    currentLeftSpeed = min(currentLeftSpeed + ACCEL_STEP, targetLeftSpeed);
  } else if (currentLeftSpeed > targetLeftSpeed) {
    currentLeftSpeed = max(currentLeftSpeed - ACCEL_STEP, targetLeftSpeed);
  }

  // Ramp right motor
  if (currentRightSpeed < targetRightSpeed) {
    currentRightSpeed = min(currentRightSpeed + ACCEL_STEP, targetRightSpeed);
  } else if (currentRightSpeed > targetRightSpeed) {
    currentRightSpeed = max(currentRightSpeed - ACCEL_STEP, targetRightSpeed);
  }

  // Apply to BTS7960 — left channel
  if (currentLeftSpeed >= 0) {
    analogWrite(L_RPWM, currentLeftSpeed);
    analogWrite(L_LPWM, 0);
  } else {
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, -currentLeftSpeed);
  }

  // Apply to BTS7960 — right channel
  if (currentRightSpeed >= 0) {
    analogWrite(R_RPWM, currentRightSpeed);
    analogWrite(R_LPWM, 0);
  } else {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, -currentRightSpeed);
  }
}

void driveForward(int speed) { setMotorTargets(speed, speed); }
void driveReverse(int speed) { setMotorTargets(-speed, -speed); }
void turnRight(int speed)    { setMotorTargets(speed, -speed); }
void turnLeft(int speed)     { setMotorTargets(-speed, speed); }
void stopMotors()            { setMotorTargets(0, 0); }

// ==========================================================
// COMPASS & HEADING HELPERS
// ==========================================================
int normalizeHeading(int h) {
  while (h < 0) h += 360;
  while (h >= 360) h -= 360;
  return h;
}

int headingError(int current, int target) {
  int err = current - target;
  if (err > 180) err -= 360;
  if (err < -180) err += 360;
  return err;
}

void initCompass() {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);          // continuous measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);          // 200 Hz, 8 G, 512 oversampling
  Wire.endTransmission();
}

bool updateCompass() {
  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  Wire.requestFrom(QMC5883L_ADDR, 6);
  if (Wire.available() < 6) return false;

  int16_t x = Wire.read() | (Wire.read() << 8);
  int16_t y = Wire.read() | (Wire.read() << 8);
  int16_t z = Wire.read() | (Wire.read() << 8);

  if (x == 0 && y == 0) return false;

  float heading = atan2((float)y, (float)x) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  heading += 90.0;
  if (heading >= 360.0) heading -= 360.0;

  lastValidHeading = (int)heading;
  compassValid = true;

  if (!headingLocked) {
    targetHeading = lastValidHeading;
    headingLocked = true;
  }
  return true;
}

void driveStraightWithCompass(int baseSpeed) {
  if (!compassValid || !headingLocked) {
    driveForward(baseSpeed);
    return;
  }
  int error = headingError(lastValidHeading, targetHeading);
  if (abs(error) <= COMPASS_DEADBAND) {
    driveForward(baseSpeed);
    return;
  }
  int correction = (int)(error * COMPASS_KP);
  int left = baseSpeed - correction;
  int right = baseSpeed + correction;
  setMotorTargets(left, right);
}

// ==========================================================
// ULTRASONIC SENSORS (Sequential, Non-Blocking)
// ==========================================================
int readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 25 ms timeout = ~4.25 m max range
  long duration = pulseIn(echoPin, HIGH, 25000);

  // Timeout = wide open space (not a fault)
  if (duration == 0) return 400;
  return (int)(duration * 0.034 / 2.0);
}

void updateUltrasonicSequenced() {
  unsigned long now = millis();
  if (now - lastUltrasonicTick < SENSOR_STAGGER_MS) return;
  lastUltrasonicTick = now;

  switch (ultrasonicCycle) {
    case 0:
      distFront = readDistance(FRONT_TRIG, FRONT_ECHO);
      frontPercent = constrain(map(distFront, 0, 100, 0, 100), 0, 100);
      ultrasonicCycle = 1;
      break;

    case 1:
      distLeft = readDistance(LEFT_TRIG, LEFT_ECHO);
      leftPercent = constrain(map(distLeft, 0, 100, 0, 100), 0, 100);
      ultrasonicCycle = 2;
      break;

    case 2:
      distRight = readDistance(RIGHT_TRIG, RIGHT_ECHO);
      rightPercent = constrain(map(distRight, 0, 100, 0, 100), 0, 100);
      ultrasonicCycle = 0;
      break;
  }
}

// ==========================================================
// SOIL SAMPLING
// ==========================================================
void readSoil() {
  // EC reading
  digitalWrite(EC_POWER_PIN, HIGH);
  delay(10);                   // let analog settle
  int rawEC = analogRead(EC_ANALOG_PIN);
  digitalWrite(EC_POWER_PIN, LOW);
  ecPercent = constrain(map(rawEC, 0, 4095, 0, 100), 0, 100);

  // Moisture reading
  int rawMoisture = analogRead(MOISTURE_PIN);
  moisturePercent = constrain(map(rawMoisture, 4095, 1000, 0, 100), 0, 100);
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialBT.begin("Agricultural_Rover");
  Wire.begin();
  initCompass();

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  pinMode(EC_POWER_PIN, OUTPUT);
  digitalWrite(EC_POWER_PIN, LOW);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  probeServo.attach(SERVO_PIN);
  probeServo.write(SERVO_UP);

  stopMotors();
  lastTurnEndTime = millis();
  agStartTime = millis();

  // Watchdog configuration (auto-detect Arduino core version)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 5000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
  #else
    esp_task_wdt_init(5, true);
  #endif
  esp_task_wdt_add(NULL);

  Serial.println("Agricultural Rover Initialized");
  SerialBT.println("SYSTEM_READY");
}

// ==========================================================
// MAIN LOOP
// ==========================================================
void loop() {
  esp_task_wdt_reset();

  // Feed GPS parser
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  // Update sensors
  updateCompass();
  updateUltrasonicSequenced();

  // ========================================================
  // NAVIGATION STATE MACHINE
  // ========================================================
  switch (currentState) {

    // ------------------------------------------------------
    case DRIVING:
      if (distFront < OBSTACLE_DISTANCE) {
        stopMotors();

        // Dead end (blocked front, left and right)
        if (distLeft < OBSTACLE_DISTANCE && distRight < OBSTACLE_DISTANCE) {
          if (hasTurnedBefore) {
            currentState = REVERSING;
            actionStartTime = millis();
            reverseDuration = min((unsigned long)(millis() - lastTurnEndTime), 5000UL);
            SerialBT.println("STATE:REVERSING");
          } else {
            currentState = WAITING_FOR_USER;
            SerialBT.println("STATE:DEAD_END");
          }
        }
        // Right side has more room → turn right
        else if (distRight >= distLeft) {
          turningRight = true;
          actionStartTime = millis();
          if (compassValid) turnTargetHeading = normalizeHeading(lastValidHeading + 90);
          currentState = TURNING;
          SerialBT.println("STATE:TURNING_RIGHT");
        }
        // Left side has more room → turn left
        else {
          turningRight = false;
          actionStartTime = millis();
          if (compassValid) turnTargetHeading = normalizeHeading(lastValidHeading - 90);
          currentState = TURNING;
          SerialBT.println("STATE:TURNING_LEFT");
        }
      } else {
        // Path is clear — drive straight with compass correction
        driveStraightWithCompass(FORWARD_SPEED);

        // Periodic soil sampling
        if (millis() - agStartTime >= SOIL_CHECK_INTERVAL) {
          stopMotors();
          probeServo.write(SERVO_DOWN);
          currentState = LOWERING;
          actionStartTime = millis();
          SerialBT.println("STATE:SOIL_CHECK");
        }
      }
      break;

    // ------------------------------------------------------
    case TURNING:
      if (turningRight) turnRight(TURN_SPEED);
      else turnLeft(TURN_SPEED);

      if (compassValid) {
        int error = headingError(lastValidHeading, turnTargetHeading);
        if (abs(error) <= TURN_TOLERANCE || (millis() - actionStartTime >= TURN_TIMEOUT)) {
          stopMotors();
          targetHeading = lastValidHeading;
          headingLocked = true;
          hasTurnedBefore = true;
          lastTurnEndTime = millis();
          agStartTime = millis();
          currentState = DRIVING;
          SerialBT.println("STATE:TURN_COMPLETE");
        }
      } else {
        // Fallback timed turn if compass unavailable
        if (millis() - actionStartTime >= FALLBACK_TURN_TIME) {
          stopMotors();
          hasTurnedBefore = true;
          lastTurnEndTime = millis();
          agStartTime = millis();
          currentState = DRIVING;
          SerialBT.println("STATE:TIMED_TURN_COMPLETE");
        }
      }
      break;

    // ------------------------------------------------------
    case REVERSING:
      driveReverse(REVERSE_SPEED);
      if (millis() - actionStartTime >= reverseDuration) {
        stopMotors();
        hasTurnedBefore = false;
        if (compassValid) {
          targetHeading = lastValidHeading;
          headingLocked = true;
        }
        agStartTime = millis();
        currentState = DRIVING;
        SerialBT.println("STATE:REVERSE_COMPLETE");
      }
      break;

    // ------------------------------------------------------
    case WAITING_FOR_USER:
      stopMotors();
      if (distFront > USER_CLEAR_DISTANCE) {
        if (compassValid) {
          targetHeading = lastValidHeading;
          headingLocked = true;
        }
        lastTurnEndTime = millis();
        agStartTime = millis();
        currentState = DRIVING;
        SerialBT.println("STATE:RESUMING");
      }
      break;

    // ------------------------------------------------------
    case LOWERING:
      if (millis() - actionStartTime >= 800) {
        currentState = READING;
        actionStartTime = millis();
      }
      break;

    // ------------------------------------------------------
    case READING:
      if (millis() - actionStartTime >= 1000) {
        readSoil();
        SerialBT.printf("MOISTURE:%d,EC:%d\n", moisturePercent, ecPercent);

        if (moisturePercent < MOISTURE_DRY_THRESHOLD) {
          digitalWrite(PUMP_PIN, HIGH);
          isWatering = true;
          currentState = WATERING;
          actionStartTime = millis();
          SerialBT.println("STATE:WATERING");
        } else {
          probeServo.write(SERVO_UP);
          currentState = RAISING;
          actionStartTime = millis();
          SerialBT.println("STATE:SOIL_OK");
        }
      }
      break;

    // ------------------------------------------------------
    case WATERING:
      if (millis() - actionStartTime >= 2500) {
        digitalWrite(PUMP_PIN, LOW);
        isWatering = false;
        probeServo.write(SERVO_UP);
        currentState = RAISING;
        actionStartTime = millis();
        SerialBT.println("STATE:WATERING_COMPLETE");
      }
      break;

    // ------------------------------------------------------
    case RAISING:
      if (millis() - actionStartTime >= 800) {
        lastTurnEndTime = millis();
        agStartTime = millis();
        if (compassValid) {
          targetHeading = lastValidHeading;
          headingLocked = true;
        }
        currentState = DRIVING;
        SerialBT.println("STATE:PROBE_UP");
      }
      break;
  }

  // Apply soft-start motor ramping every loop
  updateMotors();

  // ========================================================
  // GPS SMOOTHING (low-pass filter)
  // ========================================================
  if (gps.location.isValid()) {
    if (smoothedLat == 0.0) {
      smoothedLat = gps.location.lat();
      smoothedLng = gps.location.lng();
    } else {
      smoothedLat = smoothedLat * 0.9 + gps.location.lat() * 0.1;
      smoothedLng = smoothedLng * 0.9 + gps.location.lng() * 0.1;
    }
  }

  // ========================================================
  // TELEMETRY PACKET (every 250 ms)
  // ========================================================
  // CSV format:
  //   lat, lng, heading, front%, left%, right%, moisture%, pump%, ec%
  // ========================================================
  if (millis() - lastTransmit >= 250) {
    lastTransmit = millis();
    char navBuffer[128];
    snprintf(navBuffer, sizeof(navBuffer),
             "%.6f,%.6f,%d,%d,%d,%d,%d,%d,%d",
             smoothedLat, smoothedLng, lastValidHeading,
             frontPercent, leftPercent, rightPercent,
             moisturePercent, isWatering ? 100 : 0, ecPercent);
    SerialBT.println(navBuffer);
    Serial.println(navBuffer);
  }
}