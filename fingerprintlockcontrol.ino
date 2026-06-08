#include <Wire.h>
#include <HardwareSerial.h>
#include <DFRobot_ID809.h>

// ======================================================
// A4988 PIN SETUP
const int STEP_PIN = 2;
const int DIR_PIN  = 4;
const int EN_PIN   = 33;

// ======================================================
// SEN0590 DISTANCE SENSOR I2C
// ======================================================

const int I2C_SDA = 21;
const int I2C_SCL = 22;

#define address 0x74

uint8_t buf[2] = {0};
uint8_t dat = 0xB0;

// ======================================================
// FINGERPRINT SENSOR UART
// ======================================================

DFRobot_ID809 fingerprint;
HardwareSerial FPSerial(2);

// Sensor TX -> ESP32 GPIO16
// Sensor RX -> ESP32 GPIO17
#define FP_RX 16
#define FP_TX 17

// ======================================================
// DEADBOLT CALIBRATION
// ======================================================

// Measured positions
const int LOCKED_DISTANCE_MM   = 109;
const int UNLOCKED_DISTANCE_MM = 94;

// Keep small because positions are only 9 mm apart
const int STOP_MARGIN_MM = 1;

// ======================================================
// A4988 MOTOR SETTINGS
// ======================================================

// If direction is reversed, swap LOW and HIGH.
const bool DIR_LOCK   = LOW;   
const bool DIR_UNLOCK = HIGH;  

const int STEP_DELAY_US = 4000;

// Step limit in case of failure
const int MAX_STEPS_PER_MOVE = 500;

// Adaptive step chunks
const int LARGE_STEP_CHUNK = 10;
const int MEDIUM_STEP_CHUNK = 4;
const int SMALL_STEP_CHUNK = 1;

// Distance error bands
const int FAR_FROM_TARGET_MM = 5;
const int NEAR_TARGET_MM = 2;

// Prevent repeated toggles while finger is held/re-scanned
const unsigned long FINGERPRINT_COOLDOWN_MS = 2500;
unsigned long lastFingerprintActionTime = 0;

// ======================================================
// SERIAL CONTROL
// ======================================================
//
// l = lock
// u = unlock
// s = status
// c = continuous distance read
//

bool continuousRead = false;

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  disableStepper();

  Wire.begin(I2C_SDA, I2C_SCL);

  FPSerial.begin(115200, SERIAL_8N1, FP_RX, FP_TX);
  fingerprint.begin(FPSerial);

  Serial.println();
  Serial.println("Checking SEN0348 / ID809 fingerprint connection...");

  while (fingerprint.isConnected() == false) {
    Serial.println("Communication with fingerprint sensor failed. Check wiring and power.");
    delay(1000);
  }

  Serial.println("Fingerprint sensor connected.");

  fingerprint.ctrlLED(
    fingerprint.eBreathing,
    fingerprint.eLEDBlue,
    0
  );

  Serial.println();
  Serial.println("ESP32 Deadbolt Controller Ready");
  Serial.println("--------------------------------");
  Serial.println("Commands:");
  Serial.println("  l = lock");
  Serial.println("  u = unlock");
  Serial.println("  s = status");
  Serial.println("  c = continuous distance read");
  Serial.println("--------------------------------");
}

// ======================================================
// MAIN LOOP
// ======================================================

void loop() {
  handleSerialCommands();
  handleFingerprint();

  if (continuousRead) {
    printStatus();
    delay(300);
  }
}

// ======================================================
// SERIAL COMMANDS
// ======================================================

void handleSerialCommands() {
  if (!Serial.available()) {
    return;
  }

  char cmd = Serial.read();

  if (cmd == 'l' || cmd == 'L') {
    lockDeadbolt();
  } 
  else if (cmd == 'u' || cmd == 'U') {
    unlockDeadbolt();
  } 
  else if (cmd == 's' || cmd == 'S') {
    printStatus();
  } 
  else if (cmd == 'c' || cmd == 'C') {
    continuousRead = !continuousRead;
    Serial.print("Continuous read: ");
    Serial.println(continuousRead ? "ON" : "OFF");
  }
}

// ======================================================
// FINGERPRINT HANDLING
// ======================================================

void handleFingerprint() {
  uint8_t ret = 0;

  fingerprint.ctrlLED(
    fingerprint.eBreathing,
    fingerprint.eLEDBlue,
    0
  );

  // Only try capture if finger is detected
  if (!fingerprint.detectFinger()) {
    return;
  }

  if (millis() - lastFingerprintActionTime < FINGERPRINT_COOLDOWN_MS) {
    return;
  }

  Serial.println();
  Serial.println("Finger detected. Capturing...");

  if (fingerprint.collectionFingerprint(0) != ERR_ID809) {
    fingerprint.ctrlLED(
      fingerprint.eFastBlink,
      fingerprint.eLEDYellow,
      3
    );

    Serial.println("Capturing succeeded.");
    Serial.println("Please release your finger.");

    while (fingerprint.detectFinger()) {
      delay(10);
    }

    ret = fingerprint.search();

    if (ret != 0) {
      fingerprint.ctrlLED(
        fingerprint.eKeepsOn,
        fingerprint.eLEDGreen,
        0
      );

      Serial.print("Fingerprint match succeeded, ID = ");
      Serial.println(ret);

      lastFingerprintActionTime = millis();

      toggleDeadbolt();

    } else {
      fingerprint.ctrlLED(
        fingerprint.eKeepsOn,
        fingerprint.eLEDRed,
        0
      );

      Serial.println("Fingerprint match failed.");
    }
  } else {
    Serial.println("Fingerprint capture failed.");
  }

  Serial.println("-----------------------------");
  delay(500);
}

// ======================================================
// TOGGLE LOGIC
// ======================================================

void toggleDeadbolt() {
  int distance = readDistanceFilteredMM();

  Serial.print("Current distance before toggle: ");
  Serial.print(distance);
  Serial.println(" mm");

  if (distance <= 0) {
    Serial.println("Distance read failed. Not moving lock.");
    return;
  }

  if (isLockedDistance(distance)) {
    Serial.println("Currently locked. Unlocking...");
    unlockDeadbolt();
  } 
  else if (isUnlockedDistance(distance)) {
    Serial.println("Currently unlocked. Locking...");
    lockDeadbolt();
  } 
  else {
    int midpoint = (LOCKED_DISTANCE_MM + UNLOCKED_DISTANCE_MM) / 2;

    if (distance >= midpoint) {
      Serial.println("Bolt is between positions but closer to locked. Unlocking...");
      unlockDeadbolt();
    } else {
      Serial.println("Bolt is between positions but closer to unlocked. Locking...");
      lockDeadbolt();
    }
  }
}

// ======================================================
// HIGH-LEVEL LOCK FUNCTIONS
// ======================================================

void lockDeadbolt() {
  Serial.println();
  Serial.println("Lock command received.");

  if (isLocked()) {
    Serial.println("Already locked.");
    return;
  }

  moveUntilLocked();
}

void unlockDeadbolt() {
  Serial.println();
  Serial.println("Unlock command received.");

  if (isUnlocked()) {
    Serial.println("Already unlocked.");
    return;
  }

  moveUntilUnlocked();
}

// ======================================================
// ADAPTIVE STEP LOGIC
// ======================================================

int getAdaptiveStepChunk(int currentDistance, int targetDistance) {
  int error = abs(targetDistance - currentDistance);

  if (error > FAR_FROM_TARGET_MM) {
    return LARGE_STEP_CHUNK;
  } 
  else if (error > NEAR_TARGET_MM) {
    return MEDIUM_STEP_CHUNK;
  } 
  else {
    return SMALL_STEP_CHUNK;
  }
}

void stepMotor(bool direction, int steps, int usDelay) {
  digitalWrite(DIR_PIN, direction);

  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(usDelay);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(usDelay);
  }
}

// ======================================================
// LOCK / UNLOCK MOVEMENT
// ======================================================

void moveUntilLocked() {
  Serial.println("Moving toward LOCKED...");
  Serial.print("Target locked distance: ");
  Serial.print(LOCKED_DISTANCE_MM);
  Serial.println(" mm");

  enableStepper();

  int totalSteps = 0;
  int lastDistance = -1;

  while (totalSteps < MAX_STEPS_PER_MOVE) {
    int distance = readDistanceFilteredMM();

    if (distance <= 0) {
      Serial.println("Distance read error while locking.");
      continue;
    }

    lastDistance = distance;

    Serial.print("Locking distance: ");
    Serial.print(distance);
    Serial.print(" mm, total steps: ");
    Serial.println(totalSteps);

    // Locked distance is greater
    if (distance >= LOCKED_DISTANCE_MM - STOP_MARGIN_MM) {
      disableStepper();

      Serial.println();
      Serial.println("Reached LOCKED position.");
      Serial.print("Final distance: ");
      Serial.print(distance);
      Serial.println(" mm");
      Serial.print("Total steps: ");
      Serial.println(totalSteps);

      return;
    }

    int stepsThisMove = getAdaptiveStepChunk(distance, LOCKED_DISTANCE_MM);

    if (totalSteps + stepsThisMove > MAX_STEPS_PER_MOVE) {
      stepsThisMove = MAX_STEPS_PER_MOVE - totalSteps;
    }

    stepMotor(DIR_LOCK, stepsThisMove, STEP_DELAY_US);
    totalSteps += stepsThisMove;
  }

  disableStepper();

  Serial.println();
  Serial.println("ERROR: lock stopped by max step limit.");
  Serial.print("Last valid distance: ");
  Serial.print(lastDistance);
  Serial.println(" mm");
}

void moveUntilUnlocked() {
  Serial.println("Moving toward UNLOCKED...");
  Serial.print("Target unlocked distance: ");
  Serial.print(UNLOCKED_DISTANCE_MM);
  Serial.println(" mm");

  enableStepper();

  int totalSteps = 0;
  int lastDistance = -1;

  while (totalSteps < MAX_STEPS_PER_MOVE) {
    int distance = readDistanceFilteredMM();

    if (distance <= 0) {
      Serial.println("Distance read error while unlocking.");
      continue;
    }

    lastDistance = distance;

    Serial.print("Unlocking distance: ");
    Serial.print(distance);
    Serial.print(" mm, total steps: ");
    Serial.println(totalSteps);

    // Unlocked distance is smaller
    if (distance <= UNLOCKED_DISTANCE_MM + STOP_MARGIN_MM) {
      disableStepper();

      Serial.println();
      Serial.println("Reached UNLOCKED position.");
      Serial.print("Final distance: ");
      Serial.print(distance);
      Serial.println(" mm");
      Serial.print("Total steps: ");
      Serial.println(totalSteps);

      return;
    }

    int stepsThisMove = getAdaptiveStepChunk(distance, UNLOCKED_DISTANCE_MM);

    if (totalSteps + stepsThisMove > MAX_STEPS_PER_MOVE) {
      stepsThisMove = MAX_STEPS_PER_MOVE - totalSteps;
    }

    stepMotor(DIR_UNLOCK, stepsThisMove, STEP_DELAY_US);
    totalSteps += stepsThisMove;
  }

  disableStepper();

  Serial.println();
  Serial.println("ERROR: unlock stopped by max step limit.");
  Serial.print("Last valid distance: ");
  Serial.print(lastDistance);
  Serial.println(" mm");
}

// ======================================================
// A4988 ENABLE FUNCTIONS
// ======================================================

void enableStepper() {
  digitalWrite(EN_PIN, LOW);   // A4988 enabled
  delay(5);
}

void disableStepper() {
  digitalWrite(EN_PIN, HIGH);  // A4988 disabled
}

// ======================================================
// SEN0590 DISTANCE FUNCTIONS
// ======================================================

int readDistanceMM() {
  bool ok = writeReg(0x10, &dat, 1);

  if (!ok) {
    return -1;
  }

  delay(25);

  uint8_t bytesRead = readReg(0x02, buf, 2);

  if (bytesRead != 2) {
    return -1;
  }

  int distance = buf[0] * 0x100 + buf[1] + 10;

  return distance;
}

int readDistanceFilteredMM() {
  const int samples = 3;
  int values[samples];
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    int d = readDistanceMM();

    if (d > 0 && d < 4000) {
      values[valid] = d;
      valid++;
    }

    delay(5);
  }

  if (valid == 0) {
    return -1;
  }

  for (int i = 0; i < valid - 1; i++) {
    for (int j = i + 1; j < valid; j++) {
      if (values[j] < values[i]) {
        int temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
    }
  }

  return values[valid / 2];
}

uint8_t readReg(uint8_t reg, const void* pBuf, size_t size) {
  if (pBuf == NULL) {
    Serial.println("pBuf ERROR!! : null pointer");
    return 0;
  }

  uint8_t* _pBuf = (uint8_t*)pBuf;

  Wire.beginTransmission(address);
  Wire.write(&reg, 1);

  if (Wire.endTransmission() != 0) {
    return 0;
  }

  delay(5);

  Wire.requestFrom(address, (uint8_t)size);

  if (Wire.available() < size) {
    return 0;
  }

  for (uint16_t i = 0; i < size; i++) {
    _pBuf[i] = Wire.read();
  }

  return size;
}

bool writeReg(uint8_t reg, const void* pBuf, size_t size) {
  if (pBuf == NULL) {
    Serial.println("pBuf ERROR!! : null pointer");
    return false;
  }

  uint8_t* _pBuf = (uint8_t*)pBuf;

  Wire.beginTransmission(address);
  Wire.write(&reg, 1);

  for (uint16_t i = 0; i < size; i++) {
    Wire.write(_pBuf[i]);
  }

  if (Wire.endTransmission() != 0) {
    return false;
  } else {
    return true;
  }
}

// ======================================================
// POSITION CHECKS
// ======================================================

bool isLockedDistance(int d) {
  return d >= LOCKED_DISTANCE_MM - STOP_MARGIN_MM;
}

bool isUnlockedDistance(int d) {
  return d <= UNLOCKED_DISTANCE_MM + STOP_MARGIN_MM;
}

bool isLocked() {
  int d = readDistanceFilteredMM();

  if (d <= 0) {
    return false;
  }

  return isLockedDistance(d);
}

bool isUnlocked() {
  int d = readDistanceFilteredMM();

  if (d <= 0) {
    return false;
  }

  return isUnlockedDistance(d);
}

// ======================================================
// STATUS
// ======================================================

void printStatus() {
  int distance = readDistanceFilteredMM();

  Serial.println();
  Serial.println("----- STATUS -----");

  Serial.print("Distance: ");
  if (distance > 0) {
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    Serial.println("sensor read error");
  }

  Serial.print("Bolt state: ");

  if (distance <= 0) {
    Serial.println("unknown");
  } 
  else if (isLockedDistance(distance)) {
    Serial.println("locked");
  } 
  else if (isUnlockedDistance(distance)) {
    Serial.println("unlocked");
  } 
  else {
    Serial.println("between positions");
  }

  Serial.println("------------------");
}