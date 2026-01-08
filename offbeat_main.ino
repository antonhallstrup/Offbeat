#include <ArduinoBLE.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <Arduino_BMI270_BMM150.h>  // IMU for Nano 33 BLE Rev2


// ---------- DEBUG TOGGLES ----------
bool DEBUG_RX  = false;   // BLE receive debug
bool DEBUG_NAV = false;   // Navigation / haptics debug
bool DEBUG_LDR = false;   // LDR / light-level debug

// ---------- CONFIG ----------

// PCA9548A I2C multiplexer address
#define MUX_ADDRESS 0x70

// BLE service and characteristic for nav data
BLEService navService("6E400001-B5A3-F393-E0A9-E50E24DCCA9F");
BLEStringCharacteristic bearingChar(
  "6E400002-B5A3-F393-E0A9-E50E24DCCA9F",
  BLEWriteWithoutResponse | BLENotify | BLERead,
  50
);

// Defining MUX channels for the 5 LRA's
const uint8_t LRA_MUX_CHANNELS[5] = {
  6, // Far left
  0, // Slight left
  7, // Straight ahead
  1, // Slight right
  2  // Far right
};

const char* LRA_LABELS[5] = {
  "far left",
  "slight left",
  "straight",
  "slight right",
  "far right"
};

Adafruit_DRV2605 drv[5];

// ---------- RAMP CONFIG & STATE ----------

enum RampStage { RAMP_IDLE, RAMP_UP, RAMP_CLICK, RAMP_DOWN };

RampStage rampStage           = RAMP_IDLE;
unsigned long lastRampMs      = 0;
int rampStepIndex             = 0;
int clickIndex                = 0;
uint8_t currentRampLraIndex   = 0;
uint8_t currentRampMuxChannel = 8;
uint8_t currentRampLevel      = 0;

// ramp resolution
const int RAMP_STEPS_UP   = 16;
const int RAMP_STEPS_DOWN = 16;

// maximum ramp level (0–127)
uint8_t rampMaxLevel = 35;

// click at the peak
const uint8_t  clickHigh   = 127;
const uint8_t  clickLow    = 80;
const unsigned long clickOnMs  = 20;
const unsigned long clickOffMs = 20;

// global control
bool paused      = false;
bool dirCalMode  = false;

// debug printing
unsigned long lastDebugMs = 0;

float debugStraightHalf = 0.0f;

// ---------- ARRIVAL MODE (all LRAs moving toward center) ----------
enum CloseStage { CLOSE_IDLE, CLOSE_G1, CLOSE_G2, CLOSE_CENTER };
CloseStage closeStage       = CLOSE_IDLE;
unsigned long lastCloseMs   = 0;
const unsigned long CLOSE_STEP_MS = 600;  // ms between arrival steps

// ---------- IMU / HEADING STATE ----------

float yawGyro = 0.0f;           // integrated yaw in degrees from gyro
float yawOffset = 0.0f;         // reference yaw set by direction calibration
unsigned long lastYawUpdateMs = 0;

float adjustedHeading   = 0.0f;
float smoothHeading     = 0.0f;
bool  smoothInitialized = false;
float headingFromPhone  = 0.0f;
bool  usePhoneHeading   = true;    // false = Nano IMU, true = phone heading

// ---------- NAV INPUT STATE (FROM PHONE) ----------

float bearingFromPhone   = 0.0f;   // destination bearing (deg)
float distanceFromPhone  = 0.0f;   // distance (m; we normalize to meters)

// Upside-down clipping flag from app
bool upsideDown = false;          // false = normal, true = upside down

// ---------- LDR GATING ----------

int ldrPin       = A0;
int ldrValue     = 0;
int ldrThreshold = 500;
bool ldrDark     = false;
bool lastLdrDark = false;  // for edge detection when touch begins

String serialCmdBuf;

// ---------- HAPTIC MAPPING STRUCTURES ----------


struct HapticCommand {
  uint8_t lraIndex;       // 1..5, 0 = off
  unsigned long cycleMs;  // click-to-click time
};

// Wrap angle to [-180, 180]
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Map bearing/heading/distance -> which LRA + tempo
HapticCommand computeHaptics(float bearingDeg,
                             float headingDeg,
                             float distanceMeters,
                             bool upsideDownFlag) {
  HapticCommand cmd;

  float diff = wrap180(bearingDeg - headingDeg);
  uint8_t idx = 0;
  // --- Dynamic forward-facing 180° (diff in [-90°, +90°]) ---
  if (diff >= -90.0f && diff <= 90.0f) {
    // Distance-dependent straight zone:
    // - Closer = narrower straight zone
    // - Farther = wider straight zone
    //
    // Tunable parameters:
    const float STRAIGHT_HALF_NEAR   = 7.5f;   // half-width (deg) when very close
    const float STRAIGHT_HALF_FAR    = 15.0f;  // half-width (deg) when far away
    const float STRAIGHT_SCALE_DIST  = 400.0f; // distance in meters over which it interpolates
    const float SLIGHT_EDGE_OUTER    = 45.0f;  // where "slight" ends and "far" begins
    const float FORWARD_EDGE_OUTER   = 90.0f;  // limit of forward cone

    float d = distanceMeters;
    if (d < 0.0f) d = 0.0f;
    if (d > STRAIGHT_SCALE_DIST) d = STRAIGHT_SCALE_DIST;

    float t = d / STRAIGHT_SCALE_DIST;
    float straightHalf = STRAIGHT_HALF_NEAR +
                         (STRAIGHT_HALF_FAR - STRAIGHT_HALF_NEAR) * t;

    debugStraightHalf = straightHalf;

    // Define dynamic boundaries
    float straightMin      = -straightHalf;
    float straightMax      =  straightHalf;
    float slightLeftMin    = -SLIGHT_EDGE_OUTER;
    float slightLeftMax    =  straightMin;
    float slightRightMin   =  straightMax;
    float slightRightMax   =  SLIGHT_EDGE_OUTER;
    float farLeftMin       = -FORWARD_EDGE_OUTER;
    float farLeftMax       =  slightLeftMin;
    float farRightMin      =  slightRightMax;
    float farRightMax      =  FORWARD_EDGE_OUTER;

    // Decide which bucket diff falls into
    if (diff >= farLeftMin && diff < farLeftMax) {
      idx = 1; // far left
    } else if (diff >= slightLeftMin && diff < slightLeftMax) {
      idx = 2; // slight left
    } else if (diff >= straightMin && diff <= straightMax) {
      idx = 3; // straight
    } else if (diff > slightRightMin && diff <= slightRightMax) {
      idx = 4; // slight right
    } else if (diff > farRightMin && diff <= farRightMax) {
      idx = 5; // far right
    } else {
      // Edge-case fallback, shouldn't normally happen
      idx = (diff < 0.0f) ? 1 : 5;
    }
  } else {
    // --- Backward-facing 180°: split evenly between LRA 1 and 5 ---
    idx = (diff < 0.0f) ? 1 : 5;
  }

  // Flip left/right when device is upside down
  if (upsideDownFlag) {
    idx = 6 - idx; // 1<->5, 2<->4, 3 stays 3
  }

  cmd.lraIndex = idx;

  // 2) Distance → tempo (click-to-click ms)
  float d = distanceMeters;

  // Clamp distance into a sensible navigation range: 0–4000 m
  if (d < 0.0f)      d = 0.0f;
  if (d > 4000.0f)   d = 4000.0f;

  // 1:1 mapping: 1 meter = 1 millisecond,
  // with a minimum of 700 ms and a maximum of 4000 ms.
  float c = d;
  if (c < 700.0f)   c = 700.0f;    // fastest allowed tempo
  if (c > 4000.0f)  c = 4000.0f;   // slowest allowed tempo

  cmd.cycleMs = (unsigned long)c;
  return cmd;
}

// ---------- I2C MUX HELPERS ----------

void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDRESS);
  if (channel > 7) {
    Wire.write(0x00);           // deselect all
  } else {
    Wire.write(1 << channel);
  }
  Wire.endTransmission();
}

// Hard stop all motors and clear mux
void stopAllMotors() {
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t muxCh = LRA_MUX_CHANNELS[i];
    selectMuxChannel(muxCh);
    drv[i].useLRA();
    drv[i].setMode(DRV2605_MODE_REALTIME);
    drv[i].setRealtimeValue(0);
    drv[i].setMode(DRV2605_MODE_INTTRIG);
    drv[i].stop();
  }
  selectMuxChannel(8); // deselect all
}

// Reset ramp state machine
void resetRampState() {
  rampStage             = RAMP_IDLE;
  rampStepIndex         = 0;
  clickIndex            = 0;
  currentRampLevel      = 0;
  currentRampLraIndex   = 0;
  currentRampMuxChannel = 8;
}

// ---------- DRV INIT ----------

bool initDrv(uint8_t idx) {
  uint8_t ch = LRA_MUX_CHANNELS[idx];
  selectMuxChannel(ch);
  if (!drv[idx].begin()) return false;
  drv[idx].selectLibrary(1);
  drv[idx].useLRA();
  drv[idx].setMode(DRV2605_MODE_INTTRIG);
  drv[idx].stop();
  return true;
}

// Play a click effect on a single LRA index using INTTRIG
void playClickOnLRA(uint8_t idx) {
  if (idx < 1 || idx > 5) return;
  uint8_t ch = LRA_MUX_CHANNELS[idx - 1];
  selectMuxChannel(ch);
  drv[idx - 1].selectLibrary(1);
  drv[idx - 1].useLRA();
  drv[idx - 1].setMode(DRV2605_MODE_INTTRIG);
  drv[idx - 1].setWaveform(0, 86); // strong click effect
  drv[idx - 1].setWaveform(1, 0);
  drv[idx - 1].go();
}

// Trigger the grouped pattern for close arrival
void triggerCloseGroup(CloseStage stage) {
  switch (stage) {
    case CLOSE_G1:
      // outer pair: 1 + 5
      playClickOnLRA(1);
      playClickOnLRA(5);
      break;
    case CLOSE_G2:
      // inner pair: 2 + 4
      playClickOnLRA(2);
      playClickOnLRA(4);
      break;
    case CLOSE_CENTER:
      // center: 3
      playClickOnLRA(3);
      break;
    default:
      break;
  }
}

// Non-blocking update for close arrival mode
void updateCloseArrival() {
  unsigned long now = millis();

  if (closeStage == CLOSE_IDLE) {
    // Start at outer pair
    closeStage    = CLOSE_G1;
    lastCloseMs   = now;
    triggerCloseGroup(closeStage);
    return;
  }

  if (now - lastCloseMs < CLOSE_STEP_MS) {
    return;
  }

  lastCloseMs = now;

  switch (closeStage) {
    case CLOSE_G1:
      closeStage = CLOSE_G2;
      break;
    case CLOSE_G2:
      closeStage = CLOSE_CENTER;
      break;
    case CLOSE_CENTER:
    default:
      closeStage = CLOSE_G1;
      break;
  }

  triggerCloseGroup(closeStage);
}

// ---------- RAMP ENGINE  ----------
// desiredLraIndex: 1..5, 0 = no vibration
// cycleMsIn: time between click peaks (full cycle)

void updateRamp(uint8_t desiredLraIndex, unsigned long cycleMsIn) {
  unsigned long now = millis();

  // No direction -> shut down and idle
  if (desiredLraIndex == 0) {
    if (rampStage != RAMP_IDLE) {
      stopAllMotors();
      resetRampState();
    }
    return;
  }

  // Interpret cycleMsIn as click-to-click period
  unsigned long cycle = cycleMsIn;
  if (cycle < 100)   cycle = 100;
  if (cycle > 60000) cycle = 60000;

  // Total explicit click time (two highs + two lows)
  const unsigned long totalClickMs =
    2UL * clickOnMs + 2UL * clickOffMs; // currently 80 ms

  unsigned long remainingMs;
  if (cycle > totalClickMs) {
    remainingMs = cycle - totalClickMs;
  } else {
    remainingMs = 0;
  }

  // Split remaining time evenly: half down, half up
  unsigned long downTimeMs = remainingMs / 2;
  unsigned long upTimeMs   = remainingMs - downTimeMs;

  // Ensure we have at least one "tick" per step
  if (upTimeMs   < (unsigned long)RAMP_STEPS_UP)   upTimeMs   = RAMP_STEPS_UP;
  if (downTimeMs < (unsigned long)RAMP_STEPS_DOWN) downTimeMs = RAMP_STEPS_DOWN;

  unsigned long stepDelayUp   = upTimeMs   / RAMP_STEPS_UP;
  unsigned long stepDelayDown = downTimeMs / RAMP_STEPS_DOWN;

  if (stepDelayUp   < 1) stepDelayUp   = 1;
  if (stepDelayDown < 1) stepDelayDown = 1;

  // First activation
  if (rampStage == RAMP_IDLE) {
    currentRampLraIndex   = desiredLraIndex;
    currentRampMuxChannel = LRA_MUX_CHANNELS[currentRampLraIndex - 1];

    selectMuxChannel(currentRampMuxChannel);
    drv[currentRampLraIndex - 1].useLRA();
    drv[currentRampLraIndex - 1].setMode(DRV2605_MODE_REALTIME);
    drv[currentRampLraIndex - 1].setRealtimeValue(0);

    currentRampLevel = 0;
    rampStage        = RAMP_UP;
    rampStepIndex    = 0;
    clickIndex       = 0;
    lastRampMs       = now;
    return;
  }

  // If direction changes mid-ramp, hand off smoothly
  if (desiredLraIndex != currentRampLraIndex) {
    // Turn off old one
    if (currentRampLraIndex >= 1 && currentRampLraIndex <= 5) {
      uint8_t oldMux = LRA_MUX_CHANNELS[currentRampLraIndex - 1];
      selectMuxChannel(oldMux);
      drv[currentRampLraIndex - 1].setMode(DRV2605_MODE_REALTIME);
      drv[currentRampLraIndex - 1].setRealtimeValue(0);
    }

    // Switch to new one, keeping currentRampLevel
    currentRampLraIndex   = desiredLraIndex;
    currentRampMuxChannel = LRA_MUX_CHANNELS[currentRampLraIndex - 1];
    selectMuxChannel(currentRampMuxChannel);
    drv[currentRampLraIndex - 1].useLRA();
    drv[currentRampLraIndex - 1].setMode(DRV2605_MODE_REALTIME);
    drv[currentRampLraIndex - 1].setRealtimeValue(currentRampLevel);
  }

  switch (rampStage) {
    case RAMP_UP: {
      if (now - lastRampMs >= stepDelayUp) {
        lastRampMs = now;
        float t = (float)rampStepIndex / (float)(RAMP_STEPS_UP - 1); // 0..1
        float env = t * t; // exponential-ish
        int level = (int)(env * rampMaxLevel + 0.5f);
        if (level < 0) level = 0;
        if (level > 127) level = 127;

        currentRampLevel = (uint8_t)level;
        selectMuxChannel(currentRampMuxChannel);
        drv[currentRampLraIndex - 1].setRealtimeValue(currentRampLevel);

        rampStepIndex++;
        if (rampStepIndex >= RAMP_STEPS_UP) {
          rampStage  = RAMP_CLICK;
          clickIndex = 0;
          currentRampLevel = clickHigh;
          selectMuxChannel(currentRampMuxChannel);
          drv[currentRampLraIndex - 1].setRealtimeValue(currentRampLevel);
          lastRampMs = now;
        }
      }
      break;
    }

    case RAMP_CLICK: {
      const uint8_t  clickLevels[4]     = { clickHigh, clickLow, clickHigh, clickLow };
      const unsigned long clickTimes[4] = { clickOnMs, clickOffMs, clickOnMs, clickOffMs };

      if (now - lastRampMs >= clickTimes[clickIndex]) {
        lastRampMs = now;
        clickIndex++;
        if (clickIndex >= 4) {
          rampStage     = RAMP_DOWN;
          rampStepIndex = 0;
        } else {
          currentRampLevel = clickLevels[clickIndex];
          selectMuxChannel(currentRampMuxChannel);
          drv[currentRampLraIndex - 1].setRealtimeValue(currentRampLevel);
        }
      }
      break;
    }

    case RAMP_DOWN: {
      if (now - lastRampMs >= stepDelayDown) {
        lastRampMs = now;
        float t = (float)rampStepIndex / (float)(RAMP_STEPS_DOWN - 1); // 0..1
        float env = (1.0f - t);
        env = env * env; // ease-out
        int level = (int)(env * rampMaxLevel + 0.5f);
        if (level < 0) level = 0;
        if (level > 127) level = 127;

        currentRampLevel = (uint8_t)level;
        selectMuxChannel(currentRampMuxChannel);
        drv[currentRampLraIndex - 1].setRealtimeValue(currentRampLevel);

        rampStepIndex++;
        if (rampStepIndex >= RAMP_STEPS_DOWN) {
          // cycle finished -> restart UP at 0
          currentRampLevel = 0;
          selectMuxChannel(currentRampMuxChannel);
          drv[currentRampLraIndex - 1].setRealtimeValue(currentRampLevel);
          rampStage     = RAMP_UP;
          rampStepIndex = 0;
          lastRampMs    = now;
        }
      }
      break;
    }

    case RAMP_IDLE:
    default:
      resetRampState();
      break;
  }
}

// ---------- SETUP & MAIN LOOP ----------

void setup() {
  Serial.begin(115200);
  pinMode(ldrPin, INPUT);

  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart < 1500)) {
    ;
  }

  Serial.println("Heading source default: PHONE");

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  Wire.begin();

  // Deselect all mux channels
  selectMuxChannel(8);

  // Initialize DRV2605 drivers
  for (uint8_t i = 0; i < 5; i++) {
    if (!initDrv(i)) {
      Serial.print("Failed to initialize DRV2605 for LRA ");
      Serial.println(i + 1);
    } else {
      Serial.print("DRV2605 ready for LRA ");
      Serial.print(i + 1);
      Serial.print(" on mux channel ");
      Serial.println(LRA_MUX_CHANNELS[i]);
    }
  }
  selectMuxChannel(8); // deselect all

  BLE.setLocalName("NanoBLE");
  BLE.setAdvertisedService(navService);
  navService.addCharacteristic(bearingChar);
  BLE.addService(navService);
  BLE.advertise();
  Serial.println("Waiting for BLE connection...");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
      Serial.print("Connected to central: ");
      Serial.println(central.address());

    while (central.connected()) {
      // --- Serial commands (debug) ---
      if (Serial.available()) {
        char input = Serial.read();
        if (input == 'p' || input == 'P') {
          paused = !paused;
          if (paused) {
            stopAllMotors();
            resetRampState();
            Serial.println("Vibration PAUSED");
          } else {
            Serial.println("Vibration RESUMED");
          }
        }
        if (input == 'h' || input == 'H') {
          usePhoneHeading = !usePhoneHeading;
          Serial.print("Heading source: ");
          Serial.println(usePhoneHeading ? "PHONE" : "NANO");
        }
        if (input == 'd' || input == 'D') {
          DEBUG_RX  = !DEBUG_RX;
          DEBUG_NAV = !DEBUG_NAV;
          Serial.print("Debug mode ");
          Serial.println(DEBUG_NAV ? "ON" : "OFF");
        }

        // Accumulate word-based commands like "ldr"
        if (input == '\n' || input == '\r') {
          if (serialCmdBuf.length() > 0) {
            serialCmdBuf.trim();
            if (serialCmdBuf.equalsIgnoreCase("ldr")) {
              DEBUG_LDR = !DEBUG_LDR;
              Serial.print("LDR debug ");
              Serial.println(DEBUG_LDR ? "ON" : "OFF");
            } else if (DEBUG_NAV) {
              Serial.print("Unknown command: ");
              Serial.println(serialCmdBuf);
            }
            serialCmdBuf = "";
          }
        } else {
          serialCmdBuf += input;
        }
      }

      // --- BLE: read bearing / distance / heading from phone ---
      if (bearingChar.written()) {
        String bearingStr = bearingChar.value();  // e.g., "73.5,2.1,20.0,180.0"
        bearingStr.trim();

        if (bearingStr.equals("CAL_START")) {
          dirCalMode = true;
          Serial.println("Direction calibration STARTED. Face your desired forward direction.");
          continue;
        }

        if (bearingStr.equals("CAL_END")) {
          yawOffset = yawGyro;          // this direction becomes 0°
          smoothInitialized = false;    // reset smoothing
          Serial.print("Direction calibration ENDED. New yawOffset: ");
          Serial.println(yawOffset);
          dirCalMode = false;
          continue;
        }

        // Remote soft reset from app
        if (bearingStr.equals("RESET")) {
          Serial.println("Remote RESET requested – rebooting Nano...");
          delay(50);
          NVIC_SystemReset();
        }

        // Orientation commands from app
        if (bearingStr.equals("UPSIDE_NORMAL")) {
          upsideDown = false;
          Serial.println("Orientation set: NORMAL (left/right as wired).");
          continue;
        }
        if (bearingStr.equals("UPSIDE_DOWN") || bearingStr.equals("USPIDE_DOWN")) {
          upsideDown = true;
          Serial.println("Orientation set: UPSIDE DOWN (LRA order flipped).");
          continue;
        }

        // Heading source commands from app
        if (bearingStr.equals("HEAD_PHONE")) {
          usePhoneHeading = true;
          Serial.println("Heading source set: PHONE (from app).");
          continue;
        }
        if (bearingStr.equals("HEAD_NANO")) {
          usePhoneHeading = false;
          Serial.println("Heading source set: NANO (onboard IMU).");
          continue;
        }

        int firstComma  = bearingStr.indexOf(',');
        int secondComma = bearingStr.indexOf(',', firstComma + 1);
        int thirdComma  = bearingStr.indexOf(',', secondComma + 1);

        if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
          // Format: bearing,declination,distance,heading
          String bearingPart     = bearingStr.substring(0, firstComma);
          String _d              = bearingStr.substring(firstComma + 1, secondComma);
          String distancePart    = bearingStr.substring(secondComma + 1, thirdComma);
          String headingPart     = bearingStr.substring(thirdComma + 1);

          bearingFromPhone    = bearingPart.toFloat();
          distanceFromPhone   = distancePart.toFloat();
          headingFromPhone    = headingPart.toFloat();
        } else if (firstComma != -1 && secondComma != -1) {
          // Format: bearing,declination,distance
          String bearingPart     = bearingStr.substring(0, firstComma);
          String _d              = bearingStr.substring(firstComma + 1, secondComma);
          String distancePart    = bearingStr.substring(secondComma + 1);

          bearingFromPhone    = bearingPart.toFloat();
          distanceFromPhone   = distancePart.toFloat();
        } else if (firstComma != -1) {
          // Format: bearing,declination
          String bearingPart     = bearingStr.substring(0, firstComma);
          String _d              = bearingStr.substring(firstComma + 1);

          bearingFromPhone    = bearingPart.toFloat();
          distanceFromPhone   = 0.0f;
        } else {
          // Format: bearing only
          bearingFromPhone    = bearingStr.toFloat();
          distanceFromPhone   = 0.0f;
        }

        if (DEBUG_RX) {
          Serial.println("\n-------------------------------");
          Serial.print("Received: bearing=");
          Serial.print(bearingFromPhone);
          Serial.print("°, distanceRaw=");
          Serial.print(distanceFromPhone);
          Serial.println(" (as sent by phone)");
        }
      }

      // --- Update gyro-based yaw (tilt-compensated) ---
      float gx, gy, gz;
      float ax, ay, az;
      if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        IMU.readAcceleration(ax, ay, az);

        unsigned long now = millis();
        if (lastYawUpdateMs == 0) lastYawUpdateMs = now;
        float dt = (now - lastYawUpdateMs) / 1000.0f;
        lastYawUpdateMs = now;

        float anorm = sqrt(ax * ax + ay * ay + az * az);
        float yawRate;
        if (anorm > 0.5f && anorm < 1.5f) {
          ax /= anorm;
          ay /= anorm;
          az /= anorm;

          float upx = -ax;
          float upy = -ay;
          float upz = -az;

          yawRate = gx * upx + gy * upy + gz * upz;
        } else {
          yawRate = gz;
        }

        yawGyro += yawRate * dt;
        while (yawGyro < 0) yawGyro += 360.0f;
        while (yawGyro >= 360.0f) yawGyro -= 360.0f;
      }

      // --- LDR gating ---
      ldrValue = analogRead(ldrPin);
      ldrDark  = (ldrValue < ldrThreshold);
      if (DEBUG_LDR) {
        static unsigned long lastLdrPrintMs = 0;
        unsigned long nowL = millis();
        if (nowL - lastLdrPrintMs >= 200) {  // print every 200 ms
          lastLdrPrintMs = nowL;
          Serial.print("LDR DEBUG: value=");
          Serial.print(ldrValue);
          Serial.print(" (threshold=");
          Serial.print(ldrThreshold);
          Serial.print(") -> ");
          Serial.println(ldrDark ? "DARK" : "LIGHT");
        }
      }

      // Edge detection on LDR state
      if (ldrDark && !lastLdrDark) {
        // Transition: light -> dark (user covers sensor / touches device)
        // Tell the phone to start continuous updates
        bearingChar.writeValue("REQ_ON");
        if (DEBUG_RX) {
          Serial.println("Sent REQ_ON to phone (LDR went dark)");
        }
      } else if (!ldrDark && lastLdrDark) {
        // Transition: dark -> light (user lets go)
        // Tell the phone to stop continuous updates
        bearingChar.writeValue("REQ_OFF");
        if (DEBUG_RX) {
          Serial.println("Sent REQ_OFF to phone (LDR went light)");
        }
      }

      lastLdrDark = ldrDark;

      // --- Heading & haptic mapping ---
      float rawHeading;
      if (usePhoneHeading) {
        rawHeading = headingFromPhone;
      } else {
        rawHeading = yawGyro - yawOffset;
      }
      while (rawHeading < 0)      rawHeading += 360.0f;
      while (rawHeading >= 360.0f) rawHeading -= 360.0f;

      if (!smoothInitialized) {
        smoothHeading = rawHeading;
        smoothInitialized = true;
      } else {
        smoothHeading = 0.7f * smoothHeading + 0.3f * rawHeading;
      }
      adjustedHeading = smoothHeading;

      float dMeters = distanceFromPhone;
      if (dMeters < 0.0f) {
        dMeters = 0.0f;
      }

      // LDR gating: only run navigation + haptics when the sensor is dark (device covered/touched)
      if (!paused && !dirCalMode && ldrDark) {
        if (dMeters <= 40.0f) {
          if (rampStage != RAMP_IDLE) {
            stopAllMotors();
            resetRampState();
          }
          updateCloseArrival();
        } else {
          closeStage = CLOSE_IDLE;
          // Compute which LRA + tempo from nav state
          HapticCommand cmd = computeHaptics(
            bearingFromPhone,
            adjustedHeading,
            dMeters,
            upsideDown
          );

          updateRamp(cmd.lraIndex, cmd.cycleMs);

          // Debug print every 500 ms (only if debug enabled)
          unsigned long nowDbg = millis();
          if (DEBUG_NAV && (nowDbg - lastDebugMs >= 500)) {
            lastDebugMs = nowDbg;

            Serial.print("NAV DEBUG: bearing=");
            Serial.print(bearingFromPhone, 1);
            Serial.print("°, heading=");
            Serial.print(adjustedHeading, 1);
            Serial.print("°, distMeters=");
            Serial.print(dMeters, 1);
            Serial.print(" m, upsideDown=");
            Serial.print(upsideDown ? "true" : "false");
            Serial.print("  -> LRA=");
            Serial.print(cmd.lraIndex);
            Serial.print(" (");
            Serial.print(LRA_LABELS[cmd.lraIndex - 1]);
            Serial.print("), cycleMs=");
            Serial.print(cmd.cycleMs);
            Serial.print(" ms, straightHalf=");
            Serial.print(debugStraightHalf, 1);
            Serial.println("°");
          }
        }
      } else {
        // Conditions not met → ensure motors are off & ramp reset
        if (rampStage != RAMP_IDLE) {
          stopAllMotors();
          resetRampState();
        }
        closeStage = CLOSE_IDLE;
      }
    }

    if (DEBUG_NAV) {
      Serial.println("Disconnected");
      Serial.println("Advertising again...");
    }
    BLE.advertise();
  }
}