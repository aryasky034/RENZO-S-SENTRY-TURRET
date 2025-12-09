//# Arduino Uno Turret Motor Controller v5.1 - Laser-Gated System
//# UPDATED: Once laser broken, system IGNORES laser restoration - PERMANENT alarm
//# Buzzer: beep-beep-beep continuously FOREVER until manual reset

#include <Servo.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define PAN_SERVO_PIN 9 // Pan servo (moves left/right)
#define TRIGGER_SERVO_PIN 10 // Trigger servo (fires projectile)
#define LDR_SENSOR_PIN A0 // Laser break detection (analog)
#define BUZZER_PIN 11 // Alert buzzer (laser break alarm)
#define STATUS_LED_PIN 13 // Built-in LED

// ============================================================================
// SERVO CONFIGURATION (CRITICAL FOR STABILITY)
// ============================================================================

#define PAN_CENTER 90 // Home position (degrees)
#define PAN_MIN 20 // Leftmost limit
#define PAN_MAX 160 // Rightmost limit
#define TRIGGER_REST 30 // Neutral/safe position
#define TRIGGER_FIRE 160 // Firing position

// ============================================================================
// TRACKING PARAMETERS
// ============================================================================

#define FRAME_WIDTH 160 // Match ESP32-CAM width
#define FRAME_CENTER_X (FRAME_WIDTH / 2) // 80
#define CENTER_TOLERANCE 25 // Pixels (±25 from center)
#define PAN_SMOOTH_STEP 3 // Degrees per update (smooth movement)
#define PAN_UPDATE_RATE 20 // ms between pan updates (50 Hz)

// ============================================================================
// FIRING PARAMETERS
// ============================================================================

#define MIN_CONFIDENCE 40 // Minimum to start tracking
#define FIRE_CONFIDENCE 50 // Minimum to fire
#define TARGET_HOLD_TIME 100 // ms target must be centered before fire
#define TRIGGER_HOLD_TIME 200 // ms to hold trigger
#define FIRE_COOLDOWN 1000 // ms between shots

// ============================================================================
// LDR SENSOR PARAMETERS - LASER BREAK DETECTION
// ============================================================================

#define LDR_THRESHOLD 400 // Analog threshold (laser blocked if < 400)
#define LDR_CHECK_RATE 100 // ms between LDR checks

// ============================================================================
// SERVO OBJECTS
// ============================================================================

Servo panServo;
Servo triggerServo;

// ============================================================================
// STATE VARIABLES
// ============================================================================

int panAngle = 90; // Current pan position
int panTarget = 90; // Target pan angle
volatile int triggerAngle = 30; // Current trigger position
bool isTracking = false; // Active tracking (laser-gated)
bool isFiring = false; // Currently firing
bool laserBroken = false; // Laser break state (NEVER RESTORED ONCE TRIGGERED)
bool systemAlarmActive = false; // ALARM ACTIVE - IGNORES LASER RESTORATION

unsigned long lastBuzzerBeepTime = 0; // NEW: Track buzzer timing

int lastTargetX = -1; // Last target X coordinate
int lastConfidence = 0; // Last confidence value
unsigned long lastDetectionTime = 0; // Time of last detection
unsigned long lastPanUpdateTime = 0; // Pan servo update timing
unsigned long lastLDRCheckTime = 0; // LDR check timing
unsigned long targetCenteredTime = 0; // When target became centered
unsigned long lastFireTime = 0; // Time of last fire
int lastLastConfidence = 0; // Track confidence changes (for motion detection)
String serialBuffer = ""; // Serial buffer for detection data

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n╔═══════════════════════════════════════╗");
  Serial.println("║ ARDUINO UNO TURRET CONTROLLER v5.1 ║");
  Serial.println("║ PERMANENT ALARM - No Restoration ║");
  Serial.println("╚═══════════════════════════════════════╝\n");

  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(LDR_SENSOR_PIN, INPUT);

  // Initialize servos with explicit pulse parameters
  panServo.attach(PAN_SERVO_PIN, 1000, 2000); // 1-2ms pulse width
  triggerServo.attach(TRIGGER_SERVO_PIN, 1000, 2000);

  // Set initial positions safely
  panServo.write(PAN_CENTER);
  triggerServo.write(TRIGGER_REST);
  delay(1000);

  Serial.println("[+] Hardware Initialization:");
  Serial.println(" ├─ Pan Servo: Pin 9 (20-160°)");
  Serial.println(" ├─ Trigger Servo: Pin 10 (30-160°)");
  Serial.println(" ├─ LDR Sensor: A0 (Laser break detection)");
  Serial.println(" ├─ Buzzer: Pin 11 (PERMANENT alarm on laser break)");
  Serial.println(" ├─ Flash Control: Serial command to ESP32");
  Serial.println(" └─ Status LED: Pin 13\n");

  Serial.println("[+] System Status:");
  Serial.println(" ├─ Pan Home: 90°");
  Serial.println(" ├─ Trigger Safe: 30°");
  Serial.println(" ├─ Tracking: DISABLED (laser clear)");
  Serial.println(" ├─ Alarm: READY (fires on laser break)");
  Serial.println(" ├─ Mode: PERMANENT ALARM");
  Serial.println(" ├─ Notes: Once triggered, alarm NEVER stops unless manual reset");
  Serial.println(" └─ Waiting for laser break detection...\n");

  digitalWrite(STATUS_LED_PIN, LOW);
  beepPattern(3, 100); // Ready signal

  lastDetectionTime = millis();
  lastPanUpdateTime = millis();
  lastLDRCheckTime = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();

  // 1. CHECK LASER SENSOR (HIGHEST PRIORITY)
  if(currentTime - lastLDRCheckTime >= LDR_CHECK_RATE) {
    checkLDRSensor();
    lastLDRCheckTime = currentTime;
  }

  // 1.5 CONTINUOUS BUZZER UPDATE (if alarm is active)
  if(systemAlarmActive) {
    updateBuzzerPattern();
  }

  // 2. RECEIVE DETECTION DATA (ONLY if alarm active)
  if(systemAlarmActive) {
    receiveDetectionData();
  }

  // 3. UPDATE PAN SERVO TO TRACK TARGET (ONLY if alarm active)
  if(currentTime - lastPanUpdateTime >= PAN_UPDATE_RATE) {
    if(systemAlarmActive) {
      updatePanTracking();
    } else {
      // Return to center if alarm not active
      returnToCenter();
    }
    lastPanUpdateTime = currentTime;
  }

  // 4. CHECK TARGET TIMEOUT (lost target)
  if(currentTime - lastDetectionTime > 3000) {
    isTracking = false;
    lastTargetX = -1;
  }

  // 5. HANDLE FIRING LOGIC (ONLY if alarm active)
  if(systemAlarmActive) {
    checkAndFire();
  }

  delay(10); // Small delay for stability
}

// ============================================================================
// RETURN PAN TO CENTER WHEN ALARM NOT ACTIVE
// ============================================================================

void returnToCenter() {
  if(panAngle > PAN_CENTER) {
    panAngle -= PAN_SMOOTH_STEP;
  } else if(panAngle < PAN_CENTER) {
    panAngle += PAN_SMOOTH_STEP;
  }

  panServo.write(panAngle);
  isTracking = false;
}

// ============================================================================
// RECEIVE DETECTION DATA FROM ESP32-CAM
// ============================================================================

void receiveDetectionData() {
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '<') {
      serialBuffer = ""; // Start new message
    }
    else if(c == '>') {
      // Message complete - parse it
      parseDetection(serialBuffer);
      serialBuffer = "";
      lastDetectionTime = millis(); // Reset timeout
    }
    else if(c != '\n' && c != '\r') {
      serialBuffer += c;
    }
  }
}

// ============================================================================
// PARSE DETECTION MESSAGE:
// ============================================================================

void parseDetection(String data) {
  int x = -1, y = -1, conf = 0;

  // Parse format: X,Y,CONF
  int commaIndex1 = data.indexOf(',');
  int commaIndex2 = data.lastIndexOf(',');

  if(commaIndex1 > 0 && commaIndex2 > commaIndex1) {
    x = data.substring(0, commaIndex1).toInt();
    y = data.substring(commaIndex1 + 1, commaIndex2).toInt();
    conf = data.substring(commaIndex2 + 1).toInt();

    // Update target (ONLY if alarm active)
    if(x >= 0 && conf > MIN_CONFIDENCE && systemAlarmActive) {
      lastTargetX = x;
      lastConfidence = conf;
      isTracking = true;

      // MOTION DETECTION: Check if human motion detected (confidence increase)
      if(conf > lastLastConfidence + 15) {
        Serial.println("[MOTION] 🔴 Human motion detected!");
        beepMotionDetected(); // Double beep for motion detection
      }

      lastLastConfidence = conf;
      digitalWrite(STATUS_LED_PIN, HIGH); // Tracking LED
    }
    else if(!systemAlarmActive) {
      // Alarm not active - ignore detection
      lastTargetX = -1;
      lastConfidence = 0;
      lastLastConfidence = 0;
      isTracking = false;
      digitalWrite(STATUS_LED_PIN, LOW);
    }
    else {
      lastTargetX = -1;
      lastConfidence = 0;
      lastLastConfidence = 0;
      isTracking = false;
      digitalWrite(STATUS_LED_PIN, LOW);
    }
  }
}

// ============================================================================
// UPDATE PAN SERVO - SMOOTH TRACKING
// ============================================================================

void updatePanTracking() {
  if(!isTracking || lastTargetX < 0) {
    // Return to center smoothly
    if(panAngle > PAN_CENTER) {
      panAngle -= PAN_SMOOTH_STEP;
    } else if(panAngle < PAN_CENTER) {
      panAngle += PAN_SMOOTH_STEP;
    }

    targetCenteredTime = 0;
  }
  else {
    // Calculate error from center
    int centerError = lastTargetX - FRAME_CENTER_X;

    // Proportional control with smooth stepping
    if(abs(centerError) > CENTER_TOLERANCE) {
      // Not centered - move towards center
      if(centerError > 0) {
        // Target to the right - move pan right
        panAngle = constrain(panAngle + PAN_SMOOTH_STEP, PAN_MIN, PAN_MAX);
      } else {
        // Target to the left - move pan left
        panAngle = constrain(panAngle - PAN_SMOOTH_STEP, PAN_MIN, PAN_MAX);
      }
      targetCenteredTime = 0; // Reset centered timer
    }
    else {
      // Target is centered
      if(targetCenteredTime == 0) {
        targetCenteredTime = millis(); // Start timer
      }
    }
  }

  // Apply pan angle (with safety bounds)
  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  panServo.write(panAngle);
}

// ============================================================================
// CHECK AND FIRE LOGIC
// ============================================================================

void checkAndFire() {
  unsigned long currentTime = millis();

  // Check if we should fire
  bool shouldFire = false;

  if(isTracking && lastTargetX >= 0 && lastConfidence >= FIRE_CONFIDENCE && systemAlarmActive) {
    // Check if target has been centered long enough
    if(targetCenteredTime > 0 &&
       (currentTime - targetCenteredTime) >= TARGET_HOLD_TIME) {
      // Check cooldown
      if((currentTime - lastFireTime) >= FIRE_COOLDOWN) {
        shouldFire = true;
      }
    }
  }

  if(shouldFire && !isFiring) {
    fireWeapon();
    lastFireTime = currentTime;
  }

  // Check if trigger needs to return to rest
  if(isFiring && (currentTime - lastFireTime) >= TRIGGER_HOLD_TIME) {
    triggerServo.write(TRIGGER_REST);
    isFiring = false;
  }
}

// ============================================================================
// FIRE WEAPON + SEND FLASH COMMAND TO ESP32
// ============================================================================

void fireWeapon() {
  isFiring = true;
  Serial.println("[!] ⚡ FIRING! Target centered, confidence high");

  // Fire sequence
  triggerServo.write(TRIGGER_FIRE);

  // Send flash command to ESP32
  Serial.println("FIRE"); // This triggers flash on ESP32

  // LED flash effects
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(50);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.println("[LED] 💡 Flash command sent to ESP32");
}

// ============================================================================
// CHECK LDR SENSOR (LASER BREAK DETECTION)
// ONCE TRIGGERED, NEVER DEACTIVATES - PERMANENT ALARM
// ============================================================================

void checkLDRSensor() {
  int ldrValue = analogRead(LDR_SENSOR_PIN);

  // If alarm is ALREADY ACTIVE - ignore laser sensor completely
  if(systemAlarmActive) {
    // Alarm is ON - STAYS ON regardless of laser state
    // Serial output for debugging (optional)
    if(ldrValue < LDR_THRESHOLD) {
      // Laser is still broken (normal)
      laserBroken = true;
    } else {
      // Laser was restored, but we IGNORE it
      laserBroken = true; // Force it to stay true
    }
    return; // EXIT - don't process sensor further
  }

  // If alarm is NOT active, check the laser sensor normally
  bool currentLDRState = (LDR_THRESHOLD < ldrValue); // True = laser broken

  if(currentLDRState) {
    // ===== LASER BEAM IS BLOCKED =====
    laserBroken = true;

    // LASER JUST BROKE - TRIGGER ALARM (first time)
    Serial.print("[LDR] ⚠ 🔴 LASER BREACHED: ");
    Serial.println(ldrValue);
    Serial.println("[ALERT] ⚠⚠⚠ INTRUDER DETECTED VIA LASER! ⚠⚠⚠");
    Serial.println("[SYSTEM] ALARM ACTIVATED - PERMANENT MODE");
    Serial.println("[SYSTEM] Laser restoration will be IGNORED");

    // Send signal to ESP32
    Serial.println("LASER_ACTIVE");

    // ACTIVATE PERMANENT ALARM
    systemAlarmActive = true;
    lastBuzzerBeepTime = millis();
  }
}

// ============================================================================
// BUZZER CONTROL
// ============================================================================

// PATTERN BEEP (system ready - 3 short beeps at startup)
void beepPattern(int count, int duration) {
  for(int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
  }
}

// CONTINUOUS BEEP PATTERN (beep-beep-beep...)
void updateBuzzerPattern() {
  unsigned long currentTime = millis();

  // Beep pattern timing:
  // Pattern: beep (100ms) - pause (100ms) - repeats continuously
  // Total cycle: 200ms

  unsigned long timeSinceBuzz = currentTime - lastBuzzerBeepTime;

  if(timeSinceBuzz < 100) {
    // BEEP ON (first 100ms of cycle)
    digitalWrite(BUZZER_PIN, HIGH);
  } else if(timeSinceBuzz < 200) {
    // BEEP OFF (second 100ms of cycle)
    digitalWrite(BUZZER_PIN, LOW);
  } else {
    // Reset cycle timer
    lastBuzzerBeepTime = currentTime;
    digitalWrite(BUZZER_PIN, HIGH); // Start new beep
  }
}

// MOTION DETECTED: Double beep (alert)
void beepMotionDetected() {
  // Double alert beep
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
}

// ============================================================================
// SYSTEM RESET (ONLY way to deactivate alarm)
// ============================================================================

void resetSystem() {
  Serial.println("[SYSTEM] ⚡⚡⚡ MANUAL RESET - Alarm deactivated");
  systemAlarmActive = false;
  laserBroken = false;
  isTracking = false;
  lastTargetX = -1;
  lastConfidence = 0;
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  panServo.write(PAN_CENTER);
  triggerServo.write(TRIGGER_REST);

  Serial.println("[SYSTEM] System reset - Ready for next breach");
}
