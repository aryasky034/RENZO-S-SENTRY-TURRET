// ============================================================================
// SERVO CONTROL AND CALIBRATION MODULE
// Dual Servo Control System for Turret
// ============================================================================

#include <Servo.h>

// Servo objects
Servo panServo;
Servo triggerServo;

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

#define PAN_SERVO_PIN 9
#define TRIGGER_SERVO_PIN 10

// Servo angle limits
#define PAN_MIN 0       // Leftmost position
#define PAN_MAX 180     // Rightmost position
#define PAN_CENTER 90   // Center position

#define TRIGGER_MIN 0     // Released position
#define TRIGGER_MAX 170   // Fully pulled position

// Servo speed control (milliseconds between increments)
#define SERVO_SPEED_SLOW 30
#define SERVO_SPEED_NORMAL 15
#define SERVO_SPEED_FAST 5

// ============================================================================
// SERVO INITIALIZATION
// ============================================================================

void initializeServos() {
  Serial.println("Initializing servos...");
  
  // Attach servos to pins
  panServo.attach(PAN_SERVO_PIN);
  triggerServo.attach(TRIGGER_SERVO_PIN);
  
  // Move to neutral position
  panServo.write(PAN_CENTER);
  triggerServo.write(TRIGGER_MIN);
  
  delay(1000); // Let servos settle
  
  Serial.println("Servos initialized - ready to operate");
}

// ============================================================================
// PAN SERVO CONTROL (Horizontal Turret Movement)
// ============================================================================

// Move pan servo to specific angle
void setPanAngle(int angle) {
  angle = constrain(angle, PAN_MIN, PAN_MAX);
  panServo.write(angle);
}

// Get current pan servo angle
int getPanAngle() {
  // Note: Servo library doesn't provide read function
  // You need to track angle manually in main code
  return panServo.read();
}

// Move pan servo smoothly to target angle
void movePanSmoothly(int targetAngle, int speed) {
  targetAngle = constrain(targetAngle, PAN_MIN, PAN_MAX);
  int currentAngle = panServo.read();
  
  while(currentAngle != targetAngle) {
    if(currentAngle < targetAngle) {
      currentAngle++;
    } else {
      currentAngle--;
    }
    
    panServo.write(currentAngle);
    delay(speed);
  }
  
  Serial.print("Pan moved to: ");
  Serial.println(currentAngle);
}

// Proportional control for smooth tracking
void setPanProportional(int targetAngle, int smoothing) {
  targetAngle = constrain(targetAngle, PAN_MIN, PAN_MAX);
  int currentAngle = panServo.read();
  
  // Calculate difference
  int difference = targetAngle - currentAngle;
  
  // Apply proportional smoothing
  int increment = difference / smoothing;
  increment = constrain(increment, -3, 3); // Limit movement per step
  
  int newAngle = currentAngle + increment;
  newAngle = constrain(newAngle, PAN_MIN, PAN_MAX);
  
  panServo.write(newAngle);
}

// ============================================================================
// TRIGGER SERVO CONTROL (Gun Fire Mechanism)
// ============================================================================

// Engage trigger (pull to fire)
void engageTrigger() {
  triggerServo.write(TRIGGER_MAX);
  Serial.println("TRIGGER ENGAGED - FIRING!");
}

// Release trigger
void releaseTrigger() {
  triggerServo.write(TRIGGER_MIN);
  Serial.println("Trigger released");
}

// Single fire pulse (pull and release)
void singleFirePulse(int holdTime) {
  engageTrigger();
  delay(holdTime);
  releaseTrigger();
  delay(100);
}

// Continuous fire (burst mode)
void burstFire(int numberOfShots, int holdTime, int betweenShotDelay) {
  for(int i = 0; i < numberOfShots; i++) {
    singleFirePulse(holdTime);
    delay(betweenShotDelay);
  }
}

// ============================================================================
// SERVO CALIBRATION FUNCTIONS
// ============================================================================

// Calibration mode for pan servo
void calibratePanServo() {
  Serial.println("\n=== PAN SERVO CALIBRATION ===");
  Serial.println("Commands:");
  Serial.println("  L - Move Left (0°)");
  Serial.println("  C - Move Center (90°)");
  Serial.println("  R - Move Right (180°)");
  Serial.println("  0-180 - Move to angle");
  Serial.println("  S - Sweep test");
  Serial.println("  Q - Quit calibration");
  
  while(true) {
    if(Serial.available()) {
      char command = Serial.read();
      
      switch(command) {
        case 'L':
        case 'l':
          setPanAngle(PAN_MIN);
          Serial.println("Pan moved to LEFT (0°)");
          break;
          
        case 'C':
        case 'c':
          setPanAngle(PAN_CENTER);
          Serial.println("Pan moved to CENTER (90°)");
          break;
          
        case 'R':
        case 'r':
          setPanAngle(PAN_MAX);
          Serial.println("Pan moved to RIGHT (180°)");
          break;
          
        case 'S':
        case 's':
          Serial.println("Starting sweep test...");
          for(int angle = 0; angle <= 180; angle += 10) {
            setPanAngle(angle);
            Serial.print("Angle: ");
            Serial.println(angle);
            delay(300);
          }
          Serial.println("Sweep complete");
          break;
          
        case 'Q':
        case 'q':
          Serial.println("Exiting calibration mode");
          return;
          
        default:
          // Try to parse as number
          if(command >= '0' && command <= '9') {
            int angle = Serial.parseInt();
            if(angle >= PAN_MIN && angle <= PAN_MAX) {
              setPanAngle(angle);
              Serial.print("Pan moved to: ");
              Serial.println(angle);
            } else {
              Serial.println("Invalid angle (0-180)");
            }
          }
          break;
      }
    }
    delay(100);
  }
}

// Calibration mode for trigger servo
void calibrateTriggerServo() {
  Serial.println("\n=== TRIGGER SERVO CALIBRATION ===");
  Serial.println("Commands:");
  Serial.println("  R - Release (0°)");
  Serial.println("  P - Pull (170°)");
  Serial.println("  0-170 - Move to angle");
  Serial.println("  F - Fire test (single shot)");
  Serial.println("  B - Burst test (3 shots)");
  Serial.println("  Q - Quit calibration");
  
  while(true) {
    if(Serial.available()) {
      char command = Serial.read();
      
      switch(command) {
        case 'R':
        case 'r':
          releaseTrigger();
          break;
          
        case 'P':
        case 'p':
          engageTrigger();
          break;
          
        case 'F':
        case 'f':
          Serial.println("Testing single fire...");
          singleFirePulse(200);
          Serial.println("Fire test complete");
          break;
          
        case 'B':
        case 'b':
          Serial.println("Testing burst fire (3 shots)...");
          burstFire(3, 150, 300);
          Serial.println("Burst test complete");
          break;
          
        case 'Q':
        case 'q':
          Serial.println("Exiting calibration mode");
          releaseTrigger(); // Always release before exit
          return;
          
        default:
          if(command >= '0' && command <= '9') {
            int angle = Serial.parseInt();
            if(angle >= TRIGGER_MIN && angle <= TRIGGER_MAX) {
              triggerServo.write(angle);
              Serial.print("Trigger moved to: ");
              Serial.println(angle);
            } else {
              Serial.println("Invalid angle (0-170)");
            }
          }
          break;
      }
    }
    delay(100);
  }
}

// Full system servo test
void testAllServos() {
  Serial.println("\n=== FULL SERVO SYSTEM TEST ===");
  
  // Pan sweep
  Serial.println("Testing pan servo sweep...");
  for(int angle = 0; angle <= 180; angle += 30) {
    setPanAngle(angle);
    Serial.print("Pan at: ");
    Serial.print(angle);
    Serial.println("°");
    delay(500);
  }
  
  // Return to center
  setPanAngle(PAN_CENTER);
  delay(300);
  
  // Trigger fire sequence
  Serial.println("Testing trigger fire sequence...");
  for(int i = 0; i < 3; i++) {
    Serial.print("Fire shot #");
    Serial.println(i + 1);
    singleFirePulse(200);
    delay(800);
  }
  
  Serial.println("Servo test complete - all systems operational");
}

// ============================================================================
// TRACKING SERVO CONTROL
// ============================================================================

// Structure to hold servo state
struct ServoState {
  int currentAngle;
  int targetAngle;
  int speed; // milliseconds between updates
  boolean isMoving;
};

ServoState panState = {90, 90, SERVO_SPEED_NORMAL, false};
ServoState triggerState = {0, 0, SERVO_SPEED_SLOW, false};

// Update servo positions (non-blocking)
void updateServoPositions() {
  static unsigned long lastUpdate = 0;
  
  if(millis() - lastUpdate < SERVO_SPEED_NORMAL) {
    return;
  }
  lastUpdate = millis();
  
  // Update pan servo
  if(panState.currentAngle != panState.targetAngle) {
    if(panState.currentAngle < panState.targetAngle) {
      panState.currentAngle++;
    } else {
      panState.currentAngle--;
    }
    panServo.write(panState.currentAngle);
  }
  
  // Update trigger servo
  if(triggerState.currentAngle != triggerState.targetAngle) {
    if(triggerState.currentAngle < triggerState.targetAngle) {
      triggerState.currentAngle++;
    } else {
      triggerState.currentAngle--;
    }
    triggerServo.write(triggerState.currentAngle);
  }
}

// Set target pan angle (non-blocking)
void setPanTarget(int angle) {
  angle = constrain(angle, PAN_MIN, PAN_MAX);
  panState.targetAngle = angle;
}

// Set target trigger angle (non-blocking)
void setTriggerTarget(int angle) {
  angle = constrain(angle, TRIGGER_MIN, TRIGGER_MAX);
  triggerState.targetAngle = angle;
}

// Get servo status
void printServoStatus() {
  Serial.print("Pan: ");
  Serial.print(panState.currentAngle);
  Serial.print("° (target: ");
  Serial.print(panState.targetAngle);
  Serial.print("°) | Trigger: ");
  Serial.print(triggerState.currentAngle);
  Serial.print("° (target: ");
  Serial.print(triggerState.targetAngle);
  Serial.println("°)");
}

// ============================================================================
// TRACKING ALGORITHM WITH SERVO CONTROL
// ============================================================================

void trackAndFire(int targetX, int frameWidth) {
  // Calculate required pan angle based on target position
  // Assuming frame center (frameWidth/2) corresponds to 90°
  
  int frameCenter = frameWidth / 2;
  int deviation = targetX - frameCenter;
  
  // Map pixel deviation to servo angle
  // Adjust scaling factor (50) based on your setup
  int angleOffset = (deviation * 90) / frameCenter;
  int newPanAngle = PAN_CENTER - angleOffset;
  newPanAngle = constrain(newPanAngle, PAN_MIN, PAN_MAX);
  
  setPanTarget(newPanAngle);
  
  // Check if aligned and fire
  int tolerance = 10; // pixels
  if(abs(deviation) < tolerance) {
    setTriggerTarget(TRIGGER_MAX); // Fire
  } else {
    setTriggerTarget(TRIGGER_MIN); // Don't fire
  }
  
  // Update servo positions
  updateServoPositions();
}

// ============================================================================
// END OF SERVO CONTROL MODULE
// ============================================================================
