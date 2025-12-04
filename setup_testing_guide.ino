// ============================================================================
// SETUP AND TESTING GUIDE
// Automated Gel Blaster Turret System
// ============================================================================

/*

████████████████████████████████████████████████████████████████████████████████
STEP 1: HARDWARE ASSEMBLY
████████████████████████████████████████████████████████████████████████████████

1.1 MOUNT SERVO MOTORS
─────────────────────
□ Mount PAN SERVO horizontally on turret base (controls left-right rotation)
  - Attach mounting bracket to turret chassis
  - Use servo horn for 180° rotation mount
  - Ensure smooth rotation without friction

□ Mount TRIGGER SERVO on gun mechanism
  - Position to engage trigger when servo extends
  - Adjust linkage so full servo travel = full trigger pull
  - Test trigger pull range (0° = released, 170° = pulled)

1.2 INSTALL CAMERA
──────────────────
□ Mount OV7670 camera on turret front
  - Point towards area of interest
  - Ensure 120-160° wide field of view
  - Secure with mounting bracket or 3D printed holder
  - Protect camera lens from dust/debris

1.3 LASER SECURITY SYSTEM
─────────────────────────
□ Position LASER EMITTER:
  - Mount on opposite side of room from turret
  - Point across doorway or target area
  - Use low-power laser (<5mW) for safety
  - Use class 3A or 3R laser (eye-safe)

□ Position LDR SENSOR:
  - Mount on turret or fixed point
  - Direct line of sight to laser beam
  - Should be in direct path of laser
  - Shield from ambient light if possible

1.4 POWER CONNECTIONS
─────────────────────
□ Connect Power Supply
  - 5V/2A for Arduino + Camera + Buzzer
  - 5V/3A separate supply for servos
  - Common ground between all supplies

□ Test voltages with multimeter
  - Arduino 5V rail: 4.8-5.2V
  - Servo 5V rail: 4.8-5.2V
  - GND connections: 0V (reference)

1.5 SIGNAL CONNECTIONS
──────────────────────
□ Connect all Arduino pins according to circuit guide
□ Double-check camera connections (8-bit parallel + I2C)
□ Verify servo signal pins (PWM)
□ Test continuity with multimeter for all connections

1.6 PHYSICAL TEST
─────────────────
□ Manually rotate turret to full range (0-180°)
□ Manually pull gun trigger without servo to ensure it works
□ Test laser beam reaches sensor
□ Verify camera lens is clean and focused

████████████████████████████████████████████████████████████████████████████████
STEP 2: SOFTWARE SETUP
████████████████████████████████████████████████████████████████████████████████

2.1 INSTALL ARDUINO IDE
───────────────────────
□ Download Arduino IDE 1.8.13+ from https://www.arduino.cc/en/software
□ Install appropriate board support if needed
□ Verify Arduino R3/Uno is recognized in Tools > Board

2.2 INSTALL LIBRARIES
─────────────────────
Open Arduino IDE → Sketch → Include Library → Manage Libraries

Required libraries:
  □ Servo by Arduino (comes pre-installed)
  □ Wire by Arduino (comes pre-installed - for I2C)
  
Optional libraries:
  □ LiquidCrystal_I2C (if adding LCD display)
  □ AccelStepper (if adding stepper motors later)

2.3 UPLOAD CODE
───────────────
□ Copy turret_main.ino content
□ Copy camera_detection.ino content
□ Copy servo_control.ino content
□ Paste all into single Arduino sketch (or use tabs)
□ Select correct board: Arduino/Genuino Uno
□ Select correct COM port (Tools > Port)
□ Click Upload (⬆️ button)

Wait for "Done uploading" message.

2.4 OPEN SERIAL MONITOR
────────────────────────
□ Tools > Serial Monitor
□ Set baud rate to 9600
□ You should see startup messages:
  "Turret System Initialized"
  "Waiting for laser breach..."

████████████████████████████████████████████████████████████████████████████████
STEP 3: INDIVIDUAL COMPONENT TESTING
████████████████████████████████████████████████████████████████████████████████

3.1 TEST PAN SERVO
───────────────────

Commands (type into Serial Monitor):
  L  = Move to left (0°)
  C  = Move to center (90°)
  R  = Move to right (180°)
  S  = Sweep test
  0-180 = Move to specific angle
  Q  = Quit calibration

Expected behavior:
  ✓ Servo moves smoothly to requested angle
  ✓ No grinding or buzzing sounds
  ✓ Full 180° range motion
  ✓ Stops at each position

Troubleshooting:
  ✗ Servo doesn't move → Check pin 9 connection
  ✗ Limited range → Check mechanical binding
  ✗ Stuttering motion → Verify power supply quality
  ✗ Buzzing sound → Servo fighting against mounting

3.2 TEST TRIGGER SERVO
──────────────────────

Commands (type into Serial Monitor):
  R  = Release trigger (0°)
  P  = Pull trigger (170°)
  F  = Single fire test
  B  = Burst fire (3 shots)
  Q  = Quit

Expected behavior:
  ✓ Servo smoothly retracts and extends
  ✓ Trigger physically pulls gun trigger
  ✓ Gun fires when trigger fully engaged
  ✓ Servo releases trigger fully
  ✓ No resistance or grinding

Troubleshooting:
  ✗ Trigger stuck → Check mechanical linkage
  ✗ Servo hums but no movement → Verify power
  ✗ Only fires partially → Adjust servo range

3.3 TEST LASER SENSOR
──────────────────────

Upload this code snippet:

void setup() {
  Serial.begin(9600);
  pinMode(A1, INPUT);
  pinMode(11, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(A1);
  Serial.println(sensorValue);
  
  // Debug: Buzzer test
  if(Serial.available()) {
    char cmd = Serial.read();
    if(cmd == 'T') {
      digitalWrite(11, HIGH);
      delay(100);
      digitalWrite(11, LOW);
    }
  }
  delay(200);
}

Expected readings:
  Laser ON (beam hitting sensor): 800-1000
  Laser OFF (beam blocked): 0-200
  Threshold should be ~500

Calibration:
  □ Note reading with laser ON
  □ Note reading with laser OFF
  □ Calculate threshold = (ON + OFF) / 2
  □ Update LASER_THRESHOLD in code

3.4 TEST CAMERA
────────────────

Upload camera test code:

void setup() {
  Serial.begin(9600);
  Wire.begin();
  initializeCamera();
}

void loop() {
  // Camera should initialize and begin capturing
  // Check Serial Monitor for initialization messages
}

Expected output:
  "Camera initialization complete"
  Frame capture begins

Note: Full image display requires:
  - SD card shield or
  - Processing application on PC or
  - Simplified serial image display

3.5 TEST BUZZER
────────────────

Add this to test buzzer:

void setup() {
  pinMode(11, OUTPUT);
}

void loop() {
  digitalWrite(11, HIGH);
  delay(100);
  digitalWrite(11, LOW);
  delay(100);
}

Expected:
  ✓ Audible beeping every 200ms
  ✓ Consistent volume
  ✓ No crackling or distortion

████████████████████████████████████████████████████████████████████████████████
STEP 4: SYSTEM INTEGRATION TESTING
████████████████████████████████████████████████████████████████████████████████

4.1 LASER BREACH DETECTION
────────────────────────────
□ Run main code
□ Observe: Steady state, pan servo at center (90°)
□ Block laser beam
□ Observe: Buzzer sounds 3 beeps, turret activates
□ Restore laser beam
□ Observe: Buzzer stops, turret returns to idle after 3 seconds

4.2 HUMAN DETECTION
─────────────────────
□ Activate turret (break laser)
□ Move in front of camera
□ Observe: Pan servo tracks your movement
□ Serial monitor shows target position

4.3 AUTO-FIRE
───────────────
□ Activate turret (break laser)
□ Move to camera center
□ Observe: Trigger servo engages (pulls trigger)
□ Gun fires gel balls at you
□ Move out of frame
□ Observe: Trigger releases, gun stops firing

4.4 CONTINUOUS OPERATION
──────────────────────────
□ Multiple activations (break/restore laser)
□ Verify laser sensor resets properly
□ Verify servo positions correct after each cycle
□ Check for memory leaks (serial monitor for crashes)

████████████████████████████████████████████████████████████████████████████████
STEP 5: PERFORMANCE CALIBRATION
████████████████████████████████████████████████████████████████████████████████

5.1 ADJUST LASER THRESHOLD
────────────────────────────
Problem: False alarms from ambient light
Solution:
  □ Increase LASER_THRESHOLD value by 50
  □ Re-test laser detection
  □ Repeat until threshold stable

Problem: Doesn't detect laser break
Solution:
  □ Decrease LASER_THRESHOLD value by 50
  □ Ensure LDR gets direct laser beam
  □ Shield sensor from ambient light

5.2 ADJUST TRACKING SPEED
──────────────────────────
Problem: Turret moves too fast, misses target
Solution: Increase SERVO_SPEED_NORMAL delay value

Problem: Turret responds too slowly to movement
Solution: Decrease SERVO_SPEED_NORMAL delay value

5.3 ADJUST FIRE ACCURACY
──────────────────────────
Problem: Turret fires without target centered
Solution: Decrease CENTER_TOLERANCE value (tighter tolerance)

Problem: Turret never fires at moving targets
Solution: Increase CENTER_TOLERANCE value (easier to fire)

5.4 ADJUST FIRING DURATION
────────────────────────────
Problem: Doesn't fire enough rounds per target
Solution: Increase holdTime in singleFirePulse()

Problem: Fires too long, wasting gel balls
Solution: Decrease holdTime in singleFirePulse()

████████████████████████████████████████████████████████████████████████████████
STEP 6: SAFETY CHECKS
████████████████████████████████████████████████████████████████████████████████

BEFORE FINAL OPERATION:
□ Verify laser power is <5mW (class 3A)
□ Ensure all components mechanically secure
□ Test all servo limits (no over-rotation)
□ Check all wiring is insulated (no shorts)
□ Verify power supply capacity (min 3A at 5V)
□ Test emergency stop (laser control)
□ Ensure turret can rotate freely
□ Check trigger mechanism pull force
□ Inspect camera field of view is unobstructed
□ Test buzzer alert is audible

SAFE OPERATION:
□ Everyone in area aware of turret operation
□ Gel balls present in magazine
□ Safety goggles worn by all people
□ No aiming at faces without protection
□ Keep clear area around turret
□ Turret mounted at safe height
□ No unattended operation
□ Laser beam doesn't exceed eye level

████████████████████████████████████████████████████████████████████████████████
STEP 7: TROUBLESHOOTING REFERENCE
████████████████████████████████████████████████████████████████████████████████

ISSUE: Arduino not recognized
SOLUTION:
  □ Install CH340 driver (Arduino R3 often uses this)
  □ Try different USB cable
  □ Try different USB port
  □ Check Device Manager for COM ports

ISSUE: Servo doesn't move
SOLUTION:
  □ Check pin is correct (9 or 10)
  □ Verify Servo.h library loaded
  □ Test with simple servo sweep code
  □ Check power supply voltage (should be 5V)
  □ Verify servo is powered separately

ISSUE: Camera not initializing
SOLUTION:
  □ Check I2C pull-up resistors (10kΩ to 3.3V)
  □ Verify all camera pins connected
  □ Test I2C communication with scanner
  □ Check camera power (should be 3.3V)
  □ Verify voltage divider on XCLK

ISSUE: LDR sensor always triggered
SOLUTION:
  □ Increase LASER_THRESHOLD value
  □ Shield sensor from ambient light
  □ Use longer distance between laser and sensor
  □ Ensure direct line of sight to laser

ISSUE: Turret fires but doesn't track
SOLUTION:
  □ Check camera is receiving power
  □ Verify camera I2C communication works
  □ Check pan servo mechanical binding
  □ Verify human detection function outputs

ISSUE: Serial monitor shows garbage
SOLUTION:
  □ Check baud rate is 9600
  □ Check COM port is correct
  □ Check USB cable connection

████████████████████████████████████████████████████████████████████████████████
MONITORING AND DEBUGGING
████████████████████████████████████████████████████████████████████████████████

Enable debug output by adding to main loop:

  Serial.print("Pan: ");
  Serial.print(panAngle);
  Serial.print(" | Target X: ");
  Serial.print(targetX);
  Serial.print(" | Laser: ");
  Serial.print(analogRead(A1));
  Serial.print(" | Firing: ");
  Serial.println(firing);

This shows real-time system state.

████████████████████████████████████████████████████████████████████████████████
FILE ORGANIZATION
████████████████████████████████████████████████████████████████████████████████

Create a single Arduino project folder with:

turret_project/
├── turret_main.ino              (Main control logic)
├── camera_detection.ino         (Human detection algorithms)
├── servo_control.ino            (Servo management)
├── circuit_wiring_guide.ino     (Electrical diagram)
├── setup_testing_guide.ino      (This file)
└── README.md                    (Project documentation)

To compile:
  1. Open turret_main.ino in Arduino IDE
  2. Sketches automatically includes other .ino files
  3. Click Upload

████████████████████████████████████████████████████████████████████████████████
NEXT STEPS
████████████████████████████████████████████████████████████████████████████████

After successful basic operation:

1. IMPROVE HUMAN DETECTION
   - Implement better edge detection algorithms
   - Add skin color detection
   - Use machine learning (TensorFlow Lite)
   - Calibrate for different lighting conditions

2. ENHANCE TRACKING
   - Add Kalman filter for smoother tracking
   - Predict target position (anticipatory tracking)
   - Multi-target tracking capability

3. ADD SAFETY FEATURES
   - Timeout after 30 seconds of inactivity
   - Manual override via remote control
   - Visual feedback (LED status indicators)
   - Logging of all events

4. EXTEND FUNCTIONALITY
   - Add range limiting (don't fire beyond 10m)
   - Ammo counter
   - Thermal camera option
   - WiFi/Bluetooth remote control
   - Video recording

████████████████████████████████████████████████████████████████████████████████
END OF SETUP GUIDE
████████████████████████████████████████████████████████████████████████████████
*/

// This file contains the complete setup and testing procedures.
// Implementation continues in turret_main.ino
