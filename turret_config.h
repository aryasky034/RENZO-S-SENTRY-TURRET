// ============================================================================
// CONFIGURATION & CONSTANTS FILE
// Automated Gel Blaster Turret System
// Adjust these values to tune system performance
// ============================================================================

#ifndef TURRET_CONFIG_H
#define TURRET_CONFIG_H

// ============================================================================
// SYSTEM BEHAVIOR SETTINGS
// ============================================================================

// LASER SECURITY SYSTEM
const int LASER_THRESHOLD = 500;           // LDR threshold (0-1023)
                                           // Adjust based on calibration
                                           // Higher = less sensitive
const int LASER_SAMPLE_DELAY = 100;        // Milliseconds between readings

// HUMAN DETECTION
const int CONFIDENCE_THRESHOLD = 30;       // Minimum confidence to track (0-100)
const int FRAME_WIDTH = 160;               // Camera frame width
const int FRAME_HEIGHT = 120;              // Camera frame height
const int CENTER_TOLERANCE = 15;           // Pixels from center to fire (0-80)
                                           // Lower = more accurate, harder to fire
                                           // Higher = easier to fire, less accurate

// TRACKING BEHAVIOR
const int TRACKING_TIMEOUT = 3000;         // Milliseconds before returning to idle
const int SERVO_SPEED_NORMAL = 15;         // Milliseconds per position update
const int SERVO_SPEED_FAST = 5;            // Quick response servo speed
const int SERVO_SPEED_SLOW = 30;           // Smooth slow servo speed

// FIRING PARAMETERS
const int FIRE_HOLD_TIME = 200;            // Milliseconds to hold trigger (per shot)
const int FIRE_RELEASE_TIME = 100;         // Milliseconds between shots
const int MAX_BURST_SHOTS = 10;            // Maximum continuous shots
const int BURST_DELAY_BETWEEN = 300;       // Milliseconds between burst cycles

// ============================================================================
// SERVO CONFIGURATION
// ============================================================================

// Pan Servo (Turret Rotation)
const int PAN_SERVO_PIN = 9;
const int PAN_MIN = 0;                     // Leftmost position
const int PAN_MAX = 180;                   // Rightmost position
const int PAN_CENTER = 90;                 // Center position
const int PAN_SPEED = 2;                   // Degrees per update

// Trigger Servo (Gun Fire)
const int TRIGGER_SERVO_PIN = 10;
const int TRIGGER_MIN = 0;                 // Released position
const int TRIGGER_MAX = 170;               // Fully pulled position
const int TRIGGER_SPEED = 5;               // Microseconds per pulse

// ============================================================================
// CAMERA CONFIGURATION
// ============================================================================

// OV7670 I2C Address
const int OV7670_I2C_ADDR = 0x21;          // 7-bit I2C address

// Camera Registers (OV7670)
const int REG_COM7 = 0x12;                 // Common control 7
const int REG_CLKRC = 0x11;                // Clock rate control
const int REG_HSTART = 0x17;               // Horizontal start
const int REG_HSTOP = 0x18;                // Horizontal stop
const int REG_VSTART = 0x19;               // Vertical start
const int REG_VSTOP = 0x1A;                // Vertical stop

// Frame Capture
const int FRAME_RATE = 5;                  // FPS (frames per second)
const int FRAME_BUFFER_SIZE = FRAME_WIDTH * FRAME_HEIGHT;

// Human Detection Sensitivity
const int EDGE_THRESHOLD = 30;             // Magnitude threshold for edges
const int MOTION_THRESHOLD = 20;           // Pixel value difference
const int MIN_REGION_SIZE = 50;            // Minimum pixels for detection

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Digital Pins
const int CAMERA_XCLK = 3;                 // Clock signal (PWM)
const int CAMERA_VSYNC = 2;                // Vertical sync (input)
const int CAMERA_HREF = 4;                 // Horizontal reference (input)
const int CAMERA_PCLK = 5;                 // Pixel clock (input)
const int CAMERA_D4 = 6;                   // Data bit 4
const int CAMERA_D5 = 7;                   // Data bit 5
const int CAMERA_D6 = 8;                   // Data bit 6
const int BUZZER_PIN = 11;                 // Alert buzzer
const int LASER_STATUS_LED = 12;           // Status indicator

// Analog Pins (used as digital for camera data)
const int CAMERA_D0 = A0;                  // Data bit 0
const int CAMERA_D1 = A2;                  // Data bit 1
const int CAMERA_D2 = A3;                  // Data bit 2
const int CAMERA_D3 = A4;                  // Data bit 3 (also SDA for I2C)
const int LDR_SENSOR_PIN = A1;             // LDR analog input
const int CAMERA_SCL = A5;                 // I2C Clock
const int CAMERA_SDA = A4;                 // I2C Data

// ============================================================================
// ALERT PATTERNS
// ============================================================================

// Buzzer alert patterns (number of beeps)
const int ALERT_LASER_BREACH = 3;          // 3 short beeps = laser broken
const int ALERT_TARGET_DETECTED = 1;       // 1 beep = target found
const int ALERT_TARGET_FIRING = 2;         // 2 beeps = firing at target
const int ALERT_SYSTEM_ERROR = 5;          // 5 beeps = error

const int BEEP_DURATION = 100;             // Milliseconds per beep
const int BEEP_INTERVAL = 100;             // Milliseconds between beeps

// ============================================================================
// PERFORMANCE LIMITS
// ============================================================================

// Maximum values to protect hardware
const int MAX_CONTINUOUS_FIRE_TIME = 5000; // Milliseconds (5 seconds max)
const int MAX_SERVO_RUNTIME = 60000;       // Milliseconds per session
const int MAX_CAMERA_FRAME_TIME = 200;     // Milliseconds per frame capture

// Temperature monitoring (if thermistor added)
const int SAFE_TEMP_CELSIUS = 45;          // Maximum safe operating temperature
const int WARNING_TEMP_CELSIUS = 40;       // Temperature warning threshold

// ============================================================================
// CALIBRATION VALUES
// ============================================================================

// Servo calibration (adjust based on your servos)
const int SERVO_MIN_PULSE_WIDTH = 500;     // Microseconds
const int SERVO_MAX_PULSE_WIDTH = 2400;    // Microseconds
const int SERVO_NEUTRAL_PULSE = 1500;      // Microseconds (90 degrees)

// Camera clock frequency
const int CAMERA_CLOCK_FREQ = 8000000;     // 8 MHz clock

// LDR calibration (auto-calibrate or set manually)
const int LDR_LASER_ON_TYPICAL = 900;      // Typical reading with laser on
const int LDR_LASER_OFF_TYPICAL = 150;     // Typical reading with laser off

// ============================================================================
// DEBUG & LOGGING SETTINGS
// ============================================================================

// Serial communication
const long SERIAL_BAUD = 9600;             // Baud rate for Serial Monitor

// Debug output levels (0=off, 1=errors, 2=warnings, 3=info, 4=debug)
const int DEBUG_LEVEL = 3;                 // Set to 4 for maximum debugging

// What to log
const boolean LOG_LASER_READINGS = true;   // Log LDR values
const boolean LOG_SERVO_POSITIONS = true;  // Log servo angles
const boolean LOG_CAMERA_FRAMES = false;   // Log frame data (verbose)
const boolean LOG_FIRING_EVENTS = true;    // Log each fire event
const boolean LOG_ERRORS = true;           // Log all errors

// ============================================================================
// DISPLAY & FEEDBACK SETTINGS
// ============================================================================

// Serial output format
const boolean USE_DETAILED_OUTPUT = true;  // Verbose vs compact output
const int OUTPUT_REFRESH_INTERVAL = 100;   // Milliseconds between updates

// Status indicators
const boolean USE_STATUS_LED = true;       // Use LED for status
const int STATUS_LED_BRIGHT = 255;         // PWM brightness (0-255)
const int STATUS_LED_DIM = 50;             // PWM dimness (0-255)

// ============================================================================
// MACHINE LEARNING / AI SETTINGS (Future expansion)
// ============================================================================

// If implementing neural network:
const int NN_INPUT_SIZE = 100;             // Number of input features
const int NN_HIDDEN_LAYERS = 2;            // Hidden layer count
const int NN_OUTPUT_SIZE = 3;              // Output: position (left/center/right)

// ============================================================================
// ADVANCED CONFIGURATION
// ============================================================================

// Kalman Filter settings (for smoothing tracking)
const float KALMAN_PROCESS_VARIANCE = 0.01f;     // Process noise
const float KALMAN_MEASUREMENT_VARIANCE = 0.1f;  // Measurement noise
const float KALMAN_INITIAL_ERROR = 0.1f;         // Initial error

// Proportional-Integral-Derivative (PID) tuning (if using PID control)
const float PID_KP = 1.0f;                       // Proportional gain
const float PID_KI = 0.1f;                       // Integral gain
const float PID_KD = 0.01f;                      // Derivative gain

// ============================================================================
// QUICK ADJUSTMENT GUIDE
// ============================================================================

/*
PROBLEM: Turret won't fire even when target is centered
ADJUST: CENTER_TOLERANCE (increase value)
EXAMPLE: 15 → 25 (easier to fire)

PROBLEM: Turret fires before target is centered
ADJUST: CENTER_TOLERANCE (decrease value)
EXAMPLE: 15 → 5 (more accurate, harder to fire)

PROBLEM: Tracking is too slow
ADJUST: SERVO_SPEED_NORMAL (decrease value)
EXAMPLE: 15 → 10 (faster response, but uses more power)

PROBLEM: Tracking is jerky/twitchy
ADJUST: SERVO_SPEED_NORMAL (increase value)
EXAMPLE: 15 → 25 (smoother motion)

PROBLEM: Laser sensor triggers too easily
ADJUST: LASER_THRESHOLD (increase value)
EXAMPLE: 500 → 550 (less sensitive to ambient light)

PROBLEM: Laser sensor doesn't detect breaks
ADJUST: LASER_THRESHOLD (decrease value)
EXAMPLE: 500 → 450 (more sensitive)

PROBLEM: Firing is weak (not enough gel balls)
ADJUST: FIRE_HOLD_TIME (increase value)
EXAMPLE: 200 → 300 (pulls trigger longer)

PROBLEM: Firing is too aggressive
ADJUST: FIRE_HOLD_TIME (decrease value)
EXAMPLE: 200 → 100 (shorter trigger pull)

PROBLEM: Target lost too quickly
ADJUST: TRACKING_TIMEOUT (increase value)
EXAMPLE: 3000 → 5000 (5 seconds before returning idle)

PROBLEM: Returns to idle too slowly
ADJUST: TRACKING_TIMEOUT (decrease value)
EXAMPLE: 3000 → 2000 (2 seconds timeout)
*/

// ============================================================================
// SAFETY CONSTANTS
// ============================================================================

// These should NOT be modified without careful consideration
const int ABSOLUTE_MAX_SERVO_ANGLE = 180;       // Never exceed
const int ABSOLUTE_MIN_SERVO_ANGLE = 0;         // Never exceed
const int FIRE_SAFETY_TIMEOUT = 10000;          // 10 second absolute max fire time
const int SYSTEM_WATCHDOG_TIMEOUT = 30000;      // 30 second system timeout

#endif // TURRET_CONFIG_H

// ============================================================================
// HOW TO USE THIS FILE
// ============================================================================

/*
1. Include this file in your main sketch:
   #include "turret_config.h"

2. Adjust configuration values based on your specific hardware:
   - Measure laser threshold values
   - Test servo response times
   - Calibrate center tolerance for your gun

3. Use constants throughout code instead of magic numbers:
   
   GOOD:
   panServo.write(PAN_CENTER);
   
   BAD:
   panServo.write(90);  // What is 90?

4. Document any changes you make:
   const int CENTER_TOLERANCE = 20;  // Increased from 15 for easier firing (2024-12-04)

5. Keep backups of known-good configurations
*/
