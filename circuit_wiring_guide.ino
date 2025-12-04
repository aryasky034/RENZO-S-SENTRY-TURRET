// ============================================================================
// AUTOMATED GEL BLASTER TURRET - CIRCUIT WIRING GUIDE
// Complete Electrical Connections for Arduino R3
// ============================================================================

/*

████████████████████████████████████████████████████████████████████████████████
COMPONENTS REQUIRED:
████████████████████████████████████████████████████████████████████████████████

1. MICROCONTROLLER:
   - Arduino R3 (ATmega328P based)
   
2. CAMERA:
   - OV7670 Camera Module (VGA camera)
   
3. SERVOS:
   - 2x SG90 Micro Servo Motors (5V)
   
4. LASER SECURITY:
   - Laser module (KY-008 or similar) - LOW POWER (<5mW)
   - LDR (Light Dependent Resistor) sensor module
   
5. ALERT SYSTEM:
   - Piezo Buzzer (5V)
   - Status LED (optional)
   
6. POWER:
   - USB Power or External 5V/12V Power Supply
   - Separate 5V supply for servos (recommended)
   - Common ground

████████████████████████████████████████████████████████████████████████████████
PIN CONFIGURATION (Arduino R3)
████████████████████████████████████████████████████████████████████████████████

DIGITAL PINS (0-13):
├─ Pin 0-1: Serial RX/TX (keep free for programming)
├─ Pin 2: Camera VSYNC (input)
├─ Pin 3: Camera XCLK (output) - PWM for clock signal
├─ Pin 4: Camera PCLK (input)
├─ Pin 5: Camera D4 or spare
├─ Pin 6: Camera D5 or spare
├─ Pin 7: Camera D6 or spare
├─ Pin 8: Camera D7 or spare
├─ Pin 9: [PWM] Pan Servo Signal
├─ Pin 10: [PWM] Trigger Servo Signal
├─ Pin 11: Buzzer (Active high)
└─ Pin 12: Laser Indicator LED (optional)

ANALOG PINS (A0-A5):
├─ A0: Camera D0 (8-bit parallel data)
├─ A1: LDR Sensor (laser detection)
├─ A2: Camera D1 (8-bit parallel data)
├─ A3: Camera D2 (8-bit parallel data)
├─ A4: Camera SDA (I2C for camera config)
└─ A5: Camera SCL (I2C for camera config)

POWER PINS:
├─ 5V: Power distribution
├─ GND: Ground (COMMON for all components)
└─ IOREF: Reference voltage (3.3V compatible)

████████████████████████████████████████████████████████████████████████████████
DETAILED WIRING DIAGRAMS
████████████████████████████████████████████████████████████████████████████████

=== 1. SERVO CONNECTIONS ===

PAN SERVO (Horizontal Rotation):
┌─────────────────────────┐
│   SG90 Servo Motor      │
├─────────────────────────┤
│ Brown (GND)  ──→  GND   │
│ Red (5V)     ──→  5V    │
│ Orange (Sig) ──→ Pin 9  │
└─────────────────────────┘

TRIGGER SERVO (Gun Trigger):
┌─────────────────────────┐
│   SG90 Servo Motor      │
├─────────────────────────┤
│ Brown (GND)  ──→  GND   │
│ Red (5V)     ──→  5V    │
│ Orange (Sig) ──→ Pin 10 │
└─────────────────────────┘

⚠️ IMPORTANT: Use separate 5V power supply for servos if drawing >500mA
   Connect Arduino GND to servo power supply GND (common ground)

=== 2. LASER SENSOR CIRCUIT ===

LASER SECURITY SYSTEM:
                    5V
                    │
        ┌───────────┴───────────┐
        │                       │
      ┌─┴─┐             ┌──────────┐
      │   │ R1 (10kΩ)   │ Laser    │
      │5V │             │ Detector │
      │   │             │ (LDR)    │
      └─┬─┘             └─────┬────┘
        │                     │
        └─────┬───────────────┘
              │
            ──┴── Analog Pin A1
              │
             GND

LDR MODULE CONNECTIONS:
┌──────────────────────┐
│  LDR Sensor Module   │
├──────────────────────┤
│ VCC (5V)   ──→  5V   │
│ GND        ──→  GND  │
│ OUT (Sig)  ──→  A1   │
└──────────────────────┘

LASER EMITTER (KY-008):
┌──────────────────────┐
│  KY-008 Laser Module │
├──────────────────────┤
│ S (Signal) ──→ [Opt] │ (Optional: can be always-on)
│ 5V         ──→  5V   │
│ GND        ──→  GND  │
└──────────────────────┘

BUZZER CIRCUIT:
         5V
         │
         │
        ┌┴────────┐
        │ Buzzer  │ (Piezo or Active Buzzer)
        │ (+)     │
        └┬────────┘
         │
       ┌─┴─┐
       │   │
       │   │ 220Ω Resistor (optional, for active buzzer)
       │   │
       └─┬─┘
         │
       Pin 11 (Turret_Buzzer)
         │
         │
        GND

=== 3. OV7670 CAMERA CONNECTIONS ===

Camera Clock Signal (XCLK) - VOLTAGE DIVIDER (5V → 3.3V):

    Arduino Pin 3 (5V PWM)
            │
            │
         ┌──┴──┐
         │R1   │ 1kΩ
         │     │
         └──┬──┘
            │
    ┌───────┼───────┬─────────────────────┐
    │       │       │                     │
    │    ┌──┴──┐    │                     │
    │    │ R2  │    │              ┌──────────┐
    │    │680Ω │    │              │ OV7670   │
    │    │     │    │              │          │
    │    └──┬──┘    │              │ XCLK     │
    │       │       │              │ (3.3V)   │
    │      GND      │              └──────────┘
    │              │                     ↑
    └──────────────┴─────────────────────┘

Camera I2C Configuration (SCCB - I2C Compatible):

Arduino               OV7670 Camera
┌──────────┐         ┌──────────────┐
│  A5 SCL  │─────────│ SCL (3.3V)   │
│  A4 SDA  │─────────│ SDA (3.3V)   │
└──────────┘         └──────────────┘

⚠️ ADD 10kΩ PULL-UP RESISTORS on SCL and SDA to 3.3V

Camera Parallel Data Interface (8-bit):

Arduino Pins         OV7670 Camera
┌─────────┐         ┌────────────────┐
│ A0      │─────────│ D0             │
│ A2      │─────────│ D1             │
│ A3      │─────────│ D2             │
│ (next)  │─────────│ D3             │
│ Pin 5   │─────────│ D4             │
│ Pin 6   │─────────│ D5             │
│ Pin 7   │─────────│ D6             │
│ (next)  │─────────│ D7             │
└─────────┘         └────────────────┘

Camera Control Signals:

Arduino               OV7670 Camera
┌──────────┐         ┌─────────────┐
│ Pin 2    │─────────│ VSYNC       │
│ Pin 4    │─────────│ PCLK        │
│ (spare)  │─────────│ HREF        │
└──────────┘         └─────────────┘

Camera Power & Reset:

OV7670 Pin           Connection
┌──────────────┐     ┌─────────────────┐
│ 3.3V         │─────│ +3.3V Regulated │
│ GND          │─────│ GND             │
│ RESET (or EN)│─────│ 3.3V (via 10kΩ) │
└──────────────┘     └─────────────────┘

⚠️ CRITICAL: OV7670 operates at 3.3V! Use voltage divider/regulator

████████████████████████████████████████████████████████████████████████████████
COMPLETE PIN MAPPING TABLE
████████████████████████████████████████████████████████████████████████████████

Arduino Pin │ Function              │ Direction │ Connected To
────────────┼───────────────────────┼───────────┼─────────────────────
0           │ RX (Serial)           │ IN        │ USB/UART (keep free)
1           │ TX (Serial)           │ OUT       │ USB/UART (keep free)
2           │ Camera VSYNC          │ IN        │ OV7670 VSYNC
3           │ Camera XCLK (PWM)     │ OUT       │ OV7670 XCLK (via divider)
4           │ Camera PCLK           │ IN        │ OV7670 PCLK
5           │ Camera D4             │ IN        │ OV7670 D4
6           │ Camera D5             │ IN        │ OV7670 D5
7           │ Camera D6             │ IN        │ OV7670 D6
8           │ Camera D7             │ IN        │ OV7670 D7
9           │ Pan Servo Signal      │ OUT(PWM)  │ SG90 Servo Pan
10          │ Trigger Servo Signal  │ OUT(PWM)  │ SG90 Servo Trigger
11          │ Buzzer                │ OUT       │ Piezo Buzzer
12          │ Laser LED (optional)  │ OUT       │ Status LED
─────────────┼───────────────────────┼───────────┼──────────────────────
A0          │ Camera D0             │ IN        │ OV7670 D0
A1          │ LDR Sensor            │ IN(ADC)   │ LDR Module OUT
A2          │ Camera D1             │ IN        │ OV7670 D1
A3          │ Camera D2             │ IN        │ OV7670 D2
A4          │ Camera SDA (I2C)      │ IN/OUT    │ OV7670 SDA
A5          │ Camera SCL (I2C)      │ IN/OUT    │ OV7670 SCL
─────────────┼───────────────────────┼───────────┼──────────────────────
5V          │ Power Rail            │ OUT       │ Servo, Camera, Buzzer
GND         │ Ground Rail           │ COM       │ All components

████████████████████████████████████████████████████████████████████████████████
POWER DISTRIBUTION
████████████████████████████████████████████████████████████████████████████████

RECOMMENDED POWER SETUP:

┌──────────────────────────────────────────────────────────────────┐
│                     POWER SUPPLY RECOMMENDATION                 │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Option 1: Single 5V Supply (if <1A total)                      │
│  ───────────────────────────────────────────                    │
│  USB 5V Adapter (2A) → Arduino → All components                 │
│                                                                  │
│  Option 2: Dual Supply (Recommended) ⭐ BEST                     │
│  ─────────────────────────────────────                          │
│  Power Supply 1 (5V/2A): Arduino + Camera + Buzzer              │
│  Power Supply 2 (5V/3A): Servo Motors (external)                │
│  ⚠️ COMMON GROUND: Connect all GND pins together                │
│                                                                  │
│  Current Budget:                                                │
│  - Arduino: ~100mA                                              │
│  - Camera: ~200mA                                               │
│  - 2x Servos: ~500-800mA (stall current)                        │
│  - Buzzer: ~50mA                                                │
│  - Total: ~1000mA (1A) minimum                                  │
│                                                                  │
│  Use 5V/2A minimum; 5V/3A recommended for safe operation       │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘

GROUND CONNECTIONS (Critical for noise immunity):

All components must share common ground:
    ┌─────────────┬─────────────┬──────────────┐
    │ Arduino GND │ Power GND   │ Servo GND    │
    └─────────────┴─────────────┴──────────────┘
                  │
                  └─────► Connect together with thick wire

████████████████████████████████████████████████████████████████████████████████
VOLTAGE LEVELS
████████████████████████████████████████████████████████████████████████████████

Component           Required Voltage    Arduino Supply    Note
─────────────────── ────────────────────  ──────────────── ─────────────────
Arduino R3          5V (USB or external)  5V               Main MCU
OV7670 Camera       3.3V                  5V (via divider) ⚠️ Use voltage divider
SG90 Servo          5V-6V                 5V external      ⚠️ Separate power supply
Piezo Buzzer        5V                    5V               Can handle Arduino GPIO
LDR Module          5V                    5V               Standard logic
Laser Module        5V                    5V (optional)    Usually always-on

████████████████████████████████████████████████████████████████████████████████
ASSEMBLY CHECKLIST
████████████████████████████████████████████████████████████████████████████████

BEFORE POWERING ON:
□ All servos connected (power + signal)
□ Pan servo moves left-right smoothly
□ Trigger servo retracts/engages
□ Camera connected and recognized
□ LDR sensor receives laser beam
□ Buzzer produces sound when tested
□ All ground connections secure
□ No loose wires touching each other
□ Voltage divider on camera XCLK installed
□ Pullup resistors on I2C lines (SCL/SDA)
□ Power supplies rated for required current

CALIBRATION AFTER ASSEMBLY:
□ Run servo calibration sketch
□ Calibrate LDR threshold with Serial monitor
□ Test camera frame capture
□ Verify laser detection works
□ Test auto-fire mechanism
□ Check turret pan range (0-180°)
□ Verify tracking accuracy

████████████████████████████████████████████████████████████████████████████████

TROUBLESHOOTING TIPS:

1. Servo Jitter:
   - Add 100µF capacitor across servo power
   - Use separate power supply for servos
   
2. Camera Not Detected:
   - Check I2C pull-up resistors (10kΩ to 3.3V)
   - Verify voltage divider on XCLK
   - Use logic analyzer to verify clock signal
   
3. LDR Not Sensing Laser:
   - Check LDR is getting direct laser beam
   - Calibrate threshold value
   - Use dark material to focus beam
   
4. Intermittent Buzzer:
   - Add 100nF capacitor for debouncing
   - Check pin can sink/source required current
   
5. Reset/Reboot Issues:
   - Add 100µF capacitor to 5V rail
   - Reduce total current draw
   - Use external power instead of USB

████████████████████████████████████████████████████████████████████████████████
END OF CIRCUIT GUIDE
████████████████████████████████████████████████████████████████████████████████
*/

// This file is for documentation. It contains the complete circuit guide
// as comments. Implementation code is in the other .ino files.

// For actual implementation, follow the pin definitions in turret_main.ino
