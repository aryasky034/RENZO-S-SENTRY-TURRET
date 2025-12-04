# Automated Gel Blaster Turret System
## Complete Project Documentation

---

## Project Overview

**Objective:** Build an autonomous AI-powered turret that detects humans via camera, tracks their movement, and fires gel blaster rounds when laser security is breached.

**Components:**
- Arduino R3 microcontroller
- OV7670 camera module (human detection via image processing)
- 2x SG90 servo motors (pan/rotation + trigger pull)
- Laser sensor module + LDR (light-dependent resistor)
- Buzzer (alert system)
- Gel blaster gun (mounted on turret)
- Power supply (5V-12V for servos, 5V for Arduino)

---

## System Architecture

### 1. **Laser Perimeter Security System**
- Laser emitter positioned in room
- LDR sensor detects laser interruption
- When laser is broken → Arduino triggers buzzer + activates tracking mode

### 2. **Human Detection & Tracking**
- OV7670 camera continuously captures frames
- Image processing detects human silhouettes/movement
- Calculates human position (left/center/right) in frame
- Pan servo rotates turret to center target
- Trigger servo fires blaster when aligned

### 3. **Auto-Fire Mechanism**
- Servo pulls trigger when human is centered in camera view
- Continues firing while target is in frame and in range
- Stops firing when target moves out of frame or range

---

## Hardware Connections

### Arduino R3 Pin Configuration

```
SERVO CONNECTIONS (PWM Pins):
- Pan Servo (Horizontal Rotation): Pin 9
- Trigger Servo: Pin 10

CAMERA CONNECTIONS (OV7670):
- SCL (Clock): A5
- SDA (Data): A4
- VSYNC: Pin 2
- HREF: Pin 3
- PCLK: Pin 4
- D0-D3: A0-A3
- D4-D7: Pins 5,6,7,8

LASER SENSOR CIRCUIT:
- LDR Output: A0 (Analog)
- Buzzer: Pin 11 (Digital)
- Laser Enable Pin: Pin 12 (Digital)

POWER:
- 5V: Servos, Camera, Arduino
- GND: Common ground for all components
```

### Laser Sensor Circuit Diagram

```
LASER PERIMETER:
[Laser Emitter] → [Room] → [LDR Sensor]
                              ↓
                          Voltage Divider
                          R1 = 10kΩ
                          R2 = LDR
                              ↓
                          Arduino A0
                              ↓
                          Threshold Detection
                              ↓
                          Buzzer (Pin 11) + Turret Activation

LDR CIRCUIT:
         +5V
          |
         R1 (10kΩ)
          |
    ○─────┼─────○ A0 (Arduino)
          |
         LDR (Variable)
          |
         GND
```

### Servo Circuit

```
Pan Servo (Horizontal):
  Red (5V) → Arduino 5V
  Black (GND) → Arduino GND
  Orange (Signal) → Arduino Pin 9

Trigger Servo:
  Red (5V) → Arduino 5V
  Black (GND) → Arduino GND
  Orange (Signal) → Arduino Pin 10
```

---

## Code Structure

### 1. **Arduino Main Code** (`turret_main.ino`)
- Initializes all components
- Reads laser sensor
- Processes camera frames
- Controls servo positions
- Manages firing logic

### 2. **Camera Module** (`camera.ino`)
- OV7670 I2C communication
- Frame capture via parallel interface
- Image processing for human detection

### 3. **Servo Control** (`servo_control.ino`)
- Dual servo management
- Smooth tracking motion
- Trigger activation

---

## How It Works (Step-by-Step)

### **IDLE STATE:**
1. Turret faces forward (center position)
2. Camera continuously monitoring
3. Laser security system active
4. Servos at rest

### **LASER BREACHED:**
1. LDR detects laser interruption
2. Analog value drops below threshold (400)
3. Arduino triggers:
   - Buzzer activates (audible alert)
   - Tracking mode enabled
   - Pan servo returns to neutral

### **TARGET DETECTION:**
1. Camera processes frames at ~5-10 fps
2. Image processing algorithm detects human shapes
3. Calculates human X-position (horizontal offset from center)
4. Provides feedback to servo control system

### **TRACKING & ALIGNMENT:**
1. Pan servo rotates left/right to center human
2. Motor smoothly adjusts angle proportional to offset
3. Goal: Place human in center of camera view

### **AUTO-FIRE WHEN ALIGNED:**
1. When human is centered ± 10-15 pixels:
   - Trigger servo engages (pulls trigger)
   - Gel blaster fires
   - Servo holds for 200ms (fire duration)
   - Servo releases

### **CONTINUOUS TRACKING:**
- While human remains in frame: Continue tracking + firing
- When human exits frame: Resume idle state + stop firing

### **OUT OF RANGE/TIMEOUT:**
- If no target detected for >3 seconds: Return to idle
- Buzzer can re-trigger if laser breaks again

---

## Safety Considerations

⚠️ **IMPORTANT:**
1. **Gel blaster operates within ~10-15 meters range only**
2. **Mounting location must be secure and stable**
3. **Gel ball capacity limits firing duration (~100-200 rounds)**
4. **Always test servo limits before deployment**
5. **LDR threshold must be calibrated to avoid false triggers**
6. **Use in safe environment only - never aim at people without protection**
7. **Laser safety: Use low-power laser (<5mW) for eye safety**

---

## Calibration Steps

### 1. **Servo Calibration:**
```
Run servo_calibration sketch
- Pan servo should rotate 0-180° smoothly
- Trigger servo should pull/release trigger correctly
```

### 2. **Laser Threshold Calibration:**
```
Upload code, open Serial Monitor
- Note LDR reading with laser ON: ~900-1000
- Note LDR reading with laser OFF: ~100-200
- Set threshold = (ON + OFF) / 2 = ~500-600
- Adjust in code: const int LASER_THRESHOLD = 500;
```

### 3. **Camera Calibration:**
```
- Adjust camera frame size: 160x120 or 320x240
- Fine-tune human detection sensitivity
- Test in different lighting conditions
```

---

## Performance Specifications

| Parameter | Value |
|-----------|-------|
| Frame Rate | 5-10 fps (OV7670 with Arduino) |
| Servo Response Time | ~50-100ms |
| Turret Rotation Range | 0-180° |
| Fire Rate | ~1-2 rounds/second |
| Detection Range | ~2-5 meters (lighting dependent) |
| Laser Security Range | ~10-15 meters |
| Power Consumption | ~500-800mA |

---

## Files Included

1. `turret_main.ino` - Main Arduino code
2. `camera.ino` - Camera initialization & frame capture
3. `servo_control.ino` - Dual servo management
4. `human_detection.cpp` - Image processing algorithm
5. `turret_project_doc.md` - This documentation

---

## Troubleshooting

### Camera Not Detecting Humans
- Check I2C connections (SCL/SDA)
- Verify PCLK, HREF, VSYNC connections
- Ensure 3.3V voltage divider on XCLK
- Test camera with separate camera test sketch first

### Servo Not Moving
- Check Pin 9 and Pin 10 connections
- Verify servo power supply (external 5V recommended)
- Test with servo sweep sketch
- Check servo center position (90°)

### Laser Not Triggering
- Calibrate LDR threshold
- Check LDR sensor placement
- Verify LDR wiring (A0)
- Test with Serial Monitor to see raw values

### Inaccurate Tracking
- Improve lighting conditions
- Increase frame processing time
- Adjust servo speed/smoothness
- Recalibrate servo center position

---

## Future Enhancements

1. **Multi-target tracking** - Track multiple humans simultaneously
2. **Machine learning** - Use trained neural networks for better detection
3. **Distance estimation** - Calculate target distance for range limitation
4. **Video logging** - Record turret activity
5. **Mobile control** - Remote activation via Bluetooth/WiFi
6. **Ammo counter** - Track gel ball capacity
7. **Self-calibration** - Automatic threshold adjustment

---

## Safety & Legal Notes

This project is for **educational and entertainment purposes only**. Ensure:
- ✓ Local laws permit autonomous weapons systems
- ✓ Used only in controlled private environments
- ✓ Proper safety protocols implemented
- ✓ All participants aware of system operation
- ✓ Gel blaster compliant with local regulations

---

**Project Difficulty:** Advanced  
**Estimated Build Time:** 8-12 hours  
**Total Cost:** $80-150 USD  

**Created for:** BCA Student Portfolio | Arduino Robotics Project

