// ============================================================================
// ADVANCED CAMERA PROCESSING MODULE
// OV7670 Human Detection Algorithm
// ============================================================================

#include <Wire.h>

// OV7670 I2C Address
#define OV7670_I2C_ADDR 0x21

// Camera Register Addresses (commonly used)
#define REG_COM7 0x12    // Common control 7
#define REG_HSTART 0x17  // Horizontal start
#define REG_HSTOP 0x18   // Horizontal stop
#define REG_VSTART 0x19  // Vertical start
#define REG_VSTOP 0x1A   // Vertical stop
#define REG_HREF 0x32    // HREF control
#define REG_CLKRC 0x11   // Clock rate control
#define REG_SCALING_XSC 0x70
#define REG_SCALING_YSC 0x71

// ============================================================================
// SCCB (I2C) COMMUNICATION FUNCTIONS
// ============================================================================

// Write to OV7670 register via I2C
bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  
  if(Wire.endTransmission() == 0) {
    return true;
  }
  return false;
}

// Read from OV7670 register via I2C
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(OV7670_I2C_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(OV7670_I2C_ADDR, 1);
  if(Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// ============================================================================
// CAMERA INITIALIZATION SEQUENCE
// ============================================================================

void initializeCameraOV7670() {
  Serial.println("Initializing OV7670 Camera...");
  delay(500);
  
  // Reset camera
  writeReg(REG_COM7, 0x80); // Software reset
  delay(500);
  
  // Set output format to RGB
  writeReg(REG_COM7, 0x05); // RGB, VGA format
  
  // Configure for QVGA (320x240)
  writeReg(REG_HSTART, 0x16);
  writeReg(REG_HSTOP, 0x04);
  writeReg(REG_VSTART, 0x02);
  writeReg(REG_VSTOP, 0x7A);
  
  // Clock divider
  writeReg(REG_CLKRC, 0x80);
  
  // Auto exposure
  writeReg(0x13, 0xC7);
  
  // Setup camera for motion detection
  configureMotionDetection();
  
  Serial.println("Camera initialized successfully!");
}

// ============================================================================
// MOTION DETECTION SETUP
// ============================================================================

void configureMotionDetection() {
  // Configure OV7670 for motion/edge detection
  // These settings optimize the camera for human detection
  
  // Brightness
  writeReg(0x55, 0x40);
  
  // Contrast
  writeReg(0x56, 0x40);
  
  // Saturation
  writeReg(0x57, 0x40);
  
  // Increase exposure for motion
  writeReg(0x10, 0x00);
}

// ============================================================================
// SIMPLIFIED FRAME ANALYSIS FOR HUMAN DETECTION
// ============================================================================

// This function analyzes a frame and detects human presence/position
// Returns: horizontal position of detected human (0-159), or -1 if none detected

int analyzeFrameForHuman(uint8_t* frameBuffer, int width, int height) {
  
  // Simple algorithm: Detect vertical edges (human outline)
  int leftEdgeCount = 0;
  int centerEdgeCount = 0;
  int rightEdgeCount = 0;
  
  // Scan frame for edge detection (simplified vertical Sobel filter)
  for(int row = 10; row < height - 10; row++) {
    for(int col = 5; col < width - 5; col++) {
      
      // Get pixel values (assuming grayscale)
      uint8_t center = frameBuffer[row * width + col];
      uint8_t left = frameBuffer[row * width + (col - 1)];
      uint8_t right = frameBuffer[row * width + (col + 1)];
      
      // Detect vertical edges (left to right gradient)
      int edgeMagnitude = abs((int)right - (int)left);
      
      if(edgeMagnitude > 30) { // Threshold for significant edges
        
        // Classify edge into zones
        if(col < width / 3) {
          leftEdgeCount++;
        } else if(col < 2 * width / 3) {
          centerEdgeCount++;
        } else {
          rightEdgeCount++;
        }
      }
    }
  }
  
  // Determine human position based on edge concentration
  int maxEdges = max(leftEdgeCount, max(centerEdgeCount, rightEdgeCount));
  
  if(maxEdges < 50) {
    return -1; // No human detected
  }
  
  // Return estimated X position
  if(leftEdgeCount == maxEdges) {
    return 40; // Human on left
  } else if(rightEdgeCount == maxEdges) {
    return 120; // Human on right
  } else {
    return 80; // Human in center
  }
}

// ============================================================================
// ALTERNATIVE: CENTER OF MASS DETECTION
// ============================================================================

// Detects human by finding center of mass in motion/brightness
int detectHumanCenterOfMass(uint8_t* frameBuffer, int width, int height) {
  
  int weightedSum = 0;
  int totalWeight = 0;
  
  // Calculate weighted average of bright regions
  for(int col = 0; col < width; col++) {
    for(int row = 0; row < height; row++) {
      uint8_t pixel = frameBuffer[row * width + col];
      
      // Weight by pixel brightness (humans are typically brighter)
      if(pixel > 100) {
        weightedSum += col * (pixel - 100);
        totalWeight += (pixel - 100);
      }
    }
  }
  
  if(totalWeight == 0) {
    return -1; // No target
  }
  
  // Return center of mass X position
  int centerX = weightedSum / totalWeight;
  return constrain(centerX, 0, width - 1);
}

// ============================================================================
// MOTION TRACKING (Frame difference)
// ============================================================================

// Detect movement by comparing current frame to previous frame
int detectMotion(uint8_t* currentFrame, uint8_t* previousFrame, int frameSize) {
  
  int leftMotion = 0;
  int centerMotion = 0;
  int rightMotion = 0;
  int width = 160;
  
  for(int i = 0; i < frameSize; i++) {
    int difference = abs((int)currentFrame[i] - (int)previousFrame[i]);
    
    if(difference > 20) { // Motion threshold
      int col = (i % width);
      
      if(col < width / 3) {
        leftMotion += difference;
      } else if(col < 2 * width / 3) {
        centerMotion += difference;
      } else {
        rightMotion += difference;
      }
    }
  }
  
  int maxMotion = max(leftMotion, max(centerMotion, rightMotion));
  
  if(maxMotion < 100) {
    return -1; // No significant motion
  }
  
  if(leftMotion == maxMotion) {
    return 40;
  } else if(rightMotion == maxMotion) {
    return 120;
  } else {
    return 80;
  }
}

// ============================================================================
// ADAPTIVE THRESHOLD ALGORITHM
// ============================================================================

// Automatically adjust detection threshold based on scene
uint8_t getAdaptiveThreshold(uint8_t* frameBuffer, int frameSize) {
  
  uint32_t sum = 0;
  uint32_t sumSquares = 0;
  
  // Calculate mean and variance
  for(int i = 0; i < frameSize; i++) {
    sum += frameBuffer[i];
    sumSquares += frameBuffer[i] * frameBuffer[i];
  }
  
  uint8_t mean = sum / frameSize;
  uint32_t variance = (sumSquares / frameSize) - (mean * mean);
  uint8_t stdDev = sqrt(variance);
  
  // Threshold = mean + 1.5 * stdDev
  return constrain(mean + (stdDev * 3 / 2), 50, 200);
}

// ============================================================================
// FRAME CAPTURE AND PROCESSING
// ============================================================================

// Variables for frame buffer
uint8_t currentFrameBuffer[160 * 120];
uint8_t previousFrameBuffer[160 * 120];

void captureFrame() {
  // This function would be called to capture a frame from the camera
  // In real implementation, this would interface with the parallel camera output
  
  // Move current to previous
  memcpy(previousFrameBuffer, currentFrameBuffer, sizeof(currentFrameBuffer));
  
  // Capture new frame (simplified - actual implementation needs parallel I/O)
  // captureFrameFromCamera(currentFrameBuffer);
}

// ============================================================================
// MAIN DETECTION FUNCTION
// ============================================================================

int detectHumanPosition(uint8_t* frameBuffer, int width, int height) {
  
  // Multiple detection methods - use the most confident one
  
  int edgeDetection = analyzeFrameForHuman(frameBuffer, width, height);
  int centerOfMass = detectHumanCenterOfMass(frameBuffer, width, height);
  
  // Use edge detection primarily, fallback to center of mass
  if(edgeDetection != -1) {
    return edgeDetection;
  } else if(centerOfMass != -1) {
    return centerOfMass;
  } else {
    return -1; // No detection
  }
}

// ============================================================================
// SERIAL OUTPUT FOR DEBUGGING
// ============================================================================

void printFrameInfo(uint8_t* frameBuffer, int frameSize) {
  Serial.print("Frame Data - ");
  Serial.print("Total pixels: ");
  Serial.println(frameSize);
  
  // Print first 20 pixel values for debugging
  Serial.print("Sample: ");
  for(int i = 0; i < 20; i++) {
    Serial.print(frameBuffer[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// ============================================================================
// END OF CAMERA MODULE
// ============================================================================
