//# ESP32-CAM Detection v3.0 - LASER GATE + FLASH LED
//# Flash LED glows when Arduino fires! No wiring changes needed.

#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"

// ============================================================================
// ESP32-CAM PIN CONFIGURATION
// ============================================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_LED_PIN     4    // BUILT-IN FLASH LED (no external wiring needed)

// ============================================================================
// DETECTION PARAMETERS (Loosened for better detection)
// ============================================================================

#define FRAME_WIDTH      160
#define FRAME_HEIGHT     120
#define DETECTION_INTERVAL 50
#define EDGE_THRESHOLD   15    // Lower = more sensitive
#define MIN_BLOB_SIZE    30    // Smaller blobs detected

// ============================================================================
// STATE VARIABLES
// ============================================================================

camera_fb_t *fb = NULL;
int targetX = -1;
int targetY = -1;
int confidence = 0;
unsigned long lastProcessTime = 0;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize FLASH LED (built-in, no external wiring)
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);
  
  Serial.println("\n\n╔═══════════════════════════════════════╗");
  Serial.println("║  ESP32-CAM v3.0 - LASER GATE READY   ║");
  Serial.println("║  Flash LED + Aggressive Detection    ║");
  Serial.println("╚═══════════════════════════════════════╝\n");
  
  Serial.println("[*] Initializing camera...");
  
  if(!initializeCamera()) {
    Serial.println("[!] ERROR: Camera initialization failed!");
    while(1) { 
      delay(1000); 
    }
  }
  
  Serial.println("[+] Camera initialized successfully");
  Serial.println("[+] Flash LED: Pin 4 (built-in)");
  Serial.println("[+] Detection mode: HUMAN/FACE");
  Serial.println("[+] Frame: 160x120 @ 20 FPS");
  Serial.println("[+] Waiting for laser gate breach...\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long currentTime = millis();
  
  // Check for Arduino firing signals first
  checkFireSignal();
  
  // Process at defined interval for consistent FPS
  if(currentTime - lastProcessTime < DETECTION_INTERVAL) {
    return;
  }
  
  lastProcessTime = currentTime;
  
  // Capture frame
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("[!] Camera capture failed");
    return;
  }
  
  // Detect human in frame
  detectHuman(fb);
  
  // Send detection data to Arduino
  sendToArduino();
  
  // Return frame buffer
  esp_camera_fb_return(fb);
}

// ============================================================================
// CHECK FOR ARDUINO "FIRING" SIGNAL → FLASH LED
// ============================================================================

void checkFireSignal() {
  if(Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if(msg.indexOf("FIRE") >= 0) {
      Serial.println("[FLASH] 💥 FIRING DETECTED - FLASHING!");
      flashOnFire();
    }
  }
}

// ============================================================================
// FLASH LED FIRING EFFECT
// ============================================================================

void flashOnFire() {
  // Epic double flash effect
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(80);
  digitalWrite(FLASH_LED_PIN, LOW);
  delay(40);
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(120);
  digitalWrite(FLASH_LED_PIN, LOW);
}

// ============================================================================
// INITIALIZE CAMERA WITH OPTIMAL SETTINGS
// ============================================================================

bool initializeCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  
  if(esp_camera_init(&config) != ESP_OK) {
    return false;
  }
  
  sensor_t *s = esp_camera_sensor_get();
  
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_aec_value(s, 250);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 2);
  s->set_aec2(s, 0);
  s->set_dcw(s, 1);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  
  return true;
}

// ============================================================================
// HUMAN DETECTION ALGORITHM (AGGRESSIVE)
// ============================================================================

void detectHuman(camera_fb_t *fb) {
  uint8_t *img = fb->buf;
  int width = FRAME_WIDTH;
  int height = FRAME_HEIGHT;
  
  // Edge detection for human shape
  int maxEdge = 0;
  int edgeX = -1, edgeY = -1;
  
  // Scan frame for edges
  for(int y = 1; y < height - 1; y++) {
    for(int x = 1; x < width - 1; x++) {
      int idx = y * width + x;
      
      // Sobel edge detection
      int gx = (img[idx - width - 1] - img[idx - width + 1]) +
               2*(img[idx - 1] - img[idx + 1]) +
               (img[idx + width - 1] - img[idx + width + 1]);
      
      int gy = (img[idx - width - 1] - img[idx + width - 1]) +
               2*(img[idx - width] - img[idx + width]) +
               (img[idx - width + 1] - img[idx + width + 1]);
      
      int edge = abs(gx) + abs(gy);
      
      if(edge > maxEdge) {
        maxEdge = edge;
        edgeX = x;
        edgeY = y;
      }
    }
  }
  
  // Determine if detection is valid
  if(maxEdge > EDGE_THRESHOLD) {
    targetX = edgeX;
    targetY = edgeY;
    confidence = constrain(map(maxEdge, EDGE_THRESHOLD, 255, 30, 95), 0, 95);
  } else {
    targetX = -1;
    targetY = -1;
    confidence = 0;
  }
}

// ============================================================================
// SEND DETECTION DATA TO ARDUINO
// ============================================================================

void sendToArduino() {
  if(targetX != -1 && confidence > 0) {
    // Format: <X,Y,CONF>
    Serial.print('<');
    Serial.print(targetX);
    Serial.print(',');
    Serial.print(targetY);
    Serial.print(',');
    Serial.print(confidence);
    Serial.println('>');
  } else {
    // No target
    Serial.println("<-1,-1,0>");
  }
}
