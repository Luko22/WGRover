// Blynk streaming footage from WGExplorer
#include <credentials.h>
#define BLYNK_PRINT Serial

/* Fill in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_NAME "WGExplorer"
#define BLYNK_AUTH_TOKEN "4WZnAisRWhzUnqEVPNjPX-fEpxUhpl3M"
#define BLYNK_TEMPLATE_ID "TMPL44pBMj3_j"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "esp_camera.h"
#include "esp_http_server.h"

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"



const char *ssid = mySSIDLap;
const char *password = myPASSWORDLap;

void startCameraServer();

void setupLedFlash(int pin);

/* Motor A connections: 
in1 and in2 pins are used to control the direction of Motor A
connected to pin 13, pin 12 */
int enA = 2; // GPIO32
int in1 = 14; // GPIO33
int in2 = 15; // GPIO25
/* Motor B connections: 
in3 and in4 pins are used to control the direction of Motor B
connected to pin 11, pin 10 */
int enB = enA; // GPIO27 
int in3 = 13; // GPIO14 
int in4 = 12 ; // GPIO12

int joyposV = 1800;
int joyposH = 1800;
int flash, pic;

BLYNK_WRITE(V1) {
  joyposV = param.asInt();
}

BLYNK_WRITE(V2) {
  joyposH = param.asInt();
}

BLYNK_WRITE(V3) {
  flash = param.asInt();
}

BLYNK_WRITE(V4) {
  pic = param.asInt();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_hmirror(s, 1);
  s->set_vflip(s, 1);

  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_SVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
    Serial.println();
    
// pins initiation
    pinMode(4, OUTPUT);

    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    initDrive(90);

    String u = "http://"+WiFi.localIP().toString()+":81/stream";
    Serial.println(u);
    Blynk.virtualWrite(V0, u);
    // Blynk.setProperty(V0, "url", u);
    delay(1000);
}

void loop() {

  Blynk.run();

  // String u = "http://"+WiFi.localIP().toString()+":81/stream";
  
  // Serial.println(u);
  // Serial.print("V0= ");
  // Serial.println(V0);

  int decLim = 1000;
  int incLim = 3000;

  int driveD = map(joyposV, decLim, 0, 90, 255);
  int driveB = map(joyposV, incLim, 4095, 90, 255);

  int driveR = map(joyposH, incLim, 0, 90, 100);
  int driveL = map(joyposH, incLim, 4095, 90, 100);

delay(50);
// to make sure joystick values are received properly
  // Serial.print(joyposV);
  // Serial.print("  |  ");
  // Serial.print(driveD);
  // Serial.print("  |  ");
  // Serial.print(driveB);
  // Serial.print("  |||  ");
  // Serial.print(joyposH);
  // Serial.print("  |  ");
  // Serial.print(driveL);
  // Serial.print("  |  ");
  // Serial.println(driveR);
  
  //Forwards
  if(joyposV<decLim){
    //map then drive backwards
    analogWrite(enA, driveB);
    analogWrite(enB, driveB);
  
    // motor A CW ^ (backwards)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // motor B CW ^ (backwards)
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  delay(10);
    } else if(joyposV>incLim){//Backwards
      

    //map then drive forwards
    analogWrite(enA, driveD);
    analogWrite(enB, driveD);
  
    // motor A CW ^ (forward)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // motor B CW ^ (forward)
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  delay(10);
  } else{
    // Turn off motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  delay(10);
  }

//Rotation (Inverted)
  if(joyposH<decLim){
    //rot L (changed from R)
    analogWrite(enA, driveR);
    analogWrite(enB, driveR);
  
    // motor A CW ^ (changed from CCW)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // motor B CCW (changed from CW ^)
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    delay(10);
  } else if(joyposH>incLim){
    //rot R (changed from L)
    analogWrite(enA, driveL);
    analogWrite(enB, driveL);
  
    // motor A CCW (changed from CW ^)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // motor B CW ^ (changed from CCW)
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    delay(10);
  } 

  digitalWrite(4, flash == 1 ? HIGH : LOW);

  if (pic == 1) {
    Serial.println("");
    Serial.println("Taking pic");
    Serial.println("");
    delay(500);
  }

  delay(50);
}

