#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xc8, 0x2e, 0x18, 0x45, 0xd7, 0x04};
// uint8_t broadcastAddress[] = {0xc8, 0x2e, 0x18, 0x26, 0x2a, 0x64}; 

typedef struct struct_message {
  int joyposV;
  int joyposH;
  int pic;
  int flash;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

const int JoyX = 36;
const int JoyY = 34;
const int picButton = 15; // Define pin for the picture button
const int flashButton = 22; // Define pin for the flash button

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(JoyY, INPUT);
  pinMode(JoyX, INPUT);
  pinMode(picButton, INPUT_PULLUP);  // Initialize picture button
  pinMode(flashButton, INPUT_PULLUP);  // Initialize flash button
}

void loop() {
  myData.joyposV = analogRead(JoyX);
  myData.joyposH = analogRead(JoyY);

  myData.pic = digitalRead(picButton) == LOW ? 1 : 0;  // Check if picture button is pressed
  myData.flash = digitalRead(flashButton) == LOW ? 1 : 0;  // Check if flash button is pressed

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  Serial.print("JoyV: ");
  Serial.print(myData.joyposV);
  Serial.print(" | JoyH: ");
  Serial.print(myData.joyposH);
  Serial.print(" | Pic: ");
  Serial.print(myData.pic);
  Serial.print(" | Flash: ");
  Serial.println(myData.flash);
  
  delay(50);
}
