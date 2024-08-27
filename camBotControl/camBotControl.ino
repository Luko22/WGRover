//Step 3: making it wireless
//ESP32 38 pnis board
//This code allows for a gyroscope to be used as a controller to the LM vehicle 
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x08, 0xf9, 0xe0, 0xd3, 0xd7, 0xe4};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int joyposV;
  int joyposH;
  // float c;
  // bool d;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// // joysick1
// const int JoyX = A0;
// const int JoyY = A14;
// //camerajoy
// const int camX = A10;
// const int camY = A11;
// int joyposV;
// int joyposH;
int JoyX = 36;
int JoyY = 34;
// only runs once
void setup() {
  Serial.begin(115200);  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  pinMode(JoyY, INPUT);
  pinMode(JoyX, INPUT);
  
}

void loop() {
  myData.joyposV = analogRead(JoyX);
  myData.joyposH = analogRead(JoyY);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
    Serial.print(myData.joyposV);
    Serial.print("  |||  ");
    Serial.println(myData.joyposH);
    delay(50);
}