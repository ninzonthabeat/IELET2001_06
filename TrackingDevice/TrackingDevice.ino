#include <WiFi.h>
#include <esp_now.h>
#include <TinyGPSPlus.h>

TinyGPSPlus gps;

const int buzzer = 23;
int i = 0;

//receivers MAC-address
// Project ESP had MAC address {0x78, 0xE3, 0x6D, 0x10, 0xF3, 0x40}
uint8_t broadcastAddress[] = {0x78, 0xE3, 0x6D, 0x10, 0xF3, 0x40};

float incomingMsg;

String success;

//creating two structured messages for incoming data andr outgoing data
typedef struct struct_message {
    float msg;
} struct_message;

typedef struct struct_message2{
  float Lat;
  float Lng;
  } struct_message2;

struct_message incoming;
struct_message2 outgoing;

////ESPNOW peer information parameters
esp_now_peer_info_t peerInfo;

//Send data to other esp32
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

//Recieve data from other esp32 and create variables
void OnDataRecv(const uint8_t * mac, const uint8_t *callbackData, int len) {
  memcpy(&incoming, callbackData, sizeof(incoming));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingMsg = incoming.msg;
  Serial.println(incomingMsg);
}

void setup() {
  Serial.begin(115200); //esp32 to laptop
  Serial2.begin(9600); //esp32 to gps-module
  WiFi.mode(WIFI_STA);

  pinMode(buzzer, OUTPUT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

   //will return ESP_NOW_SEND_SUCCESS in sending callback function 
  //if the data is received successfully on the MAC layer
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false; //Encryption is disabled
  
  //Add peer (other ESP32)     
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  //Register other ESP32 for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {

 // if incoming message is not 0, outgoing message equals longitude and latitude
  if (incomingMsg != 0){
    outgoing.Lat = gps.location.lat()*1000000; //multiplying with 1 000 000 to ensure 
    outgoing.Lng = gps.location.lng()*1000000; //no missing decimals
    
    //Send the message via ESP-NOW protocol
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));

    if (result == ESP_OK) {
      Serial.println("Sent successfully");
      }
    else {
      Serial.println("Error sending the data");
      }

 // Buzzer sequence
    for(i = 0; i < 5; i++){
      digitalWrite(buzzer, HIGH); // Send 1KHz sound signal...
      delay(1000);        // ...for 1 sec
      digitalWrite(buzzer, LOW);     // Stop sound...
      delay(1000);
      }
    delay(5000);
   }

  else {
    
   }
}
