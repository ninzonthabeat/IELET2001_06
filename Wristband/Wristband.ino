/* Include Libraries */
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ICM_20948.h"
#include "UbidotsEsp32Mqtt.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

/* Define Constants */

#define WIRE_PORT Wire // Desired Wire port for I2C
//#define AD0_VAL 1     //  Value of the last bit of I2C address
#define AD0_VAL 0x38    // I2C address of ICM

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define BUTTON_PIN 4 // GPIO4 pin connected to button

ICM_20948_I2C myICM; // Create an ICM_20948_I2C object

TinyGPSPlus gps;    // Create a GPS object


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

bool hasFallen = false;   // store boolean variable about fall
bool trig1 = false;        // lower threshold
bool trig2 = false;        // higher threshold
bool trig3 = false;        // trigger about about orientation 

int c1; // count for trigger1
int c2; // count for trigger2
int c3; // count for trigger3

/* Defining values for Ubidots connection */ 
const char *UBIDOTS_TOKEN = ""; // Ubidots token
const char *WIFI_SSID = ""; // Wi-Fi SSID
const char *WIFI_PASS = ""; // Wi-Fi password
const char *DEVICE_LABEL = ""; // Device Label to data published
const char *VARIABLE_LABEL = ""; // Variable Label to data published

Ubidots ubidots(UBIDOTS_TOKEN);

int angChange;  // orientation change


int state = LOW; //button
uint8_t broadcastAddress[] = {0x0C, 0xB8, 0x15, 0xC3, 0x2E, 0x0C}; //MAC for ESP-NOW

float incomingLat;
float incomingLng;
String success;

// outgoing message
typedef struct struct_message {
    float t;
} struct_message;

// incoming message
typedef struct struct_message2 {
    float Lat;
    float Lng;
} struct_message2;

struct_message outgoing;
struct_message2 incoming;

esp_now_peer_info_t peerInfo;


/* Auxiliar Functions */
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/* ESP32_NOW function to send packets to other ESP32*/
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

// Receive data from other ESP32 and update coordinate variable values 
void OnDataRecv(const uint8_t * mac, const uint8_t *callbackData, int len) {
  memcpy(&incoming, callbackData, sizeof(incoming));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingLng = (incoming.Lng/1000000);
  incomingLat = (incoming.Lat/1000000);
}

// Display coordinates from other ESP32 on OLED
void display_Coords(){
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Koordinater: ");
  display.setCursor(0, 10);
  display.println("lat: ");
  display.print(incomingLat, 6);
  display.setCursor(0, 30);
  display.println("lng: ");
  display.print(incomingLng, 6);
  display.display();
}

/* SETUP FUNCTION */
void setup() {
  Serial.begin(115200);   // ESP32 to USB
  Serial2.begin(9600);    // ESP32 to GPS-module

  while (!Serial || !Serial2)
  {
  };

  // Ubidots
  ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  
  pinMode(4, INPUT_PULLUP);// initialize the pushbutton pin as an pull-up input

  /* OLED DISPLAY*/
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64 OLED
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
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

  /* gyroscope*/
  WIRE_PORT.begin(21, 22);    // I2C pinout on ESP32
  WIRE_PORT.setClock(400000);

  // Initializing ICM_20948 / gyro
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else 
    {
      initialized = true;
    }
  }

  // SW reset on ICM_20948 so that device is in known state
  myICM.swReset(); // SW reset so device is in known state
  if (myICM.status != ICM_20948_Stat_Ok) 
  {
    Serial.println(F("Software Reset returned: "));
    Serial.println(myICM.statusString());
  }
  delay(300);

  myICM.sleep(false); // Wake sensor

  // Set full scale ranges for accelerometer and gyroscope
  ICM_20948_fss_t FSS; // Full Scale Settings structure

  FSS.a = gpm4; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
               // gpm2
               // gpm4
               // gpm8
               // gpm16
               
  FSS.g = dps250; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                  // dps250
                  // dps500
                  // dps1000
                  // dps2000
}


/* LOOP FUNCTION*/
void loop() {
  
  //Set values to send from this ESP32 to other ESP32s
  if(digitalRead(BUTTON_PIN)==LOW) 
  { //if button is pressed
    outgoing.t = 1; //Activates airtag 
    display_Coords();  //displays coords
  } 
  else
  {
    outgoing.t = 0; //Deactive airtag, Idle state
    display.clearDisplay();
    display.display();
  }

  // gyroscope logic  
  hasFallen = true;
  
  if (myICM.dataReady())
  { 
    myICM.getAGMT();    // Updating values

    printScaledAGMT(&myICM);  // Function that also regards scale settings from measurements

    // Calculate magnitude vector for acceleration
    float RawAM = pow((pow(myICM.accX(), 2) + pow(myICM.accY(), 2) + pow(myICM.accZ(), 2)), 0.5);

    // 1. If AM exceeds lower threshold
    if (RawAM > 3000)     // lower threshold
    {
      trig1 = true;  
      Serial.println("Trigger 1 activated");
    }

    // 2.
    if (trig1 == true) 
    {
      c1++;   // increment counter for trigger 1

      // if AM exceeds upper threshold
      if (RawAM > 5000) 
      {
        trig2 = true;
        Serial.println("Trigger 2 activated");
      }
    }

    // Time interval to exceed upper threshold is 0.5 sec
    if (c1 >= 6) {
      trig1 == false;
      c1 = 0;
      Serial.println("Trigger 1 deactivated");
    }

    // 3. If orientation doesn't change
    if (trig2 == true)
    {
      c2++; // increment counter for trigger 2

      angChange = pow((pow(myICM.gyrX(),2) + pow(myICM.gyrY(), 2) + pow(myICM.gyrZ(), 2)), 0.5);

      // if orientation changes between 80-100 degrees, activate trigger 3. 
      if (angChange >= 30 && angChange <= 400)
      {
       trig3 = true;
       trig2 = false; //  reset trigger and count 2
       c2 = false; 
       Serial.println(angChange);
       Serial.println("Trigger 3 activated");
      }
    }

    // time interval to exceed orientation change is 0.5 sec
    if (c2 >= 6) {
      trig2 = false;
      c2 = 0;
      Serial.println("Trigger 2 deactivated");
    }

    // 4.
    if (trig3 == true)
    {
      c3++; // increment counter for trigger 3

      if (c3 >= 10) {
        
        angChange = pow((pow(myICM.gyrX(),2) + pow(myICM.gyrY(), 2) + pow(myICM.gyrZ(), 2)), 0.5);

        if (angChange >= 0 && angChange <= 10)  // if orientation change remains between 0-10 degrees
        {
          hasFallen = true;
          Serial.println("Fall detected");
          trig3 = false;
          c3 = 0;
        } else    // client has regained normal orientation
        {
          trig3 = false;
          c3 = 0;
          Serial.println("Trigger 3 deactivated");
        }
      }
    }

    // Fall detection alert
    if (hasFallen == true) 
    {
      unsigned long prevTime;
      prevTime = millis();
      // 10 seconds to cancel alert
      while (millis() - prevTime <= 10000)
      {
        unsigned long counter = 10 - ((millis() - prevTime)/1000);

        // convert float counter to String
        char alertMsg [100];
        sprintf(alertMsg, "Fall detected\n\nSending alert within %d seconds\n\nPush button to cancel action", counter);

        display.clearDisplay();
        display.setTextSize(1.5);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println(alertMsg);
        display.display();
  
        if (digitalRead(BUTTON_PIN) == LOW)  // alert is canceled, stop sending coordinates
        {
          hasFallen = false;
          display.clearDisplay();
          display.setTextSize(1.5);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.print("Fall detection alert cancelled");
          display.display();
          delay(1000);
          
          break;
        }
      }
  
      // Check GPS module
      while (Serial2.available() > 0 && hasFallen == true) 
      {
        if (gps.encode(Serial2.read())) 
        {
          publishToUbidots();
        }
  
        if (ubidots.connected())
        {
          ubidots.loop();
        }
      }
      
      hasFallen = false;    //reset hasFallen
    }
  }


  // Set values to send from this ESP32 to other ESP32s
  if (digitalRead(BUTTON_PIN) == LOW)
  {
    outgoing.t = 1;   // Activate tracker
    display_Coords(); // Displays coordinates on OLED
  } 
  else {
    outgoing.t = 0;   // Deactivate tracker - enter idle state
    display.clearDisplay();
    display.display();
  }

  // Send message via ESP-NOW protocol
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));

  if (result == ESP_OK) 
  {
    Serial.println("Sent successfully");
  }
  else {
    Serial.println("Error sending the data");
  }

  // delay for troubleshooting
  delay(1000);
}

// Send ESP32's location to Ubidots 
void publishToUbidots()
{
  if (!ubidots.connected())
  {
    ubidots.connect();
  }

  if (ubidots.connected())
  {
    Serial.println("Connected to Ubidots");

    // Publish value to trigger fall_detection event
    ubidots.add("fall_detected", true);
    ubidots.publish("fall-detection");

    // Start publishing values to the DEVICE_LABEL
    float lat = float(gps.location.lat());
    float lng = float(gps.location.lng());

    char* str_lat = (char*)malloc(sizeof(char) * 30);
    char* str_lng = (char*)malloc(sizeof(char) * 30);

    sprintf(str_lat, "%f", lat);
    sprintf(str_lng, "%f", lng);

    char* context = (char*)malloc(sizeof(char) * 60);
    ubidots.addContext("lat", str_lat);
    ubidots.addContext("lng", str_lng);
    ubidots.getContext(context);
    Serial.println(context);

    ubidots.add("position", 2, context);
    ubidots.publish(DEVICE_LABEL);
    free(str_lat);
    free(str_lng);
    free(context); 
  } else {
    Serial.println("Could not connect to Ubidots");
    ubidots.disconnect();
  }
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    Serial.print("-");
  }
  else
  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial.print(-val, decimals);
  }
  else
  {
    Serial.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  Serial.print("Scaled. Acc (mg) [");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print("] Gyr (DPS) [");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print("]");
  Serial.println();
}
