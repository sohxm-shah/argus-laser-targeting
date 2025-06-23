#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

const char* ssid = "connectile dysfunction"; //wifi network
const char* password = "hotsignalsinyourarea";

//rpi recever info
const char* udpAddress = "192.168.189.197";  //rpi ip
const int udpPort = 4210;                  //rpi's listening port

WiFiUDP udp;  // UDP object

const int flexThumbPin = 32;
const int flexMiddlePin = 33;
const int flexRingPin = 34;

const int straightThreshold = 2300; //these values are the limits which would determine whether the sensor is bent or not.
const int bentThreshold = 2290;

String gesture = "undefined";

bool straightStatus(int flexValue) {
  return (flexValue > straightThreshold);
}

bool bentStatus(int flexValue) {
  return (flexValue < bentThreshold);
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password); //connecting to wifi
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() { 
  int flexThumbValue = analogRead(flexThumbPin);
  int flexMiddleValue = analogRead(flexMiddlePin);
  int flexRingValue = analogRead(flexRingPin);

  Serial.print("Thumb: ");
  Serial.print(flexThumbValue);
  Serial.print(" | Middle: ");
  Serial.print(flexMiddleValue);
  Serial.print(" | Ring: ");
  Serial.println(flexRingValue);

  //Gesture detection logic
  //killswitch - fist or palm open
  if ((bentStatus(flexThumbValue) && bentStatus(flexMiddleValue) && bentStatus(flexRingValue)) || 
      (straightStatus(flexThumbValue) && straightStatus(flexMiddleValue) && straightStatus(flexRingValue))) {
    gesture = "Killswitch";
  }
  //aim ("x" logic) - ring finger bent (gun looking hand sign)
  else if (straightStatus(flexThumbValue) && straightStatus(flexMiddleValue) && bentStatus(flexRingValue)) {
    gesture = "Aim";
  }
  //shoot - thumb also bent in aim position
  else if (bentStatus(flexThumbValue) && straightStatus(flexMiddleValue) && bentStatus(flexRingValue)) {
    gesture = "Shoot";
  }
  else {
    gesture = "Undefined";
  }

  Serial.print("Detected Gesture: ");
  Serial.println(gesture);  

  StaticJsonDocument<200> doc;
  doc["gesture"] = gesture;
  doc["thumb"] = flexThumbValue;
  doc["middle"] = flexMiddleValue;
  doc["ring"] = flexRingValue;

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);

  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t*)jsonBuffer, strlen(jsonBuffer));
  udp.endPacket();
  
  delay(2000);
}
