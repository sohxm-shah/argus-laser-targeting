#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>


const int flexThumbPin = 32;
const int flexMiddlePin = 33;
const int flexRingPin = 34;

const int straightThreshold = 1800; //these values are the limits which would determine whether the sensor is bent or not.
const int bentThreshold = 1600;

String gesture = "undefined";

bool straightStatus(int flexValue) {
  return (flexValue > straightThreshold);
}

bool bentStatus(int flexValue) {
  return (flexValue < bentThreshold);
}

void setup() { 
  Serial.begin(115200);
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

  delay(1000);
}
