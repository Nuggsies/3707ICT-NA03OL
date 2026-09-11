#include <Arduino.h>
#include "DHTesp.h"

const int DHT_PIN = 15; // DHT1 SDA   ->  ESP32 GPIO 15
const int PIR_PIN = 13; // PIR1 OUT   ->  ESP32 GPIO 13

DHTesp dhtSensor;

void setup() {
  Serial.begin(115200);

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(PIR_PIN, INPUT);

}

void loop() {
  TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  bool motionDetected = digitalRead(PIR_PIN);


  Serial.println("Motion detected: " + String(motionDetected ? "YES" : "NO"));

  Serial.println("Temp: " + String(data.temperature, 2) + "°C");
  Serial.println("Humidity: " + String(data.humidity, 1) + "%");
  Serial.println("---");



  delay(2000); // this speeds up the simulation
  
}
