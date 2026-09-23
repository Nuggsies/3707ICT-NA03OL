#include <Arduino.h>
#include "DHTesp.h"
#include <ESP32Servo.h>

const int DHT_PIN = 15;   // DHT1 SDA   ->  ESP32 GPIO 15
const int PIR_PIN = 13;   // PIR1 OUT   ->  ESP32 GPIO 13
const int RELAY_PIN = 26; // Cut machinery power when unsafe conditions are met
const int ALERT_PIN = 27; // Buzzer/LED alert
const int GAS_PIN = 34;   // MQ2 gas sensor analog output pin
const int SERVO_PIN = 25; // Vent servo

DHTesp dhtSensor;
Servo ventServo;

// automation states 
enum State { SURVEY, CAUTION, EMERGENCY, FAILSAFE };
State currentState = SURVEY;

// temperature thresholds with dead bands
const float CAUTION_ENTER = 40.0;   // enters CAUTION state at 40 degrees 
const float CAUTION_EXIT = 35.0;    // must drop below this value to return to SURVEY state
const float EMERGENCY_ENTER = 60.0;
const float EMERGENCY_EXIT = 55.0;  // must drop below this value to leave EMERGENCY state

// gas thresholds (analog 0 - 4095 may need tuning)
const int GAS_EMERGENCY_THRESHOLD = 1800;

// adaptive polling / edge intelligence
unsigned long lastReadTime = 0;
unsigned long pollInterval = 2000; // default SURVEY state polling rate

void updateState(float temperature, bool sensorFault, bool gasHigh) {
  if (sensorFault) {
    currentState = FAILSAFE;
    return; // FAILSAFE state will be enabled until manual intervention (loop() has the reset logic)
  }

  //elevated gas reading enables EMERGENCY state regardless of the temperature state
  if (gasHigh) {
    currentState = EMERGENCY;
    return;
  }

  switch (currentState) {
    case SURVEY:
      if (temperature > CAUTION_ENTER) currentState = CAUTION;
      break;

      case CAUTION:
      if (temperature > EMERGENCY_ENTER) currentState = EMERGENCY;
      else if (temperature < CAUTION_EXIT) currentState = SURVEY;
      break;

      case EMERGENCY:
      if (temperature < EMERGENCY_EXIT) currentState = CAUTION;
      break;

      case FAILSAFE:
      // requires sensorFault to clear naturally next read
      if (!sensorFault) currentState = EMERGENCY; // failsafe: re-enter via EMERGENCY, not straight to SURVEY
      break;
  }
}

void updateActuators() {
  switch (currentState) {
    case SURVEY:
      digitalWrite(RELAY_PIN, HIGH); //machine powered
      digitalWrite(ALERT_PIN, LOW);
      ventServo.write(0); //vent closed
      pollInterval = 2000;
      break;

    case CAUTION:
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(ALERT_PIN, HIGH); // alert active
      ventServo.write(0);
      pollInterval = 1000;            //poll faster to catch any trends
      break;

    case EMERGENCY:
      digitalWrite(RELAY_PIN, LOW); //cut power to the machine
      digitalWrite(ALERT_PIN, HIGH);
      ventServo.write(90); //open vent to clear heat/gas
      pollInterval = 500;
      break;

    case FAILSAFE:
      digitalWrite(RELAY_PIN, LOW); // treat as emergency
      digitalWrite(ALERT_PIN, HIGH);
      ventServo.write(90);
      pollInterval = 500;
      break;
  }
}

void printStatus(float temperature, float humidity, bool motionDetected, int gasReading) {
  const char* stateNames[] = {"SURVEY", "CAUTION", "EMERGENCY", "FAILSAFE"};
  Serial.print("State: ");
  Serial.print(stateNames[currentState]);
  Serial.print(" | Temp: ");
  Serial.print(temperature, 2);
  Serial.print("C | Humidity: ");
  Serial.print(humidity, 1);
  Serial.print("% | Motion: ");
  Serial.println(motionDetected ? "YES" : "NO");
  Serial.print(" | Gas: ");
  Serial.println(gasReading);
}

void setup() {
  Serial.begin(115200);

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(PIR_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ALERT_PIN, OUTPUT);

  ventServo.attach(SERVO_PIN);
  ventServo.write(0);

  digitalWrite(RELAY_PIN, HIGH); //start in SURVEY state
  digitalWrite(ALERT_PIN, LOW);
}

void loop() {
  //only take a new reading ocne pollInterval has elapsed
  if (millis() - lastReadTime >= pollInterval) {
    lastReadTime = millis();

  TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  bool motionDetected = digitalRead(PIR_PIN);
  bool sensorFault = isnan(data.temperature) || isnan(data.humidity);

  int gasReading = analogRead(GAS_PIN);
  bool gasHigh = gasReading > GAS_EMERGENCY_THRESHOLD;

  updateState(data.temperature, sensorFault, gasHigh);
  updateActuators();
  printStatus(data.temperature, data.humidity, motionDetected, gasReading);
  }
}
