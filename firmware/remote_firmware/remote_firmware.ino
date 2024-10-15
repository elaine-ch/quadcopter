#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include "rf_packet.h"
#include <serLCD.h>
#include <EEPROM.h>
SerLCD lcd1;
bool calibrate = false;
bool armed = false;

struct MinMaxGimbals {
  int throttleMax;
  int throttleMin;
  int yawMax;
  int yawMin;
  int rollMax;
  int rollMin;
  int pitchMax;
  int pitchMin;
};

MinMaxGimbals cValues;

void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
	quad_remote_setup();
  rfBegin(21);
  analogReference(INTERNAL);
  pinMode(BATTERY_SENSE_PIN, INPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
}

void loop() {
  // Serial.print("Throttle: ");
  // // Serial.println(analogRead(PIN_THROTTLE));
  // Serial.println(constrain(map(analogRead(PIN_THROTTLE), 96, 750, 0, 255), 0, 255));
  // Serial.print("Yaw: ");
  // // Serial.println(analogRead(PIN_YAW));
  // Serial.println(constrain(map(analogRead(PIN_YAW), 0, 1023, 0, 255), 0, 255));
  // Serial.print("Roll: ");
  // Serial.println(constrain(map(analogRead(PIN_ROLL), 113, 771, 0, 255), 0, 255));
  // Serial.print("Pitch: ");
  // Serial.println(constrain(map(analogRead(PIN_PITCH), 66, 721, 0, 255), 0, 255));

  //get previous calibration values from EEPROM
  if (EEPROM.read(0) == 255){
    //if EEPROM memory location has not been written to yet, use default values
    cValues.throttleMax = 750;
    cValues.throttleMin = 96;
    cValues.yawMax = 1023;
    cValues.yawMin = 0;
    cValues.rollMax = 771;
    cValues.rollMin = 113;
    cValues.pitchMax = 721;
    cValues.pitchMin = 66;
  } else {
    EEPROM.get(0, cValues);
  }

  //readBattery();

  if(digitalRead(BUTTON1_PIN) == 0 && !armed) {
    calibrate = true;
    calibrationMode();
  }
}

void readBattery() {
  Serial.println(analogRead(BATTERY_SENSE_PIN));
}

void calibrationMode() {
  //if not in calibration mode, exit function
  Serial.println("Entering calibration mode");
  cValues.throttleMax = analogRead(PIN_THROTTLE);
  cValues.throttleMin = analogRead(PIN_THROTTLE);
  cValues.yawMax = analogRead(PIN_YAW);
  cValues.yawMin = analogRead(PIN_YAW);
  cValues.rollMax = analogRead(PIN_ROLL);
  cValues.rollMin = analogRead(PIN_ROLL);
  cValues.pitchMax = analogRead(PIN_PITCH);
  cValues.pitchMin = analogRead(PIN_PITCH);

  while(calibrate){
    cValues.throttleMax = max(cValues.throttleMax, analogRead(PIN_THROTTLE));
    cValues.throttleMin = min(cValues.throttleMin, analogRead(PIN_THROTTLE));
    cValues.yawMax = max(cValues.yawMax, analogRead(PIN_YAW));
    cValues.yawMin = min(cValues.yawMin, analogRead(PIN_YAW));
    cValues.rollMax = max(cValues.rollMax, analogRead(PIN_ROLL));
    cValues.rollMin = min(cValues.rollMin, analogRead(PIN_ROLL));
    cValues.pitchMax = max(cValues.pitchMax, analogRead(PIN_PITCH));
    cValues.pitchMin = min(cValues.pitchMin, analogRead(PIN_PITCH));

    if(digitalRead(BUTTON2_PIN) == 0){
      EEPROM.put(0, cValues);
      calibrate = false;
    }
  }
  Serial.println("Exiting calibration mode");
  Serial.print("Throttle: ");
  Serial.println(cValues.throttleMin);
  Serial.println(cValues.throttleMax);
  Serial.print("Yaw: ");
  Serial.println(cValues.yawMin);
  Serial.println(cValues.yawMax);
  Serial.print("Roll: ");
  Serial.println(cValues.rollMin);
  Serial.println(cValues.rollMax);
  Serial.print("Pitch: ");
  Serial.println(cValues.pitchMin);
  Serial.println(cValues.pitchMax);
}