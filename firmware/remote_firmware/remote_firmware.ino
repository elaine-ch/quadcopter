#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include <serLCD.h>
SerLCD lcd1;

void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
	quad_remote_setup();
}

void loop() {
  Serial.print("Throttle: ");
  Serial.println(analogRead(PIN_THROTTLE));
  Serial.println(constrain(map(analogRead(PIN_THROTTLE), 96, 750, 0, 255), 0, 255));
  Serial.print("Yaw: ");
  Serial.println(analogRead(PIN_YAW));
  Serial.println(constrain(map(analogRead(PIN_YAW), 0, 1023, 0, 255), 0, 255));
  Serial.print("Roll: ");
  Serial.println(constrain(map(analogRead(PIN_ROLL), 113, 771, 0, 255), 0, 255));
  Serial.print("Pitch: ");
  Serial.println(constrain(map(analogRead(PIN_PITCH), 66, 721, 0, 255), 0, 255));
}