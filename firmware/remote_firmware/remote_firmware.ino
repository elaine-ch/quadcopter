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
  Serial.print("Throttle: " + constrain(analogRead(PIN_THROTTLE), 0, 255));
}
