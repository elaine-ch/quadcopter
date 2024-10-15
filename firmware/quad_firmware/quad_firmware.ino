#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include "rf_packet.h"
#include <serLCD.h>
SerLCD lcd1;

int propBackRightPin = 8;
int propFrontRightPin = 3;
int propFrontLeftPin = 4;
int propBackLeftPin = 5;

void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
  quad_remote_setup();
  rfBegin();
  pinMode(propBackRightPin, OUTPUT);
  pinMode(propFrontRightPin, OUTPUT);
  pinMode(propFrontLeftPin, OUTPUT);
  pinMode(propBackLeftPin, OUTPUT);
}


void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(propBackRightPin, 10);
  analogWrite(propFrontRightPin, 10);
  analogWrite(propFrontLeftPin, 10);
  analogWrite(propBackLeftPin, 10);
}
