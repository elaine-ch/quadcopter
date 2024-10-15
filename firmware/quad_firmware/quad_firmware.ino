#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include "rf_packet.h"

#define LED_ARMED 16

int propBackRightPin = 8;
int propFrontRightPin = 3;
int propFrontLeftPin = 4;
int propBackLeftPin = 5;
bool armed;
int magicNumber;

int bRValue = 0;
int bLValue = 0;
int fLValue = 0;
int fRValue = 0;

int batteryCount = 0;

unsigned long timeSinceLastPacket;

void setup() {
  const int SERIAL_BAUD = 9600 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
  quad_remote_setup();
  rfBegin(21);

  armed = false;

  analogReference(INTERNAL);
  pinMode(propBackRightPin, OUTPUT);
  pinMode(propFrontRightPin, OUTPUT);
  pinMode(propFrontLeftPin, OUTPUT);
  pinMode(propBackLeftPin, OUTPUT);
  pinMode(BATTERY_SENSE_PIN, INPUT);
  pinMode(LED_ARMED, OUTPUT);

  analogWrite(propBackRightPin, 0);
  analogWrite(propFrontRightPin, 0);
  analogWrite(propFrontLeftPin, 0);
  analogWrite(propBackLeftPin, 0);

  //send one packet on setup to tell remote quad is disarmed
  Packet packet;
  packet.propFrontLeft = 0;
  packet.propFrontRight = 0;
  packet.propBackLeft = 0;
  packet.propBackRight = 0;
  packet.magicNumber = 2025;
  packet.battery = 0;
  packet.armed = armed;
  rfWrite((uint8_t*) (&packet), sizeof(packet));
}


void loop() {
  //battery stuff
  batteryCount++;
  if(batteryCount == 5000) {
    readBattery();
    batteryCount = 0;
  }

  //read from radio
  uint8_t buf[sizeof(Packet)];
  if(rfAvailable()) {
    rfRead(buf, sizeof(Packet));
    Packet* packet = (Packet*) buf;
    bRValue = packet->propBackRight;
    bLValue = packet->propBackLeft;
    fLValue = packet->propFrontLeft;
    fRValue = packet->propFrontRight;
    armed = packet->armed;
    magicNumber = packet->magicNumber;
    timeSinceLastPacket = millis();
  }

  //if haven't recieved packet from remote in a while, disarm
  if(millis() - timeSinceLastPacket > 500){
    armed = false;
  }

  if (armed) {
    digitalWrite(LED_ARMED, HIGH);
    analogWrite(propBackRightPin, bRValue);
    analogWrite(propFrontRightPin, fRValue);
    analogWrite(propFrontLeftPin, fLValue);
    analogWrite(propBackLeftPin, bLValue);
  } else {
    digitalWrite(LED_ARMED, LOW);
    analogWrite(propBackRightPin, 0);
    analogWrite(propFrontRightPin, 0);
    analogWrite(propFrontLeftPin, 0);
    analogWrite(propBackLeftPin, 0);
  }
}

void readBattery() {
  Serial.print("Battery reading: ");
  int batteryRead = analogRead(BATTERY_SENSE_PIN);
  int batteryLevel = map(batteryRead, 792, 890, 0, 100);
  Serial.println(batteryLevel);

  Packet packet;
  packet.propFrontLeft = 0;
  packet.propFrontRight = 0;
  packet.propBackLeft = 0;
  packet.propBackRight = 0;
  packet.magicNumber = 2025;
  packet.battery = batteryLevel;
  packet.armed = armed;
  rfWrite((uint8_t*) (&packet), sizeof(packet));
}
