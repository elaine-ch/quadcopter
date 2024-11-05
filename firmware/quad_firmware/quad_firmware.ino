#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include "rf_packet.h"
#include <Adafruit_Sensor.h>
#include <QuadClass_LSM6DSOX.h>
#include <Adafruit_Simple_AHRS.h>
#include <Wire.h>

#define LED_ARMED 16
#define RAD_TO_DEG 57.295779513082320876798154814105

int propBackRightPin = 8;
int propFrontRightPin = 3;
int propFrontLeftPin = 4;
int propBackLeftPin = 5;
bool armed;

int bRValue = 0;
int bLValue = 0;
int fLValue = 0;
int fRValue = 0;

int batteryCount = 0;

unsigned long current;
unsigned long dt;
unsigned long lastTime;

double gain = 0.95;

float cf_angle_pitch;
float cf_angle_roll;


//just to see if the gyro angles are near accelerometer angles and how the gain draws the complimentary gain between them
float gyro_angle_pitch;
float gyro_angle_roll;

unsigned long timeSinceLastPacket;


// Create LSM9DS0 board instance.
QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation | Always NULL

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

  // analogWrite(propBackRightPin, 0);
  // analogWrite(propFrontRightPin, 0);
  // analogWrite(propFrontLeftPin, 0);
  // analogWrite(propBackLeftPin, 0);

  rfFlush();

  //send one packet on setup to tell remote quad is disarmed
  Packet packet;
  packet.propFrontLeft = 0;
  packet.propFrontRight = 0;
  packet.propBackLeft = 0;
  packet.propBackRight = 0;
  packet.magicNumber = 1829;
  packet.battery = 0;
  packet.armed = armed;
  rfWrite((uint8_t*) (&packet), sizeof(packet));

  if (!lsm.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  _accel = lsm.getAccelerometerSensor();
  _gyro = lsm.getGyroSensor();
  ahrs = new Adafruit_Simple_AHRS(_accel, _mag, _gyro);
  #
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
    int magicNumber = packet->magicNumber;
    if (magicNumber == 1829){
      bRValue = packet->propBackRight;
      bLValue = packet->propBackLeft;
      fLValue = packet->propFrontLeft;
      fRValue = packet->propFrontRight;
      armed = packet->armed;
    }
    timeSinceLastPacket = millis();
  }

  //if haven't recieved packet from remote in a while, disarm
  if(millis() - timeSinceLastPacket > 500){
    armed = false;
  }

  if (armed) {
    digitalWrite(LED_ARMED, HIGH);
    analogWrite(propBackRightPin, bRValue);
    Serial.print(bRValue);
    Serial.print(" ");
    analogWrite(propFrontRightPin, fRValue);
    Serial.print(fRValue);
    Serial.print(" ");
    analogWrite(propFrontLeftPin, fLValue);
    Serial.print(fLValue);
    Serial.print(" ");
    analogWrite(propBackLeftPin, bLValue);
    Serial.println(bLValue);
  } else {
    digitalWrite(LED_ARMED, LOW);
    analogWrite(propBackRightPin, 0);
    analogWrite(propFrontRightPin, 0);
    analogWrite(propFrontLeftPin, 0);
    analogWrite(propBackLeftPin, 0);
  }

  //plot data
  quad_data_t orientation;
  
  // Use the simple AHRS function to get the current orientation.
  if (ahrs->getQuadOrientation(&orientation))
  {
    current = millis();
    dt = current-lastTime;
    lastTime = current;

//cf_ange = (gain) * (cf_angle + (gyro_raw * RAD_TO_DEG * dt)) + (1-gain) * (acc_angle)
cf_angle_pitch = (gain) * (cf_angle_pitch + (orientation.pitch * RAD_TO_DEG * dt)) + (1-gain) * (orientation.pitch_rate);
cf_angle_roll = (gain) * (cf_angle_roll + (orientation.roll * RAD_TO_DEG * dt)) + (1-gain) * (orientation.roll_rate);


//just to see if the gyro angles are near accelerometer angles and how the gain draws the complimentary gain between them
gyro_angle_pitch = gyro_angle_pitch + orientation.pitch * RAD_TO_DEG * dt;
gyro_angle_roll = gyro_angle_roll + orientation.roll * RAD_TO_DEG * dt;
//gyro_angle = gyro_angle + gyro_raw * RAD_TO_DEG * dt


    Serial.print("gyro pitch angle = ");
    Serial.print(gyro_angle_pitch);
    Serial.print(F(" cf pitch angle = "));
    Serial.print(cf_angle_pitch);
    Serial.print(F(" accel pitch = "));
    Serial.println(orientation.pitch_rate);

    Serial.print(F(" gyro roll angle = "));
    Serial.print(gyro_angle_roll);
    Serial.print(F(" cf roll angle = "));
    Serial.println(cf_angle_roll);
    Serial.print(F(" accel roll = "));
    Serial.print(orientation.roll_rate);



    /* 'orientation' should have valid .roll and .pitch fields */
    // Serial.print(now - last);
    // Serial.print(F(" "));
    // Serial.print(orientation.roll);
    // Serial.print(F(" "));
    // Serial.print(orientation.pitch);
    // Serial.print(F(" "));
    // Serial.print(orientation.roll_rate);
    // Serial.print(F(" "));
    // Serial.print(orientation.pitch_rate);
    // Serial.print(F(" "));
    // Serial.print(orientation.yaw_rate);
    // Serial.println(F(""));
    // Serial.print(F(" "));
    // Serial.print(orientation.pitch);
    // Serial.print(F(" "));
    // Serial.print(orientation.pitch_rate);
  }
}

void readBattery() {
  int batteryRead = analogRead(BATTERY_SENSE_PIN);
  int batteryLevel = map(batteryRead, 792, 890, 0, 100);

  Packet packet;
  packet.propFrontLeft = 0;
  packet.propFrontRight = 0;
  packet.propBackLeft = 0;
  packet.propBackRight = 0;
  packet.magicNumber = 1829;
  packet.battery = batteryLevel;
  packet.armed = armed;
  rfWrite((uint8_t*) (&packet), sizeof(packet));
}
