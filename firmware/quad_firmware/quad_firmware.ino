#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include "rf_packet.h"
#include <Adafruit_Sensor.h>
// #include <Adafruit_LSM6DS.h>
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

int throttle_stick = 0;
int yaw_stick = 0;
int pitch_stick = 0;
int roll_stick = 0;

int throttle = 0;
int yaw = 0;
int pitch = 0;
int roll = 0;

float iTermPitch = 0;
float iTermYaw = 0;

int deadzone = 3;

int min_gimbal = -128;
int max_gimbal = 128;

int batteryCount = 0;

unsigned long current;
unsigned long dt;
unsigned long lastTime;

double gain = 0.95;

float cf_angle_pitch;
float cf_angle_roll;
float angle_yaw;

float pitchPrevError = 0;
float yawPrevError = 0;
float rollPrevError = 0;

//just to see if the gyro angles are near accelerometer angles and how the gain draws the complimentary gain between them
float gyro_angle_pitch;
float gyro_angle_roll;

int counter = 0;

unsigned long timeSinceLastPacket;

struct PIDVals {
  float Pr, Ir, Dr, Py, Iy, Dy;
};

PIDVals pvals;

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
  // quad_remote_setup();  don't you dare add this line back in it will break everything
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
  packet.throttle_stick  = 0;
  packet.yaw_stick = 0;
  packet.roll_stick = 0;
  packet.pitch_stick = 0;
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
  // lsm6ds_gyro_range_t gyRange = ISM330DHCX_GYRO_RANGE_4000_DPS; got rid of because Steve said so.
  // lsm6ds_accel_range_t accRange = LSM6DS_ACCEL_RANGE_8_G;
  lsm6ds_data_rate_t ODR = LSM6DS_RATE_208_HZ;
  accelCompositeFilter_t accCF = LSM6DS_CompositeFilter_LPF2;
  accelCompositeFilter_range_t accCF2= LSM6DS_CompositeFilter_ODR_DIV_20; //ODR/4 we don't know why. now 20 because 208/10.4=20

  // lsm.setGyroRange(gyRange);
  // lsm.setAccelRange(accRange); steve said remove them. paired with above
  lsm.setAccelDataRate(ODR);
  lsm.setGyroDataRate(ODR);
  lsm.setAccelCompositeFilter(accCF, accCF2);
    // lsm.setGyroLPF1();
  // lsm.setGyroHPF(bool enable, gyroHPF_range_t filter_range); do not use
  
  
  #
}

void loop() {
  //battery stuff
  // batteryCount++;
  // if(batteryCount == 5000) {
  //   readBattery();
  //   batteryCount = 0;
  // }

  //read from radio
  uint8_t buf[sizeof(Packet)];
  if(rfAvailable()) {
    rfRead(buf, sizeof(Packet));
    Packet* packet = (Packet*) buf;
    int magicNumber = packet->magicNumber;
    if (magicNumber == 1829){
      // bRValue = packet->propBackRight;
      // bLValue = packet->propBackLeft;
      // fLValue = packet->propFrontLeft;
      // fRValue = packet->propFrontRight;
      throttle_stick = packet->throttle_stick;
      yaw_stick = packet->yaw_stick;
      pitch_stick = packet->pitch_stick;
      roll_stick = packet->roll_stick;
      pvals.Pr = packet->Pr;
      pvals.Ir = packet->Ir;
      pvals.Dr = packet->Dr;
      pvals.Py = packet->Py;
      pvals.Iy = packet->Iy;
      pvals.Dy = packet->Dy;
      armed = packet->armed;
      timeSinceLastPacket = millis();
    }
  }

  //if haven't recieved packet from remote in a while, disarm
  if(millis() - timeSinceLastPacket > 500){
    armed = false;
  }
  

  //calculate the wanted thrust, yaw rate, pitch and roll values.
  throttle = throttle_stick;
  yaw = map(yaw_stick, 0, 255, min_gimbal, max_gimbal); //TODO split yaw away from pitch and roll
  pitch = map(pitch_stick, 0, 255, min_gimbal, max_gimbal);
  roll = map(roll_stick, 0, 255, min_gimbal, max_gimbal);
  if(yaw > -deadzone && yaw < deadzone){ yaw = 0; }
  if(roll > -deadzone && roll < deadzone){ roll = 0; }
  if(pitch > -deadzone && pitch < deadzone){ pitch = 0; }

  

  //plot data
  quad_data_t orientation;
  
  // Use the simple AHRS function to get the current orientation.
  if (ahrs->getQuadOrientation(&orientation))
  {
    current = millis();
    dt = (current-lastTime) / 1000.0;
    lastTime = current;

    //cf_ange = (gain) * (cf_angle + (gyro_raw * RAD_TO_DEG * dt)) + (1-gain) * (acc_angle)
    cf_angle_pitch = (gain) * (cf_angle_pitch + (orientation.pitch * RAD_TO_DEG * dt)) + (1-gain) * (orientation.pitch_rate);
    cf_angle_roll = (gain) * (cf_angle_roll + (orientation.roll * RAD_TO_DEG * dt)) + (1-gain) * (orientation.roll_rate);
    angle_yaw = orientation.yaw_rate;

    //just to see if the gyro angles are near accelerometer angles and how the gain draws the complimentary gain between them
    // gyro_angle_pitch = gyro_angle_pitch + orientation.pitch * RAD_TO_DEG * dt;
    // gyro_angle_roll = gyro_angle_roll + orientation.roll * RAD_TO_DEG * dt;
    //gyro_angle = gyro_angle + gyro_raw * RAD_TO_DEG * dt
    float pTerm = pvals.Pr * (cf_angle_pitch - pitch);
    iTermPitch = iTermPitch + pvals.Ir * (cf_angle_pitch - pitch);
    float dTerm = pvals.Dr * ((cf_angle_pitch - pitch) - pitchPrevError);
    pitchPrevError = (cf_angle_pitch - pitch);
    float pitchPIDCorrection = pTerm + iTermPitch + dTerm;
  
  
    pTerm = pvals.Py * (angle_yaw - yaw);
    iTermYaw = iTermYaw + pvals.Iy * (angle_yaw - yaw);
    dTerm = pvals.Dy * ((angle_yaw - yaw) - yawPrevError);
    yawPrevError = (angle_yaw - yaw);
    float yawPIDCorrection = pTerm + iTermYaw + dTerm;

    fRValue = throttle - pitchPIDCorrection - yawPIDCorrection;
    fLValue = throttle - pitchPIDCorrection + yawPIDCorrection;
    bRValue = throttle + pitchPIDCorrection + yawPIDCorrection;
    bLValue = throttle + pitchPIDCorrection - yawPIDCorrection;

    if(throttle < deadzone){
      fRValue = 0;
      fLValue = 0;
      bRValue = 0;
      bLValue = 0;
      iTermPitch = 0;
      iTermYaw = 0;
    }

    Serial.print(cf_angle_pitch);
    Serial.print(F(" "));
    Serial.print(cf_angle_roll);
    Serial.println(F(" "));

  }

    if (armed) {
    digitalWrite(LED_ARMED, HIGH);
    analogWrite(propBackRightPin, bRValue);
    // Serial.print(bRValue);
    // Serial.print(" ");
    analogWrite(propFrontRightPin, fRValue);
    // Serial.print(fRValue);
    // Serial.print(" ");
    analogWrite(propFrontLeftPin, fLValue);
    // Serial.print(fLValue);
    // Serial.print(" ");
    analogWrite(propBackLeftPin, bLValue);
    // Serial.println(bLValue);
  } else {
    digitalWrite(LED_ARMED, LOW);
    analogWrite(propBackRightPin, 0);
    analogWrite(propFrontRightPin, 0);
    analogWrite(propFrontLeftPin, 0);
    analogWrite(propBackLeftPin, 0);
  }
}


void readBattery() {
  int batteryRead = analogRead(BATTERY_SENSE_PIN);
  int batteryLevel = map(batteryRead, 792, 890, 0, 100);

  Packet packet;
  packet.throttle_stick  = 0;
  packet.yaw_stick = 0;
  packet.roll_stick = 0;
  packet.pitch_stick = 0;
  packet.magicNumber = 1829;
  packet.battery = batteryLevel;
  packet.armed = armed;
  rfWrite((uint8_t*) (&packet), sizeof(packet));
}
