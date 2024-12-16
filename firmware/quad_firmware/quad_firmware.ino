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

//assignments for our quad
int propBackLeftPin = 19;
int propFrontLeftPin = 3;
int propFrontRightPin = 4;
int propBackRightPin = 5;

//assignments for fcb
// int propBackRightPin = 8;
// int propFrontRightPin = 3;
// int propFrontLeftPin = 4;
// int propBackLeftPin = 5;

bool armed;
bool wink;

//LED assignments for our quad
int RGB_GRN = 18;
int RGB_RED = 34;
int RGB_BLU = 35;
int EYE1 = 8;
int EYE2 = 9;

bool rgbInc = true;
int rgbRed = 180;

//LED assignments for given fcb
// int RGB_GRN = 18;
// int RGB_RED = 18;
// int RGB_BLU = 18;
// int EYE1 = 18;
// int EYE2 = 18;

int bRValue = 0;
int bLValue = 0;
int fLValue = 0;
int fRValue = 0;

int pitchTrim = 0;
int yawTrim = 0;
int rollTrim = 0;

int throttle_stick = 0;
int yaw_stick = 0;
int pitch_stick = 0;
int roll_stick = 0;

int throttle = 0;
int yaw = 0;
int pitch = 0;
int roll = 0;

float prevPitchError = 0;
float iTermPitch = 0;
float iTermRoll = 0;
float iTermYaw = 0;
float iTolerance = 3;

int deadzone = 3;

int min_gimbal = -10;
int max_gimbal = 10;

int yaw_min_gimbal = -180;
int yaw_max_gimbal = 180;

int batteryCount = 0;

unsigned long current;
float dt;
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
  float Pr, Ir, Dr, Pp, Ip, Dp, Py, Iy, Dy;
};

PIDVals pvals;

// Create LSM9DS0 board instance.
QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation | Always NULL

void setup() {
  const int SERIAL_BAUD = 19200 ;        // Baud rate for serial port 
	Serial.begin(SERIAL_BAUD);           // Start up serial
	delay(100);
  // quad_remote_setup();  don't you dare add this line back in it will break everything
  rfBegin(21);

  armed = false;
  wink = false;

  analogReference(INTERNAL);
  pinMode(propBackRightPin, OUTPUT);
  pinMode(propFrontRightPin, OUTPUT);
  pinMode(propFrontLeftPin, OUTPUT);
  pinMode(propBackLeftPin, OUTPUT);
  pinMode(BATTERY_SENSE_PIN, INPUT);
  pinMode(LED_ARMED, OUTPUT);
  pinMode(RGB_GRN, OUTPUT);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_BLU, OUTPUT);
  pinMode(EYE1, OUTPUT);
  pinMode(EYE2, OUTPUT);

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
  batteryCount++;
  if(batteryCount == 5000) {
    readBattery();
    batteryCount = 0;
  }

  analogWrite(RGB_RED, rgbRed);
  if(rgbRed >= 250) {
    rgbInc = false;
  } else if (rgbRed <= 0){
    rgbInc = true;
  }

  if(rgbInc){
    rgbRed++;
  } else {
    rgbRed--;
  }

  Serial.println(rgbRed);

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
      if(packet->wink){
        wink = true;
      } else {
        throttle_stick = packet->throttle_stick;
        yaw_stick = packet->yaw_stick;
        pitch_stick = packet->pitch_stick;
        roll_stick = packet->roll_stick;
        pvals.Pr = packet->Pr;
        pvals.Ir = packet->Ir;
        pvals.Dr = packet->Dr;
        pvals.Pp = packet->Py;
        pvals.Ip = packet->Iy;
        pvals.Dp = packet->Dy;
        armed = packet->armed;
        //pitch 2.25, roll 1.5
        pitchTrim = packet->pitchTrim;
        yawTrim = packet->yawTrim;
        rollTrim = packet->rollTrim;
        timeSinceLastPacket = millis();
      }
    }
  }

  if(wink) {
    analogWrite(EYE1, 0);
  }

  if(wink && millis() % 100 == 0){
    wink = false;
  }

  //if haven't recieved packet from remote in a while, disarm
  if(millis() - timeSinceLastPacket > 500){
    armed = false;
  }
  

  //calculate the wanted thrust, yaw rate, pitch and roll values.
  throttle = throttle_stick;
  yaw = map(yaw_stick, 0, 255, yaw_min_gimbal, yaw_max_gimbal); //TODO split yaw away from pitch and roll
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
    //dt = (current-lastTime) / 1000.0;
    dt = (current-lastTime) / 1000.0;
    lastTime = current;

    // Serial.print(orientation.pitch_rate * RAD_TO_DEG);
    // Serial.println(F(" "));

    //cf_ange = (gain) * (cf_angle + (gyro_raw * RAD_TO_DEG * dt)) + (1-gain) * (acc_angle)
    //cf_angle_pitch = ((gain) * (cf_angle_pitch + (orientation.pitch * RAD_TO_DEG * dt)) + (1-gain) * (orientation.pitch_rate)) / 1000.0 + pitchTrim;
    //cf_angle_roll = ((gain) * (cf_angle_roll + (orientation.roll * RAD_TO_DEG * dt)) + (1-gain) * (orientation.roll_rate)) / 1000.0;
    cf_angle_pitch = ((gain) * (cf_angle_pitch + (orientation.pitch_rate * RAD_TO_DEG * dt)) + (1-gain) * (orientation.pitch)) + pitchTrim / 10.0;
    cf_angle_roll = ((gain) * (cf_angle_roll + (orientation.roll_rate * RAD_TO_DEG * dt)) + (1-gain) * (orientation.roll)) + rollTrim / 10.0;
    angle_yaw = orientation.yaw_rate * RAD_TO_DEG + yawTrim;

    // Serial.print(yaw_min_gimbal);
    // Serial.print(F(" "));
    // Serial.print(yaw_max_gimbal);
    // Serial.print(F(" "));
    // Serial.print(angle_yaw);
    // Serial.println(F(" "));

    //just to see if the gyro angles are near accelerometer angles and how the gain draws the complimentary gain between them
    // gyro_angle_pitch = gyro_angle_pitch + orientation.pitch * RAD_TO_DEG * dt;
    // gyro_angle_roll = gyro_angle_roll + orientation.roll * RAD_TO_DEG * dt;
    //gyro_angle = gyro_angle + gyro_raw * RAD_TO_DEG * dt
    //PID FOR PITCH
    //GOOD PITCH for other pcb p=0.9 i=0.66 d=0.19
    
    float pTerm = pvals.Pp * (cf_angle_pitch - pitch);
    iTermPitch = iTermPitch * 0.95 + pvals.Ip * (cf_angle_pitch - pitch) * dt;
    float dTerm = pvals.Dp * ((cf_angle_pitch - pitch) - pitchPrevError) / dt;
    pitchPrevError = (cf_angle_pitch - pitch);
    if((cf_angle_pitch - pitch) < iTolerance && (cf_angle_pitch - pitch) > (0.0-iTolerance)){
      // iTermPitch = 0;
    }
    // if((prevPitchError > 0 && pTerm < 0) || (prevPitchError < 0 && pTerm > 0)){
    //   iTermPitch = 0;
    // }
    float pitchPIDCorrection = pTerm + iTermPitch + dTerm;
    // prevPitchError = pvals.Pr * (cf_angle_pitch - pitch);

    //PID FOR ROLL

    pvals.Py = 6.6;
    pvals.Iy = 0.0;
    pvals.Dy = 0.0;
    //GOOD ROLL for other pcb p=0.8, i=0.7 d=0.2
    pTerm = pvals.Pr * (cf_angle_roll - roll);
    iTermRoll = iTermRoll * 0.95 + pvals.Ir * (cf_angle_roll - roll) * dt;
    dTerm = pvals.Dr * ((cf_angle_roll - roll) - rollPrevError) / dt;
    rollPrevError = (cf_angle_roll - roll);
    if((cf_angle_roll - roll) < iTolerance && (cf_angle_roll - roll) > (0.0-iTolerance)){
      // iTermRoll = 0;
    }
    float rollPIDCorrection = pTerm + iTermRoll + dTerm;

    //PID FOR YAW

    pTerm = pvals.Py * (angle_yaw - yaw);
    iTermYaw = iTermYaw + pvals.Iy * (angle_yaw - yaw) * dt;
    dTerm = pvals.Dy * ((angle_yaw - yaw) - yawPrevError) / dt;
    yawPrevError = (angle_yaw - yaw);
    if((angle_yaw - yaw) < iTolerance && (angle_yaw - yaw) > (0.0-iTolerance)){
      iTermYaw = 0;
    }
    float yawPIDCorrection = pTerm + iTermYaw + dTerm;

    // pTerm = pvals.Py * (angle_yaw - yaw);
    // iTermYaw = iTermYaw *0.75 + pvals.Iy * (angle_yaw - yaw);
    // dTerm = pvals.Dy * ((angle_yaw - yaw) - yawPrevError);
    // yawPrevError = (angle_yaw - yaw);
    // if((angle_yaw - yaw) < iTolerance && (angle_yaw - yaw) > (0.0-iTolerance)){
    //   iTermYaw = 0;
    // }
    // float yawPIDCorrection = pTerm + iTermYaw + dTerm;

    // pitchPIDCorrection = 0;
    //GOOD PITCH for other pcb p=0.9 i=0.66 d=0.19
    //GOOD ROLL for other pcb p=0.8, i=0.7 d=0.2
    // yawPIDCorrection = 0;
    // pitchPIDCorrection = 0;
    //FRONT POSITIVE PITCH CORRECTION, BACK NEGATIVE PITCH CORRECTION
    //YAW is POSITIVE FR and BL
    //WEIRD RADIANS YAW is p=7.5 i=0.58 d=0 and need trim of 1 to 3
    //GOOD YAW IS p=4.6, i=1.15, d=0 and trim of -0.5
    fRValue = throttle + pitchPIDCorrection + yawPIDCorrection + rollPIDCorrection;
    fLValue = throttle + pitchPIDCorrection - yawPIDCorrection - rollPIDCorrection;
    bRValue = throttle - pitchPIDCorrection - yawPIDCorrection + rollPIDCorrection;
    bLValue = throttle - pitchPIDCorrection + yawPIDCorrection - rollPIDCorrection;

    //GOOD VALUES AT MAX-ISH BATTERY: ROUND 1
    /*
    Pr: 6.7
    Ir: 0.6
    Dr: 0.7
    same for p
    pitch trim: 3
    roll trim: 1.25
    */

    if(throttle < deadzone){
      fRValue = 0;
      fLValue = 0;
      bRValue = 0;
      bLValue = 0;
      iTermPitch = 0;
      iTermRoll = 0;
      iTermYaw = 0;
    }

    //Serial.print(cf_angle_pitch);
    //Serial.println(F(" "));
  }

    if (armed) {
    digitalWrite(LED_ARMED, HIGH);
    if(!wink){
      analogWrite(EYE1, 160);
    }
    analogWrite(EYE2, 160);
    bRValue = constrain(bRValue, 0, 255);
    bLValue = constrain(bLValue, 0, 255);
    fRValue = constrain(fRValue, 0, 255);
    fLValue = constrain(fLValue, 0, 255);
    
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
    if(!wink){
      analogWrite(EYE1, 30);
    }
    analogWrite(EYE2, 30);
    analogWrite(propBackRightPin, 0);
    analogWrite(propFrontRightPin, 0);
    analogWrite(propFrontLeftPin, 0);
    analogWrite(propBackLeftPin, 0);
  }

  // Packet packet;
  //   packet.magicNumber = 1829;
  //   packet.yawValue = angle_yaw;
  //   rfWrite((uint8_t*) (&packet), sizeof(packet));
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