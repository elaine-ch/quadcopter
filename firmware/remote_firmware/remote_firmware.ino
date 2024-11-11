#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include "rf_packet.h"
#include <EEPROM.h>
bool calibrate = false;
bool pidCalibrate = false;
int yawMode = 0;
int pidCalMode = 0; //0 for Pr 1 for Ir 2 for Dr 3 for Py 4 for Iy 5 for Dy
int prevDownPushed = 0;
int prevLeftPushed = 0;
int prevRightPushed = 0;
int prevUpPushed = 0;
int prevOnePushed = 0;
int prevEncPushed = 0;
int prevCenterPushed = 0;
bool armed = false;
bool printOnce = true;
int batteryCount = 0;
int quadBattery = 0;
int batteryLevel = 0;

float pInc = 0.1;
float iInc = 0.1;
float dInc = 0.1;

extern RotaryEncoder knob1;
int lastKnobPos;

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

struct PIDVals {
  float Pr, Ir, Dr, Py, Iy, Dy;
};

struct WholeEeprom {
  MinMaxGimbals gimbalVals;
  PIDVals pidVals;
};

MinMaxGimbals cValues;
PIDVals pvals;
WholeEeprom wholeEeprom;

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
  pinMode(ENC1_A_PIN, INPUT);
  pinMode(ENC1_B_PIN, INPUT);
  calibrate = false;
  armed = false;
  lcd.setFastBacklight(255, 255, 255);

  rfFlush();

  //tell quad about reset
  Packet packet;
  packet.throttle_stick = 0;
  packet.yaw_stick = 0;
  packet.pitch_stick = 0;
  packet.roll_stick = 0;
  packet.magicNumber = 1829;
  packet.battery = 0;
  packet.armed = armed;
  rfWrite((uint8_t*) (&packet), sizeof(packet));
}

void loop() {
  batteryCount++;
  // Serial.print("Throttle: ");
  // Serial.println(constrain(map(analogRead(PIN_THROTTLE), 96, 750, 0, 255), 0, 255));
  // Serial.print("Yaw: ");
  // Serial.println(analogRead(PIN_YAW));
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
    cValues.yawMax = 840;
    cValues.yawMin = 96;
    cValues.rollMax = 771;
    cValues.rollMin = 113;
    cValues.pitchMax = 721;
    cValues.pitchMin = 66;
    pvals.Pr = 2.0;
    pvals.Ir = 0;
    pvals.Dr = 0;
    pvals.Py = 2;
    pvals.Iy = 0;
    pvals.Dy = 0;

  } else {
    EEPROM.get(0, wholeEeprom);
    cValues = wholeEeprom.gimbalVals;
    pvals = wholeEeprom.pidVals;
  }
  
  //declaring additional variables to avoid calling functions inside of constrain
  int tMap = map(analogRead(PIN_THROTTLE), cValues.throttleMin, cValues.throttleMax, 0, 255);
  int throttle = constrain(tMap, 0, 255);
  int yMap = map(analogRead(PIN_YAW), cValues.yawMin, cValues.yawMax, 0, 255);
  int yaw = constrain(yMap, 0, 255);
  int rMap = map(analogRead(PIN_ROLL), cValues.rollMin, cValues.rollMax, 0, 255);
  int roll = constrain(rMap, 0, 255);
  int pMap = map(analogRead(PIN_PITCH), cValues.pitchMin, cValues.pitchMax, 0, 255);
  int pitch = constrain(pMap, 0, 255);

  //transmitting to quad
  //for now, send throttle value
  if(millis() % 100 == 0) {
    Packet packet;
    packet.throttle_stick = throttle;
    packet.yaw_stick = yaw;
    packet.roll_stick = roll;
    packet.pitch_stick = pitch;
    packet.magicNumber = 1829;
    packet.battery = 0;
    packet.armed = armed;
    packet.Pr = pvals.Pr;
    packet.Ir = pvals.Ir;
    packet.Dr = pvals.Dr;
    packet.Py = pvals.Py;
    packet.Iy = pvals.Iy;
    packet.Dy = pvals.Dy;
    rfWrite((uint8_t*) (&packet), sizeof(packet));
  }

  //arm quadcopter from remote
  //our yaw and pitch are backwards
  if(!armed && throttle >= 0 && throttle <= 5 && yaw <= 255 && yaw >= 250 && roll <= 255  && roll >= 250 && pitch >= 250 && pitch <= 255){
    armed = true;
    Serial.println("armed!");
    char* armedMessage = "Quad armed!";
    lcd.clear();
    lcd.write(armedMessage);

    lcd.setFastBacklight(255, 0, 0);
  }

  //disarm using button 2
  if(armed && digitalRead(BUTTON2_PIN) == 0){
    armed = false;
    Serial.println("disarmed!");
    char* disarmed = "Quad disarmed!";
    lcd.clear();
    lcd.write(disarmed);

    lcd.setCursor(0, 1);
    lcd.print(batteryLevel);

    lcd.setCursor(13, 1);
    lcd.print(quadBattery);

    lcd.setFastBacklight(255, 255, 255);
  }

  //receive message from quadcopter
  bool prevArmed = armed;
  uint8_t buf[sizeof(Packet)];
  if(rfAvailable()) {
    rfRead(buf, sizeof(Packet));
    Packet* packet = (Packet*) buf;
    int magicNumber = packet->magicNumber;
    if (magicNumber == 1829) {
      armed = packet->armed;
      quadBattery = packet->battery;
    }
    if(!armed && prevArmed) {
      char* disarmed = "Quad disarmed!";
      lcd.clear();
      lcd.write(disarmed);

      lcd.setCursor(0, 1);
      lcd.print(batteryLevel);

      lcd.setCursor(13, 1);
      lcd.print(quadBattery);

      lcd.setFastBacklight(255, 255, 255);
    }
  }

  //for debugging calibration
  if (printOnce) {
    Serial.println("Before calibration mode");
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
    printOnce = false;
  }

  //print battery reading
  if(batteryCount == 1000) {
    readBattery();
    batteryCount = 0;
  }
  

  //enter calibration mode
  if(digitalRead(BUTTON1_PIN) == 0 && !armed) {
    calibrate = true;
    calibrationMode();
  }
  else if(digitalRead(BUTTON_CENTER_PIN) == 0 && prevCenterPushed != 0) {
    pidCalibrate = true;
    lastKnobPos = knob1.getCurrentPos();
    prevCenterPushed = digitalRead(BUTTON_CENTER_PIN);
    pidCalibrationMode();
  }

  //calibration fails if quad is armed
  // if(digitalRead(BUTTON1_PIN) == 0 && !armed) {
  //   char* deny = "Calibration failed quad is armed";
  //   lcd.clear();
  //   lcd.write(deny);
  // }
}

void readBattery() {
  Serial.print("Battery reading: ");
  int batteryRead = analogRead(BATTERY_SENSE_PIN);
  batteryLevel = map(batteryRead, 39, 57, 0, 100);
  Serial.println(batteryLevel);

  lcd.setCursor(0, 1);
  lcd.print(batteryLevel);

  lcd.setCursor(13, 1);
  lcd.print(quadBattery);
}


void pidCalibrationMode() {
  //write to lcd
  lcd.clear();
  char* string = "PID Calibrating...";
  lcd.write(string);
  // lcd.setFastBacklight(0, 255, 150);
  int toAdd = 0;
  while(pidCalibrate){
    lcd.setCursor(0, 1);
    if(knob1.getCurrentPos() > lastKnobPos){
      lastKnobPos = knob1.getCurrentPos();
      switch(pidCalMode){
        case 0: pvals.Pr += pInc; break;
        case 1: pvals.Ir += iInc; break;
        case 2: pvals.Dr += dInc; break;
        case 3: pvals.Py += pInc; break;
        case 4: pvals.Iy += iInc; break;
        case 5: pvals.Dy += dInc; break;
      }
    }
    else if(knob1.getCurrentPos() < lastKnobPos){
      lastKnobPos = knob1.getCurrentPos();
      switch(pidCalMode){
        case 0: pvals.Pr -= pInc; break;
        case 1: pvals.Ir -= iInc; break;
        case 2: pvals.Dr -= dInc; break;
        case 3: pvals.Py -= pInc; break;
        case 4: pvals.Iy -= iInc; break;
        case 5: pvals.Dy -= dInc; break;
      }
    }
    if(digitalRead(ENC1_BUTTON_PIN) == 0 && prevEncPushed !=0){
      switch(pidCalMode){
        case 0: pvals.Pr = 0; break;
        case 1: pvals.Ir = 0; break;
        case 2: pvals.Dr = 0; break;
        case 3: pvals.Py = 0; break;
        case 4: pvals.Iy = 0; break;
        case 5: pvals.Dy = 0; break;
      }
    }
    if(digitalRead(BUTTON1_PIN) == 0 && prevOnePushed !=0){
      switch(pidCalMode){
        case 0: pvals.Pr += pInc; break;
        case 1: pvals.Ir += iInc; break;
        case 2: pvals.Dr += dInc; break;
        case 3: pvals.Py += pInc; break;
        case 4: pvals.Iy += iInc; break;
        case 5: pvals.Dy += dInc; break;
      }
    }
    if(digitalRead(BUTTON_LEFT_PIN) == 0 && prevLeftPushed){
      switch(pidCalMode){
        case 0: pvals.Pr -= pInc; break;
        case 1: pvals.Ir -= iInc; break;
        case 2: pvals.Dr -= dInc; break;
        case 3: pvals.Py -= pInc; break;
        case 4: pvals.Iy -= iInc; break;
        case 5: pvals.Dy -= dInc; break;
      }
    }
    else if(digitalRead(BUTTON_UP_PIN) == 0 && prevUpPushed){
      switch(pidCalMode){
        case 0: pvals.Pr += (10 * pInc); break;
        case 1: pvals.Ir += (10 * iInc); break;
        case 2: pvals.Dr += (10 * dInc); break;
        case 3: pvals.Py += (10 * pInc); break;
        case 4: pvals.Iy += (10 * iInc); break;
        case 5: pvals.Dy += (10 * dInc); break;
      }
    }
    else if(digitalRead(BUTTON_RIGHT_PIN) == 0 && prevRightPushed != 0){
      switch(pidCalMode){
        case 0: pvals.Pr -= (10 * pInc); break;
        case 1: pvals.Ir -= (10 * iInc); break;
        case 2: pvals.Dr -= (10 * dInc); break;
        case 3: pvals.Py -= (10 * pInc); break;
        case 4: pvals.Iy -= (10 * iInc); break;
        case 5: pvals.Dy -= (10 * dInc); break;
      }
    }

    else if(digitalRead(BUTTON_DOWN_PIN) == 0 && prevDownPushed !=0){
      pidCalMode ++;
      if(pidCalMode > 5){
        pidCalMode = 0;
      }
    }
    
    prevOnePushed = digitalRead(BUTTON1_PIN);
    prevEncPushed = digitalRead(ENC1_BUTTON_PIN);
    prevDownPushed = digitalRead(BUTTON_DOWN_PIN);
    prevUpPushed = digitalRead(BUTTON_UP_PIN);
    prevLeftPushed = digitalRead(BUTTON_LEFT_PIN);
    prevRightPushed = digitalRead(BUTTON_RIGHT_PIN);
    switch(pidCalMode){
        case 0: {
          lcd.print("Pr: ");
          lcd.print(pvals.Pr);
          break;
        }
        case 1: {
          lcd.print("Ir: ");
          lcd.print(pvals.Ir);
          break;
        }
        case 2: {
          lcd.print("Dr: ");
          lcd.print(pvals.Dr);
          break;
        }
        case 3:{
          lcd.print("Py: ");
          lcd.print(pvals.Py);
          break;
        }
        case 4: {
          lcd.print("Iy: ");
          lcd.print(pvals.Iy);
          break;
        }
        case 5: {
          lcd.print("Dy: ");
          lcd.print(pvals.Dy);
          break;
        }
      }
    if(digitalRead(BUTTON_CENTER_PIN) == 0 && prevCenterPushed){
      wholeEeprom.gimbalVals = cValues;
      wholeEeprom.pidVals = pvals;
      EEPROM.put(0, wholeEeprom);
      pidCalibrate = false;
      calibrate = false;
      prevCenterPushed = digitalRead(BUTTON_CENTER_PIN); 
    }
    prevCenterPushed = digitalRead(BUTTON_CENTER_PIN);
  }

  lcd.clear();
  string = "Finished!";
  lcd.write(string);
  lcd.setFastBacklight(255, 255, 255);
  // Serial.println("Exiting calibration mode");
  // Serial.print("Throttle: ");
  // Serial.println(cValues.throttleMin);
  // Serial.println(cValues.throttleMax);
  // Serial.print("Yaw: ");
  // Serial.println(cValues.yawMin);
  // Serial.println(cValues.yawMax);
  // Serial.print("Roll: ");
  // Serial.println(cValues.rollMin);
  // Serial.println(cValues.rollMax);
  // Serial.print("Pitch: ");
  // Serial.println(cValues.pitchMin);
  // Serial.println(cValues.pitchMax);
}


void calibrationMode() {
  //write to lcd
  lcd.clear();
  char* string = "Calibrating...";
  lcd.write(string);
  lcd.setFastBacklight(0, 0, 255);

  int tDefault = analogRead(PIN_THROTTLE);
  int yDefault = analogRead(PIN_YAW);
  int rDefault = analogRead(PIN_ROLL);
  int pDefault = analogRead(PIN_PITCH);

  cValues.throttleMax = tDefault;
  cValues.throttleMin = tDefault;
  cValues.yawMax = yDefault;
  cValues.yawMin = yDefault;
  cValues.pitchMax = pDefault;
  cValues.rollMax = rDefault;
  cValues.rollMin = rDefault;

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
      wholeEeprom.gimbalVals = cValues;
      wholeEeprom.pidVals = pvals;
      EEPROM.put(0, wholeEeprom);
      calibrate = false;
    }
  }

  lcd.clear();
  string = "Finished!";
  lcd.write(string);
  lcd.setFastBacklight(255, 255, 255);
  // Serial.println("Exiting calibration mode");
  // Serial.print("Throttle: ");
  // Serial.println(cValues.throttleMin);
  // Serial.println(cValues.throttleMax);
  // Serial.print("Yaw: ");
  // Serial.println(cValues.yawMin);
  // Serial.println(cValues.yawMax);
  // Serial.print("Roll: ");
  // Serial.println(cValues.rollMin);
  // Serial.println(cValues.rollMax);
  // Serial.print("Pitch: ");
  // Serial.println(cValues.pitchMin);
  // Serial.println(cValues.pitchMax);
}