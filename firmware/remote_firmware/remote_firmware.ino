#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include "rf_packet.h"
#include <EEPROM.h>
bool calibrate = false;
bool armed = false;
bool printOnce = true;
int batteryCount = 0;
int quadBattery = 0;
int batteryLevel = 0;

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

  calibrate = false;
  armed = false;
  lcd.setFastBacklight(255, 255, 255);

  rfFlush();

  //tell quad about reset
  Packet packet;
  packet.propFrontLeft = 0;
  packet.propFrontRight = 0;
  packet.propBackLeft = 0;
  packet.propBackRight = 0;
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
  } else {
    EEPROM.get(0, cValues);
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
    packet.propFrontLeft = throttle;
    packet.propFrontRight = throttle;
    packet.propBackLeft = throttle;
    packet.propBackRight = throttle;
    packet.magicNumber = 1829;
    packet.battery = 0;
    packet.armed = armed;
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
      EEPROM.put(0, cValues);
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