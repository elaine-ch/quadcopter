#ifndef rf_packet_h
#define rf_packet_h

#include <Arduino.h>

struct Packet {
    int throttle_stick;
    int yaw_stick;
    int pitch_stick;
    int roll_stick;
    //int LED[8];
    int magicNumber;
    int battery;
    bool armed;
    float Pr, Ir, Dr, Py, Iy, Dy;
    float yawTrim, rollTrim, pitchTrim;
    bool wink;
};

#endif