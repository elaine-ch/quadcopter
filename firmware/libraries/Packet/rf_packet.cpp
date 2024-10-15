#ifndef rf_packet_h
#define rf_packet_h

#include <Arduino.h>

struct Packet {
    int propFrontLeft;
    int propFrontRight;
    int propBackLeft;
    int propBackRight;
    //int LED[8];
    int magicNumber;
    int battery;
    bool armed;
};

#endif