#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "Arduino.h"
// built in led pin
int led = 13;

// e-stop button pin
int e_stop_pin = 2;
bool e_stop_pressed = false;

const float MAX_CURRENT = 30.0;
const float MAX_ANGULAR_VEL = 500; // deg per sec

const int8_t LM_CHANNEL_ID = 0;
const int8_t RM_CHANNEL_ID = 1;

// N side motor
// increases CW when facing N side
const float RIGHT_VESC_DOWN = 226;
const float RIGHT_VESC_UP = 36;

// K side motor
// increases CCW when facing N side
const float LEFT_VESC_DOWN = 140;
const float LEFT_VESC_UP = 320;

#endif
