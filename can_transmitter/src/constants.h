#ifndef CONSTANTS_H
#define CONSTANTS_H
#include "Arduino.h"
// built in led pin
int led = 13;

// e-stop button pin
int e_stop_pin = 33; // Pin 2 on old teensy 3.2
bool e_stop_pressed = false;

const float MAX_CURRENT = 8.0; // 30 amps seems the max
const float MAX_ANGULAR_VEL = 500;  // deg per sec

const float KP = -0.07;
const float KD = -0.001;//-0.0005;


const int8_t LM_CHANNEL_ID = 0;
const int8_t RM_CHANNEL_ID = 1;

// N side motor
// increases CW when facing N side
const float RIGHT_VESC_DOWN = 226;
const float RIGHT_VESC_UP = 36;

const float RIGHT_VESC_OFFSET = 306.0;
const int RIGHT_VESC_DIRECTION = -1;

// K side motor
// increases CCW when facing N side
const float LEFT_VESC_DOWN = 140;
const float LEFT_VESC_UP = 320;

const float LEFT_VESC_OFFSET = 314.0;
const int LEFT_VESC_DIRECTION = 1;


const int PID_PERIOD = 1000; // us

#endif
