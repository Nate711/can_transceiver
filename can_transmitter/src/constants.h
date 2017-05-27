#include <Arduino.h>

// built in led pin
int led = 13;

// e-stop button pin
int e_stop_pin = 2;
bool e_stop_pressed = false;

const float MAX_CURRENT = 8.0;
const float MAX_ANGULAR_VEL = 2000; // deg per sec

const int8_t LM_CHANNEL_ID = 0;
const int8_t RM_CHANNEL_ID = 1;

const int PID_PERIOD = 1000; // us
