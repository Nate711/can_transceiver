#include <Arduino.h>

// built in led pin
int led = 13;

// e-stop button pin
int e_stop_pin = 2;
bool e_stop_pressed = false;

const float MAX_CURRENT = 8.0;
const float MAX_ANGULAR_VEL = 2000; // deg per sec

const int8_t CONTROLLER_ID_FOR_LEFT_VESC = 0;
const int8_t CONTROLLER_ID_FOR_RIGHT_VESC = 1;
