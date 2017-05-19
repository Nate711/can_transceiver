#ifndef VESC_H
#define VESC_H

#include <Arduino.h>
#include <FlexCAN.h>
#include "buffer.h"
#include <kinetis_flexcan.h>
#include "datatypes.h"

class VESC {
private:
  float last_angle;
	float last_degps;
	long time_last_angle_read;

  float encoder_offset;

  // it might be a bad idea to have direction multipliers... think about it
  int encoder_direction;

  int8_t controller_channel_ID;

  float max_current = 10; // amps
  float max_speed = 1000; // deg per sec

  // corrected position of the motor
  float true_deg;

  // FlexCAN object to use for comms
  FlexCAN& CANtx;

public:
  VESC(float encoder_offset1,
            int encoder_direction1,
            int8_t controller_channel_ID1,
            float max_current1,
            float max_speed1,
            FlexCAN& cantx);

  void update_deg(float deg);
  float read_corrected_deg();

  void set_current(float current);
};

#endif
