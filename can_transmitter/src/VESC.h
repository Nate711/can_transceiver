#ifndef VESC_H
#define VESC_H

#include <Arduino.h>
#include <FlexCAN.h>
#include "buffer.h"
#include <kinetis_flexcan.h>
#include "datatypes.h"
#include "AngularPDController.h"
#include "utils.h"

class VESC {
private:
  // Holds time in micros of the last time the object got an angle measurement
  // It's important that this time is consistent
	long time_last_angle_read;

  // Holds the most recent time delta between measurements in MICROS
  long last_time_delta;

  // Offset and direction for encoder
  float encoder_offset;
  // it might be a bad idea to have direction multipliers... think about it
  int encoder_direction;

  int8_t controller_channel_ID;

  float max_current = 10; // amps
  float max_speed = 1000; // deg per sec

  // corrected position of the motor
  float true_deg;
  float true_degps;


  // FlexCAN object to use for comms
  FlexCAN& CANtx;

  // PID object
  AngularPDController pos_controller;

  //
  unsigned long last_print_debug = 0;

public:
  VESC(float encoder_offset1,
            int encoder_direction1,
            float max_current1,
            float max_speed1,
            float Kp, float Kd, float threshold,
            int8_t controller_channel_ID1,
            FlexCAN& cantx);

  void update_deg(float deg);
  float read_corrected_deg();

  void set_current(float current);

  void pid_update(float set_point);

  void print_debug();
};

#endif
