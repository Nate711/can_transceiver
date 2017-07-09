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
  long last_time_delta_micros=30;

  // Offset and direction for encoder
  float encoder_offset;
  // it might be a bad idea to have direction multipliers... think about it
  int encoder_direction;

  int8_t controller_channel_ID;

  float max_current = 10; // amps
  float max_speed = 2000; // deg per sec

  // corrected position of the motor
  float true_deg=0;

  float true_degps=0;


  // FlexCAN object to use for comms
  FlexCAN& CANtx;

  // PID object to control angular position
  AngularPDController pos_controller;

  //
  elapsedMillis last_print_debug=0;

	elapsedMillis print_w=0;


public:
  VESC(float encoder_offset1,
            int encoder_direction1,
            float max_current1,
            float max_speed1,
            float Kp, float Kd,
            int8_t controller_channel_ID1,
            FlexCAN& cantx);

  void update_deg(const float& deg);
  float read_corrected_deg();
	float read_corrected_degps();

	void reset(int sleep_time);

  void set_current(const float& current);

	void set_pid_position_constants(const float& kp, const float& ki, const float& kd);

	void set_position(const float& pos);

  void pid_update(const float& set_point);

  void print_debug();
};

#endif
