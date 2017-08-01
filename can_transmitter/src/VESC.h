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
	// Position target used for either on-teensy pos control (not yet implemented)
	// or for on-vesc position control
	float normalized_position_target;

	// VESC position pid constants
	float vesc_kp=0,vesc_ki=0,vesc_kd=0;

	// Flag to tell if position pid constants need to be sent to VESC
	bool update_pid_constants = true;

  // Holds time in micros of the last time the object got an angle measurement
  // It's important that this time is consistent
	long time_last_angle_read;

  // Holds the most recent time delta between measurements in MICROS
  long last_time_delta_micros=30;

  // Offset and direction for encoder
  float encoder_offset;
  // it might be a bad idea to have direction multipliers... think about it
  int encoder_direction;

	// CAN bus channel ID
  int8_t controller_channel_ID;

  float max_current = 10; // amps
  float max_speed = 2000; // deg per sec

  // corrected position and velocity of the motor
  float vesc_angle=0;
  float true_degps=0;

  // FlexCAN object to use for comms
  FlexCAN& CANtx;

  // PID object to control angular position
  AngularPDController pos_controller;

  // keep track of elapsed milliseconds since last prints
  elapsedMillis last_print_debug=0;
	elapsedMillis print_w=0;

	float vesc_to_normalized_angle(const float& raw_angle);
	float normalized_to_vesc_angle(const float& normalized_angle);

public:
  VESC(float encoder_offset1,
            int encoder_direction1,
            float max_current1,
            float max_speed1,
            float Kp, float Kd,
            int8_t controller_channel_ID1,
            FlexCAN& cantx);

	/**
	 * Updates rotation state and calculates speed
	 * @param deg absolute encoder angle sent over CAN
	 */
  void update_deg(const float& deg);

	/**
	 * Gets the absolute angle
	 * @return [description]
	 */
  float read_vesc_angle();

	/**
	 * Returns the speed of the motor. 'corrected' no longer means anything
	 * @return speed of motor
	 */
	float read_corrected_degps();

  /**
   * Not implemented?
   * @param sleep_time [description]
   */
	void reset(int sleep_time);

	/**
	 * Sends a CAN message to VESC to set the motor current
	 * @param current [description]
	 */
  void set_current(const float& current);

	/**
	 * Setter for normalized position target variable. Holds on to the target so
	 * you don't have to worry about making a variable in meain to set the target.
	 * @param target
	 */
	void set_norm_position_target(float target);

	/**
	 * Getter for normalized position target variable. Holds on to the target so
	 * you don't have to worry about making a variable in meain to set the target.
	 * @param target
	 */
	float get_norm_position_target();

	/**
	 * Updates internal memory of vesc pid constants and sets flag to send CAN
	 * message with new values.
	 * @param kp [description]
	 * @param ki [description]
	 * @param kd [description]
	 */
	void update_vesc_position_pid_constants(const float& kp, const float& ki, const float& kd);

	/**
	 * Sends a CAN message with the new pid constants to the VESC
	 * @param kp [description]
	 * @param ki [description]
	 * @param kd [description]
	 */
	void set_position_pid_constants(const float& kp, const float& ki, const float& kd);

	/**
	 * Sends CAN message using stored pid constants to the VESC
	 */
	void set_position_pid_constants();

	/**
	 * Sends position CAN message to VESC given a normalized angle
	 * @param pos [description]
	 */
	void set_position_normalized(const float& pos);
	/**
	 * Uses private normalized target position variable
	 */
	void set_position_normalized();

	/**
	 * Sends position CAN message to VESC given an absolute angle
	 * @param pos [description]
	 */
	void set_position(const float& pos);

	/**
	 * Sends CAN message to VESC with position and pid constants if pid constants
	 * need updating.
	 */
	void set_normalized_position_with_constants();

	/**
	 * Uses teensy pid to send current command to VESC given a normalized
	 * angle target
	 * @param set_point [description]
	 */
	void pid_update_normalized(const float& set_point);

	/**
	 * Same as above except uses absolute angle
	 * @param set_point [description]
	 */
  void pid_update(const float& set_point);

	/**
	 * Prints VESC object state
	 */
  void print_debug();
};

#endif
