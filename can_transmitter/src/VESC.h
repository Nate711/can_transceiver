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
	float pos=0;

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

	float vesc_to_normalized_angle(float raw_angle);
	float normalized_to_vesc_angle(float normalized_angle);

	/**
	 * Sends a CAN message to VESC to set the motor current
	 * @param current : desired signed current
	 */
	void send_current(float current);

	/**
	 * Sends position CAN message to VESC given an absolute angle
	 * @param pos [description]
	 */
	void send_position(float pos);

	/**
	 * Sends a CAN message with the new pid constants and position to the VESC
	 * @param kp [description]
	 * @param ki [description]
	 * @param kd [description]
	 * @param pos
	 */
	void send_position_pid_constants(float kp, float ki, float kd,
		float pos);


public:
  VESC(float encoder_offset1,
        int encoder_direction1,
        float max_current1,
        float max_speed1,
        float Kp, float Kd,
        int8_t controller_channel_ID1,
        FlexCAN& cantx);

	/**
	 * Sends position CAN message to motor to update position hold command.
	 * Currently only implements VESC-side position hold
	 * @param deg normalized target angle in degrees
	 */
	void write(float deg);

	/**
	 * Sends CAN message to set current
	 * @param current desired current in amps
	 */
	void write_current(float current);
	/**
	 * Sends CAN message to update position PID gains and position
	 * @param kp P term gain
	 * @param ki I term gain
	 * @param kd D term gain
	 * @param pos : normalized target position
	 */
	void write_pos_and_pid_gains(float kp, float ki,
		float kd,float pos);

	/**
	 * Returns the last read normalized motor position in degrees. Note
	 * that the motor position read is not the commanded position, but
	 * the actual, last-read motor position
	 * @return motor position
	 */
	float read();

	/**
	 * Sets up the vesc object to talk over this CAN ID channel
	 * @param CANID : Which CAN ID to use for this communication channel
	 */
	void attach(int CANID);

	/**
	 * De-initializes the VESC object and sends a zero-current command to halt
	 * the VESC
	 */
	void detach();

	/**
	 * Updates rotation state and calculates speed
	 * @param deg absolute encoder angle sent over CAN
	 */
  void update_deg(const float& deg);

  /**
   * Not implemented?
   * @param sleep_time [description]
   */
	void reset(int sleep_time);

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
