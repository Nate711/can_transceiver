#include "VESC.h"
#include "utils.h"
// #include <FlexCAN.h>

/**
 * Convert a vesc raw angle to a normalized degree angle,
 * which are defined as the angle
 * from the horizontal to the motor with positive downward, to
 **/
float VESC::vesc_to_normalized_angle(const float& raw_angle) {
  float normalized = raw_angle;

  // correct for encoder direction, ie if + angle is CW or CCW
  if(encoder_direction == -1) {
    normalized = utils_angle_difference(0, normalized);
  }

  // add encoder offset
  normalized += encoder_offset;

  // normalize to [0 360)
  utils_norm_angle(normalized);

  return normalized;
}

/**
 * Converts normalized angle to raw vesc angle
 * @param  normalized_angle [description]
 * @return                  [description]
 */
float VESC::normalized_to_vesc_angle(const float& normalized_angle) {
  float raw_angle = normalized_angle;

  // subtract offset
  raw_angle -= encoder_offset;

  // reverse if opposite direction
  if(encoder_direction == -1) {
    raw_angle = utils_angle_difference(0, raw_angle);
  }

  // normalize angle to 0 to 360
  utils_norm_angle(raw_angle);

  return raw_angle;
}

/**
 * Constructs VESC object with initial params
 * I dont understand member initializer lists :( but CANtx breaks without it
 */
VESC::VESC(float encoder_offset1,
          int encoder_direction1,
          float max_current1,
          float max_speed1,
          float Kp, float Kd,
          int8_t controller_channel_ID1,
          FlexCAN& cantx) : CANtx(cantx), pos_controller(Kp,Kd){

  encoder_offset = encoder_offset1;
  encoder_direction = encoder_direction1;

  controller_channel_ID = controller_channel_ID1;

  max_current = max_current1;
  max_speed = max_speed1;

  CANtx = cantx;

  // the first time_delta will be large and will give small deg per sec which is ok
  time_last_angle_read = 0;

  vesc_angle = 0.0;
  true_degps = 0;
}

/**
 * Compute PID output and send to VESC given a normalized angle set
 * point. Uses last given position values.
 * @param set_point normalized angle set point
 */
void VESC::pid_update_normalized(const float& set_point) {
  pid_update(normalized_to_vesc_angle(set_point));
}

/**
 * Compute PID output and send to VESC. Uses last given values
 */
void VESC::pid_update(const float& set_point) {
  float error = utils_angle_difference(vesc_angle,set_point);
  float cur_command = max_current *
            pos_controller.compute_command(error,last_time_delta_micros);

  // FUCK EVERYTHINGGGGG COMPARING FLOATS DOESNT WORK
  set_current(cur_command);
}

/**
 * Updates the VESC objects knowledge of the motor angle
 * Takes between 4 and 5 us with while loop norm angle
 * @param raw_deg measured position in raw vesc angle coordinates.
 * automatically normalizes the given angle (no > 180deg moves)
 */
void VESC::update_deg(const float& raw_deg) {
  unsigned long now_time = micros();

  float corrected = raw_deg;

  // 26 us without this line, 31 with this line = 5 us time
  utils_norm_angle(corrected);

  // Compute time since last deg update and
  last_time_delta_micros = now_time - time_last_angle_read;
  // POTENTIAL PROBLEM: if the main loop calls update_deg with the same angle
  // at different times then the VESC object will think the speed is 0
  time_last_angle_read = now_time;


  // Compute velocity in deg per s
  // This computation is subject to noise!
  // 37-38 us loop time
  // this line takes 6-8 us
  if(last_time_delta_micros==0) last_time_delta_micros = 1;

  true_degps = (int)(1000000*utils_angle_difference(corrected,vesc_angle)) /
                                        (int)(last_time_delta_micros);


  // 38-39 us loop time
  // true_degps = utils_angle_difference(corrected,true_deg) /
  //                                     (last_time_delta/1000000.0);

  // Update degree state
  vesc_angle = corrected;
}

/**
 * Returns the VESC object's last known knowledge of motor position
 */
float VESC::read_vesc_angle() {
  return vesc_angle;
}

float VESC::read_corrected_degps() {
  return true_degps;
}

void VESC::set_norm_position_target(float target) {
  normalized_position_target = target;
}

float VESC::get_norm_position_target() {
  return normalized_position_target;
}

void VESC::update_vesc_position_pid_constants(const float& kp, const float& ki, const float& kd) {
  if(vesc_kp == kp && vesc_ki == ki && vesc_kd == kd) {
    // Nothing to do, same values as last time
  } else {
    // Update internal memory and set flag to send CAN message
    vesc_kp = kp;
    vesc_ki = ki;
    vesc_kd = kd;
    update_pid_constants = true;
  }
}

void VESC::set_position_pid_constants() {
  set_position_pid_constants(vesc_kp, vesc_ki, vesc_kd,
    normalized_to_vesc_angle(normalized_position_target));
}

void VESC::set_position_pid_constants(float kp, float ki, float kd, float pos) {
  CAN_message_t msg;
  int MULTIPLIER = 100000; // max valu is .3 for any value (2^15 / 100000)
  msg.id = controller_channel_ID | ((int32_t) CAN_PACKET_SET_P_PID_K<<8);
  msg.len = 8;
  int32_t index = 0;
  buffer_append_int16(msg.buf,(int16_t)(kp*MULTIPLIER),&index);
  buffer_append_int16(msg.buf,(int16_t)(ki*MULTIPLIER),&index);
  buffer_append_int16(msg.buf,(int16_t)(kd*MULTIPLIER),&index);


  // JANK AF ADDING POS TO CONSTANT COMMAND
  int POS_MULTIPLIER = 50;
  // Multiplying by 50 means resolution of 0.02 degrees, which is less than the
  // encoder resolution of 0.07 degrees
  buffer_append_int16(msg.buf,(int16_t)(pos*POS_MULTIPLIER),&index);

  /*
	WACKO fix, doesn't work without the wait
	*/
	long time = micros();
	while(time - micros() < 1) {}

	CANtx.write(msg);
}

void VESC::set_normalized_position_with_constants() {
  // JUST USING SET POSITION PID CONSTANTS DOES NOT WORK
  if(update_pid_constants) {
    set_position_pid_constants();
  }
  set_position_normalized();
}

void VESC::set_position_normalized(const float& pos) {
  set_position(normalized_to_vesc_angle(pos));
}

void VESC::set_position_normalized() {
  set_position_normalized(normalized_position_target);
}


/**
* Sends position command to the VESC
**/
/**
 * Sends CAN position command to the VESC
 * @param pos desired position, automatically normalizes it
 */
void VESC::set_position(const float& pos) {
  float norm_pos = pos;
  utils_norm_angle(norm_pos);

  CAN_message_t msg;
  int MULTIPLIER = 1000000;
  msg.id = controller_channel_ID | ((int32_t) CAN_PACKET_SET_POS<<8);
  msg.len = 8;
  int32_t index = 0;
  buffer_append_int32(msg.buf,(int32_t)(pos*MULTIPLIER),&index);
  /*
	WACKO fix, doesn't work without the wait
	*/
	long time = micros();
	while(time - micros() < 1) {}

	CANtx.write(msg);
}

/**
 * Sends current command to the VESC
 */
void VESC::set_current(const float& current) {
  CAN_message_t msg;
  int MULTIPLIER = 1000;

	// (VESC will accept all 255 ID messages)
	// CAN_PACKET_SET_CURRENT byte goes on the left of the ID byte
	msg.id = controller_channel_ID | ((int32_t)CAN_PACKET_SET_CURRENT << 8);
	msg.len = 4; // 4 byte int

	int32_t index=0;
	buffer_append_int32(msg.buf, (int32_t)(current*MULTIPLIER), &index);

  /*
	WACKO fix, doesn't work without the wait
	*/
	long time = micros();
	while(time - micros() < 1) {}

	CANtx.write(msg);
}

void VESC::print_debug() {
	if(last_print_debug > 100) {
		last_print_debug = 0;

		Serial.print("O: ");
		Serial.print(pos_controller.get_command());
		Serial.print(" \tEr: ");
		Serial.print(pos_controller.get_error());
		Serial.print(" \tEr.w:  ");
		Serial.print(pos_controller.get_error_deriv());
    Serial.print(" \tw: ");
    Serial.print(true_degps);
		Serial.print(" \tKp: ");
		Serial.print(pos_controller.get_pterm());
		Serial.print(" \tKd: ");
		Serial.println(pos_controller.get_dterm());
	}
}
