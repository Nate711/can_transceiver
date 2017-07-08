#include "VESC.h"
// #include <FlexCAN.h>

void utils_norm_angle(float& angle);

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

  true_deg = 0.0;
  true_degps = 0;
}

/**
 * Compute PID output and send to VESC. Uses last given values
 */

void VESC::pid_update(const float& set_point) {

  float error = utils_angle_difference(read_corrected_deg(),set_point);
  float cur_command = max_current *
            pos_controller.compute_command(error,last_time_delta_micros);

  // FUCK EVERYTHINGGGGG COMPARING FLOATS DOESNT WORK
  set_current(cur_command);
}

/**
 * Updates the VESC objects knowledge of the motor angle
 * Takes between 4 and 5 us with while loop norm angle
 */
void VESC::update_deg(const float& raw_deg) {
  unsigned long now_time = micros();

  // Apply correction formula
  float corrected = (encoder_direction==1)?raw_deg:(-raw_deg) + encoder_offset;
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

  true_degps = (int)(1000000*utils_angle_difference(corrected,true_deg)) /
                                        (int)(last_time_delta_micros);


  // 38-39 us loop time
  // true_degps = utils_angle_difference(corrected,true_deg) /
  //                                     (last_time_delta/1000000.0);

  // Update degree state
  true_deg = corrected;
}

/**
 * Returns the VESC object's last known knowledge of motor position
 */
float VESC::read_corrected_deg() {
  return true_deg;
}

float VESC::read_corrected_degps() {
  return true_degps;
}

/**
* Sends position command to the VESC
**/

void VESC::set_position(const float& pos) {
  CAN_message_t msg;
  int MULTIPLIER = 1000000;
  msg.id = controller_channel_ID | ((int32_t) CAN_PACKET_SET_POS<<8);
  msg.len = 4;
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
