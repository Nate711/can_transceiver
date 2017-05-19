#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include "datatypes.h"
#include "AngularPDController.h"
#include "constants.h"

#include "buffer.h"
#include "utils.h"


// Teensy State variable
enum controller_state {
	IDLE,
	IDLE_BUTTON_DOWN,
	RUNNING,
	ESTOP
};
controller_state teensy_state = IDLE;

// Create CAN object
FlexCAN CANTransceiver(500000); // default is 500k baud
static CAN_message_t msg;

// Longs to keep track of the last time a debugging print message was sent
// and last time
unsigned long last_send = micros();
unsigned long last_print_debug = micros();
unsigned long last_print_motor_status = micros();

/**
 * Struct to keep track of PID constants: Should be obsolete with PD class
 **/
struct PD_Constants {
	float Kp;
	// float Ki;
	float Kd;
} VESC_pd_k;

/**
 * Struct to keep track of the motor state: Should replace with a motor class
 **/
struct Motor_State {
	float last_angle;
	float last_degps;
	long time_last_angle_read;
} left_tmotor_state,right_tmotor_state;

// PD controller objects to control the VESCs
AngularPDController right_vesc_pid_controller(-0.07,
																							-0.0005,
																							1.0);
AngularPDController left_vesc_pid_controller(-0.07,
																							-0.0005,
																							1.0);

/**
* frequency is in regular freq, not angular freq, so like 1Hz = 1 RPS
* multiplier cannot be bigger than 3 million
**/
int continuousRotationCommand(const float& seconds, const int& multiplier, const float& frequency) {
	// millis/1000 gives seconds, multiplying by 360 gives rotations
	// multiplying by frequency gives rotations/sec
	// %36000 /100 wraps the angle around 360 while at the same time giving
	// resolution of 0.01 deg
	return multiplier / 100 * ((int)(frequency*360.0
		*seconds*100.0)%36000);
}

/**
* freq is not angular freq
**/
int squareWaveCommand(const float& seconds, const int& multiplier, float frequency, float amplitude) {
	//
	int timestamp = (int)(seconds*10000); // in tenths of milliseconds
	int period = (int)(10000/frequency); // in tenths of milliseconds
	int half_period = period/2;

	return multiplier * ((timestamp % period)/half_period)
		*amplitude;
}
/**
 * Check and read rotor angle over CAN
 * Returns true and stores value if message received, otherwise ret false
 **/
bool readAngleOverCAN(FlexCAN& CANrx, float& last_angle_received, int& transmitter_ID) {
	// NOTE: will parse thru multiple angles if >1 messages received btn loops

	// keep track if a value was received
	bool read = false;

	// parse all available can messages
	while( CANrx.read(msg)) {
		read = true;

		// Very useful, this is the ID encoded by the transitting CAN device
		transmitter_ID = msg.id & 0xFF;

    // toggle LEDs
    // digitalWrite(led, !digitalRead(led));

		// dummy variable for buffer get functino
		int32_t index=0;

		// parse a 32bit float from the can message data buffer
		last_angle_received = buffer_get_float32(msg.buf, 100000, &index);
  }
	return read;
}

/**
 * Sends a rotor position command over CAN
 * rotor angle must already be multiplied by 1 million
**/
void sendMultipliedAngleOverCAN(FlexCAN& CANtx, const uint32_t& multiplied_angle, const int8_t& controller_id) {
	CAN_message_t msg;

	// use a controller ID of 255 (VESC will accept all 255 ID messages)
	// CAN_PACKET_SET_POS byte goes on the left of the ID byte
	msg.id = controller_id | ((int32_t)CAN_PACKET_SET_POS << 8);
	msg.len = 4; // 4 byte int

	int32_t index=0;

	// set message data buffer to multiplied angle, multiple must be 1million
	buffer_append_int32(msg.buf, multiplied_angle, &index);

	// send!
	CANtx.write(msg);

	// toggle LED
	// digitalWrite(led, !digitalRead(led));
}

/**
 * Sends current command over CANtx
 * Multiplier is 1000
 **/
void sendMultipliedCurrentOverCAN(FlexCAN& CANtx, const int32_t& current, const int8_t& controller_id) {

	CAN_message_t msg;

	// use a controller ID of 255 (VESC will accept all 255 ID messages)
	// CAN_PACKET_SET_CURRENT byte goes on the left of the ID byte
	msg.id = controller_id | ((int32_t)CAN_PACKET_SET_CURRENT << 8);
	msg.len = 4; // 4 byte int

	int32_t index=0;

	buffer_append_int32(msg.buf, current, &index);

	long time = micros();
	while(time - micros() < 1) {}
	/*
	WEIRDEST FUCKING THING EVER:
	if I delete Serial.print(" ");
	and Serial.println();

	Neither line alone suffices, but adding more print statements also works

	the esc freezes up

	i suspect it's a can error where I can't send a message right after I receive
	one

	OK: waiting for 1 microsecond also works fine
	*/

	CANtx.write(msg);
}

/*
float current_command = MAX_CURRENT*positionPidControl(last_read_angle,
		tmotor_state.last_angle,
		angle_set_point,
		time_delta,
		VESC_pd_k);
*/

// float positionPidControl(const float& angle_now,
// 	float& angle_last,
// 	const float& set_point,
// 	const int& microsTimeDelta,
// 	const PD_Constants& pd_k) {
//
// 	/**
// 	* fucking weird error:
// 	* if angle_diff defined and calced up here (before error is calced), the vesc freezes up
// 	TODO: figure out what the freeze actually is!!
// 	*/
//
// 	float error = utils_angle_difference(angle_now, set_point);
// 	float angle_diff = utils_angle_difference(angle_now, angle_last);
//
//
// 	float error_deriv = (angle_diff)*1000000.0 /
// 		(float)microsTimeDelta; // deg /s
//
// 	// error_deriv = 0.5*error_deriv + 0.5*last_degps;
// 	tmotor_state.last_degps = error_deriv;
//
// 	angle_last = angle_now;
//
// 	// NEGATIVE SIGN IS IMPORTANT
// 	float res =  -(error*pd_k.Kp + error_deriv*pd_k.Kd);
// 	res = constrain(res, -1.0, 1.0);
//
// 	if(micros() - last_send > 50*1000) {
// 		last_send = micros();
//
// 		Serial.print("O: ");
// 		Serial.print(res);
// 		Serial.print(" \tEr: ");
// 		Serial.print(error);
// 		Serial.print(" \tw:  ");
// 		Serial.print(error_deriv);
// 		Serial.print(" \tKp: ");
// 		Serial.print(error*pd_k.Kp);
// 		Serial.print(" \tKd: ");
// 		Serial.println(error_deriv*pd_k.Kd);
// 	}
//
// 	/**
// 	* sets a zero current if the speed is less than or greater than 100deg/s
// 	**/
// 	if(abs(error_deriv) > 1000) return 0;
//
// 	// if(abs(error) < 1.0) {
// 		// res = constrain(res, -0.5, 0.5);
// 	// }
//
// 	return res;
// }

void setup() {
	// init stop button
	pinMode(e_stop_pin,INPUT_PULLUP);

  // init CAN bus
  CANTransceiver.begin();
  pinMode(led, OUTPUT);
	digitalWrite(led, HIGH);
  delay(1000);
  Serial.println("CAN Transmitter Initialized");

	left_tmotor_state.time_last_angle_read = micros();
	right_tmotor_state.time_last_angle_read = micros();


	// VESC_pd_k.Kp = 0.07;
	// VESC_pd_k.Kd = 0.0015;
	VESC_pd_k.Kp = 0.05;
	VESC_pd_k.Kd = 0.0005;

	// INITIALIZE ALL YOUR THINGS
	left_tmotor_state.last_angle = 0.0; // 0 degrees is a safe angle for the motor
	left_tmotor_state.last_degps = 0.0;
	left_tmotor_state.time_last_angle_read = micros();

	right_tmotor_state.last_angle = 180.0; // 180 is safe for the motor
	right_tmotor_state.last_degps = 0.0;
	right_tmotor_state.time_last_angle_read = micros();
}

void print_motor_status() {
	if(micros() - last_print_motor_status > 100*1000) {
		last_print_motor_status = micros();
		Serial.print("Left Angle: ");
		Serial.print(left_tmotor_state.last_angle);
		Serial.print("\tRight Angle: ");
		Serial.print(right_tmotor_state.last_angle);
		Serial.print("\tLeft Angle Target: ");
		Serial.print(utils_angle_difference(180.0, right_tmotor_state.last_angle));
		Serial.print("\tRight Angle Target: ");
		Serial.println(utils_angle_difference(180.0, left_tmotor_state.last_angle));
	}
}

void print_debug() {
	if(micros()-last_print_debug > 100*1000) {
		last_print_debug = micros();

		Serial.print("O: ");
		Serial.print(left_vesc_pid_controller.get_command());
		Serial.print(" \tEr: ");
		Serial.print(left_vesc_pid_controller.get_error());
		Serial.print(" \tw:  ");
		Serial.print(left_vesc_pid_controller.get_error_deriv());
		Serial.print(" \tKp: ");
		Serial.print(left_vesc_pid_controller.get_pterm());
		Serial.print(" \tKd: ");
		Serial.print(left_vesc_pid_controller.get_dterm());

		Serial.print("\tO: ");
		Serial.print(right_vesc_pid_controller.get_command());
		Serial.print(" \tEr: ");
		Serial.print(right_vesc_pid_controller.get_error());
		Serial.print(" \tw:  ");
		Serial.print(right_vesc_pid_controller.get_error_deriv());
		Serial.print(" \tKp: ");
		Serial.print(right_vesc_pid_controller.get_pterm());
		Serial.print(" \tKd: ");
		Serial.println(right_vesc_pid_controller.get_dterm());
	}
}

void print_status() {
	if(micros() - last_print_debug > 100*1000) {
		last_print_debug = micros();

		Serial.print("State: ");
		switch(teensy_state) {
			case IDLE:
				Serial.println("IDLE");
				break;
			case IDLE_BUTTON_DOWN:
				Serial.println("IDLE BUTTON DOWN");
				break;
			case ESTOP:
				Serial.println("E-STOP");
				break;
			case RUNNING:
				Serial.println("RUNNING!");
				break;
		}
	}
}

void loop() {
	switch(teensy_state) {
		case IDLE:
			print_status();
		  if(digitalReadFast(e_stop_pin) == LOW) {
				teensy_state = IDLE_BUTTON_DOWN;
			}
			break;

		case IDLE_BUTTON_DOWN:
			print_status();
		  if(digitalReadFast(e_stop_pin) == HIGH) {
				teensy_state = RUNNING;
				delay(100);

			}
			break;

		case ESTOP:
			// Handle the ESTOP behavior: if it has been pressed or is currently pressed
			// then send a zero current command over to the VESC and delay 500ms
			sendMultipliedCurrentOverCAN(CANTransceiver,0,CONTROLLER_ID_FOR_RIGHT_VESC);
			sendMultipliedCurrentOverCAN(CANTransceiver,0,CONTROLLER_ID_FOR_LEFT_VESC);

			print_status();
			delay(100);

			break;

		case RUNNING:
			if(digitalReadFast(e_stop_pin) == LOW) {
				teensy_state = ESTOP;
				break; // redundant
			} else {

				float last_read_angle;
				int transmitter_ID;
				if(readAngleOverCAN(CANTransceiver,last_read_angle,transmitter_ID)) {
					if(transmitter_ID == CONTROLLER_ID_FOR_RIGHT_VESC) {
						right_tmotor_state.last_angle = last_read_angle;

						/************ PID CONTROL **********/

						// float angle = continuousRotationCommand(millis()/1000.0, 1000000, 0.5)/1000000.0;
						// float angle = squareWaveCommand(millis()/1000.0, 1000, 25, 5)/1000.0;
						// RIGHT MOTOR CENTERS AT 180
						// haptic feedback version
						float angle_set_point = utils_angle_difference(180, left_tmotor_state.last_angle);
						// angle_set_point = constrain(angle_set_point, 140, 220);
						// float angle_set_point = 180.0-60.0;

						long time_delta = micros() - right_tmotor_state.time_last_angle_read;

						float error = utils_angle_difference(right_tmotor_state.last_angle, angle_set_point);
						float current_command = MAX_CURRENT*right_vesc_pid_controller.compute_command(error, time_delta/(1000000.0));

						// NOT WORKING BC FUCKING GET_ERROR_DERIV
						// SAFETY: set current to zero if ang vel > 1000 degree per second
						// long now = micros();
						float d = right_vesc_pid_controller.get_error_deriv();
						// long then = micros();
						// Serial.println(then-now);
						if(d > MAX_ANGULAR_VEL || d < -MAX_ANGULAR_VEL) {
							Serial.print("ERROR DERIV: ");
							Serial.println(d);
							// e_stop_pressed = true;
							current_command = 0.0;
						}

						right_tmotor_state.time_last_angle_read = micros();

						sendMultipliedCurrentOverCAN(CANTransceiver,
								(int32_t)(current_command * 1000.0),
								CONTROLLER_ID_FOR_RIGHT_VESC);
						// print_debug();

						print_motor_status();
						/*

						float offset_angle = 180 - right_tmotor_state.last_angle;
						//offset_angle = constrain(offset_angle, 160, 220);

						int32_t command = (offset_angle) * 1000000;
						sendMultipliedAngleOverCAN(CANTransceiver, command,CONTROLLER_ID_FOR_LEFT_VESC);
						*/
					}
					if(transmitter_ID == CONTROLLER_ID_FOR_LEFT_VESC) {
						left_tmotor_state.last_angle = last_read_angle;

						/************ PID CONTROL **********/

						// follow the right motor for haptic like behavior
						float angle_set_point = utils_angle_difference(180,
																									right_tmotor_state.last_angle);
						angle_set_point = constrain(angle_set_point, -60, 60);
						// so if right motor is at 170 degrees, the left motor should be at 10 degrees

						// float angle_set_point = 0.0;

						long time_delta = micros() - left_tmotor_state.time_last_angle_read;

						float error = utils_angle_difference(left_tmotor_state.last_angle, angle_set_point);
						float current_command = MAX_CURRENT*left_vesc_pid_controller.compute_command(error, time_delta/(1000000.0));

						left_tmotor_state.time_last_angle_read = micros();

						// SAFETY: set current to zero if ang vel > 1000 degree per second
						float d = left_vesc_pid_controller.get_error_deriv();
						if(d > MAX_ANGULAR_VEL || d < -MAX_ANGULAR_VEL) {
							Serial.print("ERROR DERIV: ");
							Serial.println(d);
							// e_stop_pressed = true;
							current_command = 0.0;
						}

						sendMultipliedCurrentOverCAN(CANTransceiver,
								(int32_t)(current_command * 1000.0),
								CONTROLLER_ID_FOR_LEFT_VESC);
						// print_debug();
						print_motor_status();
						/*

						int32_t command = (180.0-left_tmotor_state.last_angle) * 1000000;
						sendMultipliedAngleOverCAN(CANTransceiver, command,CONTROLLER_ID_FOR_RIGHT_VESC);
						*/
					}
				}
			}
			break;

/* OLD ESTOP CODE
	// }
	// // Handle the ESTOP behavior: if it has been pressed or is currently pressed
	// // then send a zero current command over to the VESC and delay 10ms
	// if(e_stop_pressed || (digitalReadFast(e_stop_pin) == LOW)) {
	// 	e_stop_pressed = true;
	// 	sendMultipliedCurrentOverCAN(CANTransceiver,0,CONTROLLER_ID_FOR_RIGHT_VESC);
	// 	sendMultipliedCurrentOverCAN(CANTransceiver,0,CONTROLLER_ID_FOR_LEFT_VESC);
	// 	Serial.println("E-Stop Pressed");
	// 	delay(200);
	// } else {
	*/

		// operate this loop at 500hz
		/*
		if(micros() - last_send > 2*1000) {
			last_send = micros();

			int32_t command = continuousRotationCommand(millis()/1000.0, 1000000, 0.5);

			sendMultipliedAngleOverCAN(CANTransceiver,command);
			// Serial.println(command/1000000);
		}
		*/
	}

}
