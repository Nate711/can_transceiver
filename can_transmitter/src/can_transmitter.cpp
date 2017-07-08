#include <FlexCAN.h>
#include "AngularPDController.h"
#include "constants.h"
#include "buffer.h"
#include "utils.h"
#include "VESC.h"

long loop_time=0;

// Teensy State variable
enum controller_state {
	IDLE,
	IDLE_BUTTON_DOWN,
	RUNNING,
	ESTOP
};
controller_state teensy_state = IDLE;

// Create CAN object, 500mbps, CAN0, use alt tx and alt rx
FlexCAN CANTransceiver(500000,0,1,1);
static CAN_message_t msg;

// Longs to keep track of the last time a debugging print message was sent
// and last time
elapsedMillis last_send;
elapsedMillis last_print_debug;
elapsedMillis last_print_motor_status;
elapsedMillis last_print_shit;

/**
 * Struct to keep track of PID constants: Should be obsolete with PD class
 **/
struct PD_Constants {
	float Kp;
	// float Ki;
	float Kd;
} VESC_pd_k;

// VESC motor objects
VESC left_vesc(0, // offset
	1, // direction
	MAX_CURRENT, // max current
	MAX_ANGULAR_VEL, // max speed
	KP, // Kp
	KD, // Kd //.5 thou default
	LM_CHANNEL_ID,CANTransceiver); // CAN channel id and flexcan
VESC right_vesc(0, // offset
	1, // direction
	MAX_CURRENT, // max current
	MAX_ANGULAR_VEL, // max speed
	KP, // Kp
	KD, // Kd
	RM_CHANNEL_ID,CANTransceiver); // CAN channel id and flexcan

// SUPER IMPORTANT PID TARGETS
float right_vesc_target = 180; // CCW turn by 40deg
float left_vesc_target = 100; // CW turn by 40 deg


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
// 	if(last_send > 50) {
// 		last_send -= 50;
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
}


void print_status() {
	if(last_print_debug > 100) {
		last_print_debug -= 100;

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

void print_shit() {
	if(last_print_shit > 10) {
		last_print_shit -= 10;

		Serial.println(loop_time);
	}
}

void process_serial() {
	if(Serial.available()) {
		char c = Serial.read();

		// Process each different command
		switch(c) {
			case 'p': // prime for jump
				right_vesc_target = 76;
				left_vesc_target = 340;
				break;

			case 'm': // midway jump position
				right_vesc_target = 130;
				left_vesc_target = 40;
				break;

			case 'j': // jump position
				right_vesc_target = 188;
				left_vesc_target = 100;
				break;

			case 'r': // neutral stance position
				right_vesc_target = 186;
				left_vesc_target = 120;
				break;

			case 'd': // vertically down position
				right_vesc_target = 226-10;
				left_vesc_target = 140-10;
			default:
				break;
		}
		// Only read the last byte in the buffer
		Serial.clear();
	}
}

void loop() {
	// State machine switch control
	switch(teensy_state) {

		// IDLE state: don't do anything but check for button press
		case IDLE:
			print_status();
		  if(digitalReadFast(e_stop_pin) == LOW) {
				teensy_state = IDLE_BUTTON_DOWN;
			}
			break;
		// IDLE Button Down State: wait until the button is released and then start
		// the running state
		case IDLE_BUTTON_DOWN:
			print_status();
		  if(digitalReadFast(e_stop_pin) == HIGH) {
				teensy_state = RUNNING;

				// clear serial data so running subroutine doesn't see it
				Serial.clear();

				delay(500);

			}
			break;

		// ESTOP: the e-stop button was pressed so stop sending current to the motors!
		case ESTOP:
			// Handle the ESTOP behavior: if it has been pressed or is currently pressed
			// then send a zero current command over to the VESC and delay 500ms
			right_vesc.set_current(0.0);
			left_vesc.set_current(0.0);

			print_status();
			delay(200);

			break;

		// RUNNING state: do whatever PID control etc needs to be done
		case RUNNING:
			// print_status();
			// check for estop
			if(digitalReadFast(e_stop_pin) == LOW) {
				teensy_state = ESTOP;
				break; // redundant
			} else {
				// IMPORTANT
				// read any jump commands
				process_serial();

				float last_read_angle;
				int transmitter_ID;

				// right_vesc.print_debug();
				if(readAngleOverCAN(CANTransceiver,last_read_angle,transmitter_ID)) { // time to read is 9 us
					if(transmitter_ID == RM_CHANNEL_ID) {

						right_vesc.update_deg(last_read_angle); // 4 micros

						// SOMETHING VERY Wrong with using has_read_angle logic

						long then = micros();
						right_vesc.pid_update(right_vesc_target); // 24 micros
						loop_time = micros() - then;
					}
					if(transmitter_ID == LM_CHANNEL_ID) {

						// left_vesc.set_current(0);
						left_vesc.update_deg(last_read_angle);
						left_vesc.pid_update(left_vesc_target);
					}
				}
			}
			break;

		// operate this loop at 500hz
		/*
		if(last_send > 2) {
			last_send -= 2;

			int32_t command = continuousRotationCommand(millis()/1000.0, 1000000, 0.5);

			sendMultipliedAngleOverCAN(CANTransceiver,command);
			// Serial.println(command/1000000);
		}
		*/
	}

}
