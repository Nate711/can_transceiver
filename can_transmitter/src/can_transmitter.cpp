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

// Variable to keep track of micros elapsed since last time sent a current
// command to the right VESC
elapsedMicros RM_current_command = 0;

// Variable to keep track of time since command to RM was sent
bool LM_current_command_sent = false;

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

		case RUNNING:
			print_status();

			if(digitalReadFast(e_stop_pin) == LOW) {
				teensy_state = ESTOP;
				break; // redundant
			} else {
				// IMPORTANT
				// read any jump commands
				process_serial();


				/****** Handle any position messages from VESCs ******/
				float last_read_angle;
				int transmitter_ID;

				// time to read angle over can is 9 us
				if(readAngleOverCAN(CANTransceiver, last_read_angle, transmitter_ID)) {
					switch(transmitter_ID) {
						case RM_CHANNEL_ID:
							right_vesc.update_deg(last_read_angle);


							break;
						case LM_CHANNEL_ID:
							left_vesc.update_deg(last_read_angle);


							break;
					}
				}
				/******* End of handling messages from VESCs ******/

				/****** Send current messages to VESCs *******/
				// Send position current commands at 1khz aka 1000 us per loop
				// LM command should be sent halfway between RM commands
				if(RM_current_command > PID_PERIOD) {
					RM_current_command = 0;
					// Start of a new cycle, LM should be sent after PID_PERIOD/2 us
					LM_current_command_sent = false;

					// START right vesc commands
					right_vesc.set_position(right_vesc_target);



					// right_vesc.pid_update(180.0); // takes 24 micros to complete
				}

				// This look should execute halfway between every RM current command
				if(RM_current_command > PID_PERIOD/2 && !LM_current_command_sent) {
					LM_current_command_sent = true;

					left_vesc.set_position(left_vesc_target);
					left_vesc.set_pid_position_constants(0.03, 0.0, 0.0002);

					// left_vesc.pid_update(0.0);
				}
				/****** End of sending current messages to VESCs *******/
			}
			break;

		/* OLD master code
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
			*/
	}

}
