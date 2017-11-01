#include <FlexCAN.h>
#include "AngularPDController.h"
#include "constants.h"
#include "buffer.h"
#include "utils.h"
#include "VESC.h"
#include "globals.h"

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

		// Print angle readings from the VESCs
		// static elapsedMillis lastCANPrint = 0;
		// if(lastCANPrint > 500) {
		// 	lastCANPrint = 0;
		// 	Serial.print(transmitter_ID);
		// 	Serial.print(" ");
		// 	Serial.println(last_angle_received);
		// }
  }
	return read;
}

// variables for debouncing
// keeps track of when the state machine went from idle to staging
long idle_to_staging;
// keeps track of when staging to running
long staging_to_running;

void E_STOP_ISR() {
	switch(teensy_state) {
	case IDLE:
		teensy_state = STAGING;
		idle_to_staging = millis();
		break;
	case STAGING:
		// checks that went from idle to staging more than 500ms ago
		// to ignore button bouncing
		if(millis() - idle_to_staging > 500) {
			teensy_state = RUNNING;
			staging_to_running = millis();
		}
		break;
	case RUNNING:
		// checks that went from staging to running more than 500ms ago
		// to ignore button bouncing
		if(millis() - staging_to_running > 500) {
			teensy_state = ESTOP;
		}
		break;
	case ESTOP:
		break;
	}
}

void print_status() {
	if(last_print_debug > 100) {
		last_print_debug -= 100;

		if(true || last_printed_state != teensy_state
			|| (teensy_state == RUNNING && last_running_state != teensy_running_state)) {

			last_printed_state = teensy_state;
			Serial.print("State: ");
			switch(teensy_state) {
				case IDLE:
					Serial.println("IDLE");
					break;

				case STAGING:
					Serial.println("STAGING");
					break;

				case ESTOP:
					Serial.println("E-STOP");
					break;

				case RUNNING:
					last_running_state = teensy_running_state;
					Serial.print("RUNNING: ");
					switch(teensy_running_state) {
					case HOP_UP:
						Serial.println("HOP_UP");
						break;
					case HOP_DOWN:
						Serial.println("HOP_DOWN");
						break;
					default:
						break;

					}
					break;
				}

		}
	}
}

void print_shit() {
	if(last_print_shit > 10) {
		last_print_shit -= 10;

		// Serial.println(loop_time);
		Serial.print(right_vesc.get_norm_position_target());
		Serial.print("\t");
		Serial.println(left_vesc.get_norm_position_target());
	}
}

void process_serial() {
	if(Serial.available()) {
		char c = Serial.read();

		// Process each different command
		switch(c) {
			case 'w': // hop up state
				teensy_running_state = HOP_UP;
				break;
			case 's': // hope down state
				teensy_running_state = HOP_DOWN;
				break;
			case 'p': // prime for jump
				left_vesc.set_norm_position_target(-45);
				right_vesc.set_norm_position_target(-135);
				break;

			case 'm': // midway jump position
				left_vesc.set_norm_position_target(0);
				right_vesc.set_norm_position_target(180);
				break;

			case 'j': // jump position
				left_vesc.set_norm_position_target(75);
				right_vesc.set_norm_position_target(105);
				break;

			case 'd': // vertically down position
				left_vesc.set_norm_position_target(90);
				right_vesc.set_norm_position_target(90);
			default:
				break;
		}
		// Only read the last byte in the buffer
		Serial.clear();
	}
}

void process_CAN_messages() {
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
}

void setup() {
	// init stop button and interrupt
	pinMode(e_stop_pin,INPUT_PULLUP);
	attachInterrupt(e_stop_pin, E_STOP_ISR, RISING);

  // init CAN bus
  CANTransceiver.begin();
  pinMode(led, OUTPUT);
	digitalWrite(led, HIGH);
  delay(1000);
  Serial.println("CAN Transmitter Initialized");
}

void loop() {
	// State machine switch control
	switch(teensy_state) {

		// IDLE state: don't do anything
		case IDLE:
			print_status();
			break;

		// STAGING state: wait until 2nd button press then go into running state
		case STAGING:
			print_status();

			Serial.clear();
			delay(500);
			break;

		// ESTOP: the e-stop button was pressed so stop sending current to the motors!
		case ESTOP:
			// Handle the ESTOP behavior: if it has been pressed or is currently pressed
			// then send a zero current command over to the VESC and delay 200ms
			right_vesc.set_current(0.0);
			left_vesc.set_current(0.0);

			print_status();
			delay(200);

			break;

		case RUNNING:
			print_status();

			// IMPORTANT: read any jump commands
			process_serial();

			process_CAN_messages();

			// Serial.println(millis());

			// For now use only hop up and down states
			switch(teensy_running_state) {
			case HOP_UP:
				// set target pid positions

				// Set to zero in order to determine offsets
				left_vesc.set_norm_position_target(90.0);
				right_vesc.set_norm_position_target(90.0);

				// high gain pid constants
				left_vesc.update_vesc_position_pid_constants(0.05,0,0.0004);
				right_vesc.update_vesc_position_pid_constants(0.05,0,0.0004);

				// AUTOMODE: transition to HOP_DOWN when leg fully extended
				break;

			case HOP_DOWN:
				// set target pid positions

				// Set to zero to find offset
				left_vesc.set_norm_position_target(0.0);
				right_vesc.set_norm_position_target(0.0);

				left_vesc.update_vesc_position_pid_constants(0.05,0,0.0008);
				right_vesc.update_vesc_position_pid_constants(0.05,0,0.0008);

			  // AUTOMODE: transition to HOP_UP when leg retracted
				break;
			default:
				break;
			}



			/****** Send current messages to VESCs *******/
			// Send position current commands at 1khz aka 1000 us per loop
			// LM command should be sent halfway between RM commands
			if(RM_current_command > UPDATE_PERIOD) {
				RM_current_command = 0;

				// Start of a new cycle, LM should be sent after PID_PERIOD/2 us
				LM_current_command_sent = false;

				right_vesc.set_normalized_position_with_constants();
			}

			// This should execute halfway between every RM current command
			if(RM_current_command > UPDATE_PERIOD/2 && !LM_current_command_sent) {
				LM_current_command_sent = true;

				left_vesc.set_normalized_position_with_constants();
			}
			/****** End of sending current messages to VESCs *******/

			break;
	}
}
