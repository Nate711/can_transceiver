#include <FlexCAN.h>
#include "AngularPDController.h"
#include "constants.h"
#include "buffer.h"
#include "utils.h"
#include "VESC.h"
#include "globals.h"

/**
 * Check for and read motor angles over CAN
 * Returns true and stores value if message received, otherwise return false
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
		Serial.print(right_vesc.read());
		Serial.print("\t");
		Serial.println(left_vesc.read());
	}
}

void process_serial() {
	if(Serial.available()) {
		char c = Serial.read();

		// Process each different command
		switch(c) {
			// TODO : update how position commands are set
			/*
			case 'w': // hop up state
				left_vesc.set_norm_position_target(90.0);
				right_vesc.set_norm_position_target(90.0);
				break;

			case 's': // hope down state
				left_vesc.set_norm_position_target(0.0);
				right_vesc.set_norm_position_target(0.0);
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
				*/
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
				right_vesc.update_angle(last_read_angle);
				break;
			case LM_CHANNEL_ID:
				left_vesc.update_angle(last_read_angle);
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


	// Initialize VESC objects
	left_vesc.attach(LM_CHANNEL_ID,
									BOOM_VESC_OFFSET,
									BOOM_VESC_DIRECTION,
									MAX_CURRENT);

	right_vesc.attach(RM_CHANNEL_ID,
										OPP_VESC_OFFSET,
										OPP_VESC_DIRECTION,
										MAX_CURRENT);
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

		// ESTOP state: the e-stop button was pressed so stop sending current to the motors!
		case ESTOP:
			// Handle the ESTOP behavior: if it has been pressed or is currently pressed
			// then send a zero current command over to the VESC and delay 200ms
			right_vesc.write_current(0.0);
			left_vesc.write_current(0.0);

			print_status();
			delay(200);

			break;

		// RUNNING state: executes motor commands
		case RUNNING:
			// Prints the status over serial every 100 ms
			print_status();

			// IMPORTANT: read any jump commands
			process_serial();

			// Process any CAN messages from the motor controllers
			process_CAN_messages();


			/****** Send current messages to VESCs *******/
			// Send position current commands at 1khz aka 1000 us per loop
			// LM command should be sent halfway between RM commands
			if(RM_current_command > UPDATE_PERIOD) {
				RM_current_command = 0;

				// Start of a new cycle, LM should be sent after PID_PERIOD/2 us
				LM_current_command_sent = false;

				// Set the PID constants and position
				right_vesc.write_pos_and_pid_gains(0.05, 0, 0.0005, 0.0);

			}

			// This should execute halfway between every RM current command
			if(RM_current_command > UPDATE_PERIOD/2 && !LM_current_command_sent) {
				LM_current_command_sent = true;

				// Set the PID constants and position
				left_vesc.write_pos_and_pid_gains(0.05, 0, 0.0005, 0.0);
			}
			/****** End of sending current messages to VESCs *******/

			break;
	}
}
