#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include "datatypes.h"

#include "buffer.h"
#include "utils.h"

int led = 13;
// create CAN object
FlexCAN CANTransceiver(500000); // default is 500k baud
static CAN_message_t msg;

unsigned long last_send = micros();

// float pid_last_angle = 0;
// long time_last_angle_read;
// float last_degps=0;

const float MAX_CURRENT = 10.0;

struct PD_Constants {
	float Kp;
	// float Ki;
	float Kd;
} VESC_pd_k;

struct Motor_State {
	float last_angle;
	float last_degps;
	long time_last_angle_read;
} tmotor_state;

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
bool readAngleOverCAN(FlexCAN& CANrx, float& last_angle_received) {
	// NOTE: will parse thru multiple angles if >1 messages received btn loops

	// keep track if a value was received
	bool read = false;

	// parse all available can messages
	while( CANrx.read(msg)) {
		read = true;

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
void sendMultipliedAngleOverCAN(FlexCAN& CANtx, const uint32_t& multiplied_angle) {
	CAN_message_t msg;

	// use a controller ID of 255 (VESC will accept all 255 ID messages)
	// CAN_PACKET_SET_POS byte goes on the left of the ID byte
	msg.id = 255 | ((int32_t)CAN_PACKET_SET_POS << 8);
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
void sendMultipliedCurrentOverCAN(FlexCAN& CANtx, const int32_t& current) {

	CAN_message_t msg;

	// use a controller ID of 255 (VESC will accept all 255 ID messages)
	// CAN_PACKET_SET_CURRENT byte goes on the left of the ID byte
	msg.id = 255 | ((int32_t)CAN_PACKET_SET_CURRENT << 8);
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

float positionPidControl(const float& angle_now,
	float& angle_last,
	const float& set_point,
	const int& microsTimeDelta,
	const PD_Constants& pd_k) {

	/**
	* fucking weird error:
	* if angle_diff defined and calced up here (before error is calced), the vesc freezes up
	TODO: figure out what the freeze actually is!!
	*/

	float error = utils_angle_difference(angle_now, set_point);
	float angle_diff = utils_angle_difference(angle_now, angle_last);


	float error_deriv = (angle_diff)*1000000.0 /
		(float)microsTimeDelta; // deg /s

	// error_deriv = 0.5*error_deriv + 0.5*last_degps;
	tmotor_state.last_degps = error_deriv;

	angle_last = angle_now;

	// NEGATIVE SIGN IS IMPORTANT
	float res =  -(error*pd_k.Kp + error_deriv*pd_k.Kd);
	res = constrain(res, -1.0, 1.0);

	if(micros() - last_send > 50*1000) {
		last_send = micros();

		Serial.print("O: ");
		Serial.print(res);
		Serial.print(" \tEr: ");
		Serial.print(error);
		Serial.print(" \tw:  ");
		Serial.print(error_deriv);
		Serial.print(" \tKp: ");
		Serial.print(error*pd_k.Kp);
		Serial.print(" \tKd: ");
		Serial.println(error_deriv*pd_k.Kd);
	}

	/**
	* sets a zero current if the speed is less than or greater than 100deg/s
	**/
	if(abs(error_deriv) > 1000) return 0;

	// if(abs(error) < 1.0) {
		// res = constrain(res, -0.5, 0.5);
	// }

	return res;
}

void setup() {
  // init CAN bus
  CANTransceiver.begin();
  pinMode(led, OUTPUT);
	digitalWrite(led, HIGH);
  delay(1000);
  Serial.println("CAN Transmitter Initialized");

	tmotor_state.time_last_angle_read = micros();

	// VESC_pd_k.Kp = 0.07;
	// VESC_pd_k.Kd = 0.0015;
	VESC_pd_k.Kp = 0.05;
	VESC_pd_k.Kd = 0.0005;
}

void loop() {

	float last_read_angle;
	if(readAngleOverCAN(CANTransceiver,last_read_angle)) {
		// Serial.println(last_read_angle);

		// float angle = continuousRotationCommand(millis()/1000.0, 1000000, 0.5)/1000000.0;
		// float angle = squareWaveCommand(millis()/1000.0, 1000, 25, 5)/1000.0;
		float angle_set_point = 180.0;
		long time_delta = micros() - tmotor_state.time_last_angle_read;

		float current_command = MAX_CURRENT*positionPidControl(last_read_angle,
				tmotor_state.last_angle,
				angle_set_point,
				time_delta,
				VESC_pd_k);

		tmotor_state.time_last_angle_read = micros();

		// sendMultipliedCurrentOverCAN(CANTransceiver,
			// (int32_t)(current_command * 1000.0));

		// Serial.print(millis());
		// Serial.print(" ");
		// Serial.println(current_command);
	}


	// operate this loop at 500hz
	/*
	if(micros() - last_send > 2*1000) {
		last_send = micros();

		int32_t command = continuousRotationCommand(millis()/1000.0, 1000000, 0.5);

		sendMultipliedAngleOverCAN(CANTransceiver,command);
	}*/

}
