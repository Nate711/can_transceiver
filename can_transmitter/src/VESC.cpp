#include "VESC.h"
// #include <FlexCAN.h>

void utils_norm_angle(float& angle);

/**
 * Constructs VESC object with initial params
 * I dont understand member initializer lists :( but CANtx breaks without it
 */
VESC::VESC(float encoder_offset1,
          int encoder_direction1,
          int8_t controller_channel_ID1,
          float max_current1,
          float max_speed1,
          FlexCAN& cantx) : CANtx(cantx){

  encoder_offset = encoder_offset1;
  encoder_direction = encoder_direction1;

  controller_channel_ID = controller_channel_ID1;

  max_current = max_current1;
  max_speed = max_speed1;

  CANtx = cantx;
}

/**
 * Updates the VESC objects knowledge of the motor angle
 */
void VESC::update_deg(float raw_deg) {
  // correct direction and apply offset
  true_deg = (encoder_direction==1)?raw_deg:(-raw_deg) + encoder_offset;
  utils_norm_angle(true_deg);
}

/**
 * Returns the VESC object's last known knowledge of motor position
 */
float VESC::read_corrected_deg() {
  return true_deg;
}

/**
 * Sends current command to the VESC
 */
void VESC::set_current(float current) {
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

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize.
 */
void utils_norm_angle(float& angle) {
	angle = fmodf(angle, 360.0);

	if (angle < 0.0) {
		angle += 360.0;
	}
}
