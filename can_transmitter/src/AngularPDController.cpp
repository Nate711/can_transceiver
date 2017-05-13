#include "AngularPDController.h"
#include <Arduino.h>

AngularPDController::AngularPDController(float kp, float kd, float threshold1) {
  pd_constants.Kp = kp;
  pd_constants.Kd = kd;

  threshold = threshold1;
}

float pid_utils_angle_difference(float angle1, float angle2) {
	// Faster in most cases
	float difference = angle1 - angle2;
	while (difference < -180.0) difference += 2.0 * 180.0;
	while (difference > 180.0) difference -= 2.0 * 180.0;
	return difference;
}

float AngularPDController::compute_command(float error, float dt) {
  float pterm = pd_constants.Kp*error;

  float angular_deriv = pid_utils_angle_difference(error, last_angular_error)/(dt);
  float dterm = pd_constants.Kd*angular_deriv;
  float command = pterm + dterm;

  last_angular_error = error;

  return constrain(command,-threshold,threshold);
}
