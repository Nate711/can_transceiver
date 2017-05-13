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
  last_error = error;

  float pterm = pd_constants.Kp*error;
  last_pterm = pterm;

  float angular_deriv = pid_utils_angle_difference(error, last_angular_error) /
      (dt);
  last_error_deriv = angular_deriv;
  last_angular_error = error;

  float dterm = pd_constants.Kd*angular_deriv;
  last_dterm = dterm;

  float command = pterm + dterm;
  last_command = constrain(command,-threshold,threshold);

  return last_command;
}

float AngularPDController::get_error() {
  return last_error;
}

float AngularPDController::get_error_deriv() {
  return last_error_deriv;
}

float AngularPDController::get_command() {
  return last_command;
}

void AngularPDController::get_command(float& command) {
  command = last_command;
}
void AngularPDController::get_error_terms(float& pterm, float& dterm) {
  pterm = last_pterm;
  dterm = last_dterm;
}

float AngularPDController::get_pterm() {
  return last_pterm;
}

float AngularPDController::get_dterm() {
  return last_dterm;
}

void AngularPDController::get_gains(float& Kp, float& Kd) {
  Kp = pd_constants.Kp;
  Kd = pd_constants.Kd;
}
