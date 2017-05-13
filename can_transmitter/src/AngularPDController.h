#ifndef ANGULARPDCONTROLLER_H
#define ANGULARPDCONTROLLER_H

class AngularPDController {
private:
  float Kp,Ki,Kd;
  float threshold;

  float last_computed_command;

  float last_angular_error;

  struct PD_Constants {
  	float Kp;
  	float Ki;
  	float Kd;
  } pd_constants;

public:
  AngularPDController(float kp, float kd, float threshold1);
  float compute_command(float error, float dt);
};

#endif /*ANGULARPDCONTROLLER_H*/
